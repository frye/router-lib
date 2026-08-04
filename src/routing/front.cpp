#include "routing/front.hpp"

#include "routing/geodesy.hpp"
#include "sailroute/front.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <tuple>
#include <vector>

namespace sailroute {
namespace detail {

namespace {

// Compute an antimeridian-aware longitude centroid from a set of coordinates
// by averaging unit vectors on the circle.
double centroid_longitude(std::span<const Coordinate> points) noexcept {
    double sum_cos = 0.0;
    double sum_sin = 0.0;
    for (const Coordinate& p : points) {
        const double lon_rad = p.longitude_degrees * std::numbers::pi / 180.0;
        sum_cos += std::cos(lon_rad);
        sum_sin += std::sin(lon_rad);
    }
    if (sum_cos == 0.0 && sum_sin == 0.0) {
        // Unit vectors cancelled exactly (e.g. antipodal symmetric inputs).
        // Return 0.0 — deterministic and independent of input order.
        return 0.0;
    }
    return std::atan2(sum_sin, sum_cos) * 180.0 / std::numbers::pi;
}

Coordinate frontier_centroid(std::span<const Coordinate> points) {
    std::vector<Coordinate> ordered{points.begin(), points.end()};
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](Coordinate left, Coordinate right) {
            return std::tie(
                       left.latitude_degrees,
                       left.longitude_degrees) <
                std::tie(
                       right.latitude_degrees,
                       right.longitude_degrees);
        });
    double sum_lat = 0.0;
    for (const Coordinate& p : ordered) {
        sum_lat += p.latitude_degrees;
    }
    const double n = static_cast<double>(ordered.size());
    return Coordinate{sum_lat / n, centroid_longitude(ordered)};
}

// Signed cross-track offset: positive = starboard (right of destination bearing),
// negative = port.  We compute it as the signed perpendicular component of the
// bearing from centroid to each point relative to the forward bearing.
double signed_cross_track(
    double point_bearing_degrees,
    double forward_bearing_degrees,
    double distance_nautical_miles) noexcept {
    // Angle from the forward axis to the point bearing (clockwise positive).
    double delta = normalize_degrees(point_bearing_degrees - forward_bearing_degrees);
    if (delta > 180.0) {
        delta -= 360.0;  // Now in (-180, 180], positive = right = starboard.
    }
    const double delta_rad = delta * std::numbers::pi / 180.0;
    return distance_nautical_miles * std::sin(delta_rad);
}

double along_track_component(
    double point_bearing_degrees,
    double forward_bearing_degrees,
    double distance_nautical_miles) noexcept {
    double delta = normalize_degrees(point_bearing_degrees - forward_bearing_degrees);
    if (delta > 180.0) {
        delta -= 360.0;
    }
    const double delta_rad = delta * std::numbers::pi / 180.0;
    return distance_nautical_miles * std::cos(delta_rad);
}

// Integer band index: floor(cross_track / band_width).
// Handles negative cross_track correctly via floor semantics.
std::int64_t band_index(double cross_track, double band_width) noexcept {
    return static_cast<std::int64_t>(std::floor(cross_track / band_width));
}

struct FrontPoint {
    Coordinate position;
    double remaining_distance{};
    double along_track{};
    double cross_track{};
    double bearing_delta{};
    bool at_centroid{};
    bool is_anchor{};
    std::int64_t band{};
};

bool same_coordinate(Coordinate left, Coordinate right) noexcept {
    return left.latitude_degrees == right.latitude_degrees &&
        left.longitude_degrees == right.longitude_degrees;
}

bool valid_segment_policy(DestinationFrontSegmentPolicy policy) noexcept {
    return policy == DestinationFrontSegmentPolicy::provisional_component ||
        policy == DestinationFrontSegmentPolicy::all_meaningful_components;
}

}  // namespace

std::optional<Error> build_destination_front_into(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles,
    const DestinationFrontOptions& options,
    std::vector<Coordinate>& front_points,
    std::vector<IsochroneFrontSegment>& segments) {
    front_points.clear();
    segments.clear();
    if (retained_points.empty() || !is_valid(destination)) {
        return build_destination_front_into(
            retained_points,
            destination,
            Coordinate{},
            band_width_nautical_miles,
            options,
            front_points,
            segments);
    }
    for (const Coordinate point : retained_points) {
        if (!is_valid(point)) {
            return Error{
                ErrorCode::invalid_argument,
                "retained_points contains an invalid coordinate"};
        }
    }

    const auto provisional = std::min_element(
        retained_points.begin(),
        retained_points.end(),
        [destination](Coordinate left, Coordinate right) {
            const double left_distance =
                great_circle_distance_nautical_miles(left, destination);
            const double right_distance =
                great_circle_distance_nautical_miles(right, destination);
            if (left_distance != right_distance) {
                return left_distance < right_distance;
            }
            return std::tie(
                       left.latitude_degrees,
                       left.longitude_degrees) <
                std::tie(
                       right.latitude_degrees,
                       right.longitude_degrees);
        });
    return build_destination_front_into(
        retained_points,
        destination,
        *provisional,
        band_width_nautical_miles,
        options,
        front_points,
        segments);
}

std::optional<Error> build_destination_front_into(
    std::span<const Coordinate> candidate_points,
    Coordinate destination,
    Coordinate anchor,
    double band_width_nautical_miles,
    const DestinationFrontOptions& options,
    std::vector<Coordinate>& front_points,
    std::vector<IsochroneFrontSegment>& segments) {
    front_points.clear();
    segments.clear();

    if (!is_valid(destination)) {
        return Error{
            ErrorCode::invalid_argument,
            "destination coordinate must contain finite latitude [-90, 90] and longitude [-180, 180]"};
    }
    if (!std::isfinite(band_width_nautical_miles) || band_width_nautical_miles <= 0.0) {
        return Error{
            ErrorCode::invalid_argument,
            "band_width_nautical_miles must be finite and positive"};
    }
    if (!std::isfinite(options.half_angle_degrees) ||
        options.half_angle_degrees <= 0.0 ||
        options.half_angle_degrees > 180.0) {
        return Error{
            ErrorCode::invalid_argument,
            "destination front half_angle_degrees must be finite and in (0, 180]"};
    }
    if (!valid_segment_policy(options.segment_policy)) {
        return Error{
            ErrorCode::invalid_argument,
            "destination front segment_policy is unsupported"};
    }

    for (const Coordinate& p : candidate_points) {
        if (!is_valid(p)) {
            return Error{
                ErrorCode::invalid_argument,
                "candidate_points contains an invalid coordinate"};
        }
    }

    if (candidate_points.empty()) {
        return std::nullopt;
    }
    if (!is_valid(anchor)) {
        return Error{
            ErrorCode::invalid_argument,
            "destination front anchor must contain finite latitude [-90, 90] and longitude [-180, 180]"};
    }
    if (std::none_of(
            candidate_points.begin(),
            candidate_points.end(),
            [anchor](Coordinate point) {
                return same_coordinate(point, anchor);
            })) {
        return Error{
            ErrorCode::invalid_argument,
            "destination front anchor must identify a candidate point"};
    }

    // Single-point degenerate case.
    if (candidate_points.size() == 1U) {
        front_points.push_back(candidate_points.front());
        segments.push_back(IsochroneFrontSegment{0U, 1U});
        return std::nullopt;
    }

    const Coordinate centroid = frontier_centroid(candidate_points);
    const double forward_bearing = initial_bearing_degrees(centroid, destination);

    std::vector<FrontPoint> projected;
    projected.reserve(candidate_points.size());
    for (const Coordinate& p : candidate_points) {
        const double dist = great_circle_distance_nautical_miles(centroid, p);
        double bearing = 0.0;
        if (dist > 0.0) {
            bearing = initial_bearing_degrees(centroid, p);
        }
        const double at = along_track_component(bearing, forward_bearing, dist);
        const double ct = signed_cross_track(bearing, forward_bearing, dist);
        projected.push_back(FrontPoint{
            p,
            great_circle_distance_nautical_miles(p, destination),
            at,
            ct,
            angular_difference_degrees(bearing, forward_bearing),
            dist == 0.0,
            same_coordinate(p, anchor),
            band_index(ct, band_width_nautical_miles)});
    }

    // Include exact angular boundaries and the centroid. If no non-centroid
    // point is within the aperture, preserve the fallback by retaining all
    // points.
    {
        // Absorb only projection-scale roundoff at an inclusive boundary.
        const double boundary_tolerance =
            32.0 * std::numeric_limits<double>::epsilon() *
            options.half_angle_degrees;
        const bool any_within = std::any_of(
            projected.begin(),
            projected.end(),
            [&options, boundary_tolerance](const FrontPoint& fp) {
                return !fp.at_centroid &&
                       fp.bearing_delta <=
                           options.half_angle_degrees + boundary_tolerance;
            });

        if (any_within) {
            projected.erase(
                std::remove_if(
                    projected.begin(),
                    projected.end(),
                    [&options, boundary_tolerance](const FrontPoint& fp) {
                        return (options.segment_policy !=
                                    DestinationFrontSegmentPolicy::
                                        all_meaningful_components ||
                                !fp.is_anchor) &&
                               !fp.at_centroid &&
                               fp.bearing_delta >
                                   options.half_angle_degrees +
                                       boundary_tolerance;
                    }),
                projected.end());
        }
        // If !any_within, per-band selection chooses the best available points.
    }

    if (projected.empty()) {
        return std::nullopt;
    }

    std::int64_t principal_band = 0;
    if (options.segment_policy ==
        DestinationFrontSegmentPolicy::all_meaningful_components) {
        const auto anchor_it = std::find_if(
            projected.begin(),
            projected.end(),
            [](const FrontPoint& point) {
                return point.is_anchor;
            });
        principal_band = anchor_it->band;
    } else {
        const auto provisional_it = std::min_element(
            projected.begin(),
            projected.end(),
            [](const FrontPoint& left, const FrontPoint& right) {
                if (left.remaining_distance != right.remaining_distance) {
                    return left.remaining_distance < right.remaining_distance;
                }
                return std::tie(
                           left.position.latitude_degrees,
                           left.position.longitude_degrees) <
                    std::tie(
                           right.position.latitude_degrees,
                           right.position.longitude_degrees);
            });
        principal_band = provisional_it->band;
    }

    // Sort by band, then remaining distance, along-track progress, and stable
    // coordinates so candidate input order cannot affect the winner.
    std::sort(
        projected.begin(),
        projected.end(),
        [&options](const FrontPoint& left, const FrontPoint& right) {
            if (left.band != right.band) {
                return left.band < right.band;
            }
            if (options.segment_policy ==
                    DestinationFrontSegmentPolicy::all_meaningful_components &&
                left.remaining_distance != right.remaining_distance) {
                return left.remaining_distance < right.remaining_distance;
            }
            if (options.segment_policy ==
                    DestinationFrontSegmentPolicy::all_meaningful_components &&
                left.is_anchor != right.is_anchor) {
                return left.is_anchor;
            }
            if (left.along_track != right.along_track) {
                return left.along_track > right.along_track;
            }
            return std::tie(
                       left.position.latitude_degrees,
                       left.position.longitude_degrees) <
                   std::tie(
                       right.position.latitude_degrees,
                       right.position.longitude_degrees);
        });

    // Keep only the first (best) entry per band.
    std::vector<FrontPoint> band_winners;
    band_winners.reserve(projected.size());
    for (const FrontPoint& fp : projected) {
        if (band_winners.empty() || band_winners.back().band != fp.band) {
            band_winners.push_back(fp);
        }
    }

    front_points.reserve(band_winners.size());
    std::size_t run_begin = 0U;
    while (run_begin < band_winners.size()) {
        std::size_t run_end = run_begin + 1U;
        while (run_end < band_winners.size() &&
               band_winners[run_end].band ==
                   band_winners[run_end - 1U].band + 1) {
            ++run_end;
        }

        const bool contains_anchor =
            band_winners[run_begin].band <= principal_band &&
            principal_band <= band_winners[run_end - 1U].band;
        const bool retain_secondary =
            options.segment_policy ==
                DestinationFrontSegmentPolicy::all_meaningful_components &&
            run_end - run_begin >=
                options.minimum_secondary_segment_points;
        if (contains_anchor || retain_secondary) {
            std::size_t segment_start = front_points.size();
            for (std::size_t index = run_begin; index < run_end; ++index) {
                const Coordinate position = band_winners[index].position;
                if (front_points.size() > segment_start &&
                   std::abs(
                       position.longitude_degrees -
                       front_points.back().longitude_degrees) > 180.0) {
                   segments.push_back(IsochroneFrontSegment{
                       segment_start,
                       front_points.size() - segment_start});
                   segment_start = front_points.size();
                }
                front_points.push_back(position);
            }
            segments.push_back(IsochroneFrontSegment{
                segment_start,
                front_points.size() - segment_start});
        }

        run_begin = run_end;
    }

    return std::nullopt;
}

}  // namespace detail

// ── Public API ──────────────────────────────────────────────────────────────

Result<IsochroneFront> build_destination_front(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles) {
    return build_destination_front(
        retained_points,
        destination,
        band_width_nautical_miles,
        DestinationFrontOptions{});
}

Result<IsochroneFront> build_destination_front(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles,
    const DestinationFrontOptions& options) {
    IsochroneFront result;
    if (auto error = detail::build_destination_front_into(
            retained_points,
            destination,
            band_width_nautical_miles,
            options,
            result.points,
            result.segments);
        error.has_value()) {
        return *error;
    }
    return result;
}

}  // namespace sailroute
