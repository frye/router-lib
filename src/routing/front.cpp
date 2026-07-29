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

Coordinate frontier_centroid(std::span<const Coordinate> points) noexcept {
    double sum_lat = 0.0;
    for (const Coordinate& p : points) {
        sum_lat += p.latitude_degrees;
    }
    const double n = static_cast<double>(points.size());
    return Coordinate{sum_lat / n, centroid_longitude(points)};
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
    double along_track{};
    double cross_track{};
    std::int64_t band{};
};

}  // namespace

std::optional<Error> build_destination_front_into(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles,
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

    for (const Coordinate& p : retained_points) {
        if (!is_valid(p)) {
            return Error{
                ErrorCode::invalid_argument,
                "retained_points contains an invalid coordinate"};
        }
    }

    if (retained_points.empty()) {
        return std::nullopt;
    }

    // Single-point degenerate case.
    if (retained_points.size() == 1U) {
        front_points.push_back(retained_points.front());
        segments.push_back(IsochroneFrontSegment{0U, 1U});
        return std::nullopt;
    }

    // ── Frame ──────────────────────────────────────────────────────────────
    const Coordinate centroid = frontier_centroid(retained_points);
    const double forward_bearing = initial_bearing_degrees(centroid, destination);

    // ── Project all points into the local frame ────────────────────────────
    std::vector<FrontPoint> projected;
    projected.reserve(retained_points.size());
    for (const Coordinate& p : retained_points) {
        const double dist = great_circle_distance_nautical_miles(centroid, p);
        double bearing = 0.0;
        if (dist > 0.0) {
            bearing = initial_bearing_degrees(centroid, p);
        }
        const double at = along_track_component(bearing, forward_bearing, dist);
        const double ct = signed_cross_track(bearing, forward_bearing, dist);
        projected.push_back(FrontPoint{p, at, ct, band_index(ct, band_width_nautical_miles)});
    }

    // ── Half-space filter: keep only destination-facing points ─────────────
    // Points with along_track <= 0 are behind or on the midpoint perpendicular.
    // Keep >= 0 so a point exactly at the centroid (dist == 0) is retained.
    // Special case: if ALL points have along_track <= 0 (destination is behind
    // the whole frontier), retain the best-progress point anyway to avoid
    // producing an empty front from a valid frontier.
    {
        const bool any_forward = std::any_of(
            projected.begin(),
            projected.end(),
            [](const FrontPoint& fp) { return fp.along_track > 0.0; });

        if (any_forward) {
            projected.erase(
                std::remove_if(
                    projected.begin(),
                    projected.end(),
                    [](const FrontPoint& fp) { return fp.along_track < 0.0; }),
                projected.end());
        }
        // If !any_forward we keep all points and let the per-band selection
        // pick the least-bad ones.
    }

    if (projected.empty()) {
        return std::nullopt;
    }

    // ── Identify provisional best (closest to destination) ────────────────
    const auto provisional_it = std::min_element(
        projected.begin(),
        projected.end(),
        [&destination](const FrontPoint& left, const FrontPoint& right) {
            return great_circle_distance_nautical_miles(
                       left.position, destination) <
                   great_circle_distance_nautical_miles(
                       right.position, destination);
        });
    const std::int64_t provisional_band = provisional_it->band;

    // ── Per-band selection: best along-track progress, deterministic tie-break
    // Sort by band, then by descending along_track, then by (lat, lon) for
    // determinism.
    std::sort(
        projected.begin(),
        projected.end(),
        [](const FrontPoint& left, const FrontPoint& right) {
            if (left.band != right.band) {
                return left.band < right.band;
            }
            if (left.along_track != right.along_track) {
                return left.along_track > right.along_track;  // descending
            }
            // Lexicographic tie-break for determinism.
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

    // ── Contiguous run containing provisional best ─────────────────────────
    // band_winners is already sorted by band ascending.
    const auto prov_band_it = std::find_if(
        band_winners.begin(),
        band_winners.end(),
        [provisional_band](const FrontPoint& fp) {
            return fp.band == provisional_band;
        });

    std::size_t run_begin = 0U;
    std::size_t run_end = band_winners.size();

    if (prov_band_it != band_winners.end()) {
        const std::size_t prov_pos =
            static_cast<std::size_t>(prov_band_it - band_winners.begin());

        // Extend left.
        run_begin = prov_pos;
        while (run_begin > 0U &&
               band_winners[run_begin - 1U].band ==
                   band_winners[run_begin].band - 1) {
            --run_begin;
        }
        // Extend right.
        run_end = prov_pos + 1U;
        while (run_end < band_winners.size() &&
               band_winners[run_end].band ==
                   band_winners[run_end - 1U].band + 1) {
            ++run_end;
        }
    }

    // band_winners[run_begin..run_end) is the contiguous run, already in
    // port-to-starboard (ascending band / cross_track) order.

    // ── Emit points and segments with antimeridian segmentation ───────────
    // A new segment begins whenever an adjacent pair of points would require
    // crossing the antimeridian (|longitude delta| > 180).
    const std::size_t run_size = run_end - run_begin;
    front_points.reserve(run_size);

    std::size_t seg_start_offset = 0U;
    for (std::size_t i = run_begin; i < run_end; ++i) {
        const Coordinate& pos = band_winners[i].position;
        if (!front_points.empty()) {
            const double lon_delta = std::abs(
                pos.longitude_degrees -
                front_points.back().longitude_degrees);
            if (lon_delta > 180.0) {
                // Antimeridian crossing: close current segment and start new one.
                segments.push_back(IsochroneFrontSegment{
                    seg_start_offset,
                    front_points.size() - seg_start_offset});
                seg_start_offset = front_points.size();
            }
        }
        front_points.push_back(pos);
    }

    if (!front_points.empty()) {
        segments.push_back(IsochroneFrontSegment{
            seg_start_offset,
            front_points.size() - seg_start_offset});
    }

    return std::nullopt;
}

}  // namespace detail

// ── Public API ──────────────────────────────────────────────────────────────

Result<IsochroneFront> build_destination_front(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles) {
    IsochroneFront result;
    if (auto error = detail::build_destination_front_into(
            retained_points,
            destination,
            band_width_nautical_miles,
            result.points,
            result.segments);
        error.has_value()) {
        return *error;
    }
    return result;
}

}  // namespace sailroute
