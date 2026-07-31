#include "sailroute/front.hpp"

#include "../src/routing/geodesy.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace {

// ── Helpers ──────────────────────────────────────────────────────────────────

std::span<const sailroute::Coordinate> segment_points(
    const sailroute::IsochroneFront& front,
    const sailroute::IsochroneFrontSegment& seg) {
    return std::span<const sailroute::Coordinate>{front.points}.subspan(
        seg.point_offset,
        seg.point_count);
}

void require_same_front(
    const sailroute::IsochroneFront& left,
    const sailroute::IsochroneFront& right) {
    REQUIRE(left.points.size() == right.points.size());
    REQUIRE(left.segments.size() == right.segments.size());
    for (std::size_t i = 0U; i < left.points.size(); ++i) {
        REQUIRE(
            left.points[i].latitude_degrees ==
            right.points[i].latitude_degrees);
        REQUIRE(
            left.points[i].longitude_degrees ==
            right.points[i].longitude_degrees);
    }
    for (std::size_t i = 0U; i < left.segments.size(); ++i) {
        REQUIRE(left.segments[i].point_offset == right.segments[i].point_offset);
        REQUIRE(left.segments[i].point_count == right.segments[i].point_count);
    }
}

bool contains_coordinate(
    const sailroute::IsochroneFront& front,
    sailroute::Coordinate expected) {
    return std::any_of(
        front.points.begin(),
        front.points.end(),
        [expected](const sailroute::Coordinate& point) {
            return point.latitude_degrees == expected.latitude_degrees &&
                   point.longitude_degrees == expected.longitude_degrees;
        });
}

std::vector<sailroute::Coordinate> angular_frontier(
    double outer_bearing_degrees) {
    const sailroute::Coordinate centroid{0.0, 0.0};
    const sailroute::Coordinate outer_starboard =
        sailroute::detail::destination_point(
            centroid, outer_bearing_degrees, 180.0);
    const sailroute::Coordinate outer_port =
        sailroute::detail::destination_point(
            centroid, 360.0 - outer_bearing_degrees, 180.0);

    // Symmetric longitudes and the compensating north point keep the computed
    // frontier centroid at the origin.
    return {
        outer_port,
        sailroute::detail::destination_point(centroid, 270.0, 120.0),
        sailroute::detail::destination_point(centroid, 270.0, 60.0),
        {-2.0 * outer_starboard.latitude_degrees, 0.0},
        sailroute::detail::destination_point(centroid, 90.0, 60.0),
        sailroute::detail::destination_point(centroid, 90.0, 120.0),
        outer_starboard,
    };
}

// ── Empty / degenerate ────────────────────────────────────────────────────────

TEST_CASE("destination front empty input produces empty result") {
    auto result = sailroute::build_destination_front(
        std::vector<sailroute::Coordinate>{},
        sailroute::Coordinate{0.0, 0.0},
        10.0);
    REQUIRE(result.has_value());
    REQUIRE(result.value().points.empty());
    REQUIRE(result.value().segments.empty());
}

TEST_CASE("destination front single point is returned as-is") {
    const sailroute::Coordinate pt{10.0, 20.0};
    auto result = sailroute::build_destination_front(
        std::vector<sailroute::Coordinate>{pt},
        sailroute::Coordinate{15.0, 20.0},
        10.0);
    REQUIRE(result.has_value());
    REQUIRE(result.value().points.size() == 1U);
    REQUIRE(result.value().segments.size() == 1U);
    REQUIRE(result.value().segments.front().point_count == 1U);
    REQUIRE(result.value().points.front().latitude_degrees == pt.latitude_degrees);
    REQUIRE(result.value().points.front().longitude_degrees == pt.longitude_degrees);
}

// ── Leading edge: only destination-facing points are selected ─────────────────
//
// Frontier: five points spread east-west at the same latitude, east of
// the destination.  The destination is due west so the forward axis is
// westward.  All five points share the same latitude (cross-track == 0),
// so they all fall in band 0.  The per-band winner is the one with the
// greatest along-track progress — i.e. the closest to the destination,
// which is lon = 2.
//
// Layout (longitude increases eastward):
//   destination at (0, 0)
//   frontier at lon = 2, 4, 6, 8, 10  (all same lat = 0)
//   All points lie on the same latitude → cross-track = 0 → all in band 0.
//   Per-band best = the one closest to the destination = lon 2.
TEST_CASE("destination front selects leading-edge point per band") {
    // Five collinear points on same latitude, destination at lat=0,lon=0.
    // All points lie along the same cross-track band (lat=0 → cross-track=0).
    // The one with greatest along-track (closest to destination) should win.
    const sailroute::Coordinate destination{0.0, 0.0};
    const std::vector<sailroute::Coordinate> frontier{
        {0.0, 10.0},
        {0.0, 8.0},
        {0.0, 6.0},
        {0.0, 4.0},
        {0.0, 2.0},  // closest to destination, should be selected
    };
    auto result = sailroute::build_destination_front(frontier, destination, 10.0);
    REQUIRE(result.has_value());
    REQUIRE(result.value().points.size() == 1U);
    // The selected point must be the one at lon=2.
    REQUIRE_NEAR(result.value().points.front().longitude_degrees, 2.0, 1e-9);
}

// ── Port-to-starboard ordering ────────────────────────────────────────────────
//
// Frontier: seven points spread east-west; destination due north.
// Points and destination on the same meridian ensure the centroid's forward
// bearing is northward.  With band_width = 65 NM (slightly wider than the
// ~60 NM degree of longitude at the equator), consecutive 1-degree steps
// fall into separate, adjacent bands, and the contiguous run from the
// provisional-best (lon=0, closest to destination) extends left and right.
//
// Expected: at least two points survive, and every segment's points are
// non-decreasing in longitude (port = west = negative, starboard = east = positive).
TEST_CASE("destination front orders port-to-starboard") {
    const sailroute::Coordinate destination{5.0, 0.0};
    const std::vector<sailroute::Coordinate> frontier{
        {0.0, -3.0},
        {0.0, -2.0},
        {0.0, -1.0},
        {0.0, 0.0},   // provisional best — closest to destination
        {0.0, 1.0},
        {0.0, 2.0},
        {0.0, 3.0},
    };
    // 65 NM puts each 1-degree step in its own adjacent band (see note above).
    auto result = sailroute::build_destination_front(frontier, destination, 65.0);
    REQUIRE(result.has_value());
    REQUIRE(!result.value().points.empty());
    // Points within each segment must be in non-decreasing longitude order
    // (port-to-starboard = west-to-east when heading north).
    for (const sailroute::IsochroneFrontSegment& seg : result.value().segments) {
        const auto pts = segment_points(result.value(), seg);
        for (std::size_t i = 1U; i < pts.size(); ++i) {
            REQUIRE(
                pts[i].longitude_degrees >=
                pts[i - 1U].longitude_degrees);
        }
    }
}

// ── Interior / disconnected cloud are excluded ────────────────────────────────
//
// Place the provisional best endpoint in a cluster of three adjacent bands
// near the front.  Add two isolated points far to the side (not adjacent to
// that cluster).  The isolated points should be dropped.
TEST_CASE("destination front drops disconnected interior points") {
    // Destination due north.
    const sailroute::Coordinate destination{10.0, 0.0};
    // Band width = 60 NM ≈ 1 degree of longitude at equator.
    // Frontier:
    //   Core cluster (bands -1, 0, +1):  lons -1, 0, +1 at lat = 0.5
    //   Isolated outlier left (band -20): lon -20 at lat = 0.3
    //   Isolated outlier right (band +15): lon +15 at lat = 0.3
    const std::vector<sailroute::Coordinate> frontier{
        {0.5, -1.0},   // core, band ~-1
        {0.5, 0.0},    // core, band 0  (provisional best - closest to destination)
        {0.5, 1.0},    // core, band ~+1
        {0.3, -20.0},  // isolated outlier, port
        {0.3, 15.0},   // isolated outlier, starboard
    };
    auto result = sailroute::build_destination_front(frontier, destination, 60.0);
    REQUIRE(result.has_value());
    // Only the three core cluster points should survive.
    REQUIRE(result.value().points.size() == 3U);
    // None of the retained points should be the outliers.
    for (const sailroute::Coordinate& pt : result.value().points) {
        REQUIRE(std::abs(pt.longitude_degrees) <= 2.0);
    }
}

// ── Input-order determinism ───────────────────────────────────────────────────
TEST_CASE("destination front is input-order deterministic") {
    const sailroute::Coordinate destination{5.0, 0.0};
    const std::vector<sailroute::Coordinate> points{
        {0.0, -2.0},
        {0.0, -1.0},
        {0.0, 0.0},
        {0.0, 1.0},
        {0.0, 2.0},
    };

    auto forward = sailroute::build_destination_front(points, destination, 60.0);
    REQUIRE(forward.has_value());

    std::vector<sailroute::Coordinate> reversed = points;
    std::reverse(reversed.begin(), reversed.end());
    auto backward = sailroute::build_destination_front(reversed, destination, 60.0);
    REQUIRE(backward.has_value());

    require_same_front(forward.value(), backward.value());
}

TEST_CASE("destination front default options preserve the three-argument result") {
    const sailroute::Coordinate destination{10.0, 0.0};
    const std::vector<sailroute::Coordinate> points = angular_frontier(120.0);

    const auto legacy =
        sailroute::build_destination_front(points, destination, 61.0);
    const auto configured = sailroute::build_destination_front(
        points,
        destination,
        61.0,
        sailroute::DestinationFrontOptions{});

    REQUIRE(legacy.has_value());
    REQUIRE(configured.has_value());
    require_same_front(legacy.value(), configured.value());
}

TEST_CASE("destination front configurable aperture includes exact 120 boundaries") {
    const sailroute::Coordinate destination{10.0, 0.0};
    const std::vector<sailroute::Coordinate> points = angular_frontier(120.0);
    const sailroute::Coordinate outer_port = points.front();
    const sailroute::Coordinate outer_starboard = points.back();

    auto default_front =
        sailroute::build_destination_front(points, destination, 61.0);
    auto wide_front = sailroute::build_destination_front(
        points,
        destination,
        61.0,
        sailroute::DestinationFrontOptions{120.0});

    REQUIRE(default_front.has_value());
    REQUIRE(wide_front.has_value());
    REQUIRE(!contains_coordinate(default_front.value(), outer_port));
    REQUIRE(!contains_coordinate(default_front.value(), outer_starboard));
    REQUIRE(contains_coordinate(wide_front.value(), outer_port));
    REQUIRE(contains_coordinate(wide_front.value(), outer_starboard));
}

TEST_CASE("destination front configurable aperture excludes points outside 120") {
    const sailroute::Coordinate destination{10.0, 0.0};
    const std::vector<sailroute::Coordinate> points = angular_frontier(121.0);

    const auto result = sailroute::build_destination_front(
        points,
        destination,
        61.0,
        sailroute::DestinationFrontOptions{120.0});

    REQUIRE(result.has_value());
    REQUIRE(!contains_coordinate(result.value(), points.front()));
    REQUIRE(!contains_coordinate(result.value(), points.back()));
}

TEST_CASE("configured destination fronts remain deterministic and segmented") {
    const sailroute::Coordinate destination{10.0, 0.0};
    const std::vector<sailroute::Coordinate> points = angular_frontier(120.0);
    std::vector<sailroute::Coordinate> reversed = points;
    std::reverse(reversed.begin(), reversed.end());

    const auto forward = sailroute::build_destination_front(
        points,
        destination,
        61.0,
        sailroute::DestinationFrontOptions{120.0});
    const auto backward = sailroute::build_destination_front(
        reversed,
        destination,
        61.0,
        sailroute::DestinationFrontOptions{120.0});

    REQUIRE(forward.has_value());
    REQUIRE(backward.has_value());
    require_same_front(forward.value(), backward.value());
    REQUIRE(forward.value().segments.size() == 1U);
    REQUIRE(
        forward.value().segments.front().point_count ==
        forward.value().points.size());
}

TEST_CASE("destination front retains centroid points inside a populated aperture") {
    const sailroute::Coordinate centroid{0.0, 0.0};
    const std::vector<sailroute::Coordinate> points{
        {1.0, -1.0},
        {1.0, -0.5},
        centroid,
        {-3.0, 0.5},
        {1.0, 1.0},
    };

    const auto result = sailroute::build_destination_front(
        points,
        sailroute::Coordinate{10.0, 0.0},
        31.0,
        sailroute::DestinationFrontOptions{90.0});

    REQUIRE(result.has_value());
    REQUIRE(contains_coordinate(result.value(), centroid));
}

TEST_CASE("destination front preserves fallback when no point is inside aperture") {
    const sailroute::Coordinate centroid{0.0, 0.0};
    const std::vector<sailroute::Coordinate> points{
        sailroute::detail::destination_point(centroid, 270.0, 120.0),
        sailroute::detail::destination_point(centroid, 270.0, 60.0),
        sailroute::detail::destination_point(centroid, 90.0, 60.0),
        sailroute::detail::destination_point(centroid, 90.0, 120.0),
    };

    const auto fallback = sailroute::build_destination_front(
        points,
        sailroute::Coordinate{10.0, 0.0},
        61.0,
        sailroute::DestinationFrontOptions{45.0});
    const auto full = sailroute::build_destination_front(
        points,
        sailroute::Coordinate{10.0, 0.0},
        61.0,
        sailroute::DestinationFrontOptions{180.0});

    REQUIRE(fallback.has_value());
    REQUIRE(full.has_value());
    require_same_front(fallback.value(), full.value());
}

TEST_CASE("destination front filters normally for very narrow valid apertures") {
    const sailroute::Coordinate forward{1.0, 0.0};
    const std::vector<sailroute::Coordinate> points{
        forward,
        {-0.5, -1.0},
        {-0.5, 1.0},
    };

    const auto result = sailroute::build_destination_front(
        points,
        sailroute::Coordinate{10.0, 0.0},
        10.0,
        sailroute::DestinationFrontOptions{1e-13});

    REQUIRE(result.has_value());
    REQUIRE(result.value().points.size() == 1U);
    REQUIRE(contains_coordinate(result.value(), forward));
}

// ── Tie-break / same-band selection is input-order deterministic ──────────────
// When two nearby points compete in the same cross-track band, the result is
// the same regardless of input order.
//
// A decoy point far to the west (lon=-12) shifts the centroid west so both
// competing points (lons 0 and 0.1) lie on the same (positive) cross-track
// side of the centroid, placing them in the same band.  The point with the
// greater along-track progress wins; the winner must be the same for both
// input orderings.
TEST_CASE("destination front ties break lexicographically and deterministically") {
    const sailroute::Coordinate destination{10.0, 0.0};
    const std::vector<sailroute::Coordinate> points_ab{
        {0.0, -12.0},  // decoy — shifts centroid west
        {1.0, 0.0},    // higher along-track candidate
        {0.0, 0.0},    // lower along-track candidate
    };
    const std::vector<sailroute::Coordinate> points_ba{
        {0.0, -12.0},
        {0.0, 0.0},
        {1.0, 0.0},
    };
    auto result_ab = sailroute::build_destination_front(points_ab, destination, 60.0);
    auto result_ba = sailroute::build_destination_front(points_ba, destination, 60.0);
    REQUIRE(result_ab.has_value());
    REQUIRE(result_ba.has_value());
    // Both orderings must produce exactly the same output.
    require_same_front(result_ab.value(), result_ba.value());
    REQUIRE(!result_ab.value().points.empty());
}

// ── Sparse / single occupied band ────────────────────────────────────────────
TEST_CASE("destination front handles two points in the same band") {
    const sailroute::Coordinate destination{5.0, 0.0};
    // Two points very close together — same band, different along_track.
    const std::vector<sailroute::Coordinate> frontier{
        {0.0, 0.1},
        {0.0, 0.2},
    };
    auto result = sailroute::build_destination_front(frontier, destination, 60.0);
    REQUIRE(result.has_value());
    // One band winner, one point.
    REQUIRE(result.value().points.size() == 1U);
    REQUIRE(result.value().segments.size() == 1U);
}

// ── Antimeridian segmentation ────────────────────────────────────────────────
//
// Five frontier points spanning the antimeridian:
//   lons 178, 179, 180(≡-180), -179, -178  at lat = 0
//   destination = (5, 180)  (due north of the antimeridian)
//
// With band_width = 60 NM ≈ 1 degree, cross-track offsets from the centroid
// (which sits on the antimeridian) are approximately:
//   178  → -120 NM → band -2
//   179  → -60 NM  → band -1
//   180  →   0 NM  → band  0  (provisional best)
//  -179  → +60 NM  → band +1
//  -178  → +120 NM → band +2
//
// All bands -2..+2 are contiguous; the contiguous run keeps all five.
// Port-to-starboard order: 178, 179, 180, -179, -178.
// The adjacent pair (180, -179) has |longitude delta| = 359 > 180 →
// antimeridian split → at least two segments.
TEST_CASE("destination front segments antimeridian crossings") {
    const sailroute::Coordinate destination{5.0, 180.0};
    const std::vector<sailroute::Coordinate> frontier{
        {0.0, 178.0},
        {0.0, 179.0},
        {0.0, 180.0},
        {0.0, -179.0},
        {0.0, -178.0},
    };
    auto result = sailroute::build_destination_front(frontier, destination, 60.0);
    REQUIRE(result.has_value());
    // At least two segments (antimeridian split).
    REQUIRE(result.value().segments.size() >= 2U);
    // No adjacent pair within a segment spans more than 180 degrees.
    for (const sailroute::IsochroneFrontSegment& seg : result.value().segments) {
        const auto pts = segment_points(result.value(), seg);
        for (std::size_t i = 1U; i < pts.size(); ++i) {
            REQUIRE(
                std::abs(
                    pts[i].longitude_degrees - pts[i - 1U].longitude_degrees) <=
                180.0);
        }
    }
}

// ── Invalid inputs are rejected ───────────────────────────────────────────────
TEST_CASE("destination front rejects invalid destination") {
    auto result = sailroute::build_destination_front(
        std::vector<sailroute::Coordinate>{{0.0, 0.0}},
        sailroute::Coordinate{0.0, 181.0},
        10.0);
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
}

TEST_CASE("destination front rejects invalid retained coordinate") {
    auto result = sailroute::build_destination_front(
        std::vector<sailroute::Coordinate>{{91.0, 0.0}},
        sailroute::Coordinate{0.0, 0.0},
        10.0);
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
}

TEST_CASE("destination front rejects non-positive band width") {
    auto result = sailroute::build_destination_front(
        std::vector<sailroute::Coordinate>{{0.0, 0.0}},
        sailroute::Coordinate{5.0, 0.0},
        0.0);
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
}

TEST_CASE("destination front rejects invalid half angles") {
    const std::array<double, 6> invalid_angles{
        0.0,
        -1.0,
        180.0001,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    for (const double half_angle : invalid_angles) {
        const auto result = sailroute::build_destination_front(
            std::vector<sailroute::Coordinate>{{0.0, 0.0}},
            sailroute::Coordinate{5.0, 0.0},
            10.0,
            sailroute::DestinationFrontOptions{half_angle});
        REQUIRE(!result.has_value());
        REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
    }
}

// ── RoutingProgressPayload integration ────────────────────────────────────────
// Verify the destination_front bit-flag is usable and the has_payload helper
// works correctly.
TEST_CASE("RoutingProgressPayload destination_front flag is composable") {
    using namespace sailroute;
    const RoutingProgressPayload combo =
        RoutingProgressPayload::retained_points |
        RoutingProgressPayload::destination_front;
    REQUIRE(has_payload(combo, RoutingProgressPayload::destination_front));
    REQUIRE(has_payload(combo, RoutingProgressPayload::retained_points));
    REQUIRE(!has_payload(combo, RoutingProgressPayload::provisional_route));
    REQUIRE(!has_payload(combo, RoutingProgressPayload::display_contours));
}

}  // namespace
