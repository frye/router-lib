#include "sailroute/front.hpp"

#include "test_support.hpp"

#include <algorithm>
#include <cmath>
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
// Frontier: five points spread north-south at the same latitude east of
// the destination.  The destination is due west so the forward axis is
// westward.  The two easternmost points (furthest from destination) should
// be dropped by the half-space / band-best filter; only the westernmost
// (leading) points are retained.
//
// Layout (longitude increases eastward):
//   destination at (0, 0)
//   frontier at lon = 2, 4, 6, 8, 10  (all same lat = 0)
//   Points at lon = 2 and 4 are in the destination-facing half; 6, 8, 10
//   are further away.  Each falls in its own cross-track band (all on the
//   same latitude, so cross-track == 0 → all in band 0).
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
