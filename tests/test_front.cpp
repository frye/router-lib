#include "sailroute/front.hpp"

#include "../src/routing/front.hpp"
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

bool same_coordinate(
    sailroute::Coordinate left,
    sailroute::Coordinate right) {
    return left.latitude_degrees == right.latitude_degrees &&
        left.longitude_degrees == right.longitude_degrees;
}

std::vector<sailroute::Coordinate> cross_track_fixture(
    sailroute::Coordinate center,
    double forward_bearing_degrees,
    std::span<const double> signed_offsets_nautical_miles) {
    std::vector<sailroute::Coordinate> points;
    points.reserve(signed_offsets_nautical_miles.size());
    for (const double offset : signed_offsets_nautical_miles) {
        if (offset == 0.0) {
            points.push_back(center);
            continue;
        }
        const double bearing = offset < 0.0
            ? forward_bearing_degrees - 90.0
            : forward_bearing_degrees + 90.0;
        points.push_back(sailroute::detail::destination_point(
            center,
            bearing,
            std::abs(offset)));
    }
    return points;
}

sailroute::IsochroneFront build_anchored_front(
    std::span<const sailroute::Coordinate> points,
    sailroute::Coordinate destination,
    sailroute::Coordinate anchor,
    double band_width_nautical_miles,
    const sailroute::DestinationFrontOptions& options) {
    sailroute::IsochroneFront front;
    const auto error = sailroute::detail::build_destination_front_into(
        points,
        destination,
        anchor,
        band_width_nautical_miles,
        options,
        front.points,
        front.segments);
    if (error.has_value()) {
        throw std::runtime_error(error->message);
    }
    return front;
}

void require_points_from_input(
    const sailroute::IsochroneFront& front,
    std::span<const sailroute::Coordinate> input) {
    for (const sailroute::Coordinate output : front.points) {
        REQUIRE(std::any_of(
            input.begin(),
            input.end(),
            [output](sailroute::Coordinate candidate) {
                return same_coordinate(output, candidate);
            }));
    }
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
    const sailroute::DestinationFrontOptions defaults;
    REQUIRE(
        defaults.segment_policy ==
        sailroute::DestinationFrontSegmentPolicy::provisional_component);
    REQUIRE(defaults.minimum_secondary_segment_points == 3U);
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

TEST_CASE("anchored destination front preserves both sides of the route endpoint") {
    const sailroute::Coordinate anchor{0.0, 0.0};
    const sailroute::Coordinate destination{10.0, 0.0};
    const std::array<double, 5> offsets{-80.0, -30.0, 0.0, 30.0, 80.0};
    const std::vector<sailroute::Coordinate> points =
        cross_track_fixture(anchor, 0.0, offsets);
    const sailroute::DestinationFrontOptions options{
        90.0,
        sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
        3U};

    const sailroute::IsochroneFront front =
        build_anchored_front(points, destination, anchor, 50.0, options);

    REQUIRE(front.segments.size() == 1U);
    REQUIRE(front.points.size() == 4U);
    const auto principal = segment_points(front, front.segments.front());
    const auto anchor_it = std::find_if(
        principal.begin(),
        principal.end(),
        [anchor](sailroute::Coordinate point) {
            return same_coordinate(point, anchor);
        });
    REQUIRE(anchor_it != principal.end());
    REQUIRE(anchor_it != principal.begin());
    REQUIRE(anchor_it + 1 != principal.end());
    for (std::size_t index = 1U; index < principal.size(); ++index) {
        REQUIRE(
            principal[index - 1U].longitude_degrees <=
            principal[index].longitude_degrees);
    }
    require_points_from_input(front, points);
}

TEST_CASE("destination front retains meaningful runs without bridging gaps") {
    const sailroute::Coordinate anchor{0.0, 0.0};
    const sailroute::Coordinate destination{10.0, 0.0};
    const std::array<double, 11> offsets{
        -330.0, -280.0, -230.0,
        -80.0, -30.0, 0.0, 30.0, 80.0,
        230.0, 280.0, 330.0};
    const std::vector<sailroute::Coordinate> points =
        cross_track_fixture(anchor, 0.0, offsets);

    const sailroute::IsochroneFront legacy = build_anchored_front(
        points,
        destination,
        anchor,
        50.0,
        sailroute::DestinationFrontOptions{});
    const sailroute::IsochroneFront all_components = build_anchored_front(
        points,
        destination,
        anchor,
        50.0,
        sailroute::DestinationFrontOptions{
            90.0,
            sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
            3U});

    REQUIRE(legacy.segments.size() == 1U);
    REQUIRE(legacy.points.size() == 4U);
    REQUIRE(all_components.segments.size() == 3U);
    REQUIRE(all_components.points.size() == 10U);
    for (const sailroute::IsochroneFrontSegment segment :
         all_components.segments) {
        REQUIRE(segment.point_count >= 3U);
    }
    require_points_from_input(all_components, points);

    const sailroute::IsochroneFront stricter = build_anchored_front(
        points,
        destination,
        anchor,
        50.0,
        sailroute::DestinationFrontOptions{
            90.0,
            sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
            4U});
    REQUIRE(stricter.segments.size() == 1U);
    REQUIRE(stricter.points.size() == 4U);

    std::vector<sailroute::Coordinate> reversed = points;
    std::reverse(reversed.begin(), reversed.end());
    const sailroute::IsochroneFront reversed_front = build_anchored_front(
        reversed,
        destination,
        anchor,
        50.0,
        sailroute::DestinationFrontOptions{
            90.0,
            sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
            3U});
    require_same_front(all_components, reversed_front);
}

TEST_CASE("destination front omits one and two point secondary hooks") {
    const sailroute::Coordinate anchor{0.0, 0.0};
    const sailroute::Coordinate destination{10.0, 0.0};
    const sailroute::DestinationFrontOptions options{
        90.0,
        sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
        3U};

    const std::array<double, 7> one_point_offsets{
        -230.0, -80.0, -30.0, 0.0, 30.0, 80.0, 230.0};
    const std::vector<sailroute::Coordinate> one_point_cloud =
        cross_track_fixture(anchor, 0.0, one_point_offsets);
    const sailroute::IsochroneFront one_point = build_anchored_front(
        one_point_cloud, destination, anchor, 50.0, options);
    REQUIRE(one_point.segments.size() == 1U);
    REQUIRE(one_point.points.size() == 4U);

    const std::array<double, 9> two_point_offsets{
        -280.0, -230.0,
        -80.0, -30.0, 0.0, 30.0, 80.0,
        230.0, 280.0};
    const std::vector<sailroute::Coordinate> two_point_cloud =
        cross_track_fixture(anchor, 0.0, two_point_offsets);
    const sailroute::IsochroneFront two_point = build_anchored_front(
        two_point_cloud, destination, anchor, 50.0, options);
    REQUIRE(two_point.segments.size() == 1U);
    REQUIRE(two_point.points.size() == 4U);
}

TEST_CASE("Race Rocks candidate fixture preserves a two-sided principal front") {
    const sailroute::Coordinate anchor{48.2975, -123.5310};
    const sailroute::Coordinate destination{48.3500, -122.9000};
    const double forward_bearing =
        sailroute::detail::initial_bearing_degrees(anchor, destination);
    const std::array<double, 11> offsets{
        -16.5, -14.0, -11.5,
        -4.0, -1.5, 0.0, 1.5, 4.0,
        11.5, 14.0, 16.5};
    const std::vector<sailroute::Coordinate> candidate_cloud =
        cross_track_fixture(anchor, forward_bearing, offsets);

    const sailroute::IsochroneFront front = build_anchored_front(
        candidate_cloud,
        destination,
        anchor,
        2.5,
        sailroute::DestinationFrontOptions{
            120.0,
            sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
            3U});

    REQUIRE(front.segments.size() == 3U);
    require_points_from_input(front, candidate_cloud);
    bool found_anchor = false;
    for (const sailroute::IsochroneFrontSegment segment : front.segments) {
        REQUIRE(segment.point_count >= 3U);
        const auto points = segment_points(front, segment);
        const auto anchor_it = std::find_if(
            points.begin(),
            points.end(),
            [anchor](sailroute::Coordinate point) {
                return same_coordinate(point, anchor);
            });
        if (anchor_it == points.end()) {
            continue;
        }
        found_anchor = true;
        REQUIRE(anchor_it != points.begin());
        REQUIRE(anchor_it + 1 != points.end());
    }
    REQUIRE(found_anchor);

    const std::array<std::size_t, 11> shuffled_indices{
        7U, 1U, 9U, 3U, 0U, 10U, 5U, 2U, 8U, 4U, 6U};
    std::vector<sailroute::Coordinate> shuffled;
    shuffled.reserve(candidate_cloud.size());
    for (const std::size_t index : shuffled_indices) {
        shuffled.push_back(candidate_cloud[index]);
    }
    const sailroute::IsochroneFront shuffled_front = build_anchored_front(
        shuffled,
        destination,
        anchor,
        2.5,
        sailroute::DestinationFrontOptions{
            120.0,
            sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
            3U});
    require_same_front(front, shuffled_front);
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

TEST_CASE("destination front splits antimeridian crossings inside retained runs") {
    const sailroute::Coordinate anchor{0.0, 180.0};
    const sailroute::Coordinate destination{10.0, 180.0};
    const std::array<double, 11> offsets{
        -330.0, -280.0, -230.0,
        -80.0, -30.0, 0.0, 30.0, 80.0,
        230.0, 280.0, 330.0};
    const std::vector<sailroute::Coordinate> points =
        cross_track_fixture(anchor, 0.0, offsets);
    const sailroute::IsochroneFront front = build_anchored_front(
        points,
        destination,
        anchor,
        50.0,
        sailroute::DestinationFrontOptions{
            90.0,
            sailroute::DestinationFrontSegmentPolicy::all_meaningful_components,
            3U});

    REQUIRE(front.segments.size() == 4U);
    require_points_from_input(front, points);
    for (const sailroute::IsochroneFrontSegment segment : front.segments) {
        const auto segment_view = segment_points(front, segment);
        for (std::size_t index = 1U; index < segment_view.size(); ++index) {
            REQUIRE(
                std::abs(
                    segment_view[index].longitude_degrees -
                    segment_view[index - 1U].longitude_degrees) <= 180.0);
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

TEST_CASE("destination front rejects unsupported segment policies and anchors") {
    const std::vector<sailroute::Coordinate> points{
        {0.0, -1.0},
        {0.0, 0.0},
        {0.0, 1.0}};
    sailroute::DestinationFrontOptions invalid_policy;
    invalid_policy.segment_policy =
        static_cast<sailroute::DestinationFrontSegmentPolicy>(99);
    const auto policy_result = sailroute::build_destination_front(
        points,
        sailroute::Coordinate{5.0, 0.0},
        60.0,
        invalid_policy);
    REQUIRE(!policy_result.has_value());
    REQUIRE(
        policy_result.error().code ==
        sailroute::ErrorCode::invalid_argument);

    sailroute::IsochroneFront front;
    const auto anchor_error = sailroute::detail::build_destination_front_into(
        points,
        sailroute::Coordinate{5.0, 0.0},
        sailroute::Coordinate{1.0, 1.0},
        60.0,
        sailroute::DestinationFrontOptions{},
        front.points,
        front.segments);
    REQUIRE(anchor_error.has_value());
    REQUIRE(anchor_error->code == sailroute::ErrorCode::invalid_argument);
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
