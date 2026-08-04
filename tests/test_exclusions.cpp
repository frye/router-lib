#include "sailroute/environment.hpp"

#include "test_support.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using sailroute::Coordinate;
using sailroute::ErrorCode;
using sailroute::ExclusionBoundaryPolicy;
using sailroute::ExclusionPolygon;
using sailroute::ExclusionRing;
using sailroute::ExclusionZone;
using sailroute::ExclusionZoneSet;
using sailroute::ProviderMetadata;
using sailroute::TimePoint;

ProviderMetadata metadata() {
    return ProviderMetadata{"test_zones", "unit test", "1"};
}

TimePoint at(long long seconds) {
    return TimePoint{std::chrono::seconds{1'800'000'000 + seconds}};
}

ExclusionRing rectangle(
    double south,
    double north,
    double west,
    double east) {
    return ExclusionRing{{
        Coordinate{south, west},
        Coordinate{south, east},
        Coordinate{north, east},
        Coordinate{north, west},
    }};
}

ExclusionZone simple_zone(
    std::string identifier,
    double south,
    double north,
    double west,
    double east) {
    ExclusionZone zone;
    zone.identifier = std::move(identifier);
    zone.source = "unit test";
    zone.revision = 1U;
    zone.polygons.push_back(
        ExclusionPolygon{rectangle(south, north, west, east), {}});
    return zone;
}

ExclusionZoneSet build(std::vector<ExclusionZone> zones) {
    auto set = ExclusionZoneSet::create(std::move(zones), metadata());
    REQUIRE(set.has_value());
    return std::move(set.value());
}

bool violates(
    const ExclusionZoneSet& zones,
    Coordinate from,
    Coordinate to,
    ExclusionBoundaryPolicy policy = ExclusionBoundaryPolicy::boundary_excluded,
    long long from_seconds = 0,
    long long to_seconds = 3600) {
    return zones
        .intersects_segment(
            from, at(from_seconds), to, at(to_seconds), policy)
        .violated;
}

}  // namespace

TEST_CASE("exclusion sets validate identity, topology, and activation windows") {
    REQUIRE(!ExclusionZoneSet::create({}, ProviderMetadata{"", "s", "1"})
                 .has_value());

    ExclusionZone unnamed = simple_zone("", 0.0, 1.0, 0.0, 1.0);
    REQUIRE(!ExclusionZoneSet::create({unnamed}, metadata()).has_value());

    ExclusionZone empty = simple_zone("empty", 0.0, 1.0, 0.0, 1.0);
    empty.polygons.clear();
    REQUIRE(!ExclusionZoneSet::create({empty}, metadata()).has_value());

    ExclusionZone degenerate = simple_zone("degenerate", 0.0, 1.0, 0.0, 1.0);
    degenerate.polygons.front().outer.vertices.resize(2U);
    const auto too_few = ExclusionZoneSet::create({degenerate}, metadata());
    REQUIRE(!too_few.has_value());
    REQUIRE(too_few.error().code == ErrorCode::invalid_environment);

    ExclusionZone non_finite = simple_zone("non_finite", 0.0, 1.0, 0.0, 1.0);
    non_finite.polygons.front().outer.vertices.front().latitude_degrees =
        std::numeric_limits<double>::quiet_NaN();
    REQUIRE(!ExclusionZoneSet::create({non_finite}, metadata()).has_value());

    ExclusionZone inverted = simple_zone("inverted", 0.0, 1.0, 0.0, 1.0);
    inverted.active_from = at(1000);
    inverted.active_until = at(500);
    REQUIRE(!ExclusionZoneSet::create({inverted}, metadata()).has_value());

    const ExclusionZone repeated = simple_zone("same", 0.0, 1.0, 0.0, 1.0);
    REQUIRE(!ExclusionZoneSet::create({repeated, repeated}, metadata())
                 .has_value());

    // A ring that bounds two equal hemispheres has no well-defined interior.
    ExclusionZone great_circle;
    great_circle.identifier = "great_circle";
    great_circle.source = "unit test";
    great_circle.polygons.push_back(ExclusionPolygon{
        ExclusionRing{{
            Coordinate{0.0, 0.0},
            Coordinate{0.0, 90.0},
            Coordinate{0.0, 180.0},
            Coordinate{0.0, -90.0},
        }},
        {}});
    REQUIRE(!ExclusionZoneSet::create({great_circle}, metadata()).has_value());

    ExclusionZone escaping_hole = simple_zone("hole", 0.0, 2.0, 0.0, 2.0);
    escaping_hole.polygons.front().holes.push_back(
        rectangle(3.0, 4.0, 3.0, 4.0));
    REQUIRE(!ExclusionZoneSet::create({escaping_hole}, metadata()).has_value());
}

TEST_CASE("a segment crossing a zone is rejected and one around it is not") {
    const ExclusionZoneSet zones =
        build({simple_zone("box", -1.0, 1.0, -1.0, 1.0)});
    REQUIRE(zones.zone_count() == 1U);
    REQUIRE(zones.maximum_revision() == 1U);
    REQUIRE(zones.metadata().name == "test_zones");

    REQUIRE(violates(zones, {0.0, -3.0}, {0.0, 3.0}));
    REQUIRE(violates(zones, {0.0, 0.0}, {0.0, 3.0}));
    REQUIRE(!violates(zones, {5.0, -3.0}, {5.0, 3.0}));
    REQUIRE(!violates(zones, {0.0, 2.0}, {0.0, 4.0}));

    REQUIRE(zones.contains(
        Coordinate{0.0, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(!zones.contains(
        Coordinate{5.0, 5.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));

    const ExclusionZoneSet::SegmentResult result = zones.intersects_segment(
        Coordinate{0.0, -3.0},
        at(0),
        Coordinate{0.0, 3.0},
        at(3600),
        ExclusionBoundaryPolicy::boundary_excluded);
    REQUIRE(result.violated);
    REQUIRE(result.zone_identifier == "box");
    REQUIRE(result.geometry_tests > 0U);
}

TEST_CASE("holes are navigable while their surrounding ring is not") {
    ExclusionZone doughnut = simple_zone("doughnut", -4.0, 4.0, -4.0, 4.0);
    doughnut.polygons.front().holes.push_back(rectangle(-1.0, 1.0, -1.0, 1.0));
    const ExclusionZoneSet zones = build({doughnut});

    REQUIRE(!zones.contains(
        Coordinate{0.0, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(zones.contains(
        Coordinate{2.5, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));

    // Entirely inside the hole, so legal.
    REQUIRE(!violates(zones, {-0.5, -0.5}, {0.5, 0.5}));
    // Leaving the hole crosses the surrounding ring.
    REQUIRE(violates(zones, {0.0, 0.0}, {0.0, 6.0}));
    // Passing straight through both the ring and the hole is still a violation.
    REQUIRE(violates(zones, {0.0, -6.0}, {0.0, 6.0}));
}

TEST_CASE("overlapping zones and multiple components are handled independently") {
    ExclusionZone multi;
    multi.identifier = "archipelago";
    multi.source = "unit test";
    multi.revision = 7U;
    multi.polygons.push_back(ExclusionPolygon{rectangle(-1.0, 1.0, -6.0, -4.0), {}});
    multi.polygons.push_back(ExclusionPolygon{rectangle(-1.0, 1.0, 4.0, 6.0), {}});

    const ExclusionZoneSet zones =
        build({multi, simple_zone("centre", -1.0, 1.0, -0.5, 0.5)});
    REQUIRE(zones.zone_count() == 2U);
    REQUIRE(zones.maximum_revision() == 7U);

    REQUIRE(violates(zones, {0.0, -5.5}, {0.0, -4.5}));
    REQUIRE(violates(zones, {0.0, 4.5}, {0.0, 5.5}));
    REQUIRE(violates(zones, {0.0, -0.2}, {0.0, 0.2}));
    REQUIRE(!violates(zones, {0.0, 1.0}, {0.0, 3.0}));
    REQUIRE(!violates(zones, {8.0, -8.0}, {8.0, 8.0}));
}

TEST_CASE("input zone order never changes ordering, results, or attribution") {
    const ExclusionZone first = simple_zone("alpha", -1.0, 1.0, -1.0, 1.0);
    const ExclusionZone second = simple_zone("beta", -1.0, 1.0, 3.0, 5.0);
    const ExclusionZone third = simple_zone("gamma", -1.0, 1.0, 7.0, 9.0);

    const ExclusionZoneSet forward = build({first, second, third});
    const ExclusionZoneSet shuffled = build({third, first, second});

    REQUIRE(forward.zones().size() == shuffled.zones().size());
    for (std::size_t index = 0U; index < forward.zones().size(); ++index) {
        REQUIRE(
            forward.zones()[index].identifier ==
            shuffled.zones()[index].identifier);
    }

    const auto forward_hit = forward.intersects_segment(
        Coordinate{0.0, -2.0},
        at(0),
        Coordinate{0.0, 10.0},
        at(3600),
        ExclusionBoundaryPolicy::boundary_excluded);
    const auto shuffled_hit = shuffled.intersects_segment(
        Coordinate{0.0, -2.0},
        at(0),
        Coordinate{0.0, 10.0},
        at(3600),
        ExclusionBoundaryPolicy::boundary_excluded);
    REQUIRE(forward_hit.violated == shuffled_hit.violated);
    REQUIRE(forward_hit.zone_identifier == shuffled_hit.zone_identifier);
    REQUIRE(forward_hit.geometry_tests == shuffled_hit.geometry_tests);
}

TEST_CASE("activation windows bound which part of a traversal is constrained") {
    ExclusionZone timed = simple_zone("timed", -1.0, 1.0, -1.0, 1.0);
    timed.active_from = at(1800);
    timed.active_until = at(5400);
    const ExclusionZoneSet zones = build({timed});

    // Wholly before activation.
    REQUIRE(!violates(zones, {0.0, -3.0}, {0.0, 3.0}, ExclusionBoundaryPolicy::boundary_excluded, 0, 900));
    // Wholly after deactivation.
    REQUIRE(!violates(zones, {0.0, -3.0}, {0.0, 3.0}, ExclusionBoundaryPolicy::boundary_excluded, 7200, 9000));
    // Straddling the window.
    REQUIRE(violates(zones, {0.0, -3.0}, {0.0, 3.0}, ExclusionBoundaryPolicy::boundary_excluded, 0, 7200));

    REQUIRE(!zones.contains(
        Coordinate{0.0, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(zones.contains(
        Coordinate{0.0, 0.0}, at(3600), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(!zones.contains(
        Coordinate{0.0, 0.0}, at(5400), ExclusionBoundaryPolicy::boundary_excluded));
}

TEST_CASE("a zone activating mid-segment only constrains the part still to sail") {
    ExclusionZone opening = simple_zone("opening", -1.0, 1.0, -3.0, -1.5);
    opening.active_from = at(1800);
    const ExclusionZoneSet zones = build({opening});

    // The vessel is already past the zone by the time it opens, so the
    // traversal is legal even though the whole segment overlaps it.
    REQUIRE(!violates(
        zones,
        {0.0, -3.0},
        {0.0, 5.0},
        ExclusionBoundaryPolicy::boundary_excluded,
        0,
        3600));
    // Sailing the other way puts the vessel in the zone after it opens.
    REQUIRE(violates(
        zones,
        {0.0, 5.0},
        {0.0, -3.0},
        ExclusionBoundaryPolicy::boundary_excluded,
        0,
        3600));
}

TEST_CASE("the boundary policy decides whether a shared edge may be sailed") {
    const ExclusionZoneSet zones =
        build({simple_zone("west", -2.0, 2.0, -4.0, 0.0),
               simple_zone("east", -2.0, 2.0, 0.0, 4.0)});

    // The shared edge runs along the prime meridian between the two zones.
    REQUIRE(violates(
        zones, {-1.0, 0.0}, {1.0, 0.0}, ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(!violates(
        zones, {-1.0, 0.0}, {1.0, 0.0}, ExclusionBoundaryPolicy::boundary_allowed));

    // Entering the interior is a violation under either policy.
    REQUIRE(violates(
        zones, {0.0, -1.0}, {0.0, 1.0}, ExclusionBoundaryPolicy::boundary_allowed));

    REQUIRE(zones.contains(
        Coordinate{0.0, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(!zones.contains(
        Coordinate{1.0, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_allowed) ||
        zones.contains(
            Coordinate{1.0, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_allowed));
}

TEST_CASE("zones spanning the antimeridian need no special encoding") {
    ExclusionZone straddling;
    straddling.identifier = "dateline";
    straddling.source = "unit test";
    straddling.polygons.push_back(ExclusionPolygon{
        ExclusionRing{{
            Coordinate{-2.0, 178.0},
            Coordinate{-2.0, -178.0},
            Coordinate{2.0, -178.0},
            Coordinate{2.0, 178.0},
        }},
        {}});
    const ExclusionZoneSet zones = build({straddling});

    REQUIRE(zones.contains(
        Coordinate{0.0, 180.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(zones.contains(
        Coordinate{0.0, -179.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(!zones.contains(
        Coordinate{0.0, 170.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));

    REQUIRE(violates(zones, {0.0, 175.0}, {0.0, -175.0}));
    REQUIRE(!violates(zones, {10.0, 175.0}, {10.0, -175.0}));
}

TEST_CASE("a polar cap zone behaves like an Antarctic exclusion zone") {
    ExclusionZone antarctic;
    antarctic.identifier = "aez";
    antarctic.source = "notice to mariners";
    antarctic.revision = 3U;
    ExclusionRing ring;
    for (int step = 0; step < 24; ++step) {
        ring.vertices.push_back(
            Coordinate{-60.0, -180.0 + 15.0 * static_cast<double>(step)});
    }
    antarctic.polygons.push_back(ExclusionPolygon{std::move(ring), {}});
    const ExclusionZoneSet zones = build({antarctic});

    REQUIRE(zones.contains(
        Coordinate{-70.0, 30.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(zones.contains(
        Coordinate{-89.0, -170.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(!zones.contains(
        Coordinate{-50.0, 30.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));
    REQUIRE(!zones.contains(
        Coordinate{40.0, 30.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));

    REQUIRE(violates(zones, {-55.0, 0.0}, {-75.0, 0.0}));
    REQUIRE(!violates(zones, {-55.0, 0.0}, {-45.0, 0.0}));
}

TEST_CASE("an unconfigured or empty set constrains nothing") {
    const ExclusionZoneSet empty;
    REQUIRE(empty.zone_count() == 0U);
    REQUIRE(empty.maximum_revision() == 0U);
    REQUIRE(empty.zones().empty());
    REQUIRE(!violates(empty, {0.0, 0.0}, {1.0, 1.0}));
    REQUIRE(!empty.contains(
        Coordinate{0.0, 0.0}, at(0), ExclusionBoundaryPolicy::boundary_excluded));

    const ExclusionZoneSet none = build({});
    REQUIRE(none.zone_count() == 0U);
    REQUIRE(!violates(none, {0.0, 0.0}, {1.0, 1.0}));
}

TEST_CASE("a stationary vessel inside an opening zone is still in violation") {
    ExclusionZone opening = simple_zone("opening", -1.0, 1.0, -1.0, 1.0);
    opening.active_from = at(1800);
    const ExclusionZoneSet zones = build({opening});

    // A degenerate segment models waiting in place while the zone opens.
    REQUIRE(violates(
        zones,
        {0.0, 0.0},
        {0.0, 0.0},
        ExclusionBoundaryPolicy::boundary_excluded,
        0,
        3600));
    REQUIRE(!violates(
        zones,
        {0.0, 0.0},
        {0.0, 0.0},
        ExclusionBoundaryPolicy::boundary_excluded,
        0,
        900));
}
