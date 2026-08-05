#include "routing/lattice.hpp"
#include "routing/mixed_lattice.hpp"

#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numbers>
#include <numeric>
#include <queue>
#include <vector>

namespace {

using sailroute::Coordinate;
using sailroute::detail::GeodesicLattice;
using sailroute::detail::MixedResolutionLattice;

sailroute::RoutePoint route_point(Coordinate coordinate) {
    sailroute::RoutePoint point;
    point.position = coordinate;
    return point;
}

Coordinate face_center(
    const GeodesicLattice& lattice,
    GeodesicLattice::Face face) {
    std::array<double, 3U> sum{};
    for (const auto cell : face) {
        const Coordinate coordinate = lattice.coordinate(cell);
        const double latitude =
            coordinate.latitude_degrees * std::numbers::pi / 180.0;
        const double longitude =
            coordinate.longitude_degrees * std::numbers::pi / 180.0;
        const double cosine_latitude = std::cos(latitude);
        sum[0U] += cosine_latitude * std::cos(longitude);
        sum[1U] += cosine_latitude * std::sin(longitude);
        sum[2U] += std::sin(latitude);
    }
    const double length =
        std::sqrt(sum[0U] * sum[0U] + sum[1U] * sum[1U] + sum[2U] * sum[2U]);
    return {
        std::asin(sum[2U] / length) * 180.0 / std::numbers::pi,
        std::atan2(sum[1U], sum[0U]) * 180.0 / std::numbers::pi};
}

void require_reciprocal_sorted_neighbors(const GeodesicLattice& lattice) {
    for (GeodesicLattice::CellIndex cell = 0U; cell < lattice.vertex_count(); ++cell) {
        const auto adjacent = lattice.neighbors(cell);
        REQUIRE(std::is_sorted(adjacent.begin(), adjacent.end()));
        REQUIRE(std::adjacent_find(adjacent.begin(), adjacent.end()) == adjacent.end());
        REQUIRE(adjacent.size() == 5U || adjacent.size() == 6U);
        for (const GeodesicLattice::CellIndex neighbor : adjacent) {
            const auto reverse = lattice.neighbors(neighbor);
            REQUIRE(std::binary_search(reverse.begin(), reverse.end(), cell));
        }
    }
}

void require_connected(const MixedResolutionLattice& lattice) {
    std::vector<std::uint8_t> visited(lattice.vertex_count(), 0U);
    std::queue<MixedResolutionLattice::CellIndex> pending;
    visited[0U] = 1U;
    pending.push(0U);
    std::size_t count = 0U;
    while (!pending.empty()) {
        const auto cell = pending.front();
        pending.pop();
        ++count;
        const auto adjacent = lattice.neighbors(cell);
        REQUIRE(std::is_sorted(adjacent.begin(), adjacent.end()));
        REQUIRE(
            std::adjacent_find(adjacent.begin(), adjacent.end()) ==
            adjacent.end());
        for (const auto neighbor : adjacent) {
            const auto reverse = lattice.neighbors(neighbor);
            REQUIRE(std::binary_search(reverse.begin(), reverse.end(), cell));
            if (!visited[neighbor]) {
                visited[neighbor] = 1U;
                pending.push(neighbor);
            }
        }
    }
    REQUIRE(count == lattice.vertex_count());
}

}  // namespace

TEST_CASE("geodesic lattice construction is deterministic with expected counts") {
    constexpr std::size_t expected_counts[]{12U, 42U, 162U, 642U, 2562U};
    for (std::size_t level = 0U; level < std::size(expected_counts); ++level) {
        const auto first = GeodesicLattice::create(level);
        const auto second = GeodesicLattice::create(level);
        REQUIRE(first.has_value());
        REQUIRE(second.has_value());
        REQUIRE(first.value().vertex_count() == expected_counts[level]);
        REQUIRE(second.value().vertex_count() == expected_counts[level]);
        for (std::size_t cell = 0U; cell < first.value().vertex_count(); ++cell) {
            const Coordinate left = first.value().coordinate(cell);
            const Coordinate right = second.value().coordinate(cell);
            REQUIRE(left.latitude_degrees == right.latitude_degrees);
            REQUIRE(left.longitude_degrees == right.longitude_degrees);
            const auto left_neighbors = first.value().neighbors(cell);
            const auto right_neighbors = second.value().neighbors(cell);
            REQUIRE(left_neighbors.size() == right_neighbors.size());
            REQUIRE(std::equal(
                left_neighbors.begin(),
                left_neighbors.end(),
                right_neighbors.begin(),
                right_neighbors.end()));
        }
    }
}

TEST_CASE("geodesic lattice lookup is independent of deterministic query order") {
    constexpr std::size_t sample_count = 1024U;
    std::vector<Coordinate> samples{
        {0.0, 0.0},
        {0.0, 180.0},
        {0.0, -180.0},
        {90.0, 137.0},
        {-90.0, -61.0},
    };
    samples.reserve(sample_count);
    std::uint64_t state = 0x4d595df4d0f33173ULL;
    while (samples.size() < sample_count) {
        state = state * 6'364'136'223'846'793'005ULL +
                1'442'695'040'888'963'407ULL;
        const double z =
            2.0 * static_cast<double>(state >> 11U) /
                static_cast<double>(std::uint64_t{1U} << 53U) -
            1.0;
        state = state * 6'364'136'223'846'793'005ULL +
                1'442'695'040'888'963'407ULL;
        const double longitude =
            360.0 * static_cast<double>(state >> 11U) /
                static_cast<double>(std::uint64_t{1U} << 53U) -
            180.0;
        samples.push_back(Coordinate{
            std::asin(std::clamp(z, -1.0, 1.0)) * 180.0 / std::numbers::pi,
            longitude});
    }

    const auto first = GeodesicLattice::create(4U);
    const auto repeated = GeodesicLattice::create(4U);
    REQUIRE(first.has_value());
    REQUIRE(repeated.has_value());

    std::vector<GeodesicLattice::CellIndex> expected;
    expected.reserve(samples.size());
    for (const Coordinate sample : samples) {
        const auto cell = first.value().nearest_cell(sample);
        REQUIRE(cell.has_value());
        expected.push_back(cell.value());
    }

    std::vector<std::size_t> order(samples.size());
    std::iota(order.begin(), order.end(), 0U);
    std::uint32_t shuffle_state = 0x5a17c9e3U;
    for (std::size_t remaining = order.size(); remaining > 1U; --remaining) {
        shuffle_state = shuffle_state * 1'664'525U + 1'013'904'223U;
        const std::size_t other = shuffle_state % remaining;
        std::swap(order[remaining - 1U], order[other]);
    }
    for (const std::size_t index : order) {
        const auto actual = repeated.value().nearest_cell(samples[index]);
        REQUIRE(actual.has_value());
        REQUIRE(actual.value() == expected[index]);
        const Coordinate first_coordinate =
            first.value().coordinate(expected[index]);
        const Coordinate repeated_coordinate =
            repeated.value().coordinate(actual.value());
        REQUIRE(
            first_coordinate.latitude_degrees ==
            repeated_coordinate.latitude_degrees);
        REQUIRE(
            first_coordinate.longitude_degrees ==
            repeated_coordinate.longitude_degrees);
        const auto first_neighbors = first.value().neighbors(expected[index]);
        const auto repeated_neighbors = repeated.value().neighbors(actual.value());
        REQUIRE(std::equal(
            first_neighbors.begin(),
            first_neighbors.end(),
            repeated_neighbors.begin(),
            repeated_neighbors.end()));
    }
}

TEST_CASE("geodesic lattice neighbors are sorted, reciprocal, and degree five or six") {
    const auto lattice = GeodesicLattice::create(3U);
    REQUIRE(lattice.has_value());
    require_reciprocal_sorted_neighbors(lattice.value());
}

TEST_CASE("geodesic lattice coordinates are finite and canonical") {
    const auto lattice = GeodesicLattice::create(4U);
    REQUIRE(lattice.has_value());
    for (std::size_t cell = 0U; cell < lattice.value().vertex_count(); ++cell) {
        const Coordinate coordinate = lattice.value().coordinate(cell);
        REQUIRE(std::isfinite(coordinate.latitude_degrees));
        REQUIRE(std::isfinite(coordinate.longitude_degrees));
        REQUIRE(coordinate.latitude_degrees >= -90.0);
        REQUIRE(coordinate.latitude_degrees <= 90.0);
        REQUIRE(coordinate.longitude_degrees >= -180.0);
        REQUIRE(coordinate.longitude_degrees <= 180.0);
    }
}

TEST_CASE("geodesic lattice nearest lookup handles antimeridian and poles stably") {
    const auto lattice = GeodesicLattice::create(4U);
    REQUIRE(lattice.has_value());

    const auto east_antimeridian = lattice.value().nearest_cell({0.0, 180.0});
    const auto west_antimeridian = lattice.value().nearest_cell({0.0, -180.0});
    REQUIRE(east_antimeridian.has_value());
    REQUIRE(east_antimeridian == west_antimeridian);

    const auto north_pole = lattice.value().nearest_cell({90.0, 0.0});
    const auto north_pole_other_longitude = lattice.value().nearest_cell({90.0, -180.0});
    REQUIRE(north_pole.has_value());
    REQUIRE(north_pole == north_pole_other_longitude);
    REQUIRE(north_pole == lattice.value().nearest_cell({90.0, 0.0}));
    REQUIRE(!lattice.value().nearest_cell(
        {std::numeric_limits<double>::quiet_NaN(), 0.0}).has_value());
}

TEST_CASE("geodesic lattice finds containing faces deterministically") {
    const auto lattice = GeodesicLattice::create(4U);
    REQUIRE(lattice.has_value());

    for (const auto face : lattice.value().faces()) {
        const auto found =
            lattice.value().containing_face(face_center(lattice.value(), face));
        REQUIRE(found.has_value());
        REQUIRE(found.value() == face);
    }
    REQUIRE(lattice.value().containing_face({0.0, 180.0}).has_value());
    REQUIRE(lattice.value().containing_face({90.0, -180.0}).has_value());
    REQUIRE(
        lattice.value().containing_face({90.0, -180.0}) ==
        lattice.value().containing_face({90.0, 137.0}));
    REQUIRE(!lattice.value().containing_face(
        {0.0, std::numeric_limits<double>::infinity()}).has_value());
}

TEST_CASE("geodesic lattice edge lengths shrink with subdivision") {
    const auto coarse = GeodesicLattice::create(0U);
    const auto fine = GeodesicLattice::create(3U);
    REQUIRE(coarse.has_value());
    REQUIRE(fine.has_value());
    REQUIRE(coarse.value().maximum_neighbor_edge_length_nautical_miles() > 0.0);
    REQUIRE(coarse.value().maximum_neighbor_edge_length_nautical_miles() < 4000.0);
    REQUIRE(fine.value().maximum_neighbor_edge_length_nautical_miles() > 0.0);
    REQUIRE(fine.value().maximum_neighbor_edge_length_nautical_miles() <
            coarse.value().maximum_neighbor_edge_length_nautical_miles());
    REQUIRE(fine.value().maximum_neighbor_edge_length_nautical_miles() < 600.0);
}

TEST_CASE("geodesic lattice preserves inherited cell indices") {
    const auto lattice = GeodesicLattice::create(3U);
    REQUIRE(lattice.has_value());
    REQUIRE(lattice.value().coincident_cell_at_level(0U, 0U) == 0U);
    REQUIRE(lattice.value().coincident_cell_at_level(161U, 2U) == 161U);
    REQUIRE(!lattice.value().coincident_cell_at_level(162U, 2U).has_value());
    REQUIRE(!lattice.value().coincident_cell_at_level(0U, 4U).has_value());
}

TEST_CASE("geodesic lattice rejects excessive subdivision levels") {
    const auto maximum = GeodesicLattice::create(
        GeodesicLattice::maximum_subdivision_level);
    REQUIRE(maximum.has_value());
    REQUIRE(maximum.value().vertex_count() == 655362U);

    const auto excessive = GeodesicLattice::create(
        GeodesicLattice::maximum_subdivision_level + 1U);
    REQUIRE(!excessive.has_value());
    REQUIRE(excessive.error().code == sailroute::ErrorCode::invalid_argument);
}

TEST_CASE("route segment distance handles seams poles and short routes") {
    const std::array antimeridian{
        route_point({0.0, 179.0}),
        route_point({0.0, -179.0})};
    REQUIRE_NEAR(
        sailroute::detail::distance_to_route_nautical_miles(
            {0.0, 180.0}, antimeridian),
        0.0,
        1.0e-6);

    const std::array polar{
        route_point({80.0, -90.0}),
        route_point({80.0, 90.0})};
    REQUIRE_NEAR(
        sailroute::detail::distance_to_route_nautical_miles(
            {90.0, 0.0}, polar),
        0.0,
        1.0e-6);

    const std::array short_route{route_point({12.0, 34.0})};
    REQUIRE_NEAR(
        sailroute::detail::distance_to_route_nautical_miles(
            {12.0, 34.0}, short_route),
        0.0,
        1.0e-6);
}

TEST_CASE("mixed lattice is deterministic connected and hierarchy local") {
    const std::array route{
        route_point({0.0, 179.0}),
        route_point({0.0, -179.0})};
    const auto first = MixedResolutionLattice::create(2U, 2U, route, 120.0);
    const auto repeated = MixedResolutionLattice::create(2U, 2U, route, 120.0);
    const auto coarse = GeodesicLattice::create(2U);
    const auto global_fine = GeodesicLattice::create(4U);
    REQUIRE(first.has_value());
    REQUIRE(repeated.has_value());
    REQUIRE(coarse.has_value());
    REQUIRE(global_fine.has_value());
    REQUIRE(first.value().coarse_vertex_count() == coarse.value().vertex_count());
    REQUIRE(first.value().vertex_count() > coarse.value().vertex_count());
    REQUIRE(first.value().vertex_count() < global_fine.value().vertex_count());
    REQUIRE(first.value().vertex_count() == repeated.value().vertex_count());
    REQUIRE(first.value().leaf_face_count() == repeated.value().leaf_face_count());

    for (std::size_t cell = 0U; cell < first.value().vertex_count(); ++cell) {
        const Coordinate left = first.value().coordinate(cell);
        const Coordinate right = repeated.value().coordinate(cell);
        REQUIRE(left.latitude_degrees == right.latitude_degrees);
        REQUIRE(left.longitude_degrees == right.longitude_degrees);
        REQUIRE(std::equal(
            first.value().neighbors(cell).begin(),
            first.value().neighbors(cell).end(),
            repeated.value().neighbors(cell).begin(),
            repeated.value().neighbors(cell).end()));
        REQUIRE(
            first.value().maximum_neighbor_edge_length_nautical_miles(cell) >
            0.0);
    }
    for (std::size_t cell = 0U; cell < coarse.value().vertex_count(); ++cell) {
        REQUIRE(
            first.value().coordinate(cell).latitude_degrees ==
            coarse.value().coordinate(cell).latitude_degrees);
        REQUIRE(
            first.value().coordinate(cell).longitude_degrees ==
            coarse.value().coordinate(cell).longitude_degrees);
    }
    require_connected(first.value());
}

TEST_CASE("mixed lattice active domain scales with corridor width") {
    const std::array route{
        route_point({70.0, -20.0}),
        route_point({85.0, 160.0}),
        route_point({70.0, 20.0})};
    const auto narrow = MixedResolutionLattice::create(2U, 2U, route, 60.0);
    const auto wide = MixedResolutionLattice::create(2U, 2U, route, 600.0);
    const auto global_fine = GeodesicLattice::create(4U);
    REQUIRE(narrow.has_value());
    REQUIRE(wide.has_value());
    REQUIRE(global_fine.has_value());
    REQUIRE(
        narrow.value().vertex_count() <= wide.value().vertex_count());
    REQUIRE(wide.value().vertex_count() < global_fine.value().vertex_count());
    require_connected(narrow.value());
    require_connected(wide.value());
}

TEST_CASE("mixed lattice containing faces follow retained leaf topology") {
    const std::array route{
        route_point({1.0, 0.1}),
        route_point({1.0, 1.9})};
    const auto first = MixedResolutionLattice::create(4U, 2U, route, 120.0);
    const auto repeated = MixedResolutionLattice::create(4U, 2U, route, 120.0);
    REQUIRE(first.has_value());
    REQUIRE(repeated.has_value());

    for (const Coordinate sample :
         std::array{
             Coordinate{1.0, 0.1},
             Coordinate{1.0, 1.9},
             Coordinate{0.0, 180.0},
             Coordinate{90.0, 0.0}}) {
        const auto first_face = first.value().containing_face(sample);
        const auto repeated_face = repeated.value().containing_face(sample);
        REQUIRE(first_face.has_value());
        REQUIRE(first_face == repeated_face);
        for (const auto cell : *first_face) {
            REQUIRE(cell < first.value().vertex_count());
        }
    }
}
