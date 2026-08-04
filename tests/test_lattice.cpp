#include "routing/lattice.hpp"

#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

using sailroute::Coordinate;
using sailroute::detail::GeodesicLattice;

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
            REQUIRE(first.value().neighbors(cell).size() == second.value().neighbors(cell).size());
        }
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
