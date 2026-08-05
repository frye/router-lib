#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace sailroute::detail {

/// A deterministic, vertex-centred geodesic lattice made by subdividing an
/// icosahedron. Cell indices are stable across subdivisions: every vertex
/// inherited from a coarser lattice keeps its index.
class GeodesicLattice {
public:
    using CellIndex = std::size_t;
    using Face = std::array<CellIndex, 3U>;

    /// Prevents unexpectedly large routing allocations (level 8 has 655,362
    /// cells). Larger resolutions are intentionally not supported internally.
    static constexpr std::size_t maximum_subdivision_level = 8U;

    /// Builds a level in [0, maximum_subdivision_level].
    [[nodiscard]] static Result<GeodesicLattice> create(
        std::size_t subdivision_level);

    [[nodiscard]] std::size_t subdivision_level() const noexcept;
    [[nodiscard]] std::size_t vertex_count() const noexcept;

    /// Returns the canonical latitude/longitude coordinate for a valid cell.
    /// Passing an index outside [0, vertex_count()) is a programmer error.
    [[nodiscard]] Coordinate coordinate(CellIndex cell) const noexcept;

    /// Returns sorted, duplicate-free neighboring cell indices for a valid
    /// cell. Every returned edge is reciprocal; cells have degree five or six.
    /// Passing an index outside [0, vertex_count()) is a programmer error.
    [[nodiscard]] std::span<const CellIndex> neighbors(CellIndex cell) const noexcept;
    [[nodiscard]] std::span<const Face> faces() const noexcept;

    /// Finds the closest vertex to a valid canonical coordinate. Invalid input
    /// yields std::nullopt. Ties are resolved by the lowest cell index.
    [[nodiscard]] std::optional<CellIndex> nearest_cell(
        Coordinate coordinate) const noexcept;
    [[nodiscard]] std::optional<Face> containing_face(
        Coordinate coordinate) const noexcept;

    /// Largest great-circle length of any lattice edge, in nautical miles.
    [[nodiscard]] double maximum_neighbor_edge_length_nautical_miles() const noexcept;
    [[nodiscard]] double maximum_neighbor_edge_length_nautical_miles(
        CellIndex cell) const noexcept;

    /// Returns the corresponding cell index at a level no finer than this
    /// lattice when this cell was inherited from that level; otherwise nullopt.
    /// The current level is accepted and maps every valid cell to itself.
    [[nodiscard]] std::optional<CellIndex> coincident_cell_at_level(
        CellIndex cell,
        std::size_t level) const noexcept;

private:
    explicit GeodesicLattice(std::size_t subdivision_level);

    std::size_t subdivision_level_{};
    std::vector<Coordinate> coordinates_;
    std::vector<std::vector<CellIndex>> neighbors_;
    std::vector<Face> faces_;
    std::vector<double> unit_vectors_;
    double maximum_neighbor_edge_length_nautical_miles_{};
};

}  // namespace sailroute::detail
