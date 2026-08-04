#pragma once

#include "routing/lattice.hpp"

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace sailroute::detail {

[[nodiscard]] double distance_to_route_nautical_miles(
    Coordinate coordinate,
    std::span<const RoutePoint> route) noexcept;

class MixedResolutionLattice {
public:
    using CellIndex = GeodesicLattice::CellIndex;
    using Face = GeodesicLattice::Face;

    [[nodiscard]] static Result<MixedResolutionLattice> create(
        std::size_t coarse_level,
        std::size_t refinement_levels,
        std::span<const RoutePoint> route,
        double corridor_width_nautical_miles);

    [[nodiscard]] std::size_t subdivision_level() const noexcept;
    [[nodiscard]] std::size_t vertex_count() const noexcept;
    [[nodiscard]] std::size_t coarse_vertex_count() const noexcept;
    [[nodiscard]] std::size_t leaf_face_count() const noexcept;
    [[nodiscard]] Coordinate coordinate(CellIndex cell) const noexcept;
    [[nodiscard]] std::span<const CellIndex> neighbors(
        CellIndex cell) const noexcept;
    [[nodiscard]] std::optional<CellIndex> nearest_cell(
        Coordinate coordinate) const noexcept;
    [[nodiscard]] double maximum_neighbor_edge_length_nautical_miles(
        CellIndex cell) const noexcept;

private:
    std::size_t subdivision_level_{};
    std::size_t coarse_vertex_count_{};
    std::size_t leaf_face_count_{};
    std::vector<Coordinate> coordinates_;
    std::vector<std::vector<CellIndex>> neighbors_;
};

}  // namespace sailroute::detail
