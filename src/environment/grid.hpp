#pragma once

#include "sailroute/environment.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace sailroute::environment_detail {

/// Bilinear weights for one located sample position.
struct GridWeights {
    std::size_t south_row{};
    std::size_t north_row{};
    std::size_t west_column{};
    std::size_t east_column{};
    double latitude_fraction{};
    double longitude_fraction{};
};

/// Shared geometry of every in-memory environmental field.
///
/// Values live at grid nodes and are blended bilinearly. Longitudes are
/// canonical, so a globally covering grid wraps its last column back to its
/// first rather than treating the antimeridian as an edge.
class SampleGrid {
public:
    SampleGrid() = default;

    [[nodiscard]] static Result<SampleGrid> create(EnvironmentGridSpec spec);

    [[nodiscard]] const EnvironmentGridSpec& spec() const noexcept {
        return spec_;
    }
    [[nodiscard]] EnvironmentCoverage coverage() const;
    /// Locates a coordinate, or reports it is outside the grid.
    [[nodiscard]] std::optional<GridWeights> locate(
        Coordinate coordinate) const noexcept;
    /// Blends one node-valued field at previously located weights.
    [[nodiscard]] double blend(
        const GridWeights& weights,
        const std::vector<double>& values) const noexcept;

private:
    EnvironmentGridSpec spec_{};
    double north_latitude_degrees_{};
    double longitude_span_degrees_{};
};

/// Converts a meteorological direction into a unit east/north vector.
void unit_from_direction(
    double direction_degrees,
    double& east,
    double& north) noexcept;

/// Converts a blended east/north unit vector back into degrees.
[[nodiscard]] double direction_from_unit(double east, double north) noexcept;

}  // namespace sailroute::environment_detail

namespace sailroute {

/// Normalizes degrees into [0, 360) for the public environment types, which
/// cannot reach the routing detail helpers.
[[nodiscard]] double detail_normalize_degrees(double degrees) noexcept;

}  // namespace sailroute
