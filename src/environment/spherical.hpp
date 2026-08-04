#pragma once

#include "sailroute/types.hpp"

#include <array>

namespace sailroute::environment_detail {

/// A position on the unit sphere in earth-centred Cartesian coordinates.
///
/// Exclusion geometry works entirely in this frame so the antimeridian and the
/// poles need no special cases: they are ordinary directions like any other.
struct UnitVector {
    double x{};
    double y{};
    double z{};
};

[[nodiscard]] UnitVector to_unit_vector(Coordinate coordinate) noexcept;
[[nodiscard]] Coordinate to_coordinate(UnitVector vector) noexcept;
[[nodiscard]] UnitVector cross(UnitVector left, UnitVector right) noexcept;
[[nodiscard]] double dot(UnitVector left, UnitVector right) noexcept;
[[nodiscard]] double norm(UnitVector vector) noexcept;
/// Returns the zero vector unchanged rather than producing a NaN direction.
[[nodiscard]] UnitVector normalize(UnitVector vector) noexcept;
[[nodiscard]] UnitVector negate(UnitVector vector) noexcept;
[[nodiscard]] UnitVector add(UnitVector left, UnitVector right) noexcept;
[[nodiscard]] UnitVector scale(UnitVector vector, double factor) noexcept;
/// Central angle between two directions, radians.
[[nodiscard]] double angle_between(UnitVector left, UnitVector right) noexcept;
/// Interpolates along the minor great-circle arc, `fraction` in [0, 1].
[[nodiscard]] UnitVector slerp(
    UnitVector from,
    UnitVector to,
    double fraction) noexcept;

/// Intersections of two minor great-circle arcs, at most two.
struct ArcIntersections {
    std::array<UnitVector, 2> points{};
    std::size_t count{};
};

/// Returns the points where arc `a`-`b` meets arc `c`-`d`.
///
/// Arcs sharing a great circle report their overlapping endpoints, so a shared
/// boundary is detected rather than silently missed.
[[nodiscard]] ArcIntersections intersect_arcs(
    UnitVector a,
    UnitVector b,
    UnitVector c,
    UnitVector d) noexcept;

/// Angular distance from a point to a minor great-circle arc, radians.
[[nodiscard]] double distance_to_arc(
    UnitVector point,
    UnitVector from,
    UnitVector to) noexcept;

}  // namespace sailroute::environment_detail
