#pragma once

#include "sailroute/types.hpp"

namespace sailroute::detail {

inline constexpr double earth_radius_nautical_miles = 3440.065;

[[nodiscard]] double normalize_degrees(double degrees) noexcept;
[[nodiscard]] double angular_difference_degrees(double first, double second) noexcept;
[[nodiscard]] double great_circle_distance_nautical_miles(
    Coordinate first,
    Coordinate second) noexcept;
[[nodiscard]] double initial_bearing_degrees(Coordinate from, Coordinate to) noexcept;
[[nodiscard]] Coordinate destination_point(
    Coordinate start,
    double bearing_degrees,
    double distance_nautical_miles) noexcept;

// A start position with its latitude trigonometry precomputed. Projecting one
// position along many bearings, as candidate expansion does, otherwise repeats
// the same sine and cosine for every bearing.
struct PreparedOrigin {
    double latitude_radians{};
    double longitude_radians{};
    double sin_latitude{};
    double cos_latitude{};
};

[[nodiscard]] PreparedOrigin prepare_origin(Coordinate start) noexcept;

// Produces the same result as destination_point(origin, ...) for the position
// the PreparedOrigin was built from.
[[nodiscard]] Coordinate destination_point_from(
    const PreparedOrigin& origin,
    double bearing_degrees,
    double distance_nautical_miles) noexcept;

}  // namespace sailroute::detail
