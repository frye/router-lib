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

}  // namespace sailroute::detail
