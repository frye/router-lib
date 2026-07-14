#include "routing/geodesy.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string_view>

namespace sailroute {
namespace {

constexpr double metres_per_second_to_knots = 1.9438444924406048;

constexpr double degrees_to_radians(double degrees) noexcept {
    return degrees * std::numbers::pi / 180.0;
}

constexpr double radians_to_degrees(double radians) noexcept {
    return radians * 180.0 / std::numbers::pi;
}

}  // namespace

bool is_valid(Coordinate coordinate) noexcept {
    return std::isfinite(coordinate.latitude_degrees) &&
           std::isfinite(coordinate.longitude_degrees) &&
           coordinate.latitude_degrees >= -90.0 &&
           coordinate.latitude_degrees <= 90.0 &&
           coordinate.longitude_degrees >= -180.0 &&
           coordinate.longitude_degrees <= 180.0;
}

double Wind::speed_knots() const noexcept {
    return std::hypot(east_mps, north_mps) * metres_per_second_to_knots;
}

double Wind::direction_from_degrees() const noexcept {
    if (east_mps == 0.0 && north_mps == 0.0) {
        return 0.0;
    }
    return detail::normalize_degrees(radians_to_degrees(std::atan2(-east_mps, -north_mps)));
}

std::string_view to_string(DepartureSource source) noexcept {
    switch (source) {
        case DepartureSource::explicit_time: return "explicit_time";
        case DepartureSource::current_time: return "current_time";
        case DepartureSource::forecast_start_fallback: return "forecast_start_fallback";
    }
    return "unknown";
}

namespace detail {

double normalize_degrees(double degrees) noexcept {
    if (!std::isfinite(degrees)) {
        return degrees;
    }
    double normalized = std::fmod(degrees, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized == 360.0 ? 0.0 : normalized;
}

double angular_difference_degrees(double first, double second) noexcept {
    const double difference = std::abs(normalize_degrees(first) - normalize_degrees(second));
    return std::min(difference, 360.0 - difference);
}

double great_circle_distance_nautical_miles(
    Coordinate first,
    Coordinate second) noexcept {
    const double first_latitude = degrees_to_radians(first.latitude_degrees);
    const double second_latitude = degrees_to_radians(second.latitude_degrees);
    const double latitude_delta = second_latitude - first_latitude;
    const double longitude_delta =
        degrees_to_radians(second.longitude_degrees - first.longitude_degrees);

    const double latitude_term = std::sin(latitude_delta / 2.0);
    const double longitude_term = std::sin(longitude_delta / 2.0);
    const double haversine =
        latitude_term * latitude_term +
        std::cos(first_latitude) * std::cos(second_latitude) *
            longitude_term * longitude_term;
    const double central_angle =
        2.0 * std::asin(std::sqrt(std::clamp(haversine, 0.0, 1.0)));
    return earth_radius_nautical_miles * central_angle;
}

double initial_bearing_degrees(Coordinate from, Coordinate to) noexcept {
    const double from_latitude = degrees_to_radians(from.latitude_degrees);
    const double to_latitude = degrees_to_radians(to.latitude_degrees);
    const double longitude_delta =
        degrees_to_radians(to.longitude_degrees - from.longitude_degrees);

    const double east = std::sin(longitude_delta) * std::cos(to_latitude);
    const double north =
        std::cos(from_latitude) * std::sin(to_latitude) -
        std::sin(from_latitude) * std::cos(to_latitude) * std::cos(longitude_delta);
    if (east == 0.0 && north == 0.0) {
        return 0.0;
    }
    return normalize_degrees(radians_to_degrees(std::atan2(east, north)));
}

Coordinate destination_point(
    Coordinate start,
    double bearing_degrees,
    double distance_nautical_miles) noexcept {
    const double start_latitude = degrees_to_radians(start.latitude_degrees);
    const double start_longitude = degrees_to_radians(start.longitude_degrees);
    const double bearing = degrees_to_radians(normalize_degrees(bearing_degrees));
    const double angular_distance = distance_nautical_miles / earth_radius_nautical_miles;

    const double destination_latitude = std::asin(std::clamp(
        std::sin(start_latitude) * std::cos(angular_distance) +
            std::cos(start_latitude) * std::sin(angular_distance) * std::cos(bearing),
        -1.0,
        1.0));
    const double destination_longitude =
        start_longitude +
        std::atan2(
            std::sin(bearing) * std::sin(angular_distance) * std::cos(start_latitude),
            std::cos(angular_distance) -
                std::sin(start_latitude) * std::sin(destination_latitude));

    double longitude = radians_to_degrees(destination_longitude);
    longitude = std::fmod(longitude + 540.0, 360.0) - 180.0;
    return Coordinate{radians_to_degrees(destination_latitude), longitude};
}

}  // namespace detail
}  // namespace sailroute
