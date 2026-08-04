#include "environment/grid.hpp"

#include "routing/geodesy.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace sailroute {

double detail_normalize_degrees(double degrees) noexcept {
    return detail::normalize_degrees(degrees);
}

namespace environment_detail {
namespace {

constexpr double longitude_epsilon_degrees = 1.0e-9;

double signed_longitude_delta(double from, double to) noexcept {
    return std::fmod(to - from + 540.0, 360.0) - 180.0;
}

}  // namespace

Result<SampleGrid> SampleGrid::create(EnvironmentGridSpec spec) {
    if (spec.latitude_count < 2U || spec.longitude_count < 2U) {
        return Error{
            ErrorCode::invalid_environment,
            "environment grid must have at least two rows and two columns"};
    }
    if (!std::isfinite(spec.south_latitude_degrees) ||
        !std::isfinite(spec.west_longitude_degrees) ||
        !std::isfinite(spec.latitude_step_degrees) ||
        !std::isfinite(spec.longitude_step_degrees)) {
        return Error{
            ErrorCode::invalid_environment,
            "environment grid origin and steps must be finite"};
    }
    if (spec.latitude_step_degrees <= 0.0 || spec.longitude_step_degrees <= 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "environment grid steps must be positive"};
    }
    if (spec.south_latitude_degrees < -90.0 ||
        spec.south_latitude_degrees > 90.0) {
        return Error{
            ErrorCode::invalid_environment,
            "environment grid south latitude must be in [-90, 90]"};
    }
    const double north = spec.south_latitude_degrees +
        spec.latitude_step_degrees *
            static_cast<double>(spec.latitude_count - 1U);
    if (north > 90.0 + longitude_epsilon_degrees) {
        return Error{
            ErrorCode::invalid_environment,
            "environment grid extends beyond the north pole"};
    }
    const double span = spec.longitude_step_degrees *
        static_cast<double>(spec.longitude_count - 1U);
    if (spec.global_longitude_coverage) {
        const double wrapped = span + spec.longitude_step_degrees;
        if (std::abs(wrapped - 360.0) > 1.0e-6) {
            return Error{
                ErrorCode::invalid_environment,
                "a globally covering environment grid must span exactly 360 "
                "degrees including its wrap-around column"};
        }
    } else if (span > 360.0 + longitude_epsilon_degrees) {
        return Error{
            ErrorCode::invalid_environment,
            "environment grid spans more than 360 degrees of longitude"};
    }
    if (spec.latitude_count >
        std::numeric_limits<std::size_t>::max() / spec.longitude_count) {
        return Error{
            ErrorCode::invalid_environment,
            "environment grid sample count overflows"};
    }

    SampleGrid grid;
    grid.spec_ = spec;
    grid.north_latitude_degrees_ = north;
    grid.longitude_span_degrees_ = span;
    return grid;
}

EnvironmentCoverage SampleGrid::coverage() const {
    EnvironmentCoverage coverage;
    coverage.south_latitude_degrees = spec_.south_latitude_degrees;
    coverage.north_latitude_degrees = north_latitude_degrees_;
    coverage.global_longitude_coverage = spec_.global_longitude_coverage;
    if (!spec_.global_longitude_coverage) {
        coverage.west_longitude_degrees =
            detail::normalize_degrees(spec_.west_longitude_degrees) > 180.0
            ? detail::normalize_degrees(spec_.west_longitude_degrees) - 360.0
            : detail::normalize_degrees(spec_.west_longitude_degrees);
        const double east = *coverage.west_longitude_degrees + longitude_span_degrees_;
        coverage.east_longitude_degrees =
            std::fmod(east + 540.0, 360.0) - 180.0;
    }
    return coverage;
}

std::optional<GridWeights> SampleGrid::locate(
    Coordinate coordinate) const noexcept {
    if (!is_valid(coordinate)) {
        return std::nullopt;
    }
    const double latitude_offset =
        (coordinate.latitude_degrees - spec_.south_latitude_degrees) /
        spec_.latitude_step_degrees;
    const double last_row = static_cast<double>(spec_.latitude_count - 1U);
    if (latitude_offset < -longitude_epsilon_degrees ||
        latitude_offset > last_row + longitude_epsilon_degrees) {
        return std::nullopt;
    }

    // Distance eastward from the grid's western edge, always non-negative.
    double longitude_delta = signed_longitude_delta(
        spec_.west_longitude_degrees, coordinate.longitude_degrees);
    if (longitude_delta < 0.0) {
        longitude_delta += 360.0;
    }
    const double longitude_offset = longitude_delta / spec_.longitude_step_degrees;
    const double last_column = static_cast<double>(spec_.longitude_count - 1U);
    if (!spec_.global_longitude_coverage &&
        longitude_offset > last_column + longitude_epsilon_degrees) {
        return std::nullopt;
    }

    GridWeights weights;
    const double clamped_latitude = std::clamp(latitude_offset, 0.0, last_row);
    const double row_floor = std::floor(clamped_latitude);
    weights.south_row = static_cast<std::size_t>(
        std::min(row_floor, std::max(0.0, last_row - 1.0)));
    weights.north_row = weights.south_row + 1U;
    weights.latitude_fraction =
        clamped_latitude - static_cast<double>(weights.south_row);

    if (spec_.global_longitude_coverage) {
        const double columns = static_cast<double>(spec_.longitude_count);
        double wrapped = std::fmod(longitude_offset, columns);
        if (wrapped < 0.0) {
            wrapped += columns;
        }
        const double column_floor = std::floor(wrapped);
        weights.west_column = static_cast<std::size_t>(column_floor) %
            spec_.longitude_count;
        weights.east_column =
            (weights.west_column + 1U) % spec_.longitude_count;
        weights.longitude_fraction = wrapped - column_floor;
    } else {
        const double clamped_longitude =
            std::clamp(longitude_offset, 0.0, last_column);
        const double column_floor = std::floor(clamped_longitude);
        weights.west_column = static_cast<std::size_t>(
            std::min(column_floor, std::max(0.0, last_column - 1.0)));
        weights.east_column = weights.west_column + 1U;
        weights.longitude_fraction =
            clamped_longitude - static_cast<double>(weights.west_column);
    }
    return weights;
}

double SampleGrid::blend(
    const GridWeights& weights,
    const std::vector<double>& values) const noexcept {
    const std::size_t south = weights.south_row * spec_.longitude_count;
    const std::size_t north = weights.north_row * spec_.longitude_count;
    const double south_west = values[south + weights.west_column];
    const double south_east = values[south + weights.east_column];
    const double north_west = values[north + weights.west_column];
    const double north_east = values[north + weights.east_column];
    const double lower = south_west +
        (south_east - south_west) * weights.longitude_fraction;
    const double upper = north_west +
        (north_east - north_west) * weights.longitude_fraction;
    return lower + (upper - lower) * weights.latitude_fraction;
}

void unit_from_direction(
    double direction_degrees,
    double& east,
    double& north) noexcept {
    const double radians =
        detail::normalize_degrees(direction_degrees) * std::numbers::pi / 180.0;
    east = std::sin(radians);
    north = std::cos(radians);
}

double direction_from_unit(double east, double north) noexcept {
    if (east == 0.0 && north == 0.0) {
        return 0.0;
    }
    return detail::normalize_degrees(
        std::atan2(east, north) * 180.0 / std::numbers::pi);
}

}  // namespace environment_detail
}  // namespace sailroute
