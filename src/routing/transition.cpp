#include "routing/transition.hpp"

#include "routing/geodesy.hpp"

#include <chrono>
#include <cmath>

namespace sailroute::detail {
namespace {

std::int8_t board_for(double heading, double wind_from) noexcept {
    const double delta = normalize_degrees(heading - wind_from);
    if (delta == 0.0 || delta == 180.0) {
        return 0;
    }
    return delta < 180.0 ? std::int8_t{1} : std::int8_t{-1};
}

std::chrono::seconds maneuver_delay(
    const ManeuverPenalties& penalties,
    std::int8_t parent_board,
    double parent_heading,
    std::int8_t candidate_board,
    double candidate_twa,
    double wind_from) noexcept {
    if (parent_board == 0 || candidate_board == 0 ||
        parent_board == candidate_board) {
        return std::chrono::seconds::zero();
    }
    const double parent_twa =
        angular_difference_degrees(parent_heading, wind_from);
    return 0.5 * (parent_twa + candidate_twa) >=
            penalties.downwind_true_wind_angle_degrees
        ? penalties.gybe_penalty
        : penalties.tack_penalty;
}

}  // namespace

Result<std::optional<VariableTransition>> evaluate_variable_transition(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutePoint& parent,
    std::int8_t parent_board,
    Coordinate destination,
    TimePoint route_end) {
    const double distance =
        great_circle_distance_nautical_miles(parent.position, destination);
    if (!(distance > 0.0)) {
        return std::optional<VariableTransition>{};
    }
    const double heading = initial_bearing_degrees(parent.position, destination);
    auto wind_result = weather.interpolate(parent.position, parent.time);
    if (!wind_result) {
        return wind_result.error();
    }
    const Wind& wind = wind_result.value();
    const double wind_speed = wind.speed_knots();
    const double wind_from = wind.direction_from_degrees();
    if (!std::isfinite(wind_speed) || !std::isfinite(wind_from)) {
        return Error{
            ErrorCode::incomplete_forecast,
            "forecast interpolation produced non-finite wind"};
    }
    if (options.maximum_true_wind_speed_knots.has_value() &&
        wind_speed > *options.maximum_true_wind_speed_knots) {
        return std::optional<VariableTransition>{};
    }
    const double twa = angular_difference_degrees(heading, wind_from);
    const std::int8_t board = board_for(heading, wind_from);
    const auto delay = maneuver_delay(
        options.maneuver,
        parent_board,
        parent.heading_degrees,
        board,
        twa,
        wind_from);
    const PolarSlice slice =
        polar.slice_at(wind_speed, options.polar_angle_interpolation);
    if (options.above_polar_range == AbovePolarRangePolicy::no_speed &&
        slice.above_tabulated_wind_speed()) {
        return std::optional<VariableTransition>{};
    }
    double boat_speed = slice.speed_knots(twa);
    if (!std::isfinite(boat_speed) ||
        boat_speed < options.minimum_boat_speed_knots) {
        return std::optional<VariableTransition>{};
    }

    double sailing_seconds = distance / boat_speed * 3600.0;
    if (options.wind_sampling == WindSampling::midpoint &&
        std::chrono::duration<double>(sailing_seconds) >=
            options.midpoint_wind_sampling_threshold) {
        const Coordinate midpoint =
            destination_point(parent.position, heading, distance * 0.5);
        const TimePoint midpoint_time =
            parent.time + delay +
            std::chrono::seconds{
                static_cast<std::chrono::seconds::rep>(
                    std::ceil(sailing_seconds * 0.5))};
        auto midpoint_wind = weather.interpolate(midpoint, midpoint_time);
        if (!midpoint_wind) {
            return midpoint_wind.error();
        }
        const double midpoint_speed_knots =
            midpoint_wind.value().speed_knots();
        const double midpoint_from =
            midpoint_wind.value().direction_from_degrees();
        if (options.maximum_true_wind_speed_knots.has_value() &&
            midpoint_speed_knots > *options.maximum_true_wind_speed_knots) {
            return std::optional<VariableTransition>{};
        }
        const PolarSlice midpoint_slice = polar.slice_at(
            midpoint_speed_knots, options.polar_angle_interpolation);
        if (options.above_polar_range == AbovePolarRangePolicy::no_speed &&
            midpoint_slice.above_tabulated_wind_speed()) {
            return std::optional<VariableTransition>{};
        }
        boat_speed = midpoint_slice.speed_knots(
            angular_difference_degrees(heading, midpoint_from));
        if (!std::isfinite(boat_speed) ||
            boat_speed < options.minimum_boat_speed_knots) {
            return std::optional<VariableTransition>{};
        }
        sailing_seconds = distance / boat_speed * 3600.0;
    }

    const auto duration = delay + std::chrono::seconds{
        static_cast<std::chrono::seconds::rep>(std::ceil(sailing_seconds))};
    if (duration <= std::chrono::seconds::zero() ||
        parent.time + duration > route_end) {
        return std::optional<VariableTransition>{};
    }
    RoutePoint point{
        destination,
        parent.time + duration,
        heading,
        boat_speed,
        wind_speed,
        wind_from,
        parent.cumulative_distance_nautical_miles + distance};
    if (options.segment_eligibility &&
        !options.segment_eligibility(RouteSegmentView{parent, point})) {
        return std::optional<VariableTransition>{};
    }
    return std::optional<VariableTransition>{
        VariableTransition{std::move(point), board}};
}

}  // namespace sailroute::detail
