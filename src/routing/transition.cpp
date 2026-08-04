#include "routing/transition.hpp"

#include "routing/geodesy.hpp"

#include <chrono>
#include <cmath>

namespace sailroute::detail {

Result<std::optional<EvaluatedWind>> evaluate_wind(
    Wind wind,
    const RoutingOptions& options) {
    const double speed_knots = wind.speed_knots();
    const double direction_from_degrees = wind.direction_from_degrees();
    if (!std::isfinite(speed_knots) ||
        !std::isfinite(direction_from_degrees)) {
        return Error{
            ErrorCode::incomplete_forecast,
            "forecast interpolation produced non-finite wind"};
    }
    if (options.maximum_true_wind_speed_knots.has_value() &&
        speed_knots > *options.maximum_true_wind_speed_knots) {
        return std::optional<EvaluatedWind>{};
    }
    return std::optional<EvaluatedWind>{
        EvaluatedWind{speed_knots, direction_from_degrees}};
}

std::int8_t board_for_heading(
    double heading_degrees,
    double wind_from_degrees) noexcept {
    const double delta =
        normalize_degrees(heading_degrees - wind_from_degrees);
    if (delta == 0.0 || delta == 180.0) {
        return 0;
    }
    return delta < 180.0 ? std::int8_t{1} : std::int8_t{-1};
}

std::chrono::seconds maneuver_delay(
    const ManeuverPenalties& penalties,
    OperationalConfiguration parent,
    double parent_true_wind_angle_degrees,
    OperationalConfiguration candidate,
    double candidate_true_wind_angle_degrees) noexcept {
    if (parent.board == 0 || candidate.board == 0 ||
        parent.board == candidate.board) {
        return std::chrono::seconds::zero();
    }
    return 0.5 *
                (parent_true_wind_angle_degrees +
                 candidate_true_wind_angle_degrees) >=
            penalties.downwind_true_wind_angle_degrees
        ? penalties.gybe_penalty
        : penalties.tack_penalty;
}

std::optional<double> boat_speed_for_angle(
    const PolarSlice& slice,
    const RoutingOptions& options,
    double true_wind_angle_degrees) noexcept {
    if (options.above_polar_range == AbovePolarRangePolicy::no_speed &&
        slice.above_tabulated_wind_speed()) {
        return std::nullopt;
    }
    const double speed_knots = slice.speed_knots(true_wind_angle_degrees);
    if (!std::isfinite(speed_knots) || speed_knots <= 0.0 ||
        speed_knots < options.minimum_boat_speed_knots) {
        return std::nullopt;
    }
    return speed_knots;
}

Result<std::optional<VariableTransition>> evaluate_variable_transition(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutePoint& parent,
    OperationalConfiguration parent_configuration,
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
        if (wind_result.error().code ==
            ErrorCode::coordinate_outside_forecast) {
            return std::optional<VariableTransition>{};
        }
        return wind_result.error();
    }
    auto evaluated_wind = evaluate_wind(wind_result.value(), options);
    if (!evaluated_wind) {
        return evaluated_wind.error();
    }
    if (!evaluated_wind.value().has_value()) {
        return std::optional<VariableTransition>{};
    }
    const double wind_speed = evaluated_wind.value()->speed_knots;
    const double wind_from =
        evaluated_wind.value()->direction_from_degrees;
    const double twa = angular_difference_degrees(heading, wind_from);
    const OperationalConfiguration configuration{
        board_for_heading(heading, wind_from),
        parent_configuration.sail,
        parent_configuration.reef};
    const auto delay = maneuver_delay(
        options.maneuver,
        parent_configuration,
        angular_difference_degrees(
            parent.heading_degrees,
            wind_from),
        configuration,
        twa);
    const PolarSlice slice =
        polar.slice_at(wind_speed, options.polar_angle_interpolation);
    const auto initial_boat_speed =
        boat_speed_for_angle(slice, options, twa);
    if (!initial_boat_speed.has_value()) {
        return std::optional<VariableTransition>{};
    }
    double boat_speed = *initial_boat_speed;

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
            if (midpoint_wind.error().code ==
                ErrorCode::coordinate_outside_forecast) {
                return std::optional<VariableTransition>{};
            }
            return midpoint_wind.error();
        }
        auto evaluated_midpoint =
            evaluate_wind(midpoint_wind.value(), options);
        if (!evaluated_midpoint) {
            return evaluated_midpoint.error();
        }
        if (!evaluated_midpoint.value().has_value()) {
            return std::optional<VariableTransition>{};
        }
        const double midpoint_speed_knots =
            evaluated_midpoint.value()->speed_knots;
        const double midpoint_from =
            evaluated_midpoint.value()->direction_from_degrees;
        const PolarSlice midpoint_slice = polar.slice_at(
            midpoint_speed_knots, options.polar_angle_interpolation);
        const auto midpoint_boat_speed = boat_speed_for_angle(
            midpoint_slice,
            options,
            angular_difference_degrees(heading, midpoint_from));
        if (!midpoint_boat_speed.has_value()) {
            return std::optional<VariableTransition>{};
        }
        boat_speed = *midpoint_boat_speed;
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
        VariableTransition{std::move(point), configuration}};
}

}  // namespace sailroute::detail
