#include "routing/transition.hpp"

#include "routing/environment_context.hpp"
#include "routing/geodesy.hpp"

#include <algorithm>
#include <cmath>

namespace sailroute::detail {
namespace {

TimePoint next_boundary(
    const WeatherDataset& weather, TimePoint start, TimePoint end,
    std::chrono::minutes maximum_step) {
    TimePoint result = std::min(end, start + maximum_step);
    const auto& times = weather.valid_times();
    const auto next = std::upper_bound(times.begin(), times.end(), start);
    if (next != times.end()) {
        result = std::min(result, *next);
    }
    return result;
}

Result<bool> validate_endpoint(
    const WeatherDataset& weather, const RoutingOptions& options,
    const RoutePoint& point) {
    auto sample = weather.interpolate(point.position, point.time);
    if (!sample) {
        return sample.error();
    }
    auto evaluated = evaluate_wind(sample.value(), options);
    if (!evaluated) {
        return evaluated.error();
    }
    return evaluated.value().has_value();
}

Result<std::optional<VariableTransition>> reject_endpoint(
    const Error& error, VariableTransitionRejection* rejection) {
    if (error.code == ErrorCode::coordinate_outside_forecast ||
        error.code == ErrorCode::departure_outside_forecast) {
        if (rejection != nullptr) {
            *rejection = error.code == ErrorCode::coordinate_outside_forecast
                ? VariableTransitionRejection::missing_data
                : VariableTransitionRejection::forecast_exhausted;
        }
        return std::optional<VariableTransition>{};
    }
    return error;
}

Result<std::optional<VariableTransition>> maneuver_phase(
    const WeatherDataset& weather, const RoutingOptions& options,
    const RoutingEnvironment& environment, EnvironmentDiagnostics& diagnostics,
    const RoutePoint& parent, OperationalConfiguration configuration,
    double heading, TimePoint action_end, VariableTransitionRejection* rejection) {
    if (!options.maneuver.active()) {
        return std::optional<VariableTransition>{VariableTransition{parent, configuration, {}}};
    }
    auto wind = weather.interpolate(parent.position, parent.time);
    if (!wind) return wind.error();
    auto valid_wind = evaluate_wind(wind.value(), options);
    if (!valid_wind) return valid_wind.error();
    if (!valid_wind.value()) return std::optional<VariableTransition>{};
    const auto fields = sample_environment(environment, parent.position, parent.time, diagnostics);
    if (fields.outcome == EnvironmentOutcome::failed) return *fields.error;
    if (fields.outcome == EnvironmentOutcome::rejected) return std::optional<VariableTransition>{};
    const Wind effective = fields.samples.has_current
        ? water_relative_wind(wind.value(), fields.samples.current) : wind.value();
    const double direction = effective.direction_from_degrees();
    const auto board = board_for_heading(heading, direction);
    const OperationalConfiguration target{
        board != 0 ? board : configuration.board, configuration.sail, configuration.reef};
    const auto delay = maneuver_delay(
        options.maneuver, configuration,
        angular_difference_degrees(parent.heading_degrees, direction), target,
        angular_difference_degrees(heading, direction));
    if (parent.time + delay >= action_end && delay > std::chrono::seconds::zero()) {
        return std::optional<VariableTransition>{};
    }
    VariableTransition result{parent, target, {}};
    const TimePoint end = parent.time + delay;
    while (result.point.time < end) {
        if (result.intermediate_points.size() + 2U > options.maximum_retained_nodes) {
            return Error{ErrorCode::resource_limit, "maneuver integration vertex limit reached"};
        }
        const TimePoint next = next_boundary(
            weather, result.point.time, end, options.maximum_integration_step);
        const auto samples = sample_environment(
            environment, result.point.position, result.point.time, diagnostics);
        if (samples.outcome == EnvironmentOutcome::failed) return *samples.error;
        if (samples.outcome == EnvironmentOutcome::rejected) return std::optional<VariableTransition>{};
        const CurrentVector current = samples.samples.current;
        RoutePoint moved = result.point;
        moved.time = next;
        moved.heading_degrees = heading;
        moved.boat_speed_knots = 0.0;
        moved.environment.reset();
        if (samples.samples.has_current) {
            const double distance = current.speed_knots() *
                std::chrono::duration<double, std::ratio<3600>>(next - result.point.time).count();
            moved.position = destination_point(
                result.point.position, current.set_toward_degrees(), distance);
            moved.cumulative_distance_nautical_miles += distance;
            RoutePointEnvironment audit;
            audit.speed_over_ground_knots = current.speed_knots();
            audit.course_over_ground_degrees = current.set_toward_degrees();
            audit.current_east_knots = current.east_knots;
            audit.current_north_knots = current.north_knots;
            audit.current_applied = true;
            audit.polar_wind_speed_knots = effective.speed_knots();
            audit.polar_wind_direction_degrees = direction;
            moved.environment = audit;
        }
        auto valid = validate_endpoint(weather, options, moved);
        if (!valid) return reject_endpoint(valid.error(), rejection);
        if (!valid.value()) return std::optional<VariableTransition>{};
        const auto geometry = check_segment_geometry(
            environment, result.point.position, result.point.time,
            moved.position, moved.time, diagnostics);
        if (geometry.outcome == EnvironmentOutcome::failed) return *geometry.error;
        if (geometry.outcome == EnvironmentOutcome::rejected ||
            (options.segment_eligibility &&
             !options.segment_eligibility({result.point, moved}))) {
            return std::optional<VariableTransition>{};
        }
        if (result.point.time != parent.time) result.intermediate_points.push_back(result.point);
        result.point = std::move(moved);
    }
    return std::optional<VariableTransition>{std::move(result)};
}

void append_phase(VariableTransition& result, VariableTransition phase, TimePoint departure) {
    if (result.point.time != departure &&
        (result.intermediate_points.empty() ||
         result.intermediate_points.back().time != result.point.time)) {
        result.intermediate_points.push_back(result.point);
    }
    result.intermediate_points.insert(result.intermediate_points.end(),
        std::make_move_iterator(phase.intermediate_points.begin()),
        std::make_move_iterator(phase.intermediate_points.end()));
    result.point = std::move(phase.point);
    result.configuration = phase.configuration;
}

}  // namespace

Result<std::optional<VariableTransition>> evaluate_heading_transition(
    const WeatherDataset& weather, const VesselPolar& polar,
    const RoutingOptions& options, const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics, const RoutePoint& parent,
    OperationalConfiguration configuration, double heading, TimePoint arrival,
    std::optional<Coordinate> destination, double radius,
    VariableTransitionRejection* rejection) {
    if (options.maximum_integration_step <= std::chrono::minutes::zero()) {
        return Error{ErrorCode::invalid_argument, "maximum integration step must be positive"};
    }
    VariableTransition result{parent, configuration, {}};
    RoutingOptions sailing_options = options;
    sailing_options.maneuver.tack_penalty = std::chrono::seconds::zero();
    sailing_options.maneuver.gybe_penalty = std::chrono::seconds::zero();
    sailing_options.segment_eligibility = {};
    while (result.point.time < arrival) {
        if (result.intermediate_points.size() + 2U > options.maximum_retained_nodes) {
            return Error{ErrorCode::resource_limit, "integration vertex limit reached"};
        }
        auto maneuver = maneuver_phase(
            weather, options, environment, diagnostics, result.point,
            result.configuration, heading, arrival, rejection);
        if (!maneuver || !maneuver.value()) return maneuver;
        if (maneuver.value()->point.time != result.point.time) {
            append_phase(result, std::move(*maneuver.value()), parent.time);
        } else {
            result.configuration = maneuver.value()->configuration;
        }
        const TimePoint end = next_boundary(
            weather, result.point.time, arrival, options.maximum_integration_step);
        if (end <= result.point.time) {
            return std::optional<VariableTransition>{};
        }
        auto step = evaluate_heading_transition_step(
            weather, polar, sailing_options, environment, diagnostics,
            result.point, result.configuration, heading, end,
            destination, radius, rejection);
        if (!step || !step.value()) {
            return step;
        }
        auto valid = validate_endpoint(weather, options, step.value()->point);
        if (!valid) {
            return reject_endpoint(valid.error(), rejection);
        }
        if (!valid.value()) {
            return std::optional<VariableTransition>{};
        }
        if (options.segment_eligibility &&
            !options.segment_eligibility({result.point, step.value()->point})) {
            return std::optional<VariableTransition>{};
        }
        append_phase(result, std::move(*step.value()), parent.time);
        if (destination &&
            great_circle_distance_nautical_miles(result.point.position, *destination)
                <= radius + 1.0e-6) {
            break;
        }
    }
    return std::optional<VariableTransition>{std::move(result)};
}

Result<std::optional<VariableTransition>> evaluate_variable_transition(
    const WeatherDataset& weather, const VesselPolar& polar,
    const RoutingOptions& options, const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics, const RoutePoint& parent,
    OperationalConfiguration configuration, Coordinate destination,
    TimePoint route_end, VariableTransitionRejection* rejection) {
    if (options.maximum_integration_step <= std::chrono::minutes::zero()) {
        return Error{ErrorCode::invalid_argument, "maximum integration step must be positive"};
    }
    VariableTransition result{parent, configuration, {}};
    RoutingOptions sailing_options = options;
    sailing_options.maneuver.tack_penalty = std::chrono::seconds::zero();
    sailing_options.maneuver.gybe_penalty = std::chrono::seconds::zero();
    sailing_options.segment_eligibility = {};
    while (great_circle_distance_nautical_miles(result.point.position, destination) > 1.0e-8) {
        if (result.intermediate_points.size() + 2U > options.maximum_retained_nodes) {
            return Error{ErrorCode::resource_limit, "integration vertex limit reached"};
        }
        if (options.maneuver.active()) {
            const double distance = great_circle_distance_nautical_miles(result.point.position, destination);
            const Coordinate probe = destination_point(result.point.position,
                initial_bearing_degrees(result.point.position, destination),
                std::min(distance, 0.001));
            RoutingOptions probe_options = sailing_options;
            probe_options.segment_eligibility = {};
            auto proposed = evaluate_variable_transition_step(
                weather, polar, probe_options, environment, diagnostics,
                result.point, result.configuration, probe, route_end, rejection);
            if (!proposed || !proposed.value()) return proposed;
            auto maneuver = maneuver_phase(
                weather, options, environment, diagnostics, result.point,
                result.configuration, proposed.value()->point.heading_degrees, route_end, rejection);
            if (!maneuver || !maneuver.value()) return maneuver;
            if (maneuver.value()->point.time != result.point.time) {
                append_phase(result, std::move(*maneuver.value()), parent.time);
            } else {
                result.configuration = maneuver.value()->configuration;
            }
        }
        const TimePoint end = next_boundary(
            weather, result.point.time, route_end, options.maximum_integration_step);
        if (end <= result.point.time) {
            if (rejection) {
                *rejection = VariableTransitionRejection::duration_exhausted;
            }
            return std::optional<VariableTransition>{};
        }
        const double remaining =
            great_circle_distance_nautical_miles(result.point.position, destination);
        const double bearing = initial_bearing_degrees(result.point.position, destination);
        const double hours = std::chrono::duration<double, std::ratio<3600>>(
            end - result.point.time).count();
        double distance = std::min(
            remaining, polar.maximum_boat_speed_knots() * options.boat_speed_factor * hours);
        std::optional<VariableTransition> accepted;
        for (int attempt = 0; attempt < 32 && distance > 1.0e-9; ++attempt) {
            const Coordinate target = distance >= remaining
                ? destination : destination_point(result.point.position, bearing, distance);
            VariableTransitionRejection reason = VariableTransitionRejection::infeasible;
            auto step = evaluate_variable_transition_step(
                weather, polar, sailing_options, environment, diagnostics,
                result.point, result.configuration, target, end, &reason);
            if (!step) {
                return step.error();
            }
            if (step.value()) {
                accepted = std::move(*step.value());
                break;
            }
            if (reason != VariableTransitionRejection::duration_exhausted &&
                reason != VariableTransitionRejection::forecast_exhausted) {
                if (rejection) {
                    *rejection = reason;
                }
                return std::optional<VariableTransition>{};
            }
            distance *= 0.5;
        }
        if (!accepted) {
            if (rejection) {
                *rejection = VariableTransitionRejection::duration_exhausted;
            }
            return std::optional<VariableTransition>{};
        }
        auto valid = validate_endpoint(weather, options, accepted->point);
        if (!valid) {
            return reject_endpoint(valid.error(), rejection);
        }
        if (!valid.value()) {
            return std::optional<VariableTransition>{};
        }
        if (options.segment_eligibility &&
            !options.segment_eligibility({result.point, accepted->point})) {
            return std::optional<VariableTransition>{};
        }
        append_phase(result, std::move(*accepted), parent.time);
    }
    return std::optional<VariableTransition>{std::move(result)};
}

Result<std::optional<VariableTransition>> evaluate_wait_transition(
    const WeatherDataset& weather, const VesselPolar&,
    const RoutingOptions& options, const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics, const RoutePoint& parent,
    OperationalConfiguration configuration, TimePoint arrival) {
    if (!options.holding_eligibility || arrival <= parent.time) {
        return std::optional<VariableTransition>{};
    }
    if (options.maximum_integration_step <= std::chrono::minutes::zero()) {
        return Error{ErrorCode::invalid_argument, "maximum integration step must be positive"};
    }
    RoutePoint point = parent;
    point.time = arrival;
    point.boat_speed_knots = 0.0;
    point.environment.reset();
    if (!options.holding_eligibility({parent, point}) ||
        (options.segment_eligibility && !options.segment_eligibility({parent, point}))) {
        return std::optional<VariableTransition>{};
    }
    for (TimePoint time = parent.time;;) {
        RoutePoint sampled_point = point;
        sampled_point.time = time;
        auto valid = validate_endpoint(weather, options, sampled_point);
        if (!valid) {
            return valid.error();
        }
        if (!valid.value()) {
            return std::optional<VariableTransition>{};
        }
        const auto fields = sample_environment(environment, parent.position, time, diagnostics);
        if (fields.outcome == EnvironmentOutcome::failed) {
            return *fields.error;
        }
        if (fields.outcome == EnvironmentOutcome::rejected) {
            return std::optional<VariableTransition>{};
        }
        if (time == arrival) {
            break;
        }
        time = next_boundary(weather, time, arrival, options.maximum_integration_step);
    }
    const auto geometry = check_segment_geometry(
        environment, parent.position, parent.time, parent.position, arrival, diagnostics);
    if (geometry.outcome == EnvironmentOutcome::failed) {
        return *geometry.error;
    }
    if (geometry.outcome == EnvironmentOutcome::rejected) {
        return std::optional<VariableTransition>{};
    }
    return std::optional<VariableTransition>{VariableTransition{std::move(point), configuration, {}}};
}

}  // namespace sailroute::detail
