#include "routing/transition.hpp"

#include "routing/environment_context.hpp"
#include "routing/geodesy.hpp"

#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>
#include <utility>

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

namespace {

// Fixed-point iterations used to solve for the water heading that holds a
// required ground track under current. The map is a contraction whenever the
// vessel can hold the track at all, and a fixed bound keeps the result
// independent of evaluation order.
constexpr int course_to_steer_iterations = 12;
constexpr double course_to_steer_tolerance_degrees = 1.0e-10;

// One solved leg toward a fixed ground target.
struct LegSolution {
    double water_heading_degrees{};
    double water_speed_knots{};
    double ground_speed_knots{};
    double flat_water_speed_knots{};
    double true_wind_angle_degrees{};
    double relative_wave_angle_degrees{};
    std::int8_t board{};
};

}  // namespace

Result<std::optional<VariableTransition>> evaluate_variable_transition(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics,
    const RoutePoint& parent,
    OperationalConfiguration parent_configuration,
    Coordinate destination,
    TimePoint route_end) {
    const double distance =
        great_circle_distance_nautical_miles(parent.position, destination);
    if (!(distance > 0.0)) {
        return std::optional<VariableTransition>{};
    }
    const double ground_course =
        initial_bearing_degrees(parent.position, destination);
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

    const bool environment_active = environment.active();
    const bool environment_fields_active =
        environment.currents.configured() || environment.waves.configured();
    EnvironmentSamples samples;
    if (environment_fields_active) {
        EnvironmentSampleResult sampled = sample_environment(
            environment, parent.position, parent.time, diagnostics);
        if (sampled.outcome == EnvironmentOutcome::failed) {
            return *sampled.error;
        }
        if (sampled.outcome == EnvironmentOutcome::rejected) {
            return std::optional<VariableTransition>{};
        }
        samples = sampled.samples;
    }

    // Resolves the leg for one wind sample and one environmental sample. With
    // no current the water heading is the ground course and the solve collapses
    // to the pre-Stage 3 arithmetic.
    std::optional<Error> solve_error;
    const auto solve = [&](double sample_wind_speed,
                           double sample_wind_from,
                           const EnvironmentSamples& state)
        -> std::optional<LegSolution> {
        if (options.maximum_true_wind_speed_knots.has_value() &&
            sample_wind_speed > *options.maximum_true_wind_speed_knots) {
            return std::nullopt;
        }
        const PolarSlice slice =
            polar.slice_at(sample_wind_speed, options.polar_angle_interpolation);

        // Speed through the water on one candidate water heading, after the
        // flat-water polar and any sea-state derating.
        const auto evaluate = [&](double offset_degrees)
            -> std::optional<LegSolution> {
            LegSolution leg;
            leg.water_heading_degrees =
                normalize_degrees(ground_course + offset_degrees);
            leg.true_wind_angle_degrees = angular_difference_degrees(
                leg.water_heading_degrees, sample_wind_from);
            const auto flat_water_speed = boat_speed_for_angle(
                slice, options, leg.true_wind_angle_degrees);
            if (!flat_water_speed.has_value()) {
                return std::nullopt;
            }
            leg.flat_water_speed_knots = *flat_water_speed;
            leg.water_speed_knots = leg.flat_water_speed_knots;
            if (state.has_wave) {
                leg.relative_wave_angle_degrees = relative_wave_angle_degrees(
                    leg.water_heading_degrees, state.wave.direction_from_degrees);
                Result<double> derated = apply_sea_state(
                    environment,
                    leg.flat_water_speed_knots,
                    sample_wind_speed,
                    leg.true_wind_angle_degrees,
                    leg.water_heading_degrees,
                    state.wave,
                    diagnostics);
                if (!derated) {
                    solve_error = derated.error();
                    return std::nullopt;
                }
                leg.water_speed_knots = derated.value();
            }
            if (!std::isfinite(leg.water_speed_knots) ||
                !(leg.water_speed_knots > 0.0) ||
                leg.water_speed_knots < options.minimum_boat_speed_knots) {
                return std::nullopt;
            }
            leg.board =
                board_for_heading(leg.water_heading_degrees, sample_wind_from);
            leg.ground_speed_knots = leg.water_speed_knots;
            return leg;
        };

        if (!state.has_current) {
            return evaluate(0.0);
        }

        const double course_radians = ground_course * std::numbers::pi / 180.0;
        // The along-track current adds to ground speed directly; the
        // cross-track component has to be steered against, which is what the
        // heading offset below cancels.
        const double along_track =
            state.current.east_knots * std::sin(course_radians) +
            state.current.north_knots * std::cos(course_radians);

        double offset_degrees = 0.0;
        std::optional<LegSolution> leg;
        for (int iteration = 0; iteration < course_to_steer_iterations;
             ++iteration) {
            leg = evaluate(offset_degrees);
            if (!leg.has_value()) {
                return std::nullopt;
            }
            const std::optional<double> next = water_heading_offset_degrees(
                ground_course, leg->water_speed_knots, state.current);
            if (!next.has_value()) {
                return std::nullopt;
            }
            const bool converged = std::abs(*next - offset_degrees) <=
                course_to_steer_tolerance_degrees;
            offset_degrees = *next;
            if (converged) {
                break;
            }
        }
        leg = evaluate(offset_degrees);
        if (!leg.has_value()) {
            return std::nullopt;
        }
        const double offset_radians =
            offset_degrees * std::numbers::pi / 180.0;
        leg->ground_speed_knots =
            leg->water_speed_knots * std::cos(offset_radians) + along_track;
        if (!std::isfinite(leg->ground_speed_knots) ||
            !(leg->ground_speed_knots > 0.0)) {
            return std::nullopt;
        }
        return leg;
    };

    std::optional<LegSolution> solution = solve(wind_speed, wind_from, samples);
    if (solve_error.has_value()) {
        return *solve_error;
    }
    if (!solution.has_value()) {
        return std::optional<VariableTransition>{};
    }

    // The board a transition is recorded on, and the maneuver it is charged
    // for, both come from the segment-start wind; a midpoint refinement
    // adjusts speed, not which side of the wind the vessel ends up on.
    const OperationalConfiguration configuration{
        solution->board,
        parent_configuration.sail,
        parent_configuration.reef};
    const auto delay = maneuver_delay(
        options.maneuver,
        parent_configuration,
        angular_difference_degrees(
            parent.heading_degrees,
            wind_from),
        configuration,
        solution->true_wind_angle_degrees);

    EnvironmentSamples applied = samples;
    double sailing_seconds = distance / solution->ground_speed_knots * 3600.0;
    const bool midpoint_wind =
        options.wind_sampling == WindSampling::midpoint &&
        std::chrono::duration<double>(sailing_seconds) >=
            options.midpoint_wind_sampling_threshold;
    const bool midpoint_environment = environment_fields_active &&
        environment.sampling == EnvironmentSampling::midpoint;
    if (midpoint_wind || midpoint_environment) {
        const Coordinate midpoint =
            destination_point(parent.position, ground_course, distance * 0.5);
        const TimePoint midpoint_time =
            parent.time + delay +
            std::chrono::seconds{
                static_cast<std::chrono::seconds::rep>(
                    std::ceil(sailing_seconds * 0.5))};
        double refined_wind_speed = wind_speed;
        double refined_wind_from = wind_from;
        if (midpoint_wind) {
            auto sampled_wind = weather.interpolate(midpoint, midpoint_time);
            if (!sampled_wind) {
                if (sampled_wind.error().code ==
                    ErrorCode::coordinate_outside_forecast) {
                    return std::optional<VariableTransition>{};
                }
                return sampled_wind.error();
            }
            auto evaluated_midpoint =
                evaluate_wind(sampled_wind.value(), options);
            if (!evaluated_midpoint) {
                return evaluated_midpoint.error();
            }
            if (!evaluated_midpoint.value().has_value()) {
                return std::optional<VariableTransition>{};
            }
            refined_wind_speed = evaluated_midpoint.value()->speed_knots;
            refined_wind_from =
                evaluated_midpoint.value()->direction_from_degrees;
        }
        if (midpoint_environment) {
            EnvironmentSampleResult sampled = sample_environment(
                environment, midpoint, midpoint_time, diagnostics);
            if (sampled.outcome == EnvironmentOutcome::failed) {
                return *sampled.error;
            }
            if (sampled.outcome == EnvironmentOutcome::rejected) {
                return std::optional<VariableTransition>{};
            }
            applied = sampled.samples;
        }
        const std::optional<LegSolution> refined = solve(
            refined_wind_speed, refined_wind_from, applied);
        if (solve_error.has_value()) {
            return *solve_error;
        }
        if (!refined.has_value()) {
            return std::optional<VariableTransition>{};
        }
        solution = refined;
        sailing_seconds = distance / solution->ground_speed_knots * 3600.0;
    }

    const auto duration = delay + std::chrono::seconds{
        static_cast<std::chrono::seconds::rep>(std::ceil(sailing_seconds))};
    if (duration <= std::chrono::seconds::zero() ||
        parent.time + duration > route_end) {
        return std::optional<VariableTransition>{};
    }
    const TimePoint arrival = parent.time + duration;

    if (environment_active) {
        const SegmentCheckResult geometry = check_segment_geometry(
            environment,
            parent.position,
            parent.time,
            destination,
            arrival,
            diagnostics);
        if (geometry.outcome == EnvironmentOutcome::failed) {
            return *geometry.error;
        }
        if (geometry.outcome == EnvironmentOutcome::rejected) {
            return std::optional<VariableTransition>{};
        }
    }

    RoutePoint point{
        destination,
        arrival,
        solution->water_heading_degrees,
        solution->water_speed_knots,
        wind_speed,
        wind_from,
        parent.cumulative_distance_nautical_miles + distance,
        std::nullopt};
    if (applied.has_current || applied.has_wave) {
        RoutePointEnvironment audit;
        audit.speed_over_ground_knots = solution->ground_speed_knots;
        audit.course_over_ground_degrees = ground_course;
        audit.current_east_knots = applied.current.east_knots;
        audit.current_north_knots = applied.current.north_knots;
        audit.flat_water_speed_knots = solution->flat_water_speed_knots;
        audit.significant_wave_height_metres =
            applied.wave.significant_height_metres;
        audit.wave_period_seconds = applied.wave.peak_period_seconds;
        audit.relative_wave_angle_degrees =
            solution->relative_wave_angle_degrees;
        audit.current_applied = applied.has_current;
        audit.wave_applied = applied.has_wave;
        point.environment = audit;
    }
    if (options.segment_eligibility &&
        !options.segment_eligibility(RouteSegmentView{parent, point})) {
        return std::optional<VariableTransition>{};
    }
    return std::optional<VariableTransition>{
        VariableTransition{std::move(point), configuration}};
}

}  // namespace sailroute::detail
