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
    if (!slice.supports_sailing_angle(true_wind_angle_degrees)) {
        return std::nullopt;
    }
    const double speed_knots =
        slice.speed_knots(true_wind_angle_degrees) * options.boat_speed_factor;
    if (!std::isfinite(speed_knots) || speed_knots <= 0.0 ||
        speed_knots < options.minimum_boat_speed_knots) {
        return std::nullopt;
    }
    return speed_knots;
}

namespace {

// A bounded iteration is a fast proposal; the resulting ground track is
// checked explicitly because polar-dependent speed need not be a contraction.
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
    double polar_wind_speed{};
    double polar_wind_from{};
};

Result<std::optional<LegSolution>> evaluate_water_heading(
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics,
    double wind_speed,
    double wind_from,
    const EnvironmentSamples& state,
    double water_heading_degrees) {
    if (options.maximum_true_wind_speed_knots.has_value() &&
        wind_speed > *options.maximum_true_wind_speed_knots) {
        return std::optional<LegSolution>{};
    }
    if (state.has_current &&
        (state.current.east_knots != 0.0 || state.current.north_knots != 0.0)) {
        constexpr double knots_to_mps = 1852.0 / 3600.0;
        const double angle = wind_from * std::numbers::pi / 180.0;
        const Wind water_wind = water_relative_wind(
            Wind{-wind_speed * std::sin(angle) * knots_to_mps,
                 -wind_speed * std::cos(angle) * knots_to_mps},
            state.current);
        wind_speed = water_wind.speed_knots();
        wind_from = water_wind.direction_from_degrees();
    }
    const PolarSlice slice =
        polar.slice_at(wind_speed, options.polar_angle_interpolation);
    LegSolution leg;
    leg.water_heading_degrees = normalize_degrees(water_heading_degrees);
    leg.polar_wind_speed = wind_speed;
    leg.polar_wind_from = wind_from;
    leg.true_wind_angle_degrees =
        angular_difference_degrees(leg.water_heading_degrees, wind_from);
    const auto flat_water_speed =
        boat_speed_for_angle(slice, options, leg.true_wind_angle_degrees);
    if (!flat_water_speed.has_value()) {
        return std::optional<LegSolution>{};
    }
    leg.flat_water_speed_knots = *flat_water_speed;
    leg.water_speed_knots = leg.flat_water_speed_knots;
    if (state.has_wave) {
        leg.relative_wave_angle_degrees = relative_wave_angle_degrees(
            leg.water_heading_degrees, state.wave.direction_from_degrees);
        Result<double> derated = apply_sea_state(
            environment,
            leg.flat_water_speed_knots,
            wind_speed,
            leg.true_wind_angle_degrees,
            leg.water_heading_degrees,
            state.wave,
            diagnostics);
        if (!derated) {
            return derated.error();
        }
        leg.water_speed_knots = derated.value();
    }
    if (!std::isfinite(leg.water_speed_knots) ||
        !(leg.water_speed_knots > 0.0) ||
        leg.water_speed_knots < options.minimum_boat_speed_knots) {
        return std::optional<LegSolution>{};
    }
    leg.board = board_for_heading(leg.water_heading_degrees, wind_from);
    leg.ground_speed_knots = leg.water_speed_knots;
    return std::optional<LegSolution>{leg};
}

}  // namespace

Result<std::optional<VariableTransition>> evaluate_variable_transition_step(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics,
    const RoutePoint& parent,
    OperationalConfiguration parent_configuration,
    Coordinate destination,
    TimePoint route_end,
    VariableTransitionRejection* rejection) {
    const auto reject = [rejection](VariableTransitionRejection reason)
        -> Result<std::optional<VariableTransition>> {
        if (rejection != nullptr) {
            *rejection = reason;
        }
        return std::optional<VariableTransition>{};
    };
    const double distance =
        great_circle_distance_nautical_miles(parent.position, destination);
    if (!(distance > 0.0)) {
        return reject(VariableTransitionRejection::infeasible);
    }
    const double ground_course =
        initial_bearing_degrees(parent.position, destination);
    auto wind_result = weather.interpolate(parent.position, parent.time);
    if (!wind_result) {
        if (wind_result.error().code ==
            ErrorCode::coordinate_outside_forecast) {
            return reject(VariableTransitionRejection::missing_data);
        }
        return wind_result.error();
    }
    auto evaluated_wind = evaluate_wind(wind_result.value(), options);
    if (!evaluated_wind) {
        return evaluated_wind.error();
    }
    if (!evaluated_wind.value().has_value()) {
        return reject(VariableTransitionRejection::infeasible);
    }
    const double wind_speed = evaluated_wind.value()->speed_knots;
    const double wind_from =
        evaluated_wind.value()->direction_from_degrees;
    double applied_wind_speed = wind_speed;
    double applied_wind_from = wind_from;

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
            return reject(VariableTransitionRejection::infeasible);
        }
        samples = sampled.samples;
    }

    // Resolves the leg for one wind sample and one environmental sample. With
    // no current the water heading is the ground course and the solve collapses
    // to the pre-Stage 3 arithmetic.
    const auto solve = [&](double sample_wind_speed,
                           double sample_wind_from,
                           const EnvironmentSamples& state)
        -> Result<std::optional<LegSolution>> {
        if (!state.has_current) {
            return evaluate_water_heading(
                polar,
                options,
                environment,
                diagnostics,
                sample_wind_speed,
                sample_wind_from,
                state,
                ground_course);
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
            auto evaluated = evaluate_water_heading(
                polar,
                options,
                environment,
                diagnostics,
                sample_wind_speed,
                sample_wind_from,
                state,
                ground_course + offset_degrees);
            if (!evaluated) {
                return evaluated.error();
            }
            leg = std::move(evaluated.value());
            if (!leg.has_value()) {
                break;
            }
            const std::optional<double> next = water_heading_offset_degrees(
                ground_course, leg->water_speed_knots, state.current);
            if (!next.has_value()) {
                break;
            }
            const bool converged = std::abs(*next - offset_degrees) <=
                course_to_steer_tolerance_degrees;
            offset_degrees = *next;
            if (converged) {
                break;
            }
        }
        auto evaluated = evaluate_water_heading(
            polar,
            options,
            environment,
            diagnostics,
            sample_wind_speed,
            sample_wind_from,
            state,
            ground_course + offset_degrees);
        if (!evaluated) {
            return evaluated.error();
        }
        leg = std::move(evaluated.value());
        if (leg) {
            const GroundVelocity actual = ground_velocity(
                leg->water_heading_degrees, leg->water_speed_knots, state.current);
            if (actual.speed_knots > 0.0 &&
                angular_difference_degrees(actual.course_degrees, ground_course) <= 1.0e-6) {
                leg->ground_speed_knots = actual.speed_knots;
                return leg;
            }
        }

        // A polar can make the fixed-point iteration oscillate. Bracket the
        // cross-track residual within contiguous sailable angular intervals.
        const double cross_track = state.current.east_knots * std::cos(course_radians) -
            state.current.north_knots * std::sin(course_radians);
        const auto residual = [cross_track](const LegSolution& value, double offset) {
            return value.water_speed_knots * std::sin(offset * std::numbers::pi / 180.0) + cross_track;
        };
        std::optional<LegSolution> best;
        std::optional<LegSolution> previous;
        double previous_offset = -180.0;
        for (double offset = -180.0; offset <= 180.0; offset += 2.0) {
            auto candidate = evaluate_water_heading(
                polar, options, environment, diagnostics, sample_wind_speed,
                sample_wind_from, state, ground_course + offset);
            if (!candidate) return candidate.error();
            if (candidate.value()) {
                auto root = candidate.value();
                double root_offset = offset;
                const double right_residual = residual(*root, offset);
                if (std::abs(right_residual) > 1.0e-9 && previous &&
                    residual(*previous, previous_offset) * right_residual < 0.0) {
                    double left = previous_offset;
                    double right = offset;
                    double left_residual = residual(*previous, left);
                    for (int count = 0; count < 60; ++count) {
                        root_offset = 0.5 * (left + right);
                        auto middle = evaluate_water_heading(
                            polar, options, environment, diagnostics, sample_wind_speed,
                            sample_wind_from, state, ground_course + root_offset);
                        if (!middle) return middle.error();
                        root = std::move(middle.value());
                        if (!root) break;
                        const double value = residual(*root, root_offset);
                        if (std::abs(value) < 1.0e-10) break;
                        if (value * left_residual > 0.0) {
                            left = root_offset;
                            left_residual = value;
                        } else {
                            right = root_offset;
                        }
                    }
                }
                if (root && std::abs(residual(*root, root_offset)) <= 1.0e-8) {
                    root->ground_speed_knots = root->water_speed_knots *
                        std::cos(root_offset * std::numbers::pi / 180.0) + along_track;
                    if (root->ground_speed_knots > 0.0 &&
                        (!best || root->ground_speed_knots > best->ground_speed_knots)) best = root;
                }
            }
            previous = std::move(candidate.value());
            previous_offset = offset;
        }
        return best;
    };

    auto solved = solve(wind_speed, wind_from, samples);
    if (!solved) {
        return solved.error();
    }
    std::optional<LegSolution> solution = std::move(solved.value());
    if (!solution.has_value()) {
        return reject(VariableTransitionRejection::infeasible);
    }

    // The board a transition is recorded on, and the maneuver it is charged
    // for, both come from the segment-start wind; a midpoint refinement
    // adjusts speed, not which side of the wind the vessel ends up on.
    const OperationalConfiguration configuration{
        solution->board != 0 ? solution->board : parent_configuration.board,
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
                    return reject(VariableTransitionRejection::missing_data);
                }
                if (sampled_wind.error().code ==
                    ErrorCode::departure_outside_forecast) {
                    return reject(
                        VariableTransitionRejection::forecast_exhausted);
                }
                return sampled_wind.error();
            }
            auto evaluated_midpoint =
                evaluate_wind(sampled_wind.value(), options);
            if (!evaluated_midpoint) {
                return evaluated_midpoint.error();
            }
            if (!evaluated_midpoint.value().has_value()) {
                return reject(VariableTransitionRejection::infeasible);
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
                return reject(VariableTransitionRejection::infeasible);
            }
            applied = sampled.samples;
        }
        auto refined_result = solve(
            refined_wind_speed, refined_wind_from, applied);
        if (!refined_result) {
            return refined_result.error();
        }
        const std::optional<LegSolution> refined =
            std::move(refined_result.value());
        if (!refined.has_value()) {
            return reject(VariableTransitionRejection::infeasible);
        }
        solution = refined;
        applied_wind_speed = refined_wind_speed;
        applied_wind_from = refined_wind_from;
        sailing_seconds = distance / solution->ground_speed_knots * 3600.0;
    }

    constexpr double time_roundoff_seconds = 1.0e-6;
    const double available_seconds =
        std::chrono::duration<double>(route_end - parent.time - delay).count();
    if (!std::isfinite(sailing_seconds) ||
        sailing_seconds > available_seconds + time_roundoff_seconds) {
        return reject(VariableTransitionRejection::duration_exhausted);
    }
    // Inverse geodesy must not turn an exact step boundary into another
    // second and force a spurious subdivision of an otherwise sailable leg.
    const auto duration = delay + std::chrono::seconds{
        static_cast<std::chrono::seconds::rep>(
            std::max(1.0, std::ceil(sailing_seconds - time_roundoff_seconds)))};
    if (duration <= std::chrono::seconds::zero() ||
        parent.time + duration > route_end) {
        return reject(
            duration <= std::chrono::seconds::zero()
                ? VariableTransitionRejection::infeasible
                : VariableTransitionRejection::duration_exhausted);
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
            return reject(VariableTransitionRejection::infeasible);
        }
    }

    RoutePoint point{
        destination,
        arrival,
        solution->water_heading_degrees,
        solution->water_speed_knots,
        applied_wind_speed,
        applied_wind_from,
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
        audit.polar_wind_speed_knots = solution->polar_wind_speed;
        audit.polar_wind_direction_degrees = solution->polar_wind_from;
        point.environment = audit;
    }
    if (options.segment_eligibility &&
        !options.segment_eligibility(RouteSegmentView{parent, point})) {
        return reject(VariableTransitionRejection::infeasible);
    }
    return std::optional<VariableTransition>{
        VariableTransition{std::move(point), configuration, {}}};
}

Result<std::optional<VariableTransition>> evaluate_heading_transition_step(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics,
    const RoutePoint& parent,
    OperationalConfiguration parent_configuration,
    double water_heading_degrees,
    TimePoint arrival,
    std::optional<Coordinate> arrival_destination,
    double arrival_radius_nautical_miles,
    VariableTransitionRejection* rejection,
    unsigned arrival_refinements) {
    const auto reject = [rejection](VariableTransitionRejection reason)
        -> Result<std::optional<VariableTransition>> {
        if (rejection != nullptr) {
            *rejection = reason;
        }
        return std::optional<VariableTransition>{};
    };
    const auto total_duration =
        std::chrono::duration_cast<std::chrono::seconds>(arrival - parent.time);
    if (total_duration <= std::chrono::seconds::zero()) {
        return reject(VariableTransitionRejection::infeasible);
    }

    auto wind_result = weather.interpolate(parent.position, parent.time);
    if (!wind_result) {
        if (wind_result.error().code == ErrorCode::coordinate_outside_forecast) {
            return reject(VariableTransitionRejection::missing_data);
        }
        return wind_result.error();
    }
    auto evaluated_wind = evaluate_wind(wind_result.value(), options);
    if (!evaluated_wind) {
        return evaluated_wind.error();
    }
    if (!evaluated_wind.value().has_value()) {
        return reject(VariableTransitionRejection::infeasible);
    }
    const double wind_speed = evaluated_wind.value()->speed_knots;
    const double wind_from =
        evaluated_wind.value()->direction_from_degrees;
    double applied_wind_speed = wind_speed;
    double applied_wind_from = wind_from;

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
            return reject(VariableTransitionRejection::infeasible);
        }
        samples = sampled.samples;
    }

    auto initial_result = evaluate_water_heading(
        polar,
        options,
        environment,
        diagnostics,
        wind_speed,
        wind_from,
        samples,
        water_heading_degrees);
    if (!initial_result) {
        return initial_result.error();
    }
    if (!initial_result.value().has_value()) {
        return reject(VariableTransitionRejection::infeasible);
    }
    LegSolution solution = std::move(*initial_result.value());
    const OperationalConfiguration configuration{
        solution.board != 0 ? solution.board : parent_configuration.board,
        parent_configuration.sail,
        parent_configuration.reef};
    const auto delay = maneuver_delay(
        options.maneuver,
        parent_configuration,
        angular_difference_degrees(parent.heading_degrees, wind_from),
        configuration,
        solution.true_wind_angle_degrees);
    if (delay >= total_duration) {
        return reject(VariableTransitionRejection::infeasible);
    }
    const auto sailing_duration = total_duration - delay;
    const double sailing_hours =
        std::chrono::duration<double, std::ratio<3600>>(sailing_duration).count();

    EnvironmentSamples applied = samples;
    GroundVelocity ground = applied.has_current
        ? ground_velocity(
              solution.water_heading_degrees,
              solution.water_speed_knots,
              applied.current)
        : GroundVelocity{
              solution.water_heading_degrees, solution.water_speed_knots};
    if (!(ground.speed_knots > 0.0)) {
        return reject(VariableTransitionRejection::infeasible);
    }

    // Bound the interval by a provisional arrival before asking for weather
    // beyond the voyage. A later midpoint correction may finish short; the
    // bounded wrapper then continues from that supported intermediate state.
    if (arrival_destination && arrival_refinements < 16U) {
        const auto origin = prepare_origin(parent.position);
        const auto fraction = arrival_fraction(
            origin, great_circle_distance_nautical_miles(parent.position, *arrival_destination),
            initial_bearing_degrees(parent.position, *arrival_destination),
            ground.course_degrees, ground.speed_knots * sailing_hours,
            *arrival_destination, arrival_radius_nautical_miles);
        if (fraction) {
            const auto shortened = parent.time + delay + std::chrono::seconds{
                static_cast<std::chrono::seconds::rep>(std::max(1.0,
                    std::ceil(static_cast<double>(sailing_duration.count()) * *fraction)))};
            if (shortened < arrival) {
                return evaluate_heading_transition_step(
                    weather, polar, options, environment, diagnostics, parent,
                    parent_configuration, water_heading_degrees, shortened,
                    arrival_destination, arrival_radius_nautical_miles, rejection,
                    arrival_refinements + 1U);
            }
        }
    }

    const bool midpoint_wind =
        options.wind_sampling == WindSampling::midpoint &&
        sailing_duration >= options.midpoint_wind_sampling_threshold;
    const bool midpoint_environment = environment_fields_active &&
        environment.sampling == EnvironmentSampling::midpoint;
    if (midpoint_wind || midpoint_environment) {
        const Coordinate midpoint = destination_point(
            parent.position,
            ground.course_degrees,
            ground.speed_knots * sailing_hours * 0.5);
        const TimePoint midpoint_time =
            parent.time + delay + sailing_duration / 2;
        double refined_wind_speed = wind_speed;
        double refined_wind_from = wind_from;
        if (midpoint_wind) {
            auto sampled_wind = weather.interpolate(midpoint, midpoint_time);
            if (!sampled_wind) {
                if (sampled_wind.error().code ==
                    ErrorCode::coordinate_outside_forecast) {
                    return reject(VariableTransitionRejection::missing_data);
                }
                if (sampled_wind.error().code ==
                    ErrorCode::departure_outside_forecast) {
                    return reject(
                        VariableTransitionRejection::forecast_exhausted);
                }
                return sampled_wind.error();
            }
            auto evaluated_midpoint =
                evaluate_wind(sampled_wind.value(), options);
            if (!evaluated_midpoint) {
                return evaluated_midpoint.error();
            }
            if (!evaluated_midpoint.value().has_value()) {
                return reject(VariableTransitionRejection::infeasible);
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
                return reject(VariableTransitionRejection::infeasible);
            }
            applied = sampled.samples;
        }
        auto refined_result = evaluate_water_heading(
            polar,
            options,
            environment,
            diagnostics,
            refined_wind_speed,
            refined_wind_from,
            applied,
            water_heading_degrees);
        if (!refined_result) {
            return refined_result.error();
        }
        if (!refined_result.value().has_value()) {
            return reject(VariableTransitionRejection::infeasible);
        }
        solution = std::move(*refined_result.value());
        applied_wind_speed = refined_wind_speed;
        applied_wind_from = refined_wind_from;
        ground = applied.has_current
            ? ground_velocity(
                  solution.water_heading_degrees,
                  solution.water_speed_knots,
                  applied.current)
            : GroundVelocity{
                  solution.water_heading_degrees,
                  solution.water_speed_knots};
        if (!(ground.speed_knots > 0.0)) {
            return reject(VariableTransitionRejection::infeasible);
        }
    }

    double distance = ground.speed_knots * sailing_hours;
    TimePoint actual_arrival = arrival;
    const PreparedOrigin origin = prepare_origin(parent.position);
    Coordinate position = destination_point_from(
        origin, ground.course_degrees, distance);
    if (arrival_destination.has_value()) {
        const double destination_distance =
            great_circle_distance_nautical_miles(
                parent.position, *arrival_destination);
        const std::optional<double> fraction = arrival_fraction(
            origin,
            destination_distance,
            initial_bearing_degrees(parent.position, *arrival_destination),
            ground.course_degrees,
            distance,
            *arrival_destination,
            arrival_radius_nautical_miles);
        if (fraction.has_value()) {
            distance *= *fraction;
            position = destination_point_from(
                origin, ground.course_degrees, distance);
            actual_arrival =
                parent.time + delay +
                std::chrono::seconds{
                    static_cast<std::chrono::seconds::rep>(
                        std::ceil(
                            static_cast<double>(sailing_duration.count()) *
                            *fraction))};
            if (actual_arrival > parent.time && actual_arrival < arrival &&
                (midpoint_wind || midpoint_environment)) {
                if (arrival_refinements >= 16U) {
                    return Error{ErrorCode::no_route, "arrival integration did not converge within its refinement budget"};
                }
                return evaluate_heading_transition_step(
                    weather, polar, options, environment, diagnostics, parent,
                    parent_configuration, water_heading_degrees, actual_arrival,
                    arrival_destination, arrival_radius_nautical_miles, rejection,
                    arrival_refinements + 1U);
            }
        }
    }
    if (environment_active) {
        const SegmentCheckResult geometry = check_segment_geometry(
            environment,
            parent.position,
            parent.time,
            position,
            actual_arrival,
            diagnostics);
        if (geometry.outcome == EnvironmentOutcome::failed) {
            return *geometry.error;
        }
        if (geometry.outcome == EnvironmentOutcome::rejected) {
            return reject(VariableTransitionRejection::infeasible);
        }
    }

    RoutePoint point{
        position,
        actual_arrival,
        solution.water_heading_degrees,
        solution.water_speed_knots,
        applied_wind_speed,
        applied_wind_from,
        parent.cumulative_distance_nautical_miles + distance,
        std::nullopt};
    if (applied.has_current || applied.has_wave) {
        RoutePointEnvironment audit;
        audit.speed_over_ground_knots = ground.speed_knots;
        audit.course_over_ground_degrees = ground.course_degrees;
        audit.current_east_knots = applied.current.east_knots;
        audit.current_north_knots = applied.current.north_knots;
        audit.flat_water_speed_knots = solution.flat_water_speed_knots;
        audit.significant_wave_height_metres =
            applied.wave.significant_height_metres;
        audit.wave_period_seconds = applied.wave.peak_period_seconds;
        audit.relative_wave_angle_degrees =
            solution.relative_wave_angle_degrees;
        audit.current_applied = applied.has_current;
        audit.wave_applied = applied.has_wave;
        audit.polar_wind_speed_knots = solution.polar_wind_speed;
        audit.polar_wind_direction_degrees = solution.polar_wind_from;
        point.environment = audit;
    }
    if (options.segment_eligibility &&
        !options.segment_eligibility(RouteSegmentView{parent, point})) {
        return reject(VariableTransitionRejection::infeasible);
    }
    return std::optional<VariableTransition>{
        VariableTransition{std::move(point), configuration, {}}};
}

}  // namespace sailroute::detail
