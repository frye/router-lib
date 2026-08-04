#include "routing/transition.hpp"

#include "routing/environment_context.hpp"
#include "routing/geodesy.hpp"

#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>
#include <utility>

namespace sailroute::detail {
namespace {

// Fixed-point iterations used to solve for the water heading that holds a
// required ground track under current. The map is a contraction whenever the
// vessel can hold the track at all, and a fixed bound keeps the result
// independent of evaluation order.
constexpr int course_to_steer_iterations = 12;
constexpr double course_to_steer_tolerance_degrees = 1.0e-10;

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
    std::int8_t parent_board,
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
        if (options.above_polar_range == AbovePolarRangePolicy::no_speed &&
            slice.above_tabulated_wind_speed()) {
            return std::nullopt;
        }

        // Speed through the water on one candidate water heading, after the
        // flat-water polar and any sea-state derating.
        const auto evaluate = [&](double offset_degrees)
            -> std::optional<LegSolution> {
            LegSolution leg;
            leg.water_heading_degrees =
                normalize_degrees(ground_course + offset_degrees);
            leg.true_wind_angle_degrees = angular_difference_degrees(
                leg.water_heading_degrees, sample_wind_from);
            leg.flat_water_speed_knots =
                slice.speed_knots(leg.true_wind_angle_degrees);
            if (!std::isfinite(leg.flat_water_speed_knots) ||
                !(leg.flat_water_speed_knots > 0.0)) {
                return std::nullopt;
            }
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
                board_for(leg.water_heading_degrees, sample_wind_from);
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
    const std::int8_t board = solution->board;
    const auto delay = maneuver_delay(
        options.maneuver,
        parent_board,
        parent.heading_degrees,
        board,
        solution->true_wind_angle_degrees,
        wind_from);

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
                return sampled_wind.error();
            }
            refined_wind_speed = sampled_wind.value().speed_knots();
            refined_wind_from =
                sampled_wind.value().direction_from_degrees();
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
        VariableTransition{std::move(point), board}};
}

}  // namespace sailroute::detail
