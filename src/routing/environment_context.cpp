#include "routing/environment_context.hpp"

#include "routing/geodesy.hpp"

#include <cmath>
#include <numbers>
#include <string>

namespace sailroute::detail {
namespace {

constexpr double degrees_to_radians = std::numbers::pi / 180.0;
constexpr double radians_to_degrees = 180.0 / std::numbers::pi;

// Slack allowed when checking that a sea-state model did not accelerate the
// vessel, so a model that returns the flat-water speed after its own rounding
// is not treated as a contract violation.
constexpr double sea_state_speed_slack_knots = 1.0e-9;

Error missing_data_error(
    std::string_view provider,
    EnvironmentSampleStatus status) {
    return Error{
        ErrorCode::environment_data_unavailable,
        std::string{provider} + " provider returned no usable sample (" +
            std::string{to_string(status)} + ")"};
}

EnvironmentSampleResult policy_result(
    MissingDataPolicy policy,
    std::string_view provider,
    EnvironmentSampleStatus status) {
    EnvironmentSampleResult result;
    if (policy == MissingDataPolicy::reject_transition) {
        result.outcome = EnvironmentOutcome::rejected;
        return result;
    }
    result.outcome = EnvironmentOutcome::failed;
    result.error = missing_data_error(provider, status);
    return result;
}

}  // namespace

EnvironmentSampleResult sample_environment(
    const RoutingEnvironment& environment,
    Coordinate coordinate,
    TimePoint time,
    EnvironmentDiagnostics& diagnostics) {
    EnvironmentSampleResult result;
    if (environment.currents.configured()) {
        ++diagnostics.current_samples;
        const EnvironmentSample<CurrentVector> sample =
            environment.currents.provider->sample(coordinate, time);
        if (!sample.has_value() || !std::isfinite(sample.value.east_knots) ||
            !std::isfinite(sample.value.north_knots)) {
            ++diagnostics.current_rejections;
            return policy_result(
                environment.currents.missing_data_policy,
                "current",
                sample.has_value() ? EnvironmentSampleStatus::invalid_data
                                   : sample.status);
        }
        result.samples.has_current = true;
        result.samples.current = sample.value;
    }
    if (environment.waves.configured()) {
        ++diagnostics.wave_samples;
        const EnvironmentSample<WaveState> sample =
            environment.waves.provider->sample(coordinate, time);
        const bool usable = sample.has_value() &&
            std::isfinite(sample.value.significant_height_metres) &&
            std::isfinite(sample.value.peak_period_seconds) &&
            std::isfinite(sample.value.direction_from_degrees) &&
            sample.value.significant_height_metres >= 0.0 &&
            sample.value.peak_period_seconds >= 0.0;
        if (!usable) {
            ++diagnostics.wave_rejections;
            return policy_result(
                environment.waves.missing_data_policy,
                "wave",
                sample.has_value() ? EnvironmentSampleStatus::invalid_data
                                   : sample.status);
        }
        result.samples.has_wave = true;
        result.samples.wave = sample.value;
    }
    return result;
}

double relative_wave_angle_degrees(
    double heading_degrees,
    double wave_direction_from_degrees) noexcept {
    const double travelling_toward =
        normalize_degrees(wave_direction_from_degrees + 180.0);
    return angular_difference_degrees(heading_degrees, travelling_toward);
}

Result<double> apply_sea_state(
    const RoutingEnvironment& environment,
    double flat_water_speed_knots,
    double true_wind_speed_knots,
    double true_wind_angle_degrees,
    double heading_degrees,
    const WaveState& wave,
    EnvironmentDiagnostics& diagnostics) {
    SeaStateInput input;
    input.flat_water_speed_knots = flat_water_speed_knots;
    input.true_wind_speed_knots = true_wind_speed_knots;
    input.true_wind_angle_degrees = true_wind_angle_degrees;
    input.heading_degrees = heading_degrees;
    input.relative_wave_angle_degrees =
        relative_wave_angle_degrees(heading_degrees, wave.direction_from_degrees);
    input.wave = wave;

    ++diagnostics.sea_state_evaluations;
    const double derated = environment.waves.model->derated_speed_knots(input);
    if (!std::isfinite(derated) || derated < 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "sea-state model '" + environment.waves.model->metadata().name +
                "' returned a non-finite or negative speed"};
    }
    if (derated > flat_water_speed_knots + sea_state_speed_slack_knots) {
        return Error{
            ErrorCode::invalid_environment,
            "sea-state model '" + environment.waves.model->metadata().name +
                "' returned a speed above the flat-water polar speed"};
    }
    return derated;
}

GroundVelocity ground_velocity(
    double water_heading_degrees,
    double water_speed_knots,
    CurrentVector current) noexcept {
    const double heading = water_heading_degrees * degrees_to_radians;
    const double east =
        water_speed_knots * std::sin(heading) + current.east_knots;
    const double north =
        water_speed_knots * std::cos(heading) + current.north_knots;
    if (east == 0.0 && north == 0.0) {
        return GroundVelocity{normalize_degrees(water_heading_degrees), 0.0};
    }
    return GroundVelocity{
        normalize_degrees(std::atan2(east, north) * radians_to_degrees),
        std::hypot(east, north)};
}

std::optional<double> water_heading_offset_degrees(
    double ground_course_degrees,
    double water_speed_knots,
    CurrentVector current) noexcept {
    if (!(water_speed_knots > 0.0)) {
        return std::nullopt;
    }
    const double course = ground_course_degrees * degrees_to_radians;
    // Current component perpendicular to the required ground track. The vessel
    // must aim far enough upstream to cancel it exactly, or the track cannot be
    // held at all.
    const double cross =
        current.east_knots * std::cos(course) - current.north_knots * std::sin(course);
    const double sine = -cross / water_speed_knots;
    if (!std::isfinite(sine) || sine < -1.0 || sine > 1.0) {
        return std::nullopt;
    }
    return std::asin(sine) * radians_to_degrees;
}

SegmentCheckResult check_segment_geometry(
    const RoutingEnvironment& environment,
    Coordinate from,
    TimePoint from_time,
    Coordinate to,
    TimePoint to_time,
    EnvironmentDiagnostics& diagnostics) {
    SegmentCheckResult result;
    if (environment.land.configured()) {
        ++diagnostics.land_checks;
        const SignedDistanceLandmask::ClearanceResult clearance =
            environment.land.landmask->certify_segment(
                from,
                to,
                environment.land.clearance_nautical_miles,
                environment.land.maximum_subdivision_depth);
        diagnostics.land_distance_queries += clearance.distance_queries;
        if (!clearance.clear) {
            ++diagnostics.land_rejections;
            if (clearance.status != EnvironmentSampleStatus::available) {
                // The mask could not answer, which is a data problem rather
                // than a proven land crossing, so the configured policy
                // decides.
                if (environment.land.missing_data_policy ==
                    MissingDataPolicy::fail_route) {
                    result.outcome = EnvironmentOutcome::failed;
                    result.error =
                        missing_data_error("landmask", clearance.status);
                    return result;
                }
            }
            result.outcome = EnvironmentOutcome::rejected;
            return result;
        }
    }
    if (environment.exclusions.configured()) {
        ++diagnostics.exclusion_checks;
        const ExclusionZoneSet::SegmentResult segment =
            environment.exclusions.zones->intersects_segment(
                from,
                from_time,
                to,
                to_time,
                environment.exclusions.boundary_policy);
        diagnostics.exclusion_geometry_tests += segment.geometry_tests;
        if (segment.violated) {
            ++diagnostics.exclusion_rejections;
            result.outcome = EnvironmentOutcome::rejected;
            return result;
        }
    }
    return result;
}

void merge(EnvironmentDiagnostics& into, const EnvironmentDiagnostics& from) noexcept {
    into.current_samples += from.current_samples;
    into.current_rejections += from.current_rejections;
    into.wave_samples += from.wave_samples;
    into.wave_rejections += from.wave_rejections;
    into.sea_state_evaluations += from.sea_state_evaluations;
    into.land_checks += from.land_checks;
    into.land_distance_queries += from.land_distance_queries;
    into.land_rejections += from.land_rejections;
    into.exclusion_checks += from.exclusion_checks;
    into.exclusion_geometry_tests += from.exclusion_geometry_tests;
    into.exclusion_rejections += from.exclusion_rejections;
}

}  // namespace sailroute::detail
