#include "sailroute/serialization.hpp"

#include "sailroute/time.hpp"
#include "serialization/numeric_encoding.hpp"
#include "serialization/text_encoding.hpp"

#include <cmath>
#include <optional>
#include <string>
#include <string_view>

namespace sailroute {
namespace {

void append_coordinate(std::string& output, Coordinate coordinate) {
    output.append("{\"latitude\":");
    serialization_detail::append_number(output, coordinate.latitude_degrees);
    output.append(",\"longitude\":");
    serialization_detail::append_number(output, coordinate.longitude_degrees);
    output.push_back('}');
}

Error invalid_numeric_value(std::string_view field) {
    return Error{
        ErrorCode::output_error,
        "cannot serialize non-finite route value: " + std::string{field}};
}

Result<std::string> validate_environment_numbers(const RoutePoint& point) {
    if (!point.environment.has_value()) {
        return std::string{};
    }
    const RoutePointEnvironment& environment = *point.environment;
    if (!std::isfinite(environment.speed_over_ground_knots)) {
        return invalid_numeric_value("speed_over_ground_knots");
    }
    if (!std::isfinite(environment.course_over_ground_degrees)) {
        return invalid_numeric_value("course_over_ground_degrees");
    }
    if (!std::isfinite(environment.current_east_knots) ||
        !std::isfinite(environment.current_north_knots)) {
        return invalid_numeric_value("current_knots");
    }
    if (!std::isfinite(environment.flat_water_speed_knots)) {
        return invalid_numeric_value("flat_water_speed_knots");
    }
    if (!std::isfinite(environment.significant_wave_height_metres)) {
        return invalid_numeric_value("significant_wave_height_metres");
    }
    if (!std::isfinite(environment.wave_period_seconds)) {
        return invalid_numeric_value("wave_period_seconds");
    }
    if (!std::isfinite(environment.relative_wave_angle_degrees)) {
        return invalid_numeric_value("relative_wave_angle_degrees");
    }
    return std::string{};
}

void append_provider(
    std::string& output,
    std::string_view key,
    const std::optional<ProviderMetadata>& provider) {
    output.push_back(',');
    serialization_detail::append_json_string(output, key);
    output.push_back(':');
    if (!provider.has_value()) {
        output.append("null");
        return;
    }
    output.append("{\"name\":");
    serialization_detail::append_json_string(output, provider->name);
    output.append(",\"source\":");
    serialization_detail::append_json_string(output, provider->source);
    output.append(",\"revision\":");
    serialization_detail::append_json_string(output, provider->revision);
    output.push_back('}');
}

Result<std::string> validate_route_numbers(const RouteResult& route) {
    for (const RoutePoint& point : route.points) {
        if (!std::isfinite(point.position.latitude_degrees)) {
            return invalid_numeric_value("latitude");
        }
        if (!std::isfinite(point.position.longitude_degrees)) {
            return invalid_numeric_value("longitude");
        }
        if (!std::isfinite(point.heading_degrees)) {
            return invalid_numeric_value("heading_degrees");
        }
        if (!std::isfinite(point.boat_speed_knots)) {
            return invalid_numeric_value("boat_speed_knots");
        }
        if (!std::isfinite(point.true_wind_speed_knots)) {
            return invalid_numeric_value("true_wind_speed_knots");
        }
        if (!std::isfinite(point.true_wind_direction_degrees)) {
            return invalid_numeric_value("true_wind_direction_degrees");
        }
        if (!std::isfinite(point.cumulative_distance_nautical_miles)) {
            return invalid_numeric_value("cumulative_distance_nautical_miles");
        }
        const Result<std::string> environment =
            validate_environment_numbers(point);
        if (!environment) {
            return environment.error();
        }
    }
    if (route.environment.has_value()) {
        const RouteEnvironmentMetadata& environment = *route.environment;
        if (!std::isfinite(environment.land_resolution_nautical_miles) ||
            !std::isfinite(
                environment.land_interpolation_error_nautical_miles) ||
            !std::isfinite(environment.land_clearance_nautical_miles)) {
            return invalid_numeric_value("land_clearance_nautical_miles");
        }
    }
    return std::string{};
}

}  // namespace

Result<std::string> route_to_json(const RouteResult& route) {
    const Result<std::string> validation = validate_route_numbers(route);
    if (!validation) {
        return validation.error();
    }

    std::string output;
    output.reserve(512 + route.points.size() * 320);
    output.append("{\n  \"completion\":");
    serialization_detail::append_json_string(output, to_string(route.completion));
    output.append(",\n  \"departure\":{\"time\":");
    serialization_detail::append_json_string(output, format_utc_time(route.departure_time));
    output.append(",\"source\":");
    serialization_detail::append_json_string(output, to_string(route.departure_source));
    output.append(",\"position\":");
    if (route.points.empty()) {
        output.append("null");
    } else {
        append_coordinate(output, route.points.front().position);
    }

    output.append("},\n  \"arrival\":{\"time\":");
    serialization_detail::append_json_string(output, format_utc_time(route.arrival_time));
    output.append(",\"position\":");
    if (route.points.empty()) {
        output.append("null");
    } else {
        append_coordinate(output, route.points.back().position);
    }

    output.append("},\n  \"forecast\":{\"source\":");
    serialization_detail::append_json_string(output, route.forecast_source);
    output.append("},\n  \"polar\":{\"source\":");
    serialization_detail::append_json_string(output, route.polar_source);
    output.append("},\n  \"diagnostics\":{\"expandedNodes\":");
    output.append(std::to_string(route.diagnostics.expanded_nodes));
    output.append(",\"generatedCandidates\":");
    output.append(std::to_string(route.diagnostics.generated_candidates));
    output.append(",\"retainedCandidates\":");
    output.append(std::to_string(route.diagnostics.retained_candidates));
    output.append(",\"timeSteps\":");
    output.append(std::to_string(route.diagnostics.time_steps));
    output.push_back('}');
    if (route.lattice_diagnostics.has_value()) {
        const LatticeRouteDiagnostics& lattice = *route.lattice_diagnostics;
        output.append(",\n  \"latticeDiagnostics\":{\"settledLabels\":");
        output.append(std::to_string(lattice.settled_labels));
        output.append(",\"queuedLabels\":");
        output.append(std::to_string(lattice.queued_labels));
        output.append(",\"relaxedLabels\":");
        output.append(std::to_string(lattice.relaxed_labels));
        output.append(",\"waitTransitions\":");
        output.append(std::to_string(lattice.wait_transitions));
        output.append(",\"refinementRuns\":");
        output.append(std::to_string(lattice.refinement_runs));
        output.append(",\"acceptedRefinements\":");
        output.append(std::to_string(lattice.accepted_refinements));
        output.append(",\"subdivisionLevel\":");
        output.append(std::to_string(lattice.subdivision_level));
        output.append(",\"refinementFallback\":");
        output.append(lattice.refinement_fallback ? "true" : "false");
        output.push_back('}');
    }
    if (route.environment_diagnostics.has_value()) {
        const EnvironmentDiagnostics& counters = *route.environment_diagnostics;
        output.append(",\n  \"environmentDiagnostics\":{\"currentSamples\":");
        output.append(std::to_string(counters.current_samples));
        output.append(",\"currentRejections\":");
        output.append(std::to_string(counters.current_rejections));
        output.append(",\"waveSamples\":");
        output.append(std::to_string(counters.wave_samples));
        output.append(",\"waveRejections\":");
        output.append(std::to_string(counters.wave_rejections));
        output.append(",\"seaStateEvaluations\":");
        output.append(std::to_string(counters.sea_state_evaluations));
        output.append(",\"landChecks\":");
        output.append(std::to_string(counters.land_checks));
        output.append(",\"landDistanceQueries\":");
        output.append(std::to_string(counters.land_distance_queries));
        output.append(",\"landRejections\":");
        output.append(std::to_string(counters.land_rejections));
        output.append(",\"exclusionChecks\":");
        output.append(std::to_string(counters.exclusion_checks));
        output.append(",\"exclusionGeometryTests\":");
        output.append(std::to_string(counters.exclusion_geometry_tests));
        output.append(",\"exclusionRejections\":");
        output.append(std::to_string(counters.exclusion_rejections));
        output.push_back('}');
    }
    if (route.environment.has_value()) {
        const RouteEnvironmentMetadata& environment = *route.environment;
        output.append(",\n  \"environment\":{\"sampling\":");
        serialization_detail::append_json_string(
            output, to_string(environment.sampling));
        append_provider(output, "currentProvider", environment.current_provider);
        append_provider(output, "waveProvider", environment.wave_provider);
        append_provider(output, "seaStateModel", environment.sea_state_model);
        append_provider(output, "landmask", environment.landmask);
        append_provider(output, "exclusions", environment.exclusions);
        output.append(",\"policies\":{\"current\":");
        serialization_detail::append_json_string(
            output, to_string(environment.current_policy));
        output.append(",\"wave\":");
        serialization_detail::append_json_string(
            output, to_string(environment.wave_policy));
        output.append(",\"land\":");
        serialization_detail::append_json_string(
            output, to_string(environment.land_policy));
        output.append(",\"exclusion\":");
        serialization_detail::append_json_string(
            output, to_string(environment.exclusion_policy));
        output.append("},\"landResolutionNauticalMiles\":");
        serialization_detail::append_number(
            output, environment.land_resolution_nautical_miles);
        output.append(",\"landInterpolationErrorNauticalMiles\":");
        serialization_detail::append_number(
            output, environment.land_interpolation_error_nautical_miles);
        output.append(",\"landClearanceNauticalMiles\":");
        serialization_detail::append_number(
            output, environment.land_clearance_nautical_miles);
        output.append(",\"exclusionBoundaryPolicy\":");
        serialization_detail::append_json_string(
            output, to_string(environment.exclusion_boundary_policy));
        output.append(",\"exclusionZoneCount\":");
        output.append(std::to_string(environment.exclusion_zone_count));
        output.append(",\"exclusionRevision\":");
        output.append(std::to_string(environment.exclusion_revision));
        output.push_back('}');
    }
    output.append(",\n  \"points\":[");

    bool first = true;
    for (const RoutePoint& point : route.points) {
        if (!first) {
            output.push_back(',');
        }
        first = false;
        output.append("\n    {\"time\":");
        serialization_detail::append_json_string(output, format_utc_time(point.time));
        output.append(",\"position\":");
        append_coordinate(output, point.position);
        output.append(",\"headingDegrees\":");
        serialization_detail::append_number(output, point.heading_degrees);
        output.append(",\"boatSpeedKnots\":");
        serialization_detail::append_number(output, point.boat_speed_knots);
        output.append(",\"trueWindSpeedKnots\":");
        serialization_detail::append_number(output, point.true_wind_speed_knots);
        output.append(",\"trueWindDirectionDegrees\":");
        serialization_detail::append_number(
            output,
            point.true_wind_direction_degrees);
        output.append(",\"cumulativeDistanceNauticalMiles\":");
        serialization_detail::append_number(
            output,
            point.cumulative_distance_nautical_miles);
        if (point.environment.has_value()) {
            const RoutePointEnvironment& environment = *point.environment;
            output.append(",\"environment\":{\"speedOverGroundKnots\":");
            serialization_detail::append_number(
                output, environment.speed_over_ground_knots);
            output.append(",\"courseOverGroundDegrees\":");
            serialization_detail::append_number(
                output, environment.course_over_ground_degrees);
            output.append(",\"currentEastKnots\":");
            serialization_detail::append_number(
                output, environment.current_east_knots);
            output.append(",\"currentNorthKnots\":");
            serialization_detail::append_number(
                output, environment.current_north_knots);
            output.append(",\"flatWaterSpeedKnots\":");
            serialization_detail::append_number(
                output, environment.flat_water_speed_knots);
            output.append(",\"significantWaveHeightMetres\":");
            serialization_detail::append_number(
                output, environment.significant_wave_height_metres);
            output.append(",\"wavePeriodSeconds\":");
            serialization_detail::append_number(
                output, environment.wave_period_seconds);
            output.append(",\"relativeWaveAngleDegrees\":");
            serialization_detail::append_number(
                output, environment.relative_wave_angle_degrees);
            output.push_back('}');
        }
        output.push_back('}');
    }
    if (!route.points.empty()) {
        output.push_back('\n');
        output.append("  ");
    }
    output.append("]\n}\n");
    return output;
}

}  // namespace sailroute
