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

Error invalid_numeric_value(std::string_view field) {
    return Error{
        ErrorCode::output_error,
        "cannot serialize non-finite route value: " + std::string{field}};
}

Result<std::string> validate_route_numbers(const RouteResult& route) {
    for (const RoutePoint& point : route.points) {
        if (!std::isfinite(point.position.latitude_degrees) ||
            point.position.latitude_degrees < -90.0 ||
            point.position.latitude_degrees > 90.0) {
            return invalid_numeric_value("latitude");
        }
        if (!std::isfinite(point.position.longitude_degrees) ||
            point.position.longitude_degrees < -180.0 ||
            point.position.longitude_degrees > 180.0) {
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
        if (point.environment.has_value()) {
            const RoutePointEnvironment& environment = *point.environment;
            if (!std::isfinite(environment.speed_over_ground_knots)) {
                return invalid_numeric_value("speed_over_ground_knots");
            }
            if (!std::isfinite(environment.course_over_ground_degrees)) {
                return invalid_numeric_value("course_over_ground_degrees");
            }
            if (environment.current_applied) {
                if (!std::isfinite(environment.current_east_knots)) {
                    return invalid_numeric_value("current_east_knots");
                }
                if (!std::isfinite(environment.current_north_knots)) {
                    return invalid_numeric_value("current_north_knots");
                }
            }
            if (!std::isfinite(environment.flat_water_speed_knots)) {
                return invalid_numeric_value("flat_water_speed_knots");
            }
            if (environment.wave_applied) {
                if (!std::isfinite(
                        environment.significant_wave_height_metres)) {
                    return invalid_numeric_value(
                        "significant_wave_height_metres");
                }
                if (!std::isfinite(environment.wave_period_seconds)) {
                    return invalid_numeric_value("wave_period_seconds");
                }
                if (!std::isfinite(environment.relative_wave_angle_degrees)) {
                    return invalid_numeric_value("relative_wave_angle_degrees");
                }
            }
        }
    }
    if (route.environment.has_value() &&
        route.environment->landmask.has_value()) {
        const RouteEnvironmentMetadata& environment = *route.environment;
        if (!std::isfinite(environment.land_resolution_nautical_miles)) {
            return invalid_numeric_value("land_resolution_nautical_miles");
        }
        if (!std::isfinite(
                environment.land_interpolation_error_nautical_miles)) {
            return invalid_numeric_value(
                "land_interpolation_error_nautical_miles");
        }
        if (!std::isfinite(environment.land_clearance_nautical_miles)) {
            return invalid_numeric_value("land_clearance_nautical_miles");
        }
    }
    return std::string{};
}


void append_element(
    std::string& output,
    std::string_view element,
    std::string_view value,
    std::string_view indentation) {
    output.append(indentation);
    output.push_back('<');
    output.append(element);
    output.push_back('>');
    serialization_detail::append_xml_text(output, value);
    output.append("</");
    output.append(element);
    output.append(">\n");
}

void append_number_element(
    std::string& output,
    std::string_view element,
    double value,
    std::string_view indentation) {
    output.append(indentation);
    output.push_back('<');
    output.append(element);
    output.push_back('>');
    serialization_detail::append_number(output, value);
    output.append("</");
    output.append(element);
    output.append(">\n");
}

void append_provider_elements(
    std::string& output,
    std::string_view prefix,
    const std::optional<ProviderMetadata>& provider,
    std::string_view indentation) {
    if (!provider.has_value()) {
        return;
    }
    append_element(
        output, std::string{prefix} + "Name", provider->name, indentation);
    append_element(
        output, std::string{prefix} + "Source", provider->source, indentation);
    append_element(
        output,
        std::string{prefix} + "Revision",
        provider->revision,
        indentation);
}

}  // namespace

Result<std::string> route_to_gpx(const RouteResult& route) {
    const Result<std::string> validation = validate_route_numbers(route);
    if (!validation) {
        return validation.error();
    }

    std::string output;
    output.reserve(768 + route.points.size() * 640);
    output.append(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<gpx version=\"1.1\" creator=\"sailroute\" "
        "xmlns=\"http://www.topografix.com/GPX/1/1\" "
        "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
        "xmlns:sailroute=\"https://sailroute.dev/xmlns/route/1\" "
        "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 "
        "http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
        "  <metadata>\n");
    append_element(output, "name", "Sailroute optimized route", "    ");
    output.append("    <desc>Forecast: ");
    serialization_detail::append_xml_text(output, route.forecast_source);
    output.append("; Polar: ");
    serialization_detail::append_xml_text(output, route.polar_source);
    output.append("</desc>\n");
    append_element(output, "time", format_utc_time(route.departure_time), "    ");
    output.append("  </metadata>\n  <trk>\n");
    append_element(output, "name", "Sailroute optimized route", "    ");
    output.append("    <desc>Departure source: ");
    serialization_detail::append_xml_text(output, to_string(route.departure_source));
    output.append("</desc>\n    <extensions>\n");
    append_element(
        output,
        "sailroute:completion",
        to_string(route.completion),
        "      ");
    if (route.environment.has_value()) {
        const RouteEnvironmentMetadata& environment = *route.environment;
        append_element(
            output,
            "sailroute:environmentSampling",
            to_string(environment.sampling),
            "      ");
        append_provider_elements(
            output, "sailroute:currentProvider", environment.current_provider, "      ");
        append_provider_elements(
            output, "sailroute:waveProvider", environment.wave_provider, "      ");
        append_provider_elements(
            output, "sailroute:seaStateModel", environment.sea_state_model, "      ");
        append_provider_elements(
            output, "sailroute:landmask", environment.landmask, "      ");
        append_provider_elements(
            output, "sailroute:exclusions", environment.exclusions, "      ");
        append_element(
            output,
            "sailroute:currentPolicy",
            to_string(environment.current_policy),
            "      ");
        append_element(
            output,
            "sailroute:wavePolicy",
            to_string(environment.wave_policy),
            "      ");
        append_element(
            output,
            "sailroute:landPolicy",
            to_string(environment.land_policy),
            "      ");
        append_element(
            output,
            "sailroute:exclusionPolicy",
            to_string(environment.exclusion_policy),
            "      ");
        if (environment.landmask.has_value()) {
            append_number_element(
                output,
                "sailroute:landResolutionNauticalMiles",
                environment.land_resolution_nautical_miles,
                "      ");
            append_number_element(
                output,
                "sailroute:landInterpolationErrorNauticalMiles",
                environment.land_interpolation_error_nautical_miles,
                "      ");
            append_number_element(
                output,
                "sailroute:landClearanceNauticalMiles",
                environment.land_clearance_nautical_miles,
                "      ");
        }
        if (environment.exclusions.has_value()) {
            append_element(
                output,
                "sailroute:exclusionBoundaryPolicy",
                to_string(environment.exclusion_boundary_policy),
                "      ");
            append_element(
                output,
                "sailroute:exclusionZoneCount",
                std::to_string(environment.exclusion_zone_count),
                "      ");
            append_element(
                output,
                "sailroute:exclusionRevision",
                std::to_string(environment.exclusion_revision),
                "      ");
        }
    }
    if (route.environment_diagnostics.has_value()) {
        const EnvironmentDiagnostics& diagnostics =
            *route.environment_diagnostics;
        append_element(
            output,
            "sailroute:currentSamples",
            std::to_string(diagnostics.current_samples),
            "      ");
        append_element(
            output,
            "sailroute:currentRejections",
            std::to_string(diagnostics.current_rejections),
            "      ");
        append_element(
            output,
            "sailroute:waveSamples",
            std::to_string(diagnostics.wave_samples),
            "      ");
        append_element(
            output,
            "sailroute:waveRejections",
            std::to_string(diagnostics.wave_rejections),
            "      ");
        append_element(
            output,
            "sailroute:seaStateEvaluations",
            std::to_string(diagnostics.sea_state_evaluations),
            "      ");
        append_element(
            output,
            "sailroute:landChecks",
            std::to_string(diagnostics.land_checks),
            "      ");
        append_element(
            output,
            "sailroute:landDistanceQueries",
            std::to_string(diagnostics.land_distance_queries),
            "      ");
        append_element(
            output,
            "sailroute:landRejections",
            std::to_string(diagnostics.land_rejections),
            "      ");
        append_element(
            output,
            "sailroute:exclusionChecks",
            std::to_string(diagnostics.exclusion_checks),
            "      ");
        append_element(
            output,
            "sailroute:exclusionGeometryTests",
            std::to_string(diagnostics.exclusion_geometry_tests),
            "      ");
        append_element(
            output,
            "sailroute:exclusionRejections",
            std::to_string(diagnostics.exclusion_rejections),
            "      ");
    }
    output.append("    </extensions>\n    <trkseg>\n");

    for (const RoutePoint& point : route.points) {
        output.append("      <trkpt lat=\"");
        serialization_detail::append_coordinate_number(
            output,
            point.position.latitude_degrees);
        output.append("\" lon=\"");
        serialization_detail::append_coordinate_number(
            output,
            point.position.longitude_degrees);
        output.append("\">\n");
        append_element(output, "time", format_utc_time(point.time), "        ");
        output.append("        <extensions>\n");
        output.append("          <sailroute:headingDegrees>");
        serialization_detail::append_number(output, point.heading_degrees);
        output.append("</sailroute:headingDegrees>\n");
        output.append("          <sailroute:boatSpeedKnots>");
        serialization_detail::append_number(output, point.boat_speed_knots);
        output.append("</sailroute:boatSpeedKnots>\n");
        output.append("          <sailroute:trueWindSpeedKnots>");
        serialization_detail::append_number(output, point.true_wind_speed_knots);
        output.append("</sailroute:trueWindSpeedKnots>\n");
        output.append("          <sailroute:trueWindDirectionDegrees>");
        serialization_detail::append_number(
            output,
            point.true_wind_direction_degrees);
        output.append("</sailroute:trueWindDirectionDegrees>\n");
        output.append("          <sailroute:cumulativeDistanceNauticalMiles>");
        serialization_detail::append_number(
            output,
            point.cumulative_distance_nautical_miles);
        output.append("</sailroute:cumulativeDistanceNauticalMiles>\n");
        if (point.environment.has_value()) {
            const RoutePointEnvironment& environment = *point.environment;
            append_number_element(
                output,
                "sailroute:speedOverGroundKnots",
                environment.speed_over_ground_knots,
                "          ");
            append_number_element(
                output,
                "sailroute:courseOverGroundDegrees",
                environment.course_over_ground_degrees,
                "          ");
            if (environment.current_applied) {
                append_number_element(
                    output,
                    "sailroute:currentEastKnots",
                    environment.current_east_knots,
                    "          ");
                append_number_element(
                    output,
                    "sailroute:currentNorthKnots",
                    environment.current_north_knots,
                    "          ");
            }
            append_number_element(
                output,
                "sailroute:flatWaterSpeedKnots",
                environment.flat_water_speed_knots,
                "          ");
            if (environment.wave_applied) {
                append_number_element(
                    output,
                    "sailroute:significantWaveHeightMetres",
                    environment.significant_wave_height_metres,
                    "          ");
                append_number_element(
                    output,
                    "sailroute:wavePeriodSeconds",
                    environment.wave_period_seconds,
                    "          ");
                append_number_element(
                    output,
                    "sailroute:relativeWaveAngleDegrees",
                    environment.relative_wave_angle_degrees,
                    "          ");
            }
        }
        output.append("        </extensions>\n      </trkpt>\n");
    }

    output.append("    </trkseg>\n  </trk>\n</gpx>\n");
    return output;
}

}  // namespace sailroute
