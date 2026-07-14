#include "sailroute/serialization.hpp"

#include "sailroute/time.hpp"
#include "serialization/text_encoding.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>

namespace sailroute {
namespace {

bool append_number(std::string& output, double value) {
    if (!std::isfinite(value)) {
        return false;
    }
    char buffer[64]{};
    const auto converted = std::to_chars(
        std::begin(buffer),
        std::end(buffer),
        value,
        std::chars_format::general,
        std::numeric_limits<double>::max_digits10);
    if (converted.ec != std::errc{}) {
        return false;
    }
    output.append(buffer, converted.ptr);
    return true;
}

void append_coordinate(std::string& output, Coordinate coordinate) {
    output.append("{\"latitude\":");
    append_number(output, coordinate.latitude_degrees);
    output.append(",\"longitude\":");
    append_number(output, coordinate.longitude_degrees);
    output.push_back('}');
}

Error invalid_numeric_value(std::string_view field) {
    return Error{
        ErrorCode::output_error,
        "cannot serialize non-finite route value: " + std::string{field}};
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
    output.append("{\n  \"departure\":{\"time\":");
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
    output.append("},\n  \"points\":[");

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
        append_number(output, point.heading_degrees);
        output.append(",\"boatSpeedKnots\":");
        append_number(output, point.boat_speed_knots);
        output.append(",\"trueWindSpeedKnots\":");
        append_number(output, point.true_wind_speed_knots);
        output.append(",\"trueWindDirectionDegrees\":");
        append_number(output, point.true_wind_direction_degrees);
        output.append(",\"cumulativeDistanceNauticalMiles\":");
        append_number(output, point.cumulative_distance_nautical_miles);
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
