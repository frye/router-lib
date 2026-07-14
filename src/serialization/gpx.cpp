#include "sailroute/serialization.hpp"

#include "sailroute/time.hpp"
#include "serialization/numeric_encoding.hpp"
#include "serialization/text_encoding.hpp"

#include <cmath>
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
    output.append("</desc>\n    <trkseg>\n");

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
        output.append("        </extensions>\n      </trkpt>\n");
    }

    output.append("    </trkseg>\n  </trk>\n</gpx>\n");
    return output;
}

}  // namespace sailroute
