#include "sailroute/serialization.hpp"

#include "sailroute/time.hpp"
#include "serialization/numeric_encoding.hpp"
#include "serialization/text_encoding.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sailroute {
namespace {

Error invalid_isochrone(std::string message) {
    return Error{ErrorCode::output_error, std::move(message)};
}

Result<std::string> validate_isochrones(const RouteResult& route) {
    for (const Isochrone& isochrone : route.isochrones) {
        if (isochrone.points.empty()) {
            return invalid_isochrone("cannot serialize an empty isochrone");
        }
        for (const Coordinate point : isochrone.points) {
            if (!is_valid(point)) {
                return invalid_isochrone(
                    "cannot serialize an isochrone coordinate outside canonical bounds");
            }
        }
    }
    return std::string{};
}

double normalized_longitude_delta(double longitude, double origin) noexcept {
    return std::fmod(longitude - origin + 540.0, 360.0) - 180.0;
}

std::vector<Coordinate> ordered_closed_points(const Isochrone& isochrone) {
    double latitude_center = 0.0;
    double longitude_sine = 0.0;
    double longitude_cosine = 0.0;
    for (const Coordinate point : isochrone.points) {
        latitude_center += point.latitude_degrees;
        const double longitude_radians =
            point.longitude_degrees * std::numbers::pi / 180.0;
        longitude_sine += std::sin(longitude_radians);
        longitude_cosine += std::cos(longitude_radians);
    }
    latitude_center /= static_cast<double>(isochrone.points.size());

    double longitude_center = isochrone.points.front().longitude_degrees;
    if (std::abs(longitude_sine) > 1.0e-12 ||
        std::abs(longitude_cosine) > 1.0e-12) {
        longitude_center =
            std::atan2(longitude_sine, longitude_cosine) * 180.0 /
            std::numbers::pi;
    }
    const double longitude_scale =
        std::cos(latitude_center * std::numbers::pi / 180.0);
    const auto angle_from_center =
        [latitude_center, longitude_center, longitude_scale](Coordinate point) {
            const double east =
                normalized_longitude_delta(
                    point.longitude_degrees,
                    longitude_center) *
                longitude_scale;
            const double north = point.latitude_degrees - latitude_center;
            return std::atan2(north, east);
        };

    std::vector<Coordinate> ordered = isochrone.points;
    std::stable_sort(
        ordered.begin(),
        ordered.end(),
        [&angle_from_center](Coordinate left, Coordinate right) {
            return angle_from_center(left) < angle_from_center(right);
        });
    ordered.push_back(ordered.front());
    return ordered;
}

void append_geojson_coordinate(std::string& output, Coordinate coordinate) {
    output.push_back('[');
    serialization_detail::append_number(output, coordinate.longitude_degrees);
    output.push_back(',');
    serialization_detail::append_number(output, coordinate.latitude_degrees);
    output.push_back(']');
}

void append_gpx_element(
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

Result<std::string> isochrones_to_json(const RouteResult& route) {
    const Result<std::string> validation = validate_isochrones(route);
    if (!validation) {
        return validation.error();
    }

    std::string output;
    output.reserve(256 + route.isochrones.size() * 1024);
    output.append("{\n  \"type\":\"FeatureCollection\",\n  \"features\":[");
    for (std::size_t index = 0U; index < route.isochrones.size(); ++index) {
        const Isochrone& isochrone = route.isochrones[index];
        const std::vector<Coordinate> ordered =
            ordered_closed_points(isochrone);
        if (index != 0U) {
            output.push_back(',');
        }
        output.append("\n    {\"type\":\"Feature\",\"properties\":{\"time\":");
        serialization_detail::append_json_string(
            output,
            format_utc_time(isochrone.time));
        output.append(",\"retainedPointCount\":");
        output.append(std::to_string(isochrone.points.size()));
        output.append(
            "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[");
        for (std::size_t point_index = 0U; point_index < ordered.size();
             ++point_index) {
            if (point_index != 0U) {
                output.push_back(',');
            }
            append_geojson_coordinate(output, ordered[point_index]);
        }
        output.append("]}}");
    }
    if (!route.isochrones.empty()) {
        output.push_back('\n');
        output.append("  ");
    }
    output.append("]\n}\n");
    return output;
}

Result<std::string> isochrones_to_gpx(const RouteResult& route) {
    const Result<std::string> validation = validate_isochrones(route);
    if (!validation) {
        return validation.error();
    }

    std::string output;
    output.reserve(512 + route.isochrones.size() * 2048);
    output.append(
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<gpx version=\"1.1\" creator=\"sailroute\" "
        "xmlns=\"http://www.topografix.com/GPX/1/1\" "
        "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" "
        "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 "
        "http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
        "  <metadata>\n");
    append_gpx_element(output, "name", "Sailroute isochrones", "    ");
    output.append("    <desc>Forecast: ");
    serialization_detail::append_xml_text(output, route.forecast_source);
    output.append("; Polar: ");
    serialization_detail::append_xml_text(output, route.polar_source);
    output.append("</desc>\n");
    append_gpx_element(
        output,
        "time",
        format_utc_time(route.departure_time),
        "    ");
    output.append("  </metadata>\n");

    for (const Isochrone& isochrone : route.isochrones) {
        const std::string formatted_time = format_utc_time(isochrone.time);
        const std::vector<Coordinate> ordered =
            ordered_closed_points(isochrone);
        output.append("  <trk>\n");
        append_gpx_element(
            output,
            "name",
            "Sailroute isochrone " + formatted_time,
            "    ");
        output.append("    <desc>Retained frontier with ");
        output.append(std::to_string(isochrone.points.size()));
        output.append(" points</desc>\n    <trkseg>\n");
        for (const Coordinate point : ordered) {
            output.append("      <trkpt lat=\"");
            serialization_detail::append_coordinate_number(
                output,
                point.latitude_degrees);
            output.append("\" lon=\"");
            serialization_detail::append_coordinate_number(
                output,
                point.longitude_degrees);
            output.append("\">\n");
            append_gpx_element(output, "time", formatted_time, "        ");
            output.append("      </trkpt>\n");
        }
        output.append("    </trkseg>\n  </trk>\n");
    }
    output.append("</gpx>\n");
    return output;
}

}  // namespace sailroute
