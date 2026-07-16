#include "sailroute/contours.hpp"
#include "sailroute/serialization.hpp"

#include "sailroute/time.hpp"
#include "serialization/numeric_encoding.hpp"
#include "serialization/text_encoding.hpp"

#include <span>
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

void append_geojson_coordinate(std::string& output, Coordinate coordinate) {
    output.push_back('[');
    serialization_detail::append_number(output, coordinate.longitude_degrees);
    output.push_back(',');
    serialization_detail::append_number(output, coordinate.latitude_degrees);
    output.push_back(']');
}

void append_geojson_segment(
    std::string& output,
    std::span<const Coordinate> points,
    bool closed) {
    output.push_back('[');
    for (std::size_t index = 0U; index < points.size(); ++index) {
        if (index != 0U) {
            output.push_back(',');
        }
        append_geojson_coordinate(output, points[index]);
    }
    if (closed && !points.empty()) {
        if (!points.empty()) {
            output.push_back(',');
        }
        append_geojson_coordinate(output, points.front());
    } else if (points.size() == 1U) {
        output.push_back(',');
        append_geojson_coordinate(output, points.front());
    }
    output.push_back(']');
}

std::span<const Coordinate> segment_points(
    const DisplayContours& contours,
    const DisplayContourSegment& segment) {
    return std::span<const Coordinate>{contours.points}.subspan(
        segment.point_offset,
        segment.point_count);
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
        auto contours_result = build_display_contours(isochrone.points);
        if (!contours_result) {
            return invalid_isochrone(contours_result.error().message);
        }
        const DisplayContours& contours = contours_result.value();
        if (index != 0U) {
            output.push_back(',');
        }
        output.append("\n    {\"type\":\"Feature\",\"properties\":{\"time\":");
        serialization_detail::append_json_string(
            output,
            format_utc_time(isochrone.time));
        output.append(",\"retainedPointCount\":");
        output.append(std::to_string(isochrone.points.size()));
        output.append("},\"geometry\":{\"type\":");
        const bool multiple = contours.segments.size() != 1U;
        output.append(multiple ? "\"MultiLineString\"" : "\"LineString\"");
        output.append(",\"coordinates\":[");
        for (std::size_t segment_index = 0U;
             segment_index < contours.segments.size();
             ++segment_index) {
            if (segment_index != 0U) {
                output.push_back(',');
            }
            const DisplayContourSegment& segment =
                contours.segments[segment_index];
            if (multiple) {
                append_geojson_segment(
                    output,
                    segment_points(contours, segment),
                    segment.closed);
            } else {
                const auto points = segment_points(contours, segment);
                for (std::size_t point_index = 0U;
                     point_index < points.size();
                     ++point_index) {
                    if (point_index != 0U) {
                        output.push_back(',');
                    }
                    append_geojson_coordinate(output, points[point_index]);
                }
                if (segment.closed && !points.empty()) {
                    output.push_back(',');
                    append_geojson_coordinate(output, points.front());
                } else if (points.size() == 1U) {
                    output.push_back(',');
                    append_geojson_coordinate(output, points.front());
                }
            }
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
        auto contours_result = build_display_contours(isochrone.points);
        if (!contours_result) {
            return invalid_isochrone(contours_result.error().message);
        }
        const DisplayContours& contours = contours_result.value();
        output.append("  <trk>\n");
        append_gpx_element(
            output,
            "name",
            "Sailroute isochrone " + formatted_time,
            "    ");
        output.append("    <desc>Retained frontier with ");
        output.append(std::to_string(isochrone.points.size()));
        output.append(" points</desc>\n");
        for (const DisplayContourSegment& segment : contours.segments) {
            output.append("    <trkseg>\n");
            const auto points = segment_points(contours, segment);
            const std::size_t serialized_count =
                points.size() +
                ((segment.closed && !points.empty()) ||
                         points.size() == 1U
                     ? 1U
                     : 0U);
            for (std::size_t point_index = 0U;
                 point_index < serialized_count;
                 ++point_index) {
                const Coordinate point =
                    points[point_index % points.size()];
                output.append("      <trkpt lat=\"");
                serialization_detail::append_coordinate_number(
                    output,
                    point.latitude_degrees);
                output.append("\" lon=\"");
                serialization_detail::append_coordinate_number(
                    output,
                    point.longitude_degrees);
                output.append("\">\n");
                append_gpx_element(
                    output,
                    "time",
                    formatted_time,
                    "        ");
                output.append("      </trkpt>\n");
            }
            output.append("    </trkseg>\n");
        }
        output.append("  </trk>\n");
    }
    output.append("</gpx>\n");
    return output;
}

}  // namespace sailroute
