#include "sailroute/serialization.hpp"

#include "sailroute/time.hpp"
#include "serialization/numeric_encoding.hpp"
#include "serialization/text_encoding.hpp"

#include <cmath>
#include <string>
#include <string_view>

namespace sailroute {
namespace {

std::string_view fallback_reason_name(
    LatticeRefinementFallbackReason reason) noexcept {
    switch (reason) {
        case LatticeRefinementFallbackReason::none:
            return "none";
        case LatticeRefinementFallbackReason::disconnected:
            return "disconnected";
        case LatticeRefinementFallbackReason::regressed:
            return "regressed";
        case LatticeRefinementFallbackReason::retry_exhausted:
            return "retry_exhausted";
    }
    return "none";
}

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

Result<std::string> validate_route_numbers(const RouteResult& route) {
    if (route.lattice_diagnostics.has_value() &&
        !std::isfinite(
            route.lattice_diagnostics
                ->accepted_corridor_width_nautical_miles)) {
        return invalid_numeric_value(
            "accepted_corridor_width_nautical_miles");
    }
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
        output.append(",\"reRelaxedLabels\":");
        output.append(std::to_string(lattice.re_relaxed_labels));
        output.append(",\"staleQueueEntries\":");
        output.append(std::to_string(lattice.stale_queue_entries));
        output.append(",\"activeCells\":");
        output.append(std::to_string(lattice.active_cells));
        output.append(",\"activeFaces\":");
        output.append(std::to_string(lattice.active_faces));
        output.append(",\"acceptedCorridorWidthNauticalMiles\":");
        serialization_detail::append_number(
            output, lattice.accepted_corridor_width_nautical_miles);
        output.append(",\"disconnectedRefinements\":");
        output.append(std::to_string(lattice.disconnected_refinements));
        output.append(",\"regressedRefinements\":");
        output.append(std::to_string(lattice.regressed_refinements));
        output.append(",\"fallbackReason\":\"");
        output.append(fallback_reason_name(lattice.fallback_reason));
        output.push_back('"');
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
