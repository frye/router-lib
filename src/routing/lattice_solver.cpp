#include "routing/lattice_solver.hpp"

#include "routing/geodesy.hpp"
#include "routing/environment_context.hpp"
#include "routing/lattice.hpp"
#include "routing/transition.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <span>
#include <tuple>
#include <utility>
#include <vector>

namespace sailroute::detail {
namespace {

using CellIndex = GeodesicLattice::CellIndex;
using LabelIndex = std::size_t;
constexpr LabelIndex no_label = std::numeric_limits<LabelIndex>::max();

struct StateKey {
    CellIndex cell{};
    std::int64_t bucket{};
    std::int8_t board{};

    friend bool operator<(const StateKey& left, const StateKey& right) noexcept {
        return std::tie(left.cell, left.bucket, left.board) <
            std::tie(right.cell, right.bucket, right.board);
    }
};

struct Label {
    StateKey state;
    RoutePoint point;
    LabelIndex parent{no_label};
    std::size_t ordinal{};
};

struct QueueEntry {
    double estimated_total_seconds{};
    TimePoint arrival;
    StateKey state;
    std::size_t ordinal{};
    LabelIndex label{};
};

struct LaterQueueEntry {
    bool operator()(const QueueEntry& left, const QueueEntry& right) const noexcept {
        return std::tie(
                   left.estimated_total_seconds,
                   left.arrival,
                   left.state.cell,
                   left.state.bucket,
                   left.state.board,
                   left.ordinal) >
            std::tie(
                   right.estimated_total_seconds,
                   right.arrival,
                   right.state.cell,
                   right.state.bucket,
                   right.state.board,
                   right.ordinal);
    }
};

struct SearchOutcome {
    RouteResult result;
    LatticeRouteDiagnostics diagnostics;
    EnvironmentDiagnostics environment;
};

std::vector<RoutePoint> reconstruct(
    const std::vector<Label>& labels,
    LabelIndex end) {
    std::vector<RoutePoint> route;
    while (end != no_label) {
        route.push_back(labels[end].point);
        end = labels[end].parent;
    }
    std::reverse(route.begin(), route.end());
    return route;
}

double heuristic_seconds(
    Coordinate position,
    Coordinate destination,
    double maximum_speed,
    LatticeSearchAlgorithm algorithm) noexcept {
    if (algorithm == LatticeSearchAlgorithm::dijkstra ||
        !(maximum_speed > 0.0)) {
        return 0.0;
    }
    return great_circle_distance_nautical_miles(position, destination) /
        maximum_speed * 3600.0;
}

std::int64_t bucket_for(
    TimePoint time,
    TimePoint departure,
    std::chrono::seconds width) noexcept {
    return std::chrono::duration_cast<std::chrono::seconds>(
               time - departure)
               .count() /
        width.count();
}

Result<SearchOutcome> search_lattice(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingEnvironment& environment,
    const RouteRequest& request,
    TimePoint departure,
    DepartureSource departure_source,
    const GeodesicLattice& lattice,
    std::span<const std::uint8_t> allowed,
    std::size_t refinement_index,
    const RoutingViewControlCallback& on_progress) {
    const ForecastMetadata& metadata = weather.metadata();
    const TimePoint horizon_end =
        departure + request.options.maximum_route_duration;
    const TimePoint route_end = std::min(horizon_end, metadata.last_valid_time);
    const bool forecast_limited = metadata.last_valid_time < horizon_end;
    const auto bucket_width = std::chrono::duration_cast<std::chrono::seconds>(
        request.options.lattice.time_bucket);
    const double maximum_speed = polar.maximum_boat_speed_knots();
    if (!(maximum_speed > 0.0)) {
        return Error{ErrorCode::invalid_polar, "polar contains no positive boat speed"};
    }

    auto start_wind = weather.interpolate(request.start, departure);
    if (!start_wind) {
        return start_wind.error();
    }
    if (great_circle_distance_nautical_miles(
            request.start, request.destination) <=
        request.options.arrival_radius_nautical_miles) {
        RouteResult result;
        result.departure_time = departure;
        result.arrival_time = departure;
        result.departure_source = departure_source;
        result.forecast_source = metadata.source;
        result.polar_source = polar.source();
        result.points.push_back(RoutePoint{
            request.start,
            departure,
            0.0,
            0.0,
            start_wind.value().speed_knots(),
            start_wind.value().direction_from_degrees(),
            0.0,
            std::nullopt});
        result.completion = RouteCompletion::destination_reached;
        const LatticeRouteDiagnostics diagnostics{
            0U, 0U, 0U, 0U, 0U, 0U, lattice.subdivision_level(), false};
        result.lattice_diagnostics = diagnostics;
        return SearchOutcome{std::move(result), diagnostics, {}};
    }
    const auto start_cell = lattice.nearest_cell(request.start);
    if (!start_cell.has_value()) {
        return Error{ErrorCode::invalid_argument, "unable to locate start lattice cell"};
    }

    std::vector<Label> labels;
    labels.reserve(lattice.vertex_count());
    labels.push_back(Label{
        StateKey{*start_cell, 0, 0},
        RoutePoint{
            request.start,
            departure,
            0.0,
            0.0,
            start_wind.value().speed_knots(),
            start_wind.value().direction_from_degrees(),
            0.0,
            std::nullopt},
        no_label,
        0U});
    std::map<StateKey, LabelIndex> best{{labels.front().state, 0U}};
    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        LaterQueueEntry>
        queue;
    queue.push(QueueEntry{
        heuristic_seconds(
            request.start,
            request.destination,
            maximum_speed,
            request.options.lattice.search_algorithm),
        departure,
        labels.front().state,
        0U,
        0U});

    LatticeRouteDiagnostics diagnostics;
    EnvironmentDiagnostics environment_diagnostics;
    diagnostics.queued_labels = 1U;
    diagnostics.subdivision_level = lattice.subdivision_level();
    RouteDiagnostics route_diagnostics;
    LabelIndex closest = 0U;
    double closest_distance =
        great_circle_distance_nautical_miles(request.start, request.destination);
    std::size_t next_ordinal = 1U;
    std::vector<Coordinate> search_points;

    const auto push_label = [&](Label label) {
        const auto found = best.find(label.state);
        if (found != best.end() &&
            labels[found->second].point.time <= label.point.time) {
            return;
        }
        const LabelIndex index = labels.size();
        labels.push_back(std::move(label));
        best[labels.back().state] = index;
        const double elapsed = std::chrono::duration<double>(
            labels.back().point.time - departure).count();
        queue.push(QueueEntry{
            elapsed +
                heuristic_seconds(
                    labels.back().point.position,
                    request.destination,
                    maximum_speed,
                    request.options.lattice.search_algorithm),
            labels.back().point.time,
            labels.back().state,
            labels.back().ordinal,
            index});
        ++diagnostics.queued_labels;
        ++diagnostics.relaxed_labels;
        ++route_diagnostics.retained_candidates;
    };

    while (!queue.empty()) {
        const QueueEntry entry = queue.top();
        queue.pop();
        const auto current_best = best.find(entry.state);
        if (current_best == best.end() || current_best->second != entry.label) {
            continue;
        }
        // Relaxation appends to labels and may reallocate it, so expansion must
        // not retain a reference into the vector across push_label calls.
        const Label current = labels[entry.label];
        ++diagnostics.settled_labels;
        ++route_diagnostics.expanded_nodes;

        const double current_distance = great_circle_distance_nautical_miles(
            current.point.position, request.destination);
        if (current_distance < closest_distance) {
            closest_distance = current_distance;
            closest = entry.label;
        }

        if (current_distance <=
            std::max(
                request.options.arrival_radius_nautical_miles,
                lattice.maximum_neighbor_edge_length_nautical_miles() * 1.75)) {
            auto arrival_result = evaluate_variable_transition(
                    weather,
                    polar,
                    request.options,
                    environment,
                    environment_diagnostics,
                    current.point,
                    current.state.board,
                    request.destination,
                    route_end);
            if (!arrival_result) {
                return arrival_result.error();
            }
            if (arrival_result.value().has_value()) {
                VariableTransition arrival =
                    std::move(*arrival_result.value());
                Label destination_label{
                    current.state,
                    std::move(arrival.point),
                    entry.label,
                    next_ordinal++};
                labels.push_back(std::move(destination_label));
                ++route_diagnostics.generated_candidates;
                ++route_diagnostics.retained_candidates;
                ++route_diagnostics.time_steps;
                RouteResult result;
                result.departure_time = departure;
                result.arrival_time = labels.back().point.time;
                result.departure_source = departure_source;
                result.forecast_source = metadata.source;
                result.polar_source = polar.source();
                result.points = reconstruct(labels, labels.size() - 1U);
                result.diagnostics = route_diagnostics;
                result.completion = RouteCompletion::destination_reached;
                result.lattice_diagnostics = diagnostics;
                return SearchOutcome{
                    std::move(result), diagnostics, environment_diagnostics};
            }
        }

        std::vector<CellIndex> targets;
        const Coordinate cell_coordinate = lattice.coordinate(current.state.cell);
        if (great_circle_distance_nautical_miles(
                current.point.position, cell_coordinate) > 1.0e-9) {
            targets.push_back(current.state.cell);
        }
        for (const CellIndex neighbor : lattice.neighbors(current.state.cell)) {
            targets.push_back(neighbor);
        }
        for (const CellIndex target : targets) {
            if (!allowed.empty() && !allowed[target]) {
                continue;
            }
            ++route_diagnostics.generated_candidates;
            auto transition_result = evaluate_variable_transition(
                weather,
                polar,
                request.options,
                environment,
                environment_diagnostics,
                current.point,
                current.state.board,
                lattice.coordinate(target),
                route_end);
            if (!transition_result ||
                !transition_result.value().has_value()) {
                continue;
            }
            VariableTransition transition =
                std::move(*transition_result.value());
            push_label(Label{
                StateKey{
                    target,
                    bucket_for(
                        transition.point.time, departure, bucket_width),
                    transition.board},
                std::move(transition.point),
                entry.label,
                next_ordinal++});
        }

        const std::int64_t next_bucket = current.state.bucket + 1;
        const TimePoint wait_until =
            departure + bucket_width * next_bucket;
        if (wait_until > current.point.time && wait_until <= route_end) {
            // Waiting still has to be legal: an exclusion zone can open around
            // a stationary vessel, so the degenerate segment is checked too.
            bool waiting_allowed = true;
            if (environment.active()) {
                const SegmentCheckResult waiting = check_segment_geometry(
                    environment,
                    current.point.position,
                    current.point.time,
                    current.point.position,
                    wait_until,
                    environment_diagnostics);
                if (waiting.outcome == EnvironmentOutcome::failed) {
                    return *waiting.error;
                }
                waiting_allowed =
                    waiting.outcome == EnvironmentOutcome::accepted;
            }
            RoutePoint waited = current.point;
            waited.time = wait_until;
            waited.boat_speed_knots = 0.0;
            waited.environment.reset();
            if (waiting_allowed) {
                push_label(Label{
                    StateKey{
                        current.state.cell, next_bucket, current.state.board},
                    std::move(waited),
                    entry.label,
                    next_ordinal++});
                ++diagnostics.wait_transitions;
            }
        }

        if (on_progress &&
            diagnostics.settled_labels %
                    request.options.lattice.progress_every_n_expansions ==
                0U) {
            search_points.assign(1U, current.point.position);
            std::vector<RoutePoint> provisional;
            if (has_payload(
                    request.options.progress.payload,
                    RoutingProgressPayload::provisional_route)) {
                provisional = reconstruct(labels, closest);
            }
            const RoutingProgressView progress{
                current.point.time,
                {},
                provisional,
                {},
                {},
                route_diagnostics,
                RoutingSolver::time_dependent_lattice,
                has_payload(
                    request.options.progress.payload,
                    RoutingProgressPayload::search_points)
                    ? std::span<const Coordinate>{search_points}
                    : std::span<const Coordinate>{},
                LatticeSearchProgress{
                    diagnostics.settled_labels,
                    diagnostics.queued_labels,
                    diagnostics.relaxed_labels,
                    refinement_index,
                    lattice.subdivision_level()}};
            if (on_progress(progress) == RoutingProgressDecision::cancel) {
                return Error{
                    ErrorCode::cancelled,
                    "lattice routing cancelled after " +
                        std::to_string(diagnostics.settled_labels) +
                        " settled labels"};
            }
        }
    }

    if (forecast_limited && closest != 0U) {
        RouteResult result;
        result.departure_time = departure;
        result.arrival_time = labels[closest].point.time;
        result.departure_source = departure_source;
        result.forecast_source = metadata.source;
        result.polar_source = polar.source();
        result.points = reconstruct(labels, closest);
        result.diagnostics = route_diagnostics;
        result.completion = RouteCompletion::forecast_exhausted;
        result.lattice_diagnostics = diagnostics;
        return SearchOutcome{
            std::move(result), diagnostics, environment_diagnostics};
    }
    return Error{
        ErrorCode::no_route,
        "time-dependent lattice search exhausted every reachable state"};
}

std::vector<std::uint8_t> corridor_cells(
    const GeodesicLattice& lattice,
    std::span<const RoutePoint> route,
    double width_nautical_miles) {
    std::vector<std::uint8_t> allowed(lattice.vertex_count(), 0U);
    for (CellIndex cell = 0U; cell < lattice.vertex_count(); ++cell) {
        const Coordinate coordinate = lattice.coordinate(cell);
        for (const RoutePoint& point : route) {
            if (great_circle_distance_nautical_miles(
                    coordinate, point.position) <= width_nautical_miles) {
                allowed[cell] = 1U;
                break;
            }
        }
    }
    return allowed;
}

}  // namespace

Result<RouteResult> optimize_lattice_route(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingEnvironment& environment,
    const RouteRequest& request,
    TimePoint departure,
    DepartureSource departure_source,
    const RoutingViewControlCallback& on_progress) {
    const std::size_t final_level =
        request.options.lattice.subdivision_level +
        request.options.lattice.refinement_levels;
    if (final_level > GeodesicLattice::maximum_subdivision_level) {
        return Error{
            ErrorCode::invalid_argument,
            "lattice subdivision and refinement levels exceed the supported maximum"};
    }
    auto coarse = GeodesicLattice::create(
        request.options.lattice.subdivision_level);
    if (!coarse) {
        return coarse.error();
    }
    auto incumbent = search_lattice(
        weather,
        polar,
        environment,
        request,
        departure,
        departure_source,
        coarse.value(),
        {},
        0U,
        on_progress);
    if (!incumbent) {
        return incumbent.error();
    }

    RouteResult result = std::move(incumbent.value().result);
    LatticeRouteDiagnostics cumulative = incumbent.value().diagnostics;
    // Environmental work is counted for every search run, including refinement
    // passes that were ultimately rejected, because it was genuinely performed.
    EnvironmentDiagnostics environment_totals = incumbent.value().environment;
    for (std::size_t refinement = 1U;
         refinement <= request.options.lattice.refinement_levels;
         ++refinement) {
        auto lattice = GeodesicLattice::create(
            request.options.lattice.subdivision_level + refinement);
        if (!lattice) {
            return lattice.error();
        }
        bool accepted = false;
        for (std::size_t retry = 0U;
             retry <= request.options.lattice.corridor_widening_retries;
             ++retry) {
            ++cumulative.refinement_runs;
            const double width =
                request.options.lattice.corridor_width_nautical_miles *
                static_cast<double>(std::size_t{1U} << retry);
            std::vector<std::uint8_t> allowed =
                corridor_cells(lattice.value(), result.points, width);
            auto refined = search_lattice(
                weather,
                polar,
                environment,
                request,
                departure,
                departure_source,
                lattice.value(),
                allowed,
                refinement,
                on_progress);
            if (!refined) {
                if (refined.error().code == ErrorCode::no_route) {
                    continue;
                }
                return refined.error();
            }
            merge(environment_totals, refined.value().environment);
            const RouteResult& candidate = refined.value().result;
            if (candidate.completion == RouteCompletion::destination_reached &&
                (result.completion != RouteCompletion::destination_reached ||
                 candidate.arrival_time <= result.arrival_time)) {
                cumulative.settled_labels +=
                    refined.value().diagnostics.settled_labels;
                cumulative.queued_labels +=
                    refined.value().diagnostics.queued_labels;
                cumulative.relaxed_labels +=
                    refined.value().diagnostics.relaxed_labels;
                cumulative.wait_transitions +=
                    refined.value().diagnostics.wait_transitions;
                ++cumulative.accepted_refinements;
                cumulative.subdivision_level =
                    lattice.value().subdivision_level();
                result = std::move(refined.value().result);
                accepted = true;
                break;
            }
        }
        if (!accepted) {
            cumulative.refinement_fallback = true;
            break;
        }
    }
    result.lattice_diagnostics = cumulative;
    if (environment.active()) {
        result.environment_diagnostics = environment_totals;
        result.environment = describe_environment(environment);
    }
    return result;
}

}  // namespace sailroute::detail
