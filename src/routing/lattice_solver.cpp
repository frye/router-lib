#include "routing/lattice_solver.hpp"

#include "routing/geodesy.hpp"
#include "routing/lattice.hpp"
#include "routing/state.hpp"
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

struct Label {
    SolverStateKey state;
    RoutePoint point;
    LabelIndex parent{no_label};
    std::size_t ordinal{};
};

SolverLabelIdentity label_identity(const Label& label) noexcept {
    return SolverLabelIdentity{
        label.state,
        label.point.time,
        label.parent,
        label.ordinal,
        label.ordinal};
}

struct QueueEntry {
    double estimated_total_seconds{};
    TimePoint arrival;
    SolverStateKey state;
    std::size_t ordinal{};
    LabelIndex label{};
};

struct LaterQueueEntry {
    bool operator()(const QueueEntry& left, const QueueEntry& right) const noexcept {
        return std::tie(
                   left.estimated_total_seconds,
                   left.arrival,
                   left.state.spatial,
                   left.state.time_bucket,
                   left.state.arrival,
                   left.state.configuration,
                   left.ordinal) >
            std::tie(
                   right.estimated_total_seconds,
                   right.arrival,
                   right.state.spatial,
                   right.state.time_bucket,
                   right.state.arrival,
                   right.state.configuration,
                   right.ordinal);
    }
};

struct SearchOutcome {
    RouteResult result;
    LatticeRouteDiagnostics diagnostics;
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
            0.0});
        result.completion = RouteCompletion::destination_reached;
        const LatticeRouteDiagnostics diagnostics{
            0U, 0U, 0U, 0U, 0U, 0U, lattice.subdivision_level(), false};
        result.lattice_diagnostics = diagnostics;
        return SearchOutcome{std::move(result), diagnostics};
    }
    const auto start_cell = lattice.nearest_cell(request.start);
    if (!start_cell.has_value()) {
        return Error{ErrorCode::invalid_argument, "unable to locate start lattice cell"};
    }

    std::vector<Label> labels;
    labels.reserve(lattice.vertex_count());
    labels.push_back(Label{
        SolverStateKey{*start_cell, 0, departure, {}},
        RoutePoint{
            request.start,
            departure,
            0.0,
            0.0,
            start_wind.value().speed_knots(),
            start_wind.value().direction_from_degrees(),
            0.0},
        no_label,
        0U});
    std::map<SolverStateKey, LabelIndex> best{{labels.front().state, 0U}};
    std::map<ContinuationStateKey, TimePoint> earliest_arrival{
        {ContinuationStateKey{
             labels.front().state.spatial,
             labels.front().state.configuration},
         departure}};
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
            dominates(
                label_identity(labels[found->second]),
                label_identity(label))) {
            return;
        }
        const LabelIndex index = labels.size();
        const ContinuationStateKey continuation{
            label.state.spatial,
            label.state.configuration};
        const auto earliest = earliest_arrival.find(continuation);
        if (earliest == earliest_arrival.end()) {
            earliest_arrival.emplace(continuation, label.point.time);
        } else if (label.point.time < earliest->second) {
            earliest->second = label.point.time;
            ++diagnostics.re_relaxed_labels;
        }
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
            ++diagnostics.stale_queue_entries;
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
                    current.point,
                    current.state.configuration,
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
                return SearchOutcome{std::move(result), diagnostics};
            }
        }

        std::vector<CellIndex> targets;
        const Coordinate cell_coordinate =
            lattice.coordinate(current.state.spatial);
        if (great_circle_distance_nautical_miles(
                current.point.position, cell_coordinate) > 1.0e-9) {
            targets.push_back(
                static_cast<CellIndex>(current.state.spatial));
        }
        for (const CellIndex neighbor :
             lattice.neighbors(
                 static_cast<CellIndex>(current.state.spatial))) {
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
                current.point,
                current.state.configuration,
                lattice.coordinate(target),
                route_end);
            if (!transition_result) {
                return transition_result.error();
            }
            if (!transition_result.value().has_value()) {
                continue;
            }
            VariableTransition transition =
                std::move(*transition_result.value());
            push_label(Label{
                SolverStateKey{
                    target,
                    bucket_for(
                        transition.point.time, departure, bucket_width),
                    transition.point.time,
                    transition.configuration},
                std::move(transition.point),
                entry.label,
                next_ordinal++});
        }

        const std::int64_t next_bucket =
            current.state.time_bucket + 1;
        const TimePoint wait_until =
            departure + bucket_width * next_bucket;
        if (wait_until > current.point.time && wait_until <= route_end) {
            RoutePoint waited = current.point;
            waited.time = wait_until;
            waited.boat_speed_knots = 0.0;
            push_label(Label{
                SolverStateKey{
                    current.state.spatial,
                    next_bucket,
                    wait_until,
                    current.state.configuration},
                std::move(waited),
                entry.label,
                next_ordinal++});
            ++diagnostics.wait_transitions;
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
        return SearchOutcome{std::move(result), diagnostics};
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
                cumulative.re_relaxed_labels +=
                    refined.value().diagnostics.re_relaxed_labels;
                cumulative.stale_queue_entries +=
                    refined.value().diagnostics.stale_queue_entries;
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
    return result;
}

}  // namespace sailroute::detail
