#include "routing/lattice_solver.hpp"

#include "routing/geodesy.hpp"
#include "routing/environment_context.hpp"
#include "routing/lattice.hpp"
#include "routing/mixed_lattice.hpp"
#include "routing/state.hpp"
#include "routing/transition.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numbers>
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
    bool goal{};
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
                   left.state.position,
                   left.state.time_bucket,
                   left.state.configuration,
                   left.ordinal) >
            std::tie(
                   right.estimated_total_seconds,
                   right.arrival,
                   right.state.spatial,
                   right.state.position,
                   right.state.time_bucket,
                   right.state.configuration,
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

SphericalPositionKey position_key(
    Coordinate position,
    double bucket_width_nautical_miles) noexcept {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    const double latitude = position.latitude_degrees * degrees_to_radians;
    const double longitude = position.longitude_degrees * degrees_to_radians;
    const double scale =
        earth_radius_nautical_miles / bucket_width_nautical_miles;
    const double cosine_latitude = std::cos(latitude);
    return SphericalPositionKey{
        std::llround(cosine_latitude * std::cos(longitude) * scale),
        std::llround(cosine_latitude * std::sin(longitude) * scale),
        std::llround(std::sin(latitude) * scale)};
}

template <typename Lattice>
Result<SearchOutcome> search_lattice(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingEnvironment& environment,
    const RouteRequest& request,
    TimePoint departure,
    DepartureSource departure_source,
    const Lattice& lattice,
    std::span<const std::uint8_t> allowed,
    std::size_t refinement_index,
    bool direct_anchor_edge,
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
    const double bucket_hours =
        std::chrono::duration<double, std::ratio<3600>>(bucket_width).count();
    // VMG successors land off-vertex. Half a maximum-speed bucket keeps nearby
    // endpoints distinct while still merging equivalent continuous paths.
    const double position_bucket_width =
        std::max(1.0, maximum_speed * bucket_hours * 0.5);

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
    const auto destination_face = lattice.containing_face(request.destination);
    if (!destination_face.has_value()) {
        return Error{
            ErrorCode::invalid_argument,
            "unable to locate destination lattice face"};
    }
    std::vector<CellIndex> destination_connectors{
        destination_face->begin(), destination_face->end()};
    for (const CellIndex cell : *destination_face) {
        const auto neighbors = lattice.neighbors(cell);
        destination_connectors.insert(
            destination_connectors.end(), neighbors.begin(), neighbors.end());
    }
    std::sort(destination_connectors.begin(), destination_connectors.end());
    destination_connectors.erase(
        std::unique(destination_connectors.begin(), destination_connectors.end()),
        destination_connectors.end());

    std::vector<Label> labels;
    labels.reserve(lattice.vertex_count());
    labels.push_back(Label{
        SolverStateKey{
            *start_cell,
            0,
            {},
            position_key(request.start, position_bucket_width)},
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
    std::map<SolverStateKey, LabelIndex> best{{labels.front().state, 0U}};
    std::map<ContinuationStateKey, TimePoint> earliest_arrival{
        {ContinuationStateKey{
             labels.front().state.spatial,
             labels.front().state.configuration,
             labels.front().state.position},
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
        if (found != best.end()) {
            const SolverLabelIdentity incumbent =
                label_identity(labels[found->second]);
            const SolverLabelIdentity candidate = label_identity(label);
            if (!strictly_dominates(candidate, incumbent)) {
                return false;
            }
        }
        const LabelIndex index = labels.size();
        const ContinuationStateKey continuation{
            label.state.spatial,
            label.state.configuration,
            label.state.position};
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
        return true;
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

        if (current.goal) {
            RouteResult result;
            result.departure_time = departure;
            result.arrival_time = current.point.time;
            result.departure_source = departure_source;
            result.forecast_source = metadata.source;
            result.polar_source = polar.source();
            result.points = reconstruct(labels, entry.label);
            result.diagnostics = route_diagnostics;
            result.completion = RouteCompletion::destination_reached;
            result.lattice_diagnostics = diagnostics;
            return SearchOutcome{
                std::move(result), diagnostics, environment_diagnostics};
        }

        const double current_distance = great_circle_distance_nautical_miles(
            current.point.position, request.destination);
        if (current_distance < closest_distance) {
            closest_distance = current_distance;
            closest = entry.label;
        }

        const Coordinate cell_coordinate =
            lattice.coordinate(current.state.spatial);
        const bool at_lattice_cell =
            great_circle_distance_nautical_miles(
                current.point.position, cell_coordinate) <= 1.0e-9;
        const auto current_face = at_lattice_cell
            ? std::optional<typename Lattice::Face>{}
            : lattice.containing_face(current.point.position);
        const bool same_destination_face =
            current_face.has_value() &&
            *current_face == *destination_face;
        const bool destination_connector =
            same_destination_face ||
            (at_lattice_cell &&
             std::binary_search(
                 destination_connectors.begin(),
                 destination_connectors.end(),
                 current.state.spatial));
        const bool same_face_anchor_edge =
            entry.label == 0U && direct_anchor_edge;
        if (destination_connector || same_face_anchor_edge) {
            ++route_diagnostics.generated_candidates;
            auto arrival_result = evaluate_variable_transition(
                weather,
                polar,
                request.options,
                environment,
                environment_diagnostics,
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
                ++route_diagnostics.time_steps;
                push_label(Label{
                    SolverStateKey{
                        lattice.vertex_count(),
                        bucket_for(
                            arrival.point.time, departure, bucket_width),
                        arrival.configuration,
                        position_key(
                            arrival.point.position, position_bucket_width)},
                    std::move(arrival.point),
                    entry.label,
                    next_ordinal++,
                    true});
            }
        }

        std::vector<CellIndex> targets;
        if (!at_lattice_cell) {
            targets.push_back(
                static_cast<CellIndex>(current.state.spatial));
        }
        for (const CellIndex neighbor :
             lattice.neighbors(
                 static_cast<CellIndex>(current.state.spatial))) {
            targets.push_back(neighbor);
        }
        bool has_spatial_successor = false;
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
            has_spatial_successor = true;
            push_label(Label{
                SolverStateKey{
                    target,
                    bucket_for(
                        transition.point.time, departure, bucket_width),
                    transition.configuration,
                    position_key(
                        transition.point.position, position_bucket_width)},
                std::move(transition.point),
                entry.label,
                next_ordinal++});
        }

        const std::int64_t next_bucket =
            current.state.time_bucket + 1;
        const TimePoint wait_until =
            departure + bucket_width * next_bucket;
        if (wait_until > current.point.time && wait_until <= route_end) {
            auto wind_result =
                weather.interpolate(current.point.position, current.point.time);
            if (!wind_result) {
                if (wind_result.error().code !=
                    ErrorCode::coordinate_outside_forecast) {
                    return wind_result.error();
                }
            } else if (
                !has_spatial_successor ||
                current_distance <
                    lattice.maximum_neighbor_edge_length_nautical_miles(
                        current.state.spatial)) {
                auto evaluated_wind =
                    evaluate_wind(wind_result.value(), request.options);
                if (!evaluated_wind) {
                    return evaluated_wind.error();
                }
                if (evaluated_wind.value().has_value()) {
                    const double wind_speed =
                        evaluated_wind.value()->speed_knots;
                    const double wind_from =
                        evaluated_wind.value()->direction_from_degrees;
                    const PolarSlice slice = polar.slice_at(
                        wind_speed,
                        request.options.polar_angle_interpolation);
                    if (!(request.options.above_polar_range ==
                              AbovePolarRangePolicy::no_speed &&
                          slice.above_tabulated_wind_speed())) {
                        const VelocityMadeGoodAngles optima =
                            slice.velocity_made_good_angles();
                        if (optima.valid && optima.upwind_degrees > 0.0) {
                            const std::array headings{
                                normalize_degrees(
                                    wind_from + optima.upwind_degrees),
                                normalize_degrees(
                                    wind_from - optima.upwind_degrees)};
                            for (const double heading : headings) {
                                ++route_diagnostics.generated_candidates;
                                auto transition_result =
                                    evaluate_heading_transition(
                                        weather,
                                        polar,
                                        request.options,
                                        environment,
                                        environment_diagnostics,
                                        current.point,
                                        current.state.configuration,
                                        heading,
                                        wait_until);
                                if (!transition_result) {
                                    return transition_result.error();
                                }
                                if (!transition_result.value().has_value()) {
                                    continue;
                                }
                                VariableTransition transition =
                                    std::move(*transition_result.value());
                                const auto target = lattice.nearest_cell(
                                    transition.point.position);
                                if (!target.has_value() ||
                                    (!allowed.empty() && !allowed[*target])) {
                                    continue;
                                }
                                push_label(Label{
                                    SolverStateKey{
                                        *target,
                                        next_bucket,
                                        transition.configuration,
                                        position_key(
                                            transition.point.position,
                                            position_bucket_width)},
                                    std::move(transition.point),
                                    entry.label,
                                    next_ordinal++});
                            }
                        }
                    }
                }
            }

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
                    SolverStateKey{
                        current.state.spatial,
                        next_bucket,
                        current.state.configuration,
                        current.state.position},
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

    if (closest != 0U) {
        RouteResult result;
        result.departure_time = departure;
        result.arrival_time = labels[closest].point.time;
        result.departure_source = departure_source;
        result.forecast_source = metadata.source;
        result.polar_source = polar.source();
        result.points = reconstruct(labels, closest);
        result.diagnostics = route_diagnostics;
        result.completion = forecast_limited
            ? RouteCompletion::forecast_exhausted
            : RouteCompletion::duration_exhausted;
        result.lattice_diagnostics = diagnostics;
        return SearchOutcome{
            std::move(result), diagnostics, environment_diagnostics};
    }
    return Error{
        ErrorCode::no_route,
        "time-dependent lattice search exhausted every reachable state"};
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
    const auto coarse_start_face =
        coarse.value().containing_face(request.start);
    const auto coarse_destination_face =
        coarse.value().containing_face(request.destination);
    if (!coarse_start_face.has_value() ||
        !coarse_destination_face.has_value()) {
        return Error{
            ErrorCode::invalid_argument,
            "unable to locate route anchors in the coarse lattice"};
    }
    const bool direct_anchor_edge =
        *coarse_start_face == *coarse_destination_face;
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
        direct_anchor_edge,
        on_progress);
    if (!incumbent) {
        return incumbent.error();
    }

    RouteResult result = std::move(incumbent.value().result);
    LatticeRouteDiagnostics cumulative = incumbent.value().diagnostics;
    // Environmental work is counted for every search run, including refinement
    // passes that were ultimately rejected, because it was genuinely performed.
    EnvironmentDiagnostics environment_totals = incumbent.value().environment;
    cumulative.active_cells = coarse.value().vertex_count();
    cumulative.active_faces = coarse.value().faces().size();
    for (std::size_t refinement = 1U;
         refinement <= request.options.lattice.refinement_levels;
         ++refinement) {
        bool accepted = false;
        LatticeRefinementFallbackReason last_failure =
            LatticeRefinementFallbackReason::none;
        for (std::size_t retry = 0U;
             retry <= request.options.lattice.corridor_widening_retries;
             ++retry) {
            ++cumulative.refinement_runs;
            const double width =
                request.options.lattice.corridor_width_nautical_miles *
                static_cast<double>(std::size_t{1U} << retry);
            auto lattice = MixedResolutionLattice::create(
                request.options.lattice.subdivision_level,
                refinement,
                result.points,
                width);
            if (!lattice) {
                return lattice.error();
            }
            auto refined = search_lattice(
                weather,
                polar,
                environment,
                request,
                departure,
                departure_source,
                lattice.value(),
                {},
                refinement,
                direct_anchor_edge,
                on_progress);
            if (!refined) {
                if (refined.error().code == ErrorCode::no_route) {
                    ++cumulative.disconnected_refinements;
                    last_failure =
                        LatticeRefinementFallbackReason::disconnected;
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
                cumulative.re_relaxed_labels +=
                    refined.value().diagnostics.re_relaxed_labels;
                cumulative.stale_queue_entries +=
                    refined.value().diagnostics.stale_queue_entries;
                ++cumulative.accepted_refinements;
                cumulative.subdivision_level =
                    lattice.value().subdivision_level();
                cumulative.active_cells = lattice.value().vertex_count();
                cumulative.active_faces = lattice.value().leaf_face_count();
                cumulative.accepted_corridor_width_nautical_miles = width;
                cumulative.fallback_reason =
                    LatticeRefinementFallbackReason::none;
                result = std::move(refined.value().result);
                accepted = true;
                break;
            }
            ++cumulative.regressed_refinements;
            last_failure = LatticeRefinementFallbackReason::regressed;
        }
        if (!accepted) {
            cumulative.refinement_fallback = true;
            cumulative.fallback_reason =
                request.options.lattice.corridor_widening_retries == 0U
                ? last_failure
                : LatticeRefinementFallbackReason::retry_exhausted;
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
