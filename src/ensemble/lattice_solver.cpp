#include "ensemble/lattice_solver.hpp"

#include "ensemble/beam_solver.hpp"
#include "ensemble/objective.hpp"
#include "ensemble/policy.hpp"
#include "ensemble/search_state.hpp"
#include "ensemble/transition.hpp"
#include "routing/environment_context.hpp"
#include "routing/geodesy.hpp"
#include "routing/lattice.hpp"
#include "routing/transition.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <queue>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace sailroute {

std::string_view to_string(EnsembleSolver solver) noexcept {
    switch (solver) {
        case EnsembleSolver::time_dependent_lattice:
            return "time_dependent_lattice";
        case EnsembleSolver::experimental_isochrone_beam:
            return "experimental_isochrone_beam";
    }
    return "unknown";
}

std::string_view to_string(EnsembleSolverPhase phase) noexcept {
    switch (phase) {
        case EnsembleSolverPhase::initializing: return "initializing";
        case EnsembleSolverPhase::searching:    return "searching";
        case EnsembleSolverPhase::finalizing:   return "finalizing";
    }
    return "unknown";
}

EnsembleRouter::EnsembleRouter(EnsembleDataset dataset, VesselPolar polar)
    : dataset_(std::move(dataset)), polar_(std::move(polar)) {}

const EnsembleDataset& EnsembleRouter::dataset() const noexcept {
    return dataset_;
}

Result<EnsembleRouteResult> EnsembleRouter::optimize(
    const EnsembleRouteRequest& request) const {
    if (request.enable_experimental_beam !=
        (request.solver == EnsembleSolver::experimental_isochrone_beam)) {
        return Error{
            ErrorCode::invalid_argument,
            request.enable_experimental_beam
                ? "enable_experimental_beam=true requires selecting experimental_isochrone_beam"
                : "experimental_isochrone_beam requires enable_experimental_beam=true"};
    }
    switch (request.solver) {
        case EnsembleSolver::time_dependent_lattice:
            return detail::optimize_ensemble_lattice_route(
                dataset_, polar_, request);
        case EnsembleSolver::experimental_isochrone_beam:
            return detail::optimize_ensemble_beam_route(
                dataset_, polar_, request);
    }
    return Error{ErrorCode::invalid_argument, "unsupported ensemble solver"};
}

Result<EnsembleRouteResult> EnsembleRouter::optimize_controlled(
    const EnsembleRouteRequest& request,
    const EnsembleControlCallback& on_progress) const {
    detail::EnsembleStepCallback step;
    if (on_progress) {
        step = [&on_progress](const EnsembleProgressView& view) {
            EnsembleProgress progress;
            progress.phase = view.phase;
            progress.active_member_count = view.active_member_count;
            progress.active_label_count = view.active_label_count;
            progress.retained_label_count = view.retained_label_count;
            progress.generated_states = view.generated_states;
            progress.settled_states = view.settled_states;
            progress.current_objective_bound = view.current_objective_bound;
            progress.policy_alternative_count = view.policy_alternative_count;
            return on_progress(progress);
        };
    }
    if (request.enable_experimental_beam !=
        (request.solver == EnsembleSolver::experimental_isochrone_beam)) {
        return Error{
            ErrorCode::invalid_argument,
            request.enable_experimental_beam
                ? "enable_experimental_beam=true requires selecting experimental_isochrone_beam"
                : "experimental_isochrone_beam requires enable_experimental_beam=true"};
    }
    switch (request.solver) {
        case EnsembleSolver::time_dependent_lattice:
            return detail::optimize_ensemble_lattice_route(
                dataset_, polar_, request, {}, step);
        case EnsembleSolver::experimental_isochrone_beam:
            return detail::optimize_ensemble_beam_route(
                dataset_, polar_, request, {}, step);
    }
    return Error{ErrorCode::invalid_argument, "unsupported ensemble solver"};
}

Result<EnsembleRouteResult> EnsembleRouter::optimize(
    const EnsembleRouteRequest& request,
    const EnsembleProgressCallback& on_progress) const {
    if (!on_progress) {
        return optimize(request);
    }
    return optimize_controlled(
        request,
        [&on_progress](const EnsembleProgress& progress) {
            on_progress(progress);
            return RoutingProgressDecision::continue_routing;
        });
}

Result<EnsembleRouteResult> EnsembleRouter::optimize_view_controlled(
    const EnsembleRouteRequest& request,
    const EnsembleViewControlCallback& on_progress) const {
    detail::EnsembleStepCallback step;
    if (on_progress) {
        step = [&on_progress](const EnsembleProgressView& view) {
            return on_progress(view);
        };
    }
    if (request.enable_experimental_beam !=
        (request.solver == EnsembleSolver::experimental_isochrone_beam)) {
        return Error{
            ErrorCode::invalid_argument,
            request.enable_experimental_beam
                ? "enable_experimental_beam=true requires selecting experimental_isochrone_beam"
                : "experimental_isochrone_beam requires enable_experimental_beam=true"};
    }
    switch (request.solver) {
        case EnsembleSolver::time_dependent_lattice:
            return detail::optimize_ensemble_lattice_route(
                dataset_, polar_, request, {}, step);
        case EnsembleSolver::experimental_isochrone_beam:
            return detail::optimize_ensemble_beam_route(
                dataset_, polar_, request, {}, step);
    }
    return Error{ErrorCode::invalid_argument, "unsupported ensemble solver"};
}

Result<EnsembleRouteResult> EnsembleRouter::optimize_view(
    const EnsembleRouteRequest& request,
    const EnsembleProgressViewCallback& on_progress) const {
    if (!on_progress) {
        return optimize(request);
    }
    return optimize_view_controlled(
        request,
        [&on_progress](const EnsembleProgressView& view) {
            on_progress(view);
            return RoutingProgressDecision::continue_routing;
        });
}

namespace detail {
namespace {

using CellIndex = GeodesicLattice::CellIndex;
using LabelIndex = std::size_t;

struct MemberStateKey {
    CellIndex spatial{};
    SphericalPositionKey position;
    std::int64_t time_bucket{};
    OperationalConfiguration configuration;
    EnsembleMemberSearchStatus status{EnsembleMemberSearchStatus::active};
    std::optional<EnsembleMemberOutcomeClass> outcome_class;
    std::optional<ErrorCode> error_code;

    friend bool operator<(const MemberStateKey& left, const MemberStateKey& right) {
        return std::tie(
                   left.spatial,
                   left.position,
                   left.time_bucket,
                   left.configuration,
                   left.status,
                   left.outcome_class,
                   left.error_code) <
            std::tie(
                   right.spatial,
                   right.position,
                   right.time_bucket,
                   right.configuration,
                   right.status,
                   right.outcome_class,
                   right.error_code);
    }
};

struct CommonStateKey {
    std::vector<MemberStateKey> members;

    friend bool operator<(const CommonStateKey& left, const CommonStateKey& right) {
        return left.members < right.members;
    }
};

struct QueueEntry {
    double optimistic_value{};
    TimePoint latest_member_time;
    std::string canonical_identity;
    LabelIndex label{};
};

struct LaterQueueEntry {
    bool operator()(const QueueEntry& left, const QueueEntry& right) const noexcept {
        return std::tie(
                   left.optimistic_value,
                   left.latest_member_time,
                   left.canonical_identity,
                   left.label) >
            std::tie(
                   right.optimistic_value,
                   right.latest_member_time,
                   right.canonical_identity,
                   right.label);
    }
};

[[nodiscard]] Error invalid_request(std::string message) {
    return Error{ErrorCode::invalid_argument, std::move(message)};
}

[[nodiscard]] Error label_limit_error(
    std::string_view limit,
    std::size_t configured,
    const EnsembleLatticeDiagnostics& diagnostics) {
    return Error{
        ErrorCode::no_route,
        "ensemble lattice " + std::string{limit} + " hard limit " +
            std::to_string(configured) + " exceeded after " +
            std::to_string(diagnostics.generated_labels) +
            " generated, " + std::to_string(diagnostics.retained_labels) +
            " retained, and " + std::to_string(diagnostics.settled_labels) +
            " settled labels"};
}

[[nodiscard]] std::int64_t bucket_for(
    TimePoint time,
    TimePoint departure,
    std::chrono::seconds width) noexcept {
    return std::chrono::duration_cast<std::chrono::seconds>(time - departure).count() /
        width.count();
}

[[nodiscard]] SphericalPositionKey position_key(
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

[[nodiscard]] bool resolved(const EnsembleSearchLabel& label) noexcept {
    return std::none_of(
        label.members.begin(),
        label.members.end(),
        [](const EnsembleMemberSearchState& member) {
            return member.status == EnsembleMemberSearchStatus::active;
        });
}

[[nodiscard]] TimePoint latest_time(const EnsembleSearchLabel& label) noexcept {
    TimePoint latest = label.members.front().point.time;
    for (const EnsembleMemberSearchState& member : label.members) {
        latest = std::max(latest, member.point.time);
    }
    return latest;
}

[[nodiscard]] Result<CommonStateKey> state_key(
    const EnsembleSearchLabel& label,
    const GeodesicLattice& lattice,
    TimePoint departure,
    std::chrono::seconds bucket_width,
    double position_bucket_width) {
    CommonStateKey key;
    key.members.reserve(label.members.size());
    for (const EnsembleMemberSearchState& member : label.members) {
        const auto cell = lattice.nearest_cell(member.point.position);
        if (!cell) {
            return invalid_request(
                "unable to locate an ensemble member in the geodesic lattice");
        }
        key.members.push_back(MemberStateKey{
            *cell,
            position_key(member.point.position, position_bucket_width),
            bucket_for(member.point.time, departure, bucket_width),
            member.configuration,
            member.status,
            member.outcome_class,
            member.error
                ? std::optional<ErrorCode>{member.error->code}
                : std::nullopt});
    }
    return key;
}

[[nodiscard]] bool uses_safe_speed_bound(
    const EnsembleDataset& dataset,
    const EnsembleRouteRequest& request) noexcept {
    if (request.lattice.search_algorithm != LatticeSearchAlgorithm::a_star ||
        request.objective.kind !=
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival) {
        return false;
    }
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        const RoutingEnvironment* environment = dataset.member_environment(index);
        if (environment != nullptr && environment->currents.configured()) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] double optimistic_value(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& label,
    const EnsembleRouteRequest& request,
    TimePoint departure,
    double maximum_speed,
    bool safe_speed_bound) {
    if (objective_direction(request.objective.kind) ==
        EnsembleObjectiveDirection::maximize) {
        return 0.0;
    }
    if (!safe_speed_bound) {
        return 0.0;
    }
    double maximum_weight = 0.0;
    for (const EnsembleMemberMetadata& member : dataset.members()) {
        maximum_weight = std::max(maximum_weight, member.original_weight);
    }
    long double accumulated_weight = 0.0L;
    double weighted_mean = 0.0;
    for (std::size_t index = 0U; index < label.members.size(); ++index) {
        const EnsembleMemberSearchState& member = label.members[index];
        double bound = std::chrono::duration<double>(
            member.point.time - departure).count();
        if (member.status == EnsembleMemberSearchStatus::failed) {
            bound = std::numeric_limits<double>::infinity();
        } else if (
            member.status == EnsembleMemberSearchStatus::active &&
            safe_speed_bound) {
            const double remaining = std::max(
                0.0,
                great_circle_distance_nautical_miles(
                    member.point.position, request.destination) -
                    request.options.arrival_radius_nautical_miles);
            bound += remaining / maximum_speed * 3600.0;
        }
        const long double weight =
            static_cast<long double>(dataset.members()[index].original_weight) /
            static_cast<long double>(maximum_weight);
        if (weight > 0.0L) {
            if (std::isinf(bound)) {
                return std::numeric_limits<double>::infinity();
            }
            const long double next_weight = accumulated_weight + weight;
            weighted_mean +=
                static_cast<double>(weight / next_weight) *
                (bound - weighted_mean);
            accumulated_weight = next_weight;
        }
    }
    return weighted_mean;
}

[[nodiscard]] std::vector<EnsembleCommonAction> actions_for(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const EnsembleRouteRequest& request,
    const GeodesicLattice& lattice,
    const EnsembleSearchLabel& label,
    std::chrono::seconds bucket_width) {
    std::vector<EnsembleCommonAction> actions;
    if (auto destination = make_common_target_action(request.destination)) {
        actions.push_back(destination.value());
    }

    std::vector<CellIndex> target_cells;
    for (std::size_t index = 0U; index < label.members.size(); ++index) {
        const EnsembleMemberSearchState& member = label.members[index];
        if (member.status != EnsembleMemberSearchStatus::active) {
            continue;
        }
        const auto cell = lattice.nearest_cell(member.point.position);
        if (!cell) {
            continue;
        }
        target_cells.push_back(*cell);
        const auto neighbors = lattice.neighbors(*cell);
        target_cells.insert(
            target_cells.end(), neighbors.begin(), neighbors.end());

        const WeatherDataset* weather = dataset.member_weather(index);
        if (weather == nullptr) {
            continue;
        }
        const auto wind = weather->interpolate(
            member.point.position, member.point.time);
        if (!wind) {
            continue;
        }
        const auto evaluated = evaluate_wind(wind.value(), request.options);
        if (!evaluated || !evaluated.value()) {
            continue;
        }
        const PolarSlice slice = polar.slice_at(
            evaluated.value()->speed_knots,
            request.options.polar_angle_interpolation);
        const VelocityMadeGoodAngles optima =
            slice.velocity_made_good_angles();
        if (!optima.valid || !(optima.upwind_degrees > 0.0)) {
            continue;
        }
        const double wind_from = evaluated.value()->direction_from_degrees;
        for (const double heading : {
                 normalize_degrees(wind_from - optima.upwind_degrees),
                 normalize_degrees(wind_from + optima.upwind_degrees)}) {
            if (auto action = make_common_heading_action(heading, bucket_width)) {
                actions.push_back(action.value());
            }
        }
    }
    std::sort(target_cells.begin(), target_cells.end());
    target_cells.erase(
        std::unique(target_cells.begin(), target_cells.end()),
        target_cells.end());
    for (const CellIndex cell : target_cells) {
        if (auto target = make_common_target_action(lattice.coordinate(cell))) {
            actions.push_back(target.value());
        }
    }
    if (auto wait = make_common_wait_action(bucket_width)) {
        actions.push_back(wait.value());
    }
    std::sort(actions.begin(), actions.end());
    actions.erase(std::unique(actions.begin(), actions.end()), actions.end());
    return actions;
}

[[nodiscard]] EnsembleRouteAction public_action(
    const EnsembleCommonAction& action) noexcept {
    EnsembleRouteAction result;
    result.target = action.target;
    result.heading_degrees = action.heading_degrees;
    result.duration = action.duration;
    switch (action.kind) {
        case EnsembleCommonActionKind::target:
            result.kind = EnsembleRouteActionKind::target;
            break;
        case EnsembleCommonActionKind::heading_for_duration:
            result.kind = EnsembleRouteActionKind::heading_for_duration;
            break;
        case EnsembleCommonActionKind::wait_for_duration:
            result.kind = EnsembleRouteActionKind::wait_for_duration;
            break;
    }
    return result;
}

[[nodiscard]] Result<EnsembleRouteResult> make_result(
    const EnsembleDataset& dataset,
    std::span<const EnsembleSearchLabel> labels,
    LabelIndex terminal,
    std::span<const LabelIndex> terminal_labels,
    const EnsembleRouteRequest& request,
    TimePoint departure,
    DepartureSource departure_source,
    const EnsembleLatticeDiagnostics& diagnostics,
    std::span<const EnvironmentDiagnostics> member_environment) {
    const EnsembleSearchLabel& selected = labels[terminal];
    if (!selected.aggregate_objective) {
        return invalid_request("selected ensemble label has no objective evaluation");
    }
    auto outcomes = ensemble_label_outcomes(dataset, selected, departure);
    if (!outcomes) {
        return outcomes.error();
    }
    auto actions = reconstruct_common_actions(labels, terminal);
    if (!actions) {
        return actions.error();
    }

    EnsembleRouteResult result;
    result.departure_time = departure;
    result.departure_source = departure_source;
    result.objective = *selected.aggregate_objective;
    result.canonical_action_sequence_identity =
        selected.canonical_action_sequence_identity;
    result.lattice_diagnostics = diagnostics;
    result.common_actions.reserve(actions.value().size());
    for (const EnsembleCommonAction& action : actions.value()) {
        result.common_actions.push_back(public_action(action));
    }
    result.members.reserve(dataset.member_count());
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        auto route = reconstruct_member_route(labels, terminal, index);
        if (!route) {
            return route.error();
        }
        result.members.push_back(EnsembleMemberRouteResult{
            outcomes.value()[index],
            std::move(route.value()),
            member_environment[index]});
    }
    auto policy = build_ensemble_policy(
        dataset,
        request.objective,
        request.policy,
        labels,
        terminal_labels,
        terminal,
        departure);
    if (!policy) {
        return policy.error();
    }
    result.policy = std::move(policy.value().graph);
    result.decision_points = std::move(policy.value().decision_points);
    result.re_evaluation = std::move(policy.value().re_evaluation);
    return result;
}

}  // namespace

Result<EnsembleRouteResult> optimize_ensemble_lattice_route(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const EnsembleRouteRequest& request,
    const EnsembleCancellationCallback& cancelled,
    const EnsembleStepCallback& on_step) {
    if (request.solver != EnsembleSolver::time_dependent_lattice ||
        request.enable_experimental_beam) {
        return invalid_request(
            "ensemble lattice requires time_dependent_lattice without the experimental beam opt in");
    }
    if (dataset.member_count() == 0U ||
        !dataset.alignment().shared_search_compatible()) {
        return invalid_request(
            "ensemble lattice routing requires a non-empty shared-search-compatible dataset");
    }
    if (!is_valid(request.start) || !is_valid(request.destination)) {
        return invalid_request(
            "ensemble route anchors must be finite canonical coordinates");
    }
    if (!std::isfinite(request.options.arrival_radius_nautical_miles) ||
        request.options.arrival_radius_nautical_miles <= 0.0 ||
        request.options.maximum_route_duration <= std::chrono::hours::zero()) {
        return invalid_request(
            "ensemble arrival radius and maximum route duration must be positive");
    }
    if (request.lattice.time_bucket <= std::chrono::minutes::zero() ||
        request.lattice.max_labels_per_state == 0U ||
        request.lattice.max_total_labels == 0U) {
        return invalid_request(
            "ensemble lattice time bucket and label limits must be positive");
    }
    if (!std::isfinite(
            request.policy.commitment_spatial_tolerance_nautical_miles) ||
        request.policy.commitment_spatial_tolerance_nautical_miles < 0.0 ||
        request.policy.commitment_time_tolerance <
            std::chrono::minutes::zero()) {
        return invalid_request(
            "ensemble policy commitment tolerances must be non-negative");
    }
    if (request.lattice.subdivision_level >
        GeodesicLattice::maximum_subdivision_level) {
        return invalid_request(
            "ensemble lattice subdivision level exceeds the supported maximum");
    }
    const double maximum_speed = polar.maximum_boat_speed_knots();
    if (!(maximum_speed > 0.0)) {
        return Error{
            ErrorCode::invalid_polar,
            "polar contains no positive boat speed"};
    }

    std::vector<EnsembleMemberOutcome> validation_outcomes;
    validation_outcomes.reserve(dataset.member_count());
    for (const EnsembleMemberMetadata& member : dataset.members()) {
        validation_outcomes.push_back(EnsembleMemberOutcome{
            member.identifier,
            EnsembleMemberOutcomeClass::infeasible_no_route,
            std::nullopt,
            std::nullopt});
    }
    auto objective_validation = evaluate_ensemble_objective(
        dataset, request.objective, validation_outcomes);
    if (!objective_validation) {
        return objective_validation.error();
    }

    auto lattice = GeodesicLattice::create(
        request.lattice.subdivision_level);
    if (!lattice) {
        return lattice.error();
    }
    const auto bucket_width = std::chrono::duration_cast<std::chrono::seconds>(
        request.lattice.time_bucket);
    const double bucket_hours =
        std::chrono::duration<double, std::ratio<3600>>(bucket_width).count();
    const double position_bucket_width =
        std::max(1.0, maximum_speed * bucket_hours * 0.5);

    const TimePoint departure = request.departure_time.value_or(
        dataset.members().front().weather.first_valid_time);
    const DepartureSource departure_source = request.departure_time
        ? DepartureSource::explicit_time
        : DepartureSource::forecast_start_fallback;
    const TimePoint duration_end =
        departure + request.options.maximum_route_duration;
    TimePoint route_end = duration_end;
    for (const EnsembleMemberMetadata& member : dataset.members()) {
        route_end = std::min(route_end, member.weather.last_valid_time);
    }
    if (route_end < departure) {
        return Error{
            ErrorCode::departure_outside_forecast,
            "ensemble departure is outside the common forecast horizon"};
    }
    const EnsembleRouteEndCause end_cause = route_end < duration_end
        ? EnsembleRouteEndCause::forecast_horizon
        : EnsembleRouteEndCause::duration_limit;

    std::vector<RoutePoint> initial_points;
    std::vector<OperationalConfiguration> configurations(dataset.member_count());
    initial_points.reserve(dataset.member_count());
    std::vector<std::optional<Error>> initial_errors(dataset.member_count());
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        const WeatherDataset* weather = dataset.member_weather(index);
        if (weather == nullptr) {
            return invalid_request("ensemble dataset is missing member weather");
        }
        const auto wind = weather->interpolate(request.start, departure);
        RoutePoint point;
        point.position = request.start;
        point.time = departure;
        if (wind) {
            point.true_wind_speed_knots = wind.value().speed_knots();
            point.true_wind_direction_degrees =
                wind.value().direction_from_degrees();
        } else {
            initial_errors[index] = wind.error();
        }
        initial_points.push_back(std::move(point));
    }
    auto initial = make_initial_ensemble_label(
        dataset, initial_points, configurations);
    if (!initial) {
        return initial.error();
    }
    const bool starts_at_destination =
        great_circle_distance_nautical_miles(
            request.start, request.destination) <=
        request.options.arrival_radius_nautical_miles;
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        EnsembleMemberSearchState& member = initial.value().members[index];
        if (initial_errors[index]) {
            member.status = EnsembleMemberSearchStatus::failed;
            member.outcome_class =
                classify_member_transition_error(*initial_errors[index]);
            member.error = initial_errors[index];
        } else if (starts_at_destination) {
            member.status = EnsembleMemberSearchStatus::completed;
            member.outcome_class = EnsembleMemberOutcomeClass::reached;
        }
    }
    canonicalize_ensemble_label(initial.value());

    EnsembleLatticeDiagnostics diagnostics;
    diagnostics.max_labels_per_state =
        request.lattice.max_labels_per_state;
    diagnostics.max_total_labels = request.lattice.max_total_labels;
    diagnostics.subdivision_level = request.lattice.subdivision_level;
    const bool safe_speed_bound =
        uses_safe_speed_bound(dataset, request);
    const bool minimizing =
        objective_direction(request.objective.kind) ==
        EnsembleObjectiveDirection::minimize;
    diagnostics.zero_heuristic = !safe_speed_bound;

    std::vector<EnsembleSearchLabel> labels;
    labels.reserve(std::min<std::size_t>(
        request.lattice.max_total_labels, lattice.value().vertex_count()));
    labels.push_back(std::move(initial.value()));
    std::vector<bool> retained{true};
    std::map<CommonStateKey, std::vector<LabelIndex>> pareto;
    auto root_key = state_key(
        labels.front(),
        lattice.value(),
        departure,
        bucket_width,
        position_bucket_width);
    if (!root_key) {
        return root_key.error();
    }
    pareto[root_key.value()].push_back(0U);
    diagnostics.retained_labels = 1U;
    diagnostics.peak_labels_per_state = 1U;

    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        LaterQueueEntry>
        queue;
    std::vector<bool> queued{false};
    std::size_t queued_retained_labels = 0U;
    std::vector<std::size_t> queued_active_member_labels(
        dataset.member_count(), 0U);
    const auto mark_queued = [&](LabelIndex label) {
        if (queued[label]) return;
        queued[label] = true;
        ++queued_retained_labels;
        for (std::size_t member = 0U;
             member < labels[label].members.size();
             ++member) {
            if (labels[label].members[member].status ==
                EnsembleMemberSearchStatus::active) {
                ++queued_active_member_labels[member];
            }
        }
    };
    const auto mark_dequeued = [&](LabelIndex label) {
        if (label >= queued.size() || !queued[label]) return;
        queued[label] = false;
        --queued_retained_labels;
        for (std::size_t member = 0U;
             member < labels[label].members.size();
             ++member) {
            if (labels[label].members[member].status ==
                EnsembleMemberSearchStatus::active) {
                --queued_active_member_labels[member];
            }
        }
    };
    std::optional<LabelIndex> best_terminal;
    std::vector<LabelIndex> terminal_labels;
    std::vector<EnvironmentDiagnostics> member_environment(
        dataset.member_count());

    const auto consider_terminal = [&](LabelIndex candidate) {
        if (std::any_of(
                terminal_labels.begin(),
                terminal_labels.end(),
                [&](LabelIndex retained_terminal) {
                    return policy_terminal_dominates(
                        dataset,
                        labels[retained_terminal],
                        labels[candidate]);
                })) {
            return;
        }
        std::erase_if(
            terminal_labels,
            [&](LabelIndex retained_terminal) {
                return policy_terminal_dominates(
                    dataset,
                    labels[candidate],
                    labels[retained_terminal]);
            });
        const auto duplicate = std::find_if(
            terminal_labels.begin(),
            terminal_labels.end(),
            [&](LabelIndex retained_terminal) {
                return labels[retained_terminal].
                    canonical_action_sequence_identity ==
                    labels[candidate].canonical_action_sequence_identity;
            });
        if (duplicate == terminal_labels.end()) {
            terminal_labels.push_back(candidate);
        }
        std::sort(
            terminal_labels.begin(),
            terminal_labels.end(),
            [&](LabelIndex left, LabelIndex right) {
                const int comparison =
                    compare_ensemble_objective_evaluations(
                        request.objective.kind,
                        *labels[left].aggregate_objective,
                        *labels[right].aggregate_objective);
                return comparison != 0
                    ? comparison < 0
                    : ensemble_label_less(labels[left], labels[right]);
            });
        const std::size_t limit =
            request.policy.max_alternatives ==
                std::numeric_limits<std::size_t>::max()
            ? request.policy.max_alternatives
            : request.policy.max_alternatives + 1U;
        if (terminal_labels.size() > limit) {
            terminal_labels.resize(limit);
        }
        best_terminal = terminal_labels.front();
    };

    if (resolved(labels.front())) {
        auto evaluation = evaluate_ensemble_label_objective(
            dataset,
            request.objective,
            labels.front(),
            departure,
            EnsembleObjectiveTieBreakInputs{0U, 0U, {}});
        if (!evaluation) {
            return evaluation.error();
        }
        consider_terminal(0U);
    } else {
        queue.push(QueueEntry{
            optimistic_value(
                dataset,
                labels.front(),
                request,
                departure,
                maximum_speed,
                safe_speed_bound),
            latest_time(labels.front()),
            labels.front().canonical_label_identity,
            0U});
        mark_queued(0U);
        diagnostics.queued_labels = 1U;
    }

    constexpr std::size_t kLatticeProgressCadence = 1000U;

    const auto build_progress_view = [&](EnsembleSolverPhase phase) {
        EnsembleProgressView view;
        view.phase = phase;
        if (phase != EnsembleSolverPhase::finalizing) {
            view.active_label_count = queued_retained_labels;
            view.active_member_count = static_cast<std::size_t>(
                std::count_if(
                    queued_active_member_labels.begin(),
                    queued_active_member_labels.end(),
                    [](std::size_t count) { return count != 0U; }));
        }
        view.retained_label_count = diagnostics.retained_labels;
        view.generated_states = diagnostics.generated_labels;
        view.settled_states = diagnostics.settled_labels;
        if (best_terminal &&
            labels[*best_terminal].aggregate_objective) {
            view.current_objective_bound =
                labels[*best_terminal].aggregate_objective->value;
        } else {
            view.current_objective_bound = EnsembleObjectiveValue{
                EnsembleObjectiveValueClass::positive_infinity, 0.0};
        }
        view.policy_alternative_count = terminal_labels.size();
        return view;
    };

    if (on_step) {
        if (on_step(build_progress_view(EnsembleSolverPhase::initializing)) ==
            RoutingProgressDecision::cancel) {
            return Error{
                ErrorCode::cancelled,
                "ensemble lattice routing cancelled before search"};
        }
    }

    while (!queue.empty()) {
        if (cancelled && cancelled()) {
            return Error{
                ErrorCode::cancelled,
                "ensemble lattice routing cancelled after " +
                    std::to_string(diagnostics.settled_labels) +
                    " settled labels"};
        }
        const std::size_t policy_limit =
            request.policy.max_alternatives ==
                std::numeric_limits<std::size_t>::max()
            ? request.policy.max_alternatives
            : request.policy.max_alternatives + 1U;
        if (best_terminal && minimizing &&
            terminal_labels.size() >= policy_limit &&
            labels[terminal_labels.back()].aggregate_objective->
                value.is_finite() &&
            queue.top().optimistic_value >
                labels[terminal_labels.back()].aggregate_objective->
                    value.finite_value) {
            break;
        }

        const QueueEntry entry = queue.top();
        queue.pop();
        mark_dequeued(entry.label);
        if (entry.label >= retained.size() || !retained[entry.label]) {
            ++diagnostics.stale_queue_entries;
            continue;
        }
        ++diagnostics.settled_labels;
        if (on_step &&
            diagnostics.settled_labels % kLatticeProgressCadence == 0U) {
            if (on_step(build_progress_view(EnsembleSolverPhase::searching)) ==
                RoutingProgressDecision::cancel) {
                return Error{
                    ErrorCode::cancelled,
                    "ensemble lattice routing cancelled after " +
                        std::to_string(diagnostics.settled_labels) +
                        " settled labels"};
            }
        }
        const EnsembleSearchLabel current = labels[entry.label];
        const std::vector<EnsembleCommonAction> actions = actions_for(
            dataset,
            polar,
            request,
            lattice.value(),
            current,
            bucket_width);
        for (const EnsembleCommonAction& action : actions) {
            ++diagnostics.generated_labels;
            auto transition = evaluate_common_transition(
                dataset,
                polar,
                request.options,
                current,
                entry.label,
                action,
                EnsembleTransitionParameters{
                    request.destination,
                    departure,
                    route_end,
                    &request.objective,
                    EnsembleObjectiveTieBreakInputs{
                        diagnostics.generated_labels,
                        diagnostics.settled_labels,
                        {}},
                    end_cause});
            if (!transition) {
                return transition.error();
            }
            for (std::size_t index = 0U;
                 index < transition.value().diagnostics.members.size();
                 ++index) {
                merge(
                    member_environment[index],
                    transition.value().diagnostics.members[index].
                        environment_diagnostics);
            }

            auto key = state_key(
                transition.value().label,
                lattice.value(),
                departure,
                bucket_width,
                position_bucket_width);
            if (!key) {
                return key.error();
            }
            std::vector<LabelIndex>& state_labels = pareto[key.value()];
            bool dominated_candidate = false;
            for (const LabelIndex incumbent : state_labels) {
                if (dominates(labels[incumbent], transition.value().label)) {
                    dominated_candidate = true;
                    break;
                }
            }
            if (dominated_candidate) {
                ++diagnostics.dominated_labels;
                continue;
            }
            std::vector<LabelIndex> survivors;
            survivors.reserve(state_labels.size() + 1U);
            for (const LabelIndex incumbent : state_labels) {
                if (dominates(transition.value().label, labels[incumbent])) {
                    retained[incumbent] = false;
                    mark_dequeued(incumbent);
                    ++diagnostics.dominated_labels;
                } else {
                    survivors.push_back(incumbent);
                }
            }
            if (survivors.size() >= request.lattice.max_labels_per_state) {
                return label_limit_error(
                    "max_labels_per_state",
                    request.lattice.max_labels_per_state,
                    diagnostics);
            }
            if (labels.size() >= request.lattice.max_total_labels) {
                return label_limit_error(
                    "max_total_labels",
                    request.lattice.max_total_labels,
                    diagnostics);
            }

            const LabelIndex child = labels.size();
            labels.push_back(std::move(transition.value().label));
            retained.push_back(true);
            queued.push_back(false);
            survivors.push_back(child);
            std::sort(
                survivors.begin(),
                survivors.end(),
                [&labels](LabelIndex left, LabelIndex right) {
                    return ensemble_label_less(labels[left], labels[right]);
                });
            state_labels = std::move(survivors);
            ++diagnostics.retained_labels;
            diagnostics.peak_labels_per_state = std::max(
                diagnostics.peak_labels_per_state, state_labels.size());

            if (resolved(labels[child])) {
                if (!labels[child].aggregate_objective) {
                    auto evaluation = evaluate_ensemble_label_objective(
                        dataset,
                        request.objective,
                        labels[child],
                        departure,
                        EnsembleObjectiveTieBreakInputs{
                            diagnostics.generated_labels,
                            diagnostics.settled_labels,
                            {}});
                    if (!evaluation) {
                        return evaluation.error();
                    }
                }
                consider_terminal(child);
                continue;
            }
            queue.push(QueueEntry{
                optimistic_value(
                    dataset,
                    labels[child],
                    request,
                    departure,
                    maximum_speed,
                    safe_speed_bound),
                latest_time(labels[child]),
                labels[child].canonical_label_identity,
                child});
            mark_queued(child);
            ++diagnostics.queued_labels;
        }
    }

    if (on_step &&
        on_step(build_progress_view(EnsembleSolverPhase::finalizing)) ==
            RoutingProgressDecision::cancel) {
        return Error{
            ErrorCode::cancelled,
            "ensemble lattice routing cancelled at finalizing after " +
                std::to_string(diagnostics.settled_labels) +
                " settled labels"};
    }

    if (!best_terminal) {
        return Error{
            ErrorCode::no_route,
            "ensemble lattice search exhausted every reachable common state"};
    }
    return make_result(
        dataset,
        labels,
        *best_terminal,
        terminal_labels,
        request,
        departure,
        departure_source,
        diagnostics,
        member_environment);
}

}  // namespace detail
}  // namespace sailroute
