#include "ensemble/beam_solver.hpp"

#include "ensemble/objective.hpp"
#include "ensemble/policy.hpp"
#include "ensemble/search_state.hpp"
#include "ensemble/transition.hpp"
#include "routing/environment_context.hpp"
#include "routing/geodesy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace sailroute::detail {
namespace {

using LabelIndex = std::size_t;

struct MemberBucketSignature {
    OperationalConfiguration configuration;
    EnsembleMemberSearchStatus status{EnsembleMemberSearchStatus::active};
    std::optional<EnsembleMemberOutcomeClass> outcome_class;
    std::optional<ErrorCode> error_code;
    TimePoint time{};

    friend bool operator<(
        const MemberBucketSignature& left,
        const MemberBucketSignature& right) noexcept {
        return std::tie(
                   left.configuration,
                   left.status,
                   left.outcome_class,
                   left.error_code,
                   left.time) <
            std::tie(
                   right.configuration,
                   right.status,
                   right.outcome_class,
                   right.error_code,
                   right.time);
    }
};

struct BeamBucketKey {
    SphericalPositionKey centroid;
    std::vector<MemberBucketSignature> members;

    friend bool operator<(
        const BeamBucketKey& left,
        const BeamBucketKey& right) noexcept {
        return std::tie(left.centroid, left.members) <
            std::tie(right.centroid, right.members);
    }
};

struct PartialRank {
    double failed_weight{};
    double completed_weight{};
    double weighted_completed_elapsed_seconds{};
    double worst_completed_elapsed_seconds{};
    double weighted_distance{};
    double worst_distance{};
    double centroid_distance{};
    std::string canonical_identity;
};

struct Candidate {
    LabelIndex label{};
    BeamBucketKey bucket;
    PartialRank rank;
    std::string heading_identity;
    bool bucket_selected{};
    bool beam_selected{};
};

struct BucketSelection {
    std::size_t count{};
    std::set<std::string> headings;
};

[[nodiscard]] Error invalid_request(std::string message) {
    return Error{ErrorCode::invalid_argument, std::move(message)};
}

[[nodiscard]] Error node_limit_error(
    const EnsembleBeamRoutingOptions& options,
    const EnsembleBeamDiagnostics& diagnostics) {
    return Error{
        ErrorCode::no_route,
        "experimental ensemble beam max_total_nodes hard limit " +
            std::to_string(options.max_total_nodes) + " exceeded after " +
            std::to_string(diagnostics.generated_nodes) +
            " generated and " + std::to_string(diagnostics.accepted_nodes) +
            " accepted nodes"};
}

[[nodiscard]] bool resolved(const EnsembleSearchLabel& label) noexcept {
    return std::none_of(
        label.members.begin(),
        label.members.end(),
        [](const EnsembleMemberSearchState& member) {
            return member.status == EnsembleMemberSearchStatus::active;
        });
}

[[nodiscard]] TimePoint active_time(const EnsembleSearchLabel& label) noexcept {
    for (const EnsembleMemberSearchState& member : label.members) {
        if (member.status == EnsembleMemberSearchStatus::active) {
            return member.point.time;
        }
    }
    return label.members.front().point.time;
}

[[nodiscard]] Coordinate weighted_spherical_centroid(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& label) noexcept {
    constexpr long double degrees_to_radians =
        std::numbers::pi_v<long double> / 180.0L;
    long double x = 0.0L;
    long double y = 0.0L;
    long double z = 0.0L;
    for (std::size_t index = 0U; index < label.members.size(); ++index) {
        const Coordinate position = label.members[index].point.position;
        const long double latitude =
            static_cast<long double>(position.latitude_degrees) *
            degrees_to_radians;
        const long double longitude =
            static_cast<long double>(position.longitude_degrees) *
            degrees_to_radians;
        const long double weight =
            static_cast<long double>(
                dataset.members()[index].normalized_weight);
        const long double cosine_latitude = std::cos(latitude);
        x += weight * cosine_latitude * std::cos(longitude);
        y += weight * cosine_latitude * std::sin(longitude);
        z += weight * std::sin(latitude);
    }
    const long double horizontal = std::hypot(x, y);
    const long double norm = std::hypot(horizontal, z);
    if (!(norm > 64.0L * std::numeric_limits<long double>::epsilon())) {
        return Coordinate{};
    }
    return Coordinate{
        static_cast<double>(
            std::atan2(z, horizontal) / degrees_to_radians),
        static_cast<double>(std::atan2(y, x) / degrees_to_radians)};
}

[[nodiscard]] SphericalPositionKey centroid_key(
    Coordinate centroid,
    double bucket_width_nautical_miles) noexcept {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    const double latitude =
        centroid.latitude_degrees * degrees_to_radians;
    const double longitude =
        centroid.longitude_degrees * degrees_to_radians;
    const double scale =
        earth_radius_nautical_miles / bucket_width_nautical_miles;
    const double cosine_latitude = std::cos(latitude);
    return SphericalPositionKey{
        std::llround(cosine_latitude * std::cos(longitude) * scale),
        std::llround(cosine_latitude * std::sin(longitude) * scale),
        std::llround(std::sin(latitude) * scale)};
}

[[nodiscard]] BeamBucketKey bucket_key(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& label,
    double bucket_width_nautical_miles) {
    BeamBucketKey key;
    key.centroid = centroid_key(
        weighted_spherical_centroid(dataset, label),
        bucket_width_nautical_miles);
    key.members.reserve(label.members.size());
    for (const EnsembleMemberSearchState& member : label.members) {
        key.members.push_back(MemberBucketSignature{
            member.configuration,
            member.status,
            member.outcome_class,
            member.error
                ? std::optional<ErrorCode>{member.error->code}
                : std::nullopt,
            member.point.time});
    }
    return key;
}

[[nodiscard]] PartialRank partial_rank(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& label,
    TimePoint departure,
    Coordinate destination) {
    PartialRank rank;
    rank.canonical_identity = label.canonical_label_identity;
    for (std::size_t index = 0U; index < label.members.size(); ++index) {
        const EnsembleMemberSearchState& member = label.members[index];
        const double weight =
            dataset.members()[index].normalized_weight;
        const double distance = great_circle_distance_nautical_miles(
            member.point.position, destination);
        rank.weighted_distance += weight * distance;
        if (weight > 0.0) {
            rank.worst_distance = std::max(rank.worst_distance, distance);
        }
        if (member.status == EnsembleMemberSearchStatus::completed) {
            rank.completed_weight += weight;
            const double elapsed = std::chrono::duration<double>(
                member.point.time - departure).count();
            rank.weighted_completed_elapsed_seconds += weight * elapsed;
            if (weight > 0.0) {
                rank.worst_completed_elapsed_seconds = std::max(
                    rank.worst_completed_elapsed_seconds, elapsed);
            }
        } else if (member.status == EnsembleMemberSearchStatus::failed) {
            rank.failed_weight += weight;
        }
    }
    rank.centroid_distance = great_circle_distance_nautical_miles(
        weighted_spherical_centroid(dataset, label), destination);
    return rank;
}

[[nodiscard]] bool rank_less(
    const PartialRank& left,
    const PartialRank& right) noexcept {
    if (left.failed_weight != right.failed_weight) {
        return left.failed_weight < right.failed_weight;
    }
    if (left.completed_weight != right.completed_weight) {
        return left.completed_weight > right.completed_weight;
    }
    return std::tie(
               left.weighted_completed_elapsed_seconds,
               left.worst_completed_elapsed_seconds,
               left.weighted_distance,
               left.worst_distance,
               left.centroid_distance,
               left.canonical_identity) <
        std::tie(
               right.weighted_completed_elapsed_seconds,
               right.worst_completed_elapsed_seconds,
               right.weighted_distance,
               right.worst_distance,
               right.centroid_distance,
               right.canonical_identity);
}

[[nodiscard]] std::vector<EnsembleCommonAction> beam_actions(
    const EnsembleSearchLabel& label,
    Coordinate destination,
    const EnsembleBeamRoutingOptions& options,
    std::chrono::seconds duration) {
    std::vector<EnsembleCommonAction> actions;
    const std::size_t heading_count = static_cast<std::size_t>(
        std::ceil(360.0 / options.heading_step_degrees));
    actions.reserve(heading_count + label.members.size());
    for (std::size_t index = 0U; index < heading_count; ++index) {
        auto action = make_common_heading_action(
            static_cast<double>(index) * options.heading_step_degrees,
            duration);
        if (action) {
            actions.push_back(action.value());
        }
    }
    for (const EnsembleMemberSearchState& member : label.members) {
        if (member.status != EnsembleMemberSearchStatus::active) {
            continue;
        }
        auto action = make_common_heading_action(
            initial_bearing_degrees(member.point.position, destination),
            duration);
        if (action) {
            actions.push_back(action.value());
        }
    }
    std::sort(actions.begin(), actions.end());
    actions.erase(std::unique(actions.begin(), actions.end()), actions.end());
    return actions;
}

[[nodiscard]] bool common_action_accepted(
    const EnsembleSearchLabel& parent,
    const EnsembleTransitionDiagnostics& diagnostics) noexcept {
    if (parent.members.size() != diagnostics.members.size()) {
        return false;
    }
    for (std::size_t index = 0U; index < parent.members.size(); ++index) {
        if (parent.members[index].status !=
            EnsembleMemberSearchStatus::active) {
            continue;
        }
        const EnsembleMemberTransitionStatus status =
            diagnostics.members[index].status;
        if (status != EnsembleMemberTransitionStatus::legal &&
            status != EnsembleMemberTransitionStatus::reached) {
            return false;
        }
    }
    return true;
}

[[nodiscard]] std::optional<Error> transition_error(
    const EnsembleSearchLabel& parent,
    const EnsembleTransitionDiagnostics& diagnostics) {
    for (std::size_t index = 0U; index < parent.members.size(); ++index) {
        if (parent.members[index].status ==
                EnsembleMemberSearchStatus::active &&
            diagnostics.members[index].status ==
                EnsembleMemberTransitionStatus::error &&
            diagnostics.members[index].error) {
            return diagnostics.members[index].error;
        }
    }
    return std::nullopt;
}

void finalize_active_members(
    EnsembleSearchLabel& label,
    EnsembleMemberOutcomeClass outcome_class) {
    for (EnsembleMemberSearchState& member : label.members) {
        if (member.status == EnsembleMemberSearchStatus::active) {
            member.status = EnsembleMemberSearchStatus::failed;
            member.outcome_class = outcome_class;
            member.error.reset();
        }
    }
    canonicalize_ensemble_label(label);
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
    const EnsembleBeamDiagnostics& diagnostics,
    std::span<const EnvironmentDiagnostics> member_environment) {
    const EnsembleSearchLabel& selected = labels[terminal];
    if (!selected.aggregate_objective) {
        return invalid_request(
            "selected experimental beam node has no objective evaluation");
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
    result.solver = EnsembleSolver::experimental_isochrone_beam;
    result.experimental = true;
    result.objective = *selected.aggregate_objective;
    result.canonical_action_sequence_identity =
        selected.canonical_action_sequence_identity;
    result.beam_diagnostics = diagnostics;
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

[[nodiscard]] Result<std::pair<TimePoint, DepartureSource>> departure_for(
    const EnsembleDataset& dataset,
    const EnsembleRouteRequest& request) {
    const TimePoint departure = request.departure_time.value_or(
        dataset.members().front().weather.first_valid_time);
    const DepartureSource source = request.departure_time
        ? DepartureSource::explicit_time
        : DepartureSource::forecast_start_fallback;
    for (const EnsembleMemberMetadata& member : dataset.members()) {
        if (departure < member.weather.first_valid_time ||
            departure > member.weather.last_valid_time) {
            return Error{
                ErrorCode::departure_outside_forecast,
                "ensemble departure is outside the common forecast horizon"};
        }
    }
    return std::pair{departure, source};
}

}  // namespace

Result<EnsembleRouteResult> optimize_ensemble_beam_route(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const EnsembleRouteRequest& request,
    const EnsembleCancellationCallback& cancelled,
    const EnsembleStepCallback& on_step) {
    if (request.solver != EnsembleSolver::experimental_isochrone_beam ||
        !request.enable_experimental_beam) {
        return invalid_request(
            "experimental ensemble beam requires explicit solver selection and enable_experimental_beam=true");
    }
    if (dataset.member_count() == 0U ||
        !dataset.alignment().shared_search_compatible()) {
        return invalid_request(
            "experimental ensemble beam requires a non-empty shared-search-compatible dataset");
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
    const EnsembleBeamRoutingOptions& options = request.beam;
    if (options.time_step <= std::chrono::minutes::zero() ||
        !std::isfinite(options.heading_step_degrees) ||
        options.heading_step_degrees <= 0.0 ||
        options.heading_step_degrees > 180.0 ||
        360.0 / options.heading_step_degrees > 1'000'000.0 ||
        !std::isfinite(options.centroid_bucket_nautical_miles) ||
        options.centroid_bucket_nautical_miles <= 0.0 ||
        options.max_nodes_per_bucket == 0U ||
        options.beam_width == 0U ||
        options.max_steps == 0U ||
        options.max_total_nodes == 0U) {
        return invalid_request(
            "experimental ensemble beam resolution, pruning, and hard limits must be finite and positive");
    }
    if (!std::isfinite(
            request.policy.commitment_spatial_tolerance_nautical_miles) ||
        request.policy.commitment_spatial_tolerance_nautical_miles < 0.0 ||
        request.policy.commitment_time_tolerance <
            std::chrono::minutes::zero()) {
        return invalid_request(
            "ensemble policy commitment tolerances must be non-negative");
    }
    if (!(polar.maximum_boat_speed_knots() > 0.0)) {
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

    auto departure = departure_for(dataset, request);
    if (!departure) {
        return departure.error();
    }
    const TimePoint departure_time = departure.value().first;
    const TimePoint duration_end =
        departure_time + request.options.maximum_route_duration;
    TimePoint route_end = duration_end;
    for (const EnsembleMemberMetadata& member : dataset.members()) {
        route_end = std::min(route_end, member.weather.last_valid_time);
    }
    const EnsembleMemberOutcomeClass end_outcome =
        route_end < duration_end
        ? EnsembleMemberOutcomeClass::forecast_exhausted
        : EnsembleMemberOutcomeClass::duration_exhausted;

    std::vector<RoutePoint> initial_points;
    std::vector<OperationalConfiguration> configurations(
        dataset.member_count());
    std::vector<std::optional<Error>> initial_errors(dataset.member_count());
    initial_points.reserve(dataset.member_count());
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        const WeatherDataset* weather = dataset.member_weather(index);
        if (weather == nullptr) {
            return invalid_request(
                "ensemble dataset is missing member weather");
        }
        const auto wind = weather->interpolate(
            request.start, departure_time);
        RoutePoint point;
        point.position = request.start;
        point.time = departure_time;
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

    EnsembleBeamDiagnostics diagnostics;
    diagnostics.beam_width = options.beam_width;
    diagnostics.max_nodes_per_bucket = options.max_nodes_per_bucket;
    diagnostics.max_steps = options.max_steps;
    diagnostics.max_total_nodes = options.max_total_nodes;
    diagnostics.retained_nodes = 1U;
    diagnostics.peak_frontier = 1U;

    std::vector<EnsembleSearchLabel> labels;
    labels.reserve(std::min(options.max_total_nodes, options.beam_width * 4U));
    labels.push_back(std::move(initial.value()));
    std::vector<LabelIndex> frontier{0U};
    std::optional<LabelIndex> best_terminal;
    std::vector<LabelIndex> terminal_labels;
    std::vector<EnvironmentDiagnostics> member_environment(
        dataset.member_count());

    const auto consider_terminal = [&](LabelIndex candidate) {
        ++diagnostics.completed_nodes;
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
            departure_time);
        if (!evaluation) {
            return evaluation.error();
        }
        consider_terminal(0U);
        frontier.clear();
    }

    const auto build_beam_progress = [&](EnsembleSolverPhase phase) {
        EnsembleProgressView view;
        view.phase = phase;
        if (phase != EnsembleSolverPhase::finalizing && !frontier.empty()) {
            const EnsembleSearchLabel& front = labels[frontier.front()];
            view.active_member_count = static_cast<std::size_t>(
                std::count_if(
                    front.members.begin(),
                    front.members.end(),
                    [](const EnsembleMemberSearchState& m) {
                        return m.status == EnsembleMemberSearchStatus::active;
                    }));
            view.active_label_count = frontier.size();
        }
        view.retained_label_count = diagnostics.retained_nodes;
        view.generated_states = diagnostics.generated_nodes;
        view.settled_states = diagnostics.expanded_nodes;
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
        if (on_step(build_beam_progress(EnsembleSolverPhase::initializing)) ==
            RoutingProgressDecision::cancel) {
            return Error{
                ErrorCode::cancelled,
                "experimental ensemble beam cancelled before search"};
        }
    }

    const auto step = std::chrono::duration_cast<std::chrono::seconds>(
        options.time_step);
    std::size_t depth = 0U;
    while (!frontier.empty()) {
        if (cancelled && cancelled()) {
            return Error{
                ErrorCode::cancelled,
                "experimental ensemble beam cancelled after " +
                    std::to_string(diagnostics.expanded_nodes) +
                    " expanded nodes"};
        }
        if (on_step) {
            if (on_step(build_beam_progress(EnsembleSolverPhase::searching)) ==
                RoutingProgressDecision::cancel) {
                return Error{
                    ErrorCode::cancelled,
                    "experimental ensemble beam cancelled after " +
                        std::to_string(diagnostics.expanded_nodes) +
                        " expanded nodes"};
            }
        }

        std::vector<LabelIndex> expandable;
        expandable.reserve(frontier.size());
        for (const LabelIndex index : frontier) {
            if (active_time(labels[index]) >= route_end) {
                finalize_active_members(labels[index], end_outcome);
                auto evaluation = evaluate_ensemble_label_objective(
                    dataset,
                    request.objective,
                    labels[index],
                    departure_time,
                    EnsembleObjectiveTieBreakInputs{
                        diagnostics.generated_nodes,
                        diagnostics.expanded_nodes,
                        {}});
                if (!evaluation) {
                    return evaluation.error();
                }
                consider_terminal(index);
            } else {
                expandable.push_back(index);
            }
        }
        if (expandable.empty()) {
            break;
        }
        if (depth >= options.max_steps) {
            return Error{
                ErrorCode::no_route,
                "experimental ensemble beam max_steps hard limit " +
                    std::to_string(options.max_steps) + " exceeded"};
        }

        std::vector<Candidate> candidates;
        for (const LabelIndex parent_index : expandable) {
            if (cancelled && cancelled()) {
                return Error{
                    ErrorCode::cancelled,
                    "experimental ensemble beam cancelled after " +
                        std::to_string(diagnostics.expanded_nodes) +
                        " expanded nodes"};
            }
            const EnsembleSearchLabel parent = labels[parent_index];
            ++diagnostics.expanded_nodes;
            const auto remaining = std::chrono::duration_cast<
                std::chrono::seconds>(
                route_end - active_time(parent));
            const std::vector<EnsembleCommonAction> actions =
                beam_actions(
                    parent,
                    request.destination,
                    options,
                    std::min(step, remaining));
            for (const EnsembleCommonAction& action : actions) {
                if (diagnostics.generated_nodes >= options.max_total_nodes) {
                    return node_limit_error(options, diagnostics);
                }
                ++diagnostics.generated_nodes;
                auto transition = evaluate_common_transition(
                    dataset,
                    polar,
                    request.options,
                    parent,
                    parent_index,
                    action,
                    EnsembleTransitionParameters{
                        request.destination,
                        departure_time,
                        route_end,
                        &request.objective,
                        EnsembleObjectiveTieBreakInputs{
                            diagnostics.generated_nodes,
                            diagnostics.expanded_nodes,
                            {}},
                        route_end < duration_end
                            ? EnsembleRouteEndCause::forecast_horizon
                            : EnsembleRouteEndCause::duration_limit});
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
                if (const auto error = transition_error(
                        parent, transition.value().diagnostics)) {
                    return *error;
                }
                if (!common_action_accepted(
                        parent, transition.value().diagnostics)) {
                    ++diagnostics.rejected_common_actions;
                    continue;
                }
                if (labels.size() >= options.max_total_nodes) {
                    return node_limit_error(options, diagnostics);
                }

                const LabelIndex child = labels.size();
                labels.push_back(std::move(transition.value().label));
                ++diagnostics.accepted_nodes;
                if (resolved(labels[child])) {
                    if (!labels[child].aggregate_objective) {
                        auto evaluation = evaluate_ensemble_label_objective(
                            dataset,
                            request.objective,
                            labels[child],
                            departure_time,
                            EnsembleObjectiveTieBreakInputs{
                                diagnostics.generated_nodes,
                                diagnostics.expanded_nodes,
                                {}});
                        if (!evaluation) {
                            return evaluation.error();
                        }
                    }
                    consider_terminal(child);
                    continue;
                }
                candidates.push_back(Candidate{
                    child,
                    bucket_key(
                        dataset,
                        labels[child],
                        options.centroid_bucket_nautical_miles),
                    partial_rank(
                        dataset,
                        labels[child],
                        departure_time,
                        request.destination),
                    common_action_identity(action)});
            }
        }

        std::sort(
            candidates.begin(),
            candidates.end(),
            [](const Candidate& left, const Candidate& right) {
                return rank_less(left.rank, right.rank);
            });

        std::map<BeamBucketKey, BucketSelection> buckets;
        for (Candidate& candidate : candidates) {
            BucketSelection& selection = buckets[candidate.bucket];
            if (selection.count < options.max_nodes_per_bucket &&
                selection.headings.insert(
                    candidate.heading_identity).second) {
                candidate.bucket_selected = true;
                ++selection.count;
            }
        }
        for (Candidate& candidate : candidates) {
            if (candidate.bucket_selected) {
                continue;
            }
            BucketSelection& selection = buckets[candidate.bucket];
            if (selection.count < options.max_nodes_per_bucket) {
                candidate.bucket_selected = true;
                ++selection.count;
            }
        }

        std::set<std::string> retained_headings;
        std::size_t selected_count = 0U;
        for (Candidate& candidate : candidates) {
            if (!candidate.bucket_selected ||
                selected_count >= options.beam_width) {
                continue;
            }
            if (retained_headings.insert(
                    candidate.heading_identity).second) {
                candidate.beam_selected = true;
                ++selected_count;
            }
        }
        for (Candidate& candidate : candidates) {
            if (!candidate.bucket_selected ||
                candidate.beam_selected ||
                selected_count >= options.beam_width) {
                continue;
            }
            candidate.beam_selected = true;
            ++selected_count;
        }

        frontier.clear();
        frontier.reserve(selected_count);
        for (const Candidate& candidate : candidates) {
            if (candidate.beam_selected) {
                frontier.push_back(candidate.label);
            } else if (candidate.bucket_selected) {
                ++diagnostics.pruned_by_beam;
            } else {
                ++diagnostics.pruned_by_bucket;
            }
        }
        diagnostics.retained_nodes += frontier.size();
        diagnostics.peak_frontier =
            std::max(diagnostics.peak_frontier, frontier.size());
        ++depth;
    }

    if (on_step &&
        on_step(build_beam_progress(EnsembleSolverPhase::finalizing)) ==
            RoutingProgressDecision::cancel) {
        return Error{
            ErrorCode::cancelled,
            "experimental ensemble beam routing cancelled at finalizing"};
    }

    if (!best_terminal) {
        return Error{
            ErrorCode::no_route,
            "experimental ensemble beam exhausted every accepted common action"};
    }
    return make_result(
        dataset,
        labels,
        *best_terminal,
        terminal_labels,
        request,
        departure_time,
        departure.value().second,
        diagnostics,
        member_environment);
}

}  // namespace sailroute::detail
