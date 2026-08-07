#include "transition.hpp"

#include "routing/environment_context.hpp"
#include "routing/geodesy.hpp"
#include "routing/transition.hpp"

#include <algorithm>
#include <string>
#include <utility>

namespace sailroute::detail {
namespace {

[[nodiscard]] Error invalid_transition(std::string message) {
    return Error{ErrorCode::invalid_argument, std::move(message)};
}

void fail_member(
    EnsembleMemberSearchState& state,
    EnsembleMemberTransitionEvaluation& evaluation,
    EnsembleMemberOutcomeClass outcome_class,
    std::optional<Error> error) {
    state.status = EnsembleMemberSearchStatus::failed;
    state.outcome_class = outcome_class;
    state.error = error;
    evaluation.outcome_class = outcome_class;
    evaluation.error = std::move(error);
}

[[nodiscard]] EnsembleMemberOutcomeClass route_end_outcome(
    const EnsembleTransitionParameters& parameters) noexcept {
    return parameters.route_end_cause == EnsembleRouteEndCause::forecast_horizon
        ? EnsembleMemberOutcomeClass::forecast_exhausted
        : EnsembleMemberOutcomeClass::duration_exhausted;
}

}  // namespace

EnsembleMemberOutcomeClass classify_member_transition_error(
    const Error& error) noexcept {
    switch (error.code) {
        case ErrorCode::forecast_exhausted:
            return EnsembleMemberOutcomeClass::forecast_exhausted;
        case ErrorCode::cancelled:
            return EnsembleMemberOutcomeClass::cancelled;
        case ErrorCode::no_route:
            return EnsembleMemberOutcomeClass::infeasible_no_route;
        case ErrorCode::incomplete_forecast:
        case ErrorCode::departure_outside_forecast:
        case ErrorCode::coordinate_outside_forecast:
        case ErrorCode::environment_data_unavailable:
        case ErrorCode::file_io:
        case ErrorCode::grib_decode:
        case ErrorCode::unsupported_grib:
            return EnsembleMemberOutcomeClass::missing_data;
        case ErrorCode::invalid_environment:
            return EnsembleMemberOutcomeClass::provider_failure;
        case ErrorCode::invalid_argument:
        case ErrorCode::invalid_polar:
        case ErrorCode::output_error:
            return EnsembleMemberOutcomeClass::other_error;
    }
    return EnsembleMemberOutcomeClass::other_error;
}

Result<EnsembleTransitionEvaluation> evaluate_common_target_transition(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const EnsembleSearchLabel& parent,
    std::size_t parent_label,
    const EnsembleCommonAction& action,
    const EnsembleTransitionParameters& parameters) {
    if (action.kind != EnsembleCommonActionKind::target) {
        return invalid_transition(
            "common target transition requires a target action");
    }
    return evaluate_common_transition(
        dataset,
        polar,
        options,
        parent,
        parent_label,
        action,
        parameters);
}

Result<EnsembleTransitionEvaluation> evaluate_common_transition(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const EnsembleSearchLabel& parent,
    std::size_t parent_label,
    const EnsembleCommonAction& action,
    const EnsembleTransitionParameters& parameters) {
    if (parent.members.size() != dataset.member_count()) {
        return invalid_transition(
            "ensemble transition parent member count does not match dataset");
    }
    if (!dataset.alignment().shared_search_compatible()) {
        return invalid_transition(
            "ensemble transition requires a shared-search-compatible dataset");
    }
    Result<EnsembleCommonAction> canonical_action =
        invalid_transition("unknown ensemble common action kind");
    switch (action.kind) {
        case EnsembleCommonActionKind::target:
            canonical_action = make_common_target_action(action.target);
            break;
        case EnsembleCommonActionKind::heading_for_duration:
            canonical_action = make_common_heading_action(
                action.heading_degrees, action.duration);
            break;
        case EnsembleCommonActionKind::wait_for_duration:
            canonical_action = make_common_wait_action(action.duration);
            break;
    }
    if (!canonical_action) {
        return canonical_action.error();
    }
    if (!is_valid(parameters.route_destination) ||
        parameters.route_end < parameters.departure) {
        return invalid_transition("ensemble transition parameters are invalid");
    }

    EnsembleTransitionEvaluation result;
    result.label = parent;
    result.label.parent_label = parent_label;
    result.label.incoming_action = canonical_action.value();
    result.label.common_action_history.push_back(canonical_action.value());
    result.label.aggregate_objective.reset();
    result.diagnostics.members.reserve(dataset.member_count());

    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        EnsembleMemberSearchState& state = result.label.members[index];
        const std::string& identifier = dataset.members()[index].identifier;
        if (state.member_identifier != identifier) {
            return invalid_transition(
                "ensemble transition members are not in dataset canonical order");
        }

        EnsembleMemberTransitionEvaluation evaluation;
        evaluation.member_identifier = identifier;
        if (state.status != EnsembleMemberSearchStatus::active) {
            result.diagnostics.members.push_back(std::move(evaluation));
            continue;
        }
        const WeatherDataset* weather = dataset.member_weather(index);
        const RoutingEnvironment* environment =
            dataset.member_environment(index);
        if (weather == nullptr || environment == nullptr) {
            return invalid_transition(
                "ensemble dataset is missing canonical member storage");
        }

        if (state.point.time >= parameters.route_end) {
            evaluation.status = EnsembleMemberTransitionStatus::infeasible;
            fail_member(
                state,
                evaluation,
                route_end_outcome(parameters),
                std::nullopt);
            result.diagnostics.members.push_back(std::move(evaluation));
            continue;
        }

        if (canonical_action.value().kind != EnsembleCommonActionKind::target &&
            state.point.time + canonical_action.value().duration >
                parameters.route_end) {
            evaluation.status = EnsembleMemberTransitionStatus::infeasible;
            fail_member(
                state,
                evaluation,
                route_end_outcome(parameters),
                std::nullopt);
            result.diagnostics.members.push_back(std::move(evaluation));
            continue;
        }

        if (canonical_action.value().kind ==
            EnsembleCommonActionKind::wait_for_duration) {
            const TimePoint arrival =
                state.point.time + canonical_action.value().duration;
            const SegmentCheckResult waiting = check_segment_geometry(
                *environment,
                state.point.position,
                state.point.time,
                state.point.position,
                arrival,
                evaluation.environment_diagnostics);
            if (waiting.outcome == EnvironmentOutcome::failed) {
                evaluation.status = EnsembleMemberTransitionStatus::error;
                fail_member(
                    state,
                    evaluation,
                    classify_member_transition_error(*waiting.error),
                    waiting.error);
            } else if (waiting.outcome == EnvironmentOutcome::rejected) {
                evaluation.status = EnsembleMemberTransitionStatus::infeasible;
                fail_member(
                    state,
                    evaluation,
                    EnsembleMemberOutcomeClass::infeasible_no_route,
                    std::nullopt);
            } else {
                state.point.time = arrival;
                state.point.boat_speed_knots = 0.0;
                state.point.environment.reset();
                state.error.reset();
                evaluation.status = EnsembleMemberTransitionStatus::legal;
            }
            merge(
                result.diagnostics.merged_environment,
                evaluation.environment_diagnostics);
            result.diagnostics.members.push_back(std::move(evaluation));
            continue;
        }

        VariableTransitionRejection rejection{
            VariableTransitionRejection::infeasible};
        Result<std::optional<VariableTransition>> transition =
            canonical_action.value().kind == EnsembleCommonActionKind::target
            ? evaluate_variable_transition(
                  *weather,
                  polar,
                  options,
                  *environment,
                  evaluation.environment_diagnostics,
                  state.point,
                  state.configuration,
                  canonical_action.value().target,
                  parameters.route_end,
                  &rejection)
            : evaluate_heading_transition(
                  *weather,
                  polar,
                  options,
                  *environment,
                  evaluation.environment_diagnostics,
                  state.point,
                  state.configuration,
                  canonical_action.value().heading_degrees,
                  state.point.time + canonical_action.value().duration,
                  parameters.route_destination,
                  options.arrival_radius_nautical_miles,
                  &rejection);
        merge(
            result.diagnostics.merged_environment,
            evaluation.environment_diagnostics);

        if (!transition) {
            evaluation.status = EnsembleMemberTransitionStatus::error;
            EnsembleMemberOutcomeClass outcome_class =
                classify_member_transition_error(transition.error());
            if (transition.error().code ==
                    ErrorCode::departure_outside_forecast &&
                !dataset.members()[index].wind_valid_times.empty() &&
                state.point.time >
                    dataset.members()[index].wind_valid_times.back()) {
                outcome_class = EnsembleMemberOutcomeClass::forecast_exhausted;
            }
            fail_member(
                state,
                evaluation,
                outcome_class,
                transition.error());
        } else if (!transition.value()) {
            evaluation.status = EnsembleMemberTransitionStatus::infeasible;
            EnsembleMemberOutcomeClass outcome_class{
                EnsembleMemberOutcomeClass::infeasible_no_route};
            switch (rejection) {
                case VariableTransitionRejection::infeasible:
                    break;
                case VariableTransitionRejection::forecast_exhausted:
                    outcome_class =
                        EnsembleMemberOutcomeClass::forecast_exhausted;
                    break;
                case VariableTransitionRejection::duration_exhausted:
                    outcome_class = route_end_outcome(parameters);
                    break;
                case VariableTransitionRejection::missing_data:
                    outcome_class = EnsembleMemberOutcomeClass::missing_data;
                    break;
            }
            fail_member(
                state,
                evaluation,
                outcome_class,
                std::nullopt);
        } else {
            state.point = std::move(transition.value()->point);
            state.configuration = transition.value()->configuration;
            state.error.reset();
            if (great_circle_distance_nautical_miles(
                    state.point.position,
                    parameters.route_destination) <=
                options.arrival_radius_nautical_miles) {
                state.status = EnsembleMemberSearchStatus::completed;
                state.outcome_class = EnsembleMemberOutcomeClass::reached;
                evaluation.status = EnsembleMemberTransitionStatus::reached;
                evaluation.outcome_class = EnsembleMemberOutcomeClass::reached;
            } else {
                state.status = EnsembleMemberSearchStatus::active;
                state.outcome_class.reset();
                evaluation.status = EnsembleMemberTransitionStatus::legal;
            }
        }
        result.diagnostics.members.push_back(std::move(evaluation));
    }

    canonicalize_ensemble_label(result.label);
    const bool resolved = std::none_of(
        result.label.members.begin(),
        result.label.members.end(),
        [](const EnsembleMemberSearchState& member) {
            return member.status == EnsembleMemberSearchStatus::active;
        });
    if (resolved && parameters.objective != nullptr) {
        auto objective = evaluate_ensemble_label_objective(
            dataset,
            *parameters.objective,
            result.label,
            parameters.departure,
            parameters.objective_tie_break);
        if (!objective) {
            return objective.error();
        }
    }
    return result;
}

}  // namespace sailroute::detail
