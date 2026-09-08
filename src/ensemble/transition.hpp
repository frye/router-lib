#pragma once

#include "ensemble/search_state.hpp"
#include "sailroute/polar.hpp"

#include <optional>
#include <vector>

namespace sailroute::detail {

enum class EnsembleMemberTransitionStatus : std::uint8_t {
    not_active,
    legal,
    reached,
    infeasible,
    error,
};

struct EnsembleMemberTransitionEvaluation {
    std::string member_identifier;
    EnsembleMemberTransitionStatus status{
        EnsembleMemberTransitionStatus::not_active};
    std::optional<EnsembleMemberOutcomeClass> outcome_class;
    std::optional<Error> error;
    EnvironmentDiagnostics environment_diagnostics;
};

struct EnsembleTransitionDiagnostics {
    std::vector<EnsembleMemberTransitionEvaluation> members;
    EnvironmentDiagnostics merged_environment;
};

enum class EnsembleRouteEndCause : std::uint8_t {
    duration_limit,
    forecast_horizon,
};

struct EnsembleTransitionParameters {
    Coordinate route_destination;
    TimePoint departure;
    TimePoint route_end;
    const EnsembleObjective* objective{nullptr};
    EnsembleObjectiveTieBreakInputs objective_tie_break;
    EnsembleRouteEndCause route_end_cause{
        EnsembleRouteEndCause::duration_limit};
};

struct EnsembleTransitionEvaluation {
    EnsembleSearchLabel label;
    EnsembleTransitionDiagnostics diagnostics;
    /// False if a positive-weight scenario violates hard action feasibility.
    /// The diagnostic label must not be enqueued or returned as a route.
    bool common_action_legal{true};
};

[[nodiscard]] EnsembleMemberOutcomeClass classify_member_transition_error(
    const Error& error) noexcept;

/// Performs initial geometry/environment/wind checks before an arrival shortcut
/// or a member-local data failure can bypass known hard constraints.
[[nodiscard]] Result<RoutePoint> evaluate_ensemble_initial_point(
    const WeatherDataset& weather, const RoutingOptions& options,
    const RoutingEnvironment& environment, EnvironmentDiagnostics& diagnostics,
    Coordinate position, TimePoint time);

/// Evaluates one target action for every active member in dataset canonical
/// order. Member-local failures are retained and never abort evaluation of later
/// members; only malformed shared state returns a top-level error. Callers must
/// check common_action_legal before accepting the returned diagnostic label.
[[nodiscard]] Result<EnsembleTransitionEvaluation>
evaluate_common_transition(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const EnsembleSearchLabel& parent,
    std::size_t parent_label,
    const EnsembleCommonAction& action,
    const EnsembleTransitionParameters& parameters);

[[nodiscard]] Result<EnsembleTransitionEvaluation>
evaluate_common_target_transition(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const EnsembleSearchLabel& parent,
    std::size_t parent_label,
    const EnsembleCommonAction& action,
    const EnsembleTransitionParameters& parameters);

}  // namespace sailroute::detail
