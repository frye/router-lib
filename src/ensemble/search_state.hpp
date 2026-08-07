#pragma once

#include "sailroute/ensemble.hpp"

#include "ensemble/objective.hpp"
#include "routing/state.hpp"

#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace sailroute::detail {

inline constexpr std::size_t no_ensemble_label =
    std::numeric_limits<std::size_t>::max();

enum class EnsembleCommonActionKind : std::uint8_t {
    target,
    heading_for_duration,
    wait_for_duration,
};

/// A solver-independent action shared by every still-active ensemble member.
///
/// Target actions support lattice edges, heading actions support beam and
/// lattice continuation moves, and waits support time-dependent lattice states.
struct EnsembleCommonAction {
    Coordinate target;
    EnsembleCommonActionKind kind{EnsembleCommonActionKind::target};
    double heading_degrees{};
    std::chrono::seconds duration{};

    friend bool operator==(
        const EnsembleCommonAction& left,
        const EnsembleCommonAction& right) noexcept;
    friend bool operator<(
        const EnsembleCommonAction& left,
        const EnsembleCommonAction& right) noexcept;
};

[[nodiscard]] Result<EnsembleCommonAction> make_common_target_action(
    Coordinate target);
[[nodiscard]] Result<EnsembleCommonAction> make_common_heading_action(
    double heading_degrees,
    std::chrono::seconds duration);
[[nodiscard]] Result<EnsembleCommonAction> make_common_wait_action(
    std::chrono::seconds duration);
[[nodiscard]] std::string common_action_identity(
    const EnsembleCommonAction& action);

enum class EnsembleMemberSearchStatus : std::uint8_t {
    active,
    completed,
    failed,
};

/// Canonical state for one dataset member at one shared-search label.
struct EnsembleMemberSearchState {
    std::string member_identifier;
    RoutePoint point;
    OperationalConfiguration configuration;
    EnsembleMemberSearchStatus status{EnsembleMemberSearchStatus::active};
    std::optional<EnsembleMemberOutcomeClass> outcome_class;
    std::optional<Error> error;
};

/// One shared-search label. There is exactly one action history for all members;
/// member-local state never carries a separate path.
struct EnsembleSearchLabel {
    std::vector<EnsembleMemberSearchState> members;
    std::vector<EnsembleCommonAction> common_action_history;
    std::string canonical_action_sequence_identity{"root"};
    std::string canonical_label_identity;
    std::optional<EnsembleObjectiveEvaluation> aggregate_objective;
    std::size_t parent_label{no_ensemble_label};
    std::optional<EnsembleCommonAction> incoming_action;
};

[[nodiscard]] Result<EnsembleSearchLabel> make_initial_ensemble_label(
    const EnsembleDataset& dataset,
    std::span<const RoutePoint> member_points,
    std::span<const OperationalConfiguration> member_configurations);

/// Refreshes identities after internal state construction or mutation.
void canonicalize_ensemble_label(EnsembleSearchLabel& label);

/// Converts a fully resolved label to canonical public objective outcomes.
[[nodiscard]] Result<std::vector<EnsembleMemberOutcome>>
ensemble_label_outcomes(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& label,
    TimePoint departure);

[[nodiscard]] Result<EnsembleObjectiveEvaluation>
evaluate_ensemble_label_objective(
    const EnsembleDataset& dataset,
    const EnsembleObjective& objective,
    EnsembleSearchLabel& label,
    TimePoint departure,
    EnsembleObjectiveTieBreakInputs tie_break = {});

/// Pareto dominance over member arrivals only. Aggregate scalar objective values
/// are intentionally ignored.
[[nodiscard]] bool dominates(
    const EnsembleSearchLabel& left,
    const EnsembleSearchLabel& right) noexcept;

[[nodiscard]] bool ensemble_label_less(
    const EnsembleSearchLabel& left,
    const EnsembleSearchLabel& right) noexcept;

[[nodiscard]] Result<std::vector<EnsembleCommonAction>>
reconstruct_common_actions(
    std::span<const EnsembleSearchLabel> labels,
    std::size_t terminal_label);

[[nodiscard]] Result<std::vector<RoutePoint>> reconstruct_member_route(
    std::span<const EnsembleSearchLabel> labels,
    std::size_t terminal_label,
    std::size_t member_index);

}  // namespace sailroute::detail
