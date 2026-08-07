#pragma once

#include "sailroute/ensemble.hpp"

#include <cstddef>
#include <span>
#include <string>

namespace sailroute::detail {

/// Search-owned fields used after the primary objective is equal.
struct EnsembleObjectiveTieBreakInputs {
    std::size_t generated_states{};
    std::size_t settled_states{};
    std::string canonical_action_sequence_identity;
};

/// Validates and evaluates one candidate in dataset canonical member order.
Result<EnsembleObjectiveEvaluation> evaluate_ensemble_objective(
    const EnsembleDataset& dataset,
    const EnsembleObjective& objective,
    std::span<const EnsembleMemberOutcome> candidate_outcomes,
    EnsembleObjectiveTieBreakInputs tie_break = {});

/// Returns -1 when `left` is preferred, 1 when `right` is preferred, and zero
/// when every primary and secondary ordering field is equal.
///
/// Primary arrival objectives minimize and probability objectives maximize.
/// Equal primary values prefer lower incomplete weight, lower finite-member
/// mean, lower worst finite arrival, fewer generated then settled states, and
/// finally lexicographically smaller canonical action identity.
int compare_ensemble_objective_evaluations(
    EnsembleObjectiveKind kind,
    const EnsembleObjectiveEvaluation& left,
    const EnsembleObjectiveEvaluation& right) noexcept;

}  // namespace sailroute::detail
