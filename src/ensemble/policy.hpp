#pragma once

#include "ensemble/search_state.hpp"

#include <span>

namespace sailroute::detail {

struct EnsemblePolicyBuildResult {
    EnsemblePolicyGraph graph;
    std::vector<EnsembleDecisionPoint> decision_points;
    EnsembleReevaluationState re_evaluation;
};

[[nodiscard]] bool policy_terminal_dominates(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& left,
    const EnsembleSearchLabel& right) noexcept;

[[nodiscard]] Result<EnsemblePolicyBuildResult> build_ensemble_policy(
    const EnsembleDataset& dataset,
    const EnsembleObjective& objective,
    const EnsemblePolicyOptions& options,
    std::span<const EnsembleSearchLabel> labels,
    std::span<const std::size_t> terminal_labels,
    std::size_t selected_terminal,
    TimePoint departure);

}  // namespace sailroute::detail
