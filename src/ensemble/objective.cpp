#include "objective.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sailroute {
namespace {

[[nodiscard]] EnsembleObjectiveValue finite_value(double value) noexcept {
    return EnsembleObjectiveValue{
        EnsembleObjectiveValueClass::finite,
        value};
}

[[nodiscard]] EnsembleObjectiveValue positive_infinity() noexcept {
    return EnsembleObjectiveValue{
        EnsembleObjectiveValueClass::positive_infinity,
        0.0};
}

[[nodiscard]] bool is_quantile_objective(EnsembleObjectiveKind kind) noexcept {
    return kind == EnsembleObjectiveKind::weighted_p75_elapsed_arrival ||
        kind == EnsembleObjectiveKind::weighted_p90_elapsed_arrival;
}

[[nodiscard]] Error invalid_objective(std::string message) {
    return Error{ErrorCode::invalid_argument, std::move(message)};
}

class ExactWeightSum {
public:
    void add(double value) noexcept {
        const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
        const std::uint64_t exponent = (bits >> 52U) & 0x7ffU;
        const std::uint64_t fraction = bits & ((std::uint64_t{1} << 52U) - 1U);
        const std::uint64_t significand = exponent == 0U
            ? fraction
            : fraction | (std::uint64_t{1} << 52U);
        add_shifted(significand, exponent == 0U ? 0U : exponent - 1U);
    }

    [[nodiscard]] ExactWeightSum multiplied(unsigned factor) const noexcept {
        ExactWeightSum result;
        std::uint64_t carry = 0U;
        for (std::size_t index = 0U; index < limbs_.size(); ++index) {
            const std::uint64_t low_product =
                (limbs_[index] & 0xffffffffU) * factor + carry;
            const std::uint64_t high_product =
                (limbs_[index] >> 32U) * factor + (low_product >> 32U);
            result.limbs_[index] =
                (high_product << 32U) | (low_product & 0xffffffffU);
            carry = high_product >> 32U;
        }
        return result;
    }

    [[nodiscard]] int compare(const ExactWeightSum& other) const noexcept {
        for (std::size_t index = limbs_.size(); index-- > 0U;) {
            if (limbs_[index] < other.limbs_[index]) {
                return -1;
            }
            if (limbs_[index] > other.limbs_[index]) {
                return 1;
            }
        }
        return 0;
    }

private:
    void add_shifted(std::uint64_t value, std::uint64_t shift) noexcept {
        if (value == 0U) {
            return;
        }
        const std::size_t limb = static_cast<std::size_t>(shift / 64U);
        const unsigned offset = static_cast<unsigned>(shift % 64U);
        const std::uint64_t low = value << offset;
        const std::uint64_t high =
            offset == 0U ? 0U : value >> (64U - offset);
        const std::uint64_t previous_low = limbs_[limb];
        limbs_[limb] += low;
        std::uint64_t carry = limbs_[limb] < previous_low ? 1U : 0U;
        std::size_t index = limb + 1U;
        if (index < limbs_.size()) {
            const std::uint64_t previous_high = limbs_[index];
            limbs_[index] += high;
            const bool high_overflow = limbs_[index] < previous_high;
            const std::uint64_t with_carry = limbs_[index] + carry;
            const bool carry_overflow = with_carry < limbs_[index];
            limbs_[index] = with_carry;
            carry = high_overflow || carry_overflow ? 1U : 0U;
            ++index;
        }
        while (carry != 0U && index < limbs_.size()) {
            ++limbs_[index];
            carry = limbs_[index] == 0U ? 1U : 0U;
            ++index;
        }
    }

    // A double spans 2,098 binary places. The extra limbs cover multiplication
    // by ten and the maximum number of members addressable by size_t.
    std::array<std::uint64_t, 35U> limbs_{};
};

[[nodiscard]] bool valid_outcome_class(
    EnsembleMemberOutcomeClass outcome_class) noexcept {
    switch (outcome_class) {
        case EnsembleMemberOutcomeClass::reached:
        case EnsembleMemberOutcomeClass::forecast_exhausted:
        case EnsembleMemberOutcomeClass::duration_exhausted:
        case EnsembleMemberOutcomeClass::infeasible_no_route:
        case EnsembleMemberOutcomeClass::missing_data:
        case EnsembleMemberOutcomeClass::provider_failure:
        case EnsembleMemberOutcomeClass::cancelled:
        case EnsembleMemberOutcomeClass::other_error:
            return true;
    }
    return false;
}

[[nodiscard]] Result<std::vector<EnsembleMemberOutcome>> canonical_outcomes(
    const EnsembleDataset& dataset,
    std::span<const EnsembleMemberOutcome> outcomes,
    std::string_view label) {
    if (outcomes.size() != dataset.member_count()) {
        return invalid_objective(
            std::string{label} +
            " outcomes must contain exactly one outcome per ensemble member");
    }

    std::vector<EnsembleMemberOutcome> canonical;
    canonical.reserve(dataset.member_count());
    for (const EnsembleMemberMetadata& member : dataset.members()) {
        const EnsembleMemberOutcome* match = nullptr;
        for (const EnsembleMemberOutcome& outcome : outcomes) {
            if (outcome.member_identifier.empty()) {
                return invalid_objective(
                    std::string{label} +
                    " outcome member identifier must not be empty");
            }
            if (outcome.member_identifier == member.identifier) {
                if (match != nullptr) {
                    return invalid_objective(
                        std::string{label} + " outcomes contain duplicate member '" +
                        member.identifier + "'");
                }
                match = &outcome;
            }
        }
        if (match == nullptr) {
            return invalid_objective(
                std::string{label} + " outcomes are missing ensemble member '" +
                member.identifier + "'");
        }
        canonical.push_back(*match);
    }

    for (const EnsembleMemberOutcome& outcome : outcomes) {
        const auto known = std::find_if(
            dataset.members().begin(),
            dataset.members().end(),
            [&outcome](const EnsembleMemberMetadata& member) {
                return member.identifier == outcome.member_identifier;
            });
        if (known == dataset.members().end()) {
            return invalid_objective(
                std::string{label} + " outcome references unknown member '" +
                outcome.member_identifier + "'");
        }
        if (!valid_outcome_class(outcome.outcome_class)) {
            return invalid_objective(
                std::string{label} + " outcome for member '" +
                outcome.member_identifier + "' has an unknown outcome class");
        }
        const bool reached =
            outcome.outcome_class == EnsembleMemberOutcomeClass::reached;
        if (reached != outcome.elapsed_arrival_seconds.has_value()) {
            return invalid_objective(
                std::string{label} + " outcome for member '" +
                outcome.member_identifier +
                (reached
                     ? "' must provide an elapsed arrival"
                     : "' must not provide an elapsed arrival"));
        }
        if (outcome.elapsed_arrival_seconds &&
            (!std::isfinite(*outcome.elapsed_arrival_seconds) ||
             *outcome.elapsed_arrival_seconds < 0.0)) {
            return invalid_objective(
                std::string{label} + " outcome for member '" +
                outcome.member_identifier +
                "' must have a finite, non-negative elapsed arrival");
        }
    }
    return canonical;
}

[[nodiscard]] Result<std::optional<std::vector<EnsembleMemberOutcome>>>
canonical_rival(
    const EnsembleDataset& dataset,
    const EnsembleObjective& objective) {
    switch (objective.kind) {
        case EnsembleObjectiveKind::weighted_mean_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p75_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p90_elapsed_arrival:
            if (objective.target || !objective.rival_outcomes.empty()) {
                return invalid_objective(
                    "arrival objectives do not accept target or rival inputs");
            }
            return std::optional<std::vector<EnsembleMemberOutcome>>{};
        case EnsembleObjectiveKind::probability_before_target:
            if (!objective.target) {
                return invalid_objective(
                    "probability_before_target requires a target");
            }
            if (!std::isfinite(objective.target->elapsed_seconds) ||
                objective.target->elapsed_seconds < 0.0) {
                return invalid_objective(
                    "probability_before_target target must be finite and non-negative");
            }
            if (!objective.rival_outcomes.empty()) {
                return invalid_objective(
                    "probability_before_target does not accept rival outcomes");
            }
            return std::optional<std::vector<EnsembleMemberOutcome>>{};
        case EnsembleObjectiveKind::probability_beating_rival: {
            if (objective.target) {
                return invalid_objective(
                    "probability_beating_rival does not accept a target");
            }
            if (objective.rival_outcomes.empty()) {
                return invalid_objective(
                    "probability_beating_rival requires rival outcomes");
            }
            auto rival = canonical_outcomes(
                dataset,
                objective.rival_outcomes,
                "rival");
            if (!rival) {
                return rival.error();
            }
            return std::optional<std::vector<EnsembleMemberOutcome>>{
                std::move(rival.value())};
        }
    }
    return invalid_objective("unknown ensemble objective kind");
}

[[nodiscard]] double rival_score(
    const EnsembleMemberOutcome& candidate,
    const EnsembleMemberOutcome& rival) noexcept {
    const bool candidate_reached =
        candidate.outcome_class == EnsembleMemberOutcomeClass::reached;
    const bool rival_reached =
        rival.outcome_class == EnsembleMemberOutcomeClass::reached;
    if (candidate_reached && rival_reached) {
        if (*candidate.elapsed_arrival_seconds < *rival.elapsed_arrival_seconds) {
            return 1.0;
        }
        if (*candidate.elapsed_arrival_seconds ==
            *rival.elapsed_arrival_seconds) {
            return 0.5;
        }
        return 0.0;
    }
    if (candidate_reached) {
        return 1.0;
    }
    if (rival_reached) {
        return 0.0;
    }
    if (candidate.outcome_class == rival.outcome_class) {
        return 0.5;
    }
    return static_cast<std::uint8_t>(candidate.outcome_class) <
            static_cast<std::uint8_t>(rival.outcome_class)
        ? 1.0
        : 0.0;
}

[[nodiscard]] int compare_value(
    const EnsembleObjectiveValue& left,
    const EnsembleObjectiveValue& right) noexcept {
    if (left.value_class != right.value_class) {
        return left.is_finite() ? -1 : 1;
    }
    if (left.is_positive_infinity()) {
        return 0;
    }
    if (left.finite_value < right.finite_value) {
        return -1;
    }
    if (left.finite_value > right.finite_value) {
        return 1;
    }
    return 0;
}

template <typename T>
[[nodiscard]] int compare_scalar(const T& left, const T& right) noexcept {
    if (left < right) {
        return -1;
    }
    if (right < left) {
        return 1;
    }
    return 0;
}

}  // namespace

std::string_view to_string(EnsembleObjectiveKind kind) noexcept {
    switch (kind) {
        case EnsembleObjectiveKind::weighted_mean_elapsed_arrival:
            return "weighted_mean_elapsed_arrival";
        case EnsembleObjectiveKind::weighted_p75_elapsed_arrival:
            return "weighted_p75_elapsed_arrival";
        case EnsembleObjectiveKind::weighted_p90_elapsed_arrival:
            return "weighted_p90_elapsed_arrival";
        case EnsembleObjectiveKind::probability_before_target:
            return "probability_before_target";
        case EnsembleObjectiveKind::probability_beating_rival:
            return "probability_beating_rival";
    }
    return "unknown";
}

std::string_view to_string(EnsembleObjectiveDirection direction) noexcept {
    switch (direction) {
        case EnsembleObjectiveDirection::minimize: return "minimize";
        case EnsembleObjectiveDirection::maximize: return "maximize";
    }
    return "unknown";
}

EnsembleObjectiveDirection objective_direction(EnsembleObjectiveKind kind) noexcept {
    switch (kind) {
        case EnsembleObjectiveKind::weighted_mean_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p75_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p90_elapsed_arrival:
            return EnsembleObjectiveDirection::minimize;
        case EnsembleObjectiveKind::probability_before_target:
        case EnsembleObjectiveKind::probability_beating_rival:
            return EnsembleObjectiveDirection::maximize;
    }
    return EnsembleObjectiveDirection::minimize;
}

std::string_view to_string(
    EnsembleMemberOutcomeClass outcome_class) noexcept {
    switch (outcome_class) {
        case EnsembleMemberOutcomeClass::reached: return "reached";
        case EnsembleMemberOutcomeClass::forecast_exhausted:
            return "forecast_exhausted";
        case EnsembleMemberOutcomeClass::duration_exhausted:
            return "duration_exhausted";
        case EnsembleMemberOutcomeClass::infeasible_no_route:
            return "infeasible_no_route";
        case EnsembleMemberOutcomeClass::missing_data: return "missing_data";
        case EnsembleMemberOutcomeClass::provider_failure:
            return "provider_failure";
        case EnsembleMemberOutcomeClass::cancelled: return "cancelled";
        case EnsembleMemberOutcomeClass::other_error: return "other_error";
    }
    return "unknown";
}

std::string_view to_string(
    EnsembleObjectiveValueClass value_class) noexcept {
    switch (value_class) {
        case EnsembleObjectiveValueClass::finite: return "finite";
        case EnsembleObjectiveValueClass::positive_infinity:
            return "positive_infinity";
    }
    return "unknown";
}

namespace detail {

Result<EnsembleObjectiveEvaluation> evaluate_ensemble_objective(
    const EnsembleDataset& dataset,
    const EnsembleObjective& objective,
    std::span<const EnsembleMemberOutcome> candidate_outcomes,
    EnsembleObjectiveTieBreakInputs tie_break) {
    if (dataset.member_count() == 0U) {
        return invalid_objective(
            "objective evaluation requires a non-empty ensemble dataset");
    }

    long double total_original_weight = 0.0L;
    double maximum_original_weight = 0.0;
    ExactWeightSum exact_total_weight;
    std::string previous_identifier;
    for (const EnsembleMemberMetadata& member : dataset.members()) {
        if (member.identifier.empty() ||
            (!previous_identifier.empty() &&
             member.identifier <= previous_identifier)) {
            return invalid_objective(
                "ensemble members must have unique identifiers in canonical order");
        }
        if (!std::isfinite(member.normalized_weight) ||
            member.normalized_weight < 0.0) {
            return invalid_objective(
                "ensemble normalized weights must be finite and non-negative");
        }
        if (!std::isfinite(member.original_weight) ||
            member.original_weight < 0.0) {
            return invalid_objective(
                "ensemble original weights must be finite and non-negative");
        }
        total_original_weight +=
            static_cast<long double>(member.original_weight);
        maximum_original_weight =
            std::max(maximum_original_weight, member.original_weight);
        exact_total_weight.add(member.original_weight);
        previous_identifier = member.identifier;
    }
    if (maximum_original_weight <= 0.0) {
        return invalid_objective(
            "ensemble original weights must have a positive finite total");
    }
    std::vector<long double> objective_weights;
    objective_weights.reserve(dataset.member_count());
    if (std::isfinite(total_original_weight)) {
        for (const EnsembleMemberMetadata& member : dataset.members()) {
            objective_weights.push_back(
                static_cast<long double>(member.original_weight));
        }
    } else {
        total_original_weight = 0.0L;
        for (const EnsembleMemberMetadata& member : dataset.members()) {
            const long double scaled =
                static_cast<long double>(member.original_weight) /
                static_cast<long double>(maximum_original_weight);
            objective_weights.push_back(scaled);
            total_original_weight += scaled;
        }
    }
    if (!std::isfinite(total_original_weight) ||
        total_original_weight <= 0.0L) {
        return invalid_objective(
            "ensemble original weights must have a positive finite total");
    }

    auto candidates = canonical_outcomes(
        dataset,
        candidate_outcomes,
        "candidate");
    if (!candidates) {
        return candidates.error();
    }
    auto rival = canonical_rival(dataset, objective);
    if (!rival) {
        return rival.error();
    }

    EnsembleObjectiveEvaluation evaluation;
    evaluation.diagnostics.generated_states = tie_break.generated_states;
    evaluation.diagnostics.settled_states = tie_break.settled_states;
    evaluation.diagnostics.canonical_action_sequence_identity =
        std::move(tie_break.canonical_action_sequence_identity);
    evaluation.diagnostics.members.reserve(dataset.member_count());

    long double finite_weight = 0.0L;
    double finite_mean = 0.0;
    double worst_finite = 0.0;
    bool has_finite = false;
    long double incomplete_weight = 0.0L;
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        const long double objective_weight =
            objective_weights[index] / total_original_weight;
        const double weight = static_cast<double>(objective_weight);
        const EnsembleMemberOutcome& candidate = candidates.value()[index];
        const bool reached =
            candidate.outcome_class == EnsembleMemberOutcomeClass::reached;
        EnsembleObjectiveMemberDiagnostic member;
        member.normalized_weight = weight;
        member.candidate = candidate;
        member.elapsed_arrival = reached
            ? finite_value(*candidate.elapsed_arrival_seconds)
            : positive_infinity();
        if (rival.value()) {
            member.rival = (*rival.value())[index];
        }

        if (weight > 0.0) {
            if (reached) {
                const double elapsed = *candidate.elapsed_arrival_seconds;
                const long double next_finite_weight =
                    finite_weight + objective_weight;
                finite_mean +=
                    static_cast<double>(objective_weight / next_finite_weight) *
                    (elapsed - finite_mean);
                finite_weight = next_finite_weight;
                worst_finite = has_finite
                    ? std::max(worst_finite, elapsed)
                    : elapsed;
                has_finite = true;
            } else {
                incomplete_weight += objective_weight;
            }
        }
        evaluation.diagnostics.members.push_back(std::move(member));
    }
    evaluation.diagnostics.incomplete_member_weight =
        static_cast<double>(incomplete_weight);

    if (has_finite) {
        evaluation.diagnostics.weighted_finite_mean_arrival =
            std::isfinite(finite_mean)
            ? finite_value(finite_mean)
            : positive_infinity();
        evaluation.diagnostics.worst_finite_arrival =
            finite_value(worst_finite);
    } else {
        evaluation.diagnostics.weighted_finite_mean_arrival =
            positive_infinity();
        evaluation.diagnostics.worst_finite_arrival = positive_infinity();
    }

    if (objective.kind ==
        EnsembleObjectiveKind::weighted_mean_elapsed_arrival) {
        for (std::size_t index = 0U;
             index < evaluation.diagnostics.members.size();
             ++index) {
            EnsembleObjectiveMemberDiagnostic& member =
                evaluation.diagnostics.members[index];
            const long double original_weight = objective_weights[index];
            if (original_weight == 0.0L ||
                member.candidate.outcome_class !=
                    EnsembleMemberOutcomeClass::reached) {
                continue;
            }
            member.weighted_contribution =
                static_cast<double>(
                    original_weight / total_original_weight) *
                *member.candidate.elapsed_arrival_seconds;
        }
        if (evaluation.diagnostics.incomplete_member_weight > 0.0) {
            evaluation.value = positive_infinity();
            return evaluation;
        }
        evaluation.value =
            evaluation.diagnostics.weighted_finite_mean_arrival;
        return evaluation;
    }

    if (is_quantile_objective(objective.kind)) {
        std::vector<std::size_t> order;
        order.reserve(dataset.member_count());
        for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
            if (objective_weights[index] > 0.0L) {
                order.push_back(index);
            }
        }
        std::stable_sort(
            order.begin(),
            order.end(),
            [&evaluation](std::size_t left, std::size_t right) {
                const EnsembleObjectiveValue& left_value =
                    evaluation.diagnostics.members[left].elapsed_arrival;
                const EnsembleObjectiveValue& right_value =
                    evaluation.diagnostics.members[right].elapsed_arrival;
                return compare_value(left_value, right_value) < 0;
            });
        const unsigned threshold_numerator =
            objective.kind ==
                EnsembleObjectiveKind::weighted_p75_elapsed_arrival
            ? 3U
            : 9U;
        const unsigned threshold_denominator =
            objective.kind ==
                EnsembleObjectiveKind::weighted_p75_elapsed_arrival
            ? 4U
            : 10U;
        const ExactWeightSum exact_threshold =
            exact_total_weight.multiplied(threshold_numerator);
        ExactWeightSum exact_cumulative;
        std::size_t selected = order.back();
        for (const std::size_t index : order) {
            exact_cumulative.add(dataset.members()[index].original_weight);
            if (exact_cumulative.multiplied(threshold_denominator).compare(
                    exact_threshold) >= 0) {
                selected = index;
                break;
            }
        }
        evaluation.diagnostics.members[selected].selected_quantile_member = true;
        evaluation.value =
            evaluation.diagnostics.members[selected].elapsed_arrival;
        return evaluation;
    }

    long double probability = 0.0L;
    for (std::size_t index = 0U;
         index < evaluation.diagnostics.members.size();
         ++index) {
        EnsembleObjectiveMemberDiagnostic& member =
            evaluation.diagnostics.members[index];
        if (objective.kind ==
            EnsembleObjectiveKind::probability_before_target) {
            member.probability_score =
                member.candidate.outcome_class ==
                        EnsembleMemberOutcomeClass::reached &&
                    *member.candidate.elapsed_arrival_seconds <
                        objective.target->elapsed_seconds
                ? 1.0
                : 0.0;
        } else {
            member.probability_score = rival_score(
                member.candidate,
                *member.rival);
        }
        const long double original_weight = objective_weights[index];
        member.weighted_contribution = static_cast<double>(
            original_weight / total_original_weight) *
            member.probability_score;
        probability +=
            original_weight *
            static_cast<long double>(member.probability_score);
    }
    evaluation.value = finite_value(
        static_cast<double>(probability / total_original_weight));
    return evaluation;
}

int compare_ensemble_objective_evaluations(
    EnsembleObjectiveKind kind,
    const EnsembleObjectiveEvaluation& left,
    const EnsembleObjectiveEvaluation& right) noexcept {
    int comparison = compare_value(left.value, right.value);
    if (objective_direction(kind) == EnsembleObjectiveDirection::maximize) {
        comparison = -comparison;
    }
    if (comparison != 0) {
        return comparison;
    }
    comparison = compare_scalar(
        left.diagnostics.incomplete_member_weight,
        right.diagnostics.incomplete_member_weight);
    if (comparison != 0) {
        return comparison;
    }
    comparison = compare_value(
        left.diagnostics.weighted_finite_mean_arrival,
        right.diagnostics.weighted_finite_mean_arrival);
    if (comparison != 0) {
        return comparison;
    }
    comparison = compare_value(
        left.diagnostics.worst_finite_arrival,
        right.diagnostics.worst_finite_arrival);
    if (comparison != 0) {
        return comparison;
    }
    comparison = compare_scalar(
        left.diagnostics.generated_states,
        right.diagnostics.generated_states);
    if (comparison != 0) {
        return comparison;
    }
    comparison = compare_scalar(
        left.diagnostics.settled_states,
        right.diagnostics.settled_states);
    if (comparison != 0) {
        return comparison;
    }
    return compare_scalar(
        left.diagnostics.canonical_action_sequence_identity,
        right.diagnostics.canonical_action_sequence_identity);
}

}  // namespace detail
}  // namespace sailroute
