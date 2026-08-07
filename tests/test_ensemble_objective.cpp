#include "sailroute/ensemble.hpp"

#include "ensemble/objective.hpp"
#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <chrono>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using sailroute::EnsembleDataset;
using sailroute::EnsembleMemberInput;
using sailroute::EnsembleMemberOutcome;
using sailroute::EnsembleMemberOutcomeClass;
using sailroute::EnsembleObjective;
using sailroute::EnsembleObjectiveDirection;
using sailroute::EnsembleObjectiveEvaluation;
using sailroute::EnsembleObjectiveKind;
using sailroute::EnsembleObjectiveValue;
using sailroute::EnsembleObjectiveValueClass;
using sailroute::EnsembleRunMetadata;
using sailroute::Error;
using sailroute::ErrorCode;
using sailroute::detail::EnsembleObjectiveTieBreakInputs;
using sailroute::detail::compare_ensemble_objective_evaluations;
using sailroute::detail::evaluate_ensemble_objective;
using sailroute::test::ConstantWindGribFixture;

EnsembleRunMetadata run_metadata() {
    return EnsembleRunMetadata{
        "objective-cycle",
        "objective-model",
        sailroute::TimePoint{std::chrono::seconds{1'784'035'200}},
        "generated objective fixture",
        1U};
}

EnsembleDataset dataset_from_weights(
    const std::vector<std::pair<std::string, double>>& members) {
    const ConstantWindGribFixture fixture;
    std::vector<EnsembleMemberInput> inputs;
    inputs.reserve(members.size());
    for (const auto& [identifier, weight] : members) {
        EnsembleMemberInput input;
        input.identifier = identifier;
        input.weight = weight;
        input.grib_path = fixture.path();
        inputs.push_back(std::move(input));
    }
    auto dataset = EnsembleDataset::load(run_metadata(), std::move(inputs));
    REQUIRE(dataset.has_value());
    return std::move(dataset.value());
}

EnsembleMemberOutcome reached(std::string identifier, double elapsed) {
    return EnsembleMemberOutcome{
        std::move(identifier),
        EnsembleMemberOutcomeClass::reached,
        elapsed,
        std::nullopt};
}

EnsembleMemberOutcome incomplete(
    std::string identifier,
    EnsembleMemberOutcomeClass outcome_class,
    std::optional<Error> error = std::nullopt) {
    return EnsembleMemberOutcome{
        std::move(identifier),
        outcome_class,
        std::nullopt,
        std::move(error)};
}

EnsembleObjective objective(EnsembleObjectiveKind kind) {
    EnsembleObjective result;
    result.kind = kind;
    return result;
}

EnsembleObjectiveValue finite(double value) {
    return EnsembleObjectiveValue{
        EnsembleObjectiveValueClass::finite,
        value};
}

EnsembleObjectiveValue infinity() {
    return EnsembleObjectiveValue{
        EnsembleObjectiveValueClass::positive_infinity,
        0.0};
}

EnsembleObjectiveEvaluation comparison_evaluation(
    EnsembleObjectiveValue value,
    double incomplete_weight,
    EnsembleObjectiveValue finite_mean,
    EnsembleObjectiveValue worst,
    std::size_t generated,
    std::size_t settled,
    std::string identity) {
    EnsembleObjectiveEvaluation evaluation;
    evaluation.value = value;
    evaluation.diagnostics.incomplete_member_weight = incomplete_weight;
    evaluation.diagnostics.weighted_finite_mean_arrival = finite_mean;
    evaluation.diagnostics.worst_finite_arrival = worst;
    evaluation.diagnostics.generated_states = generated;
    evaluation.diagnostics.settled_states = settled;
    evaluation.diagnostics.canonical_action_sequence_identity =
        std::move(identity);
    return evaluation;
}

}  // namespace

TEST_CASE("ensemble objective identifiers directions and value classes are stable") {
    REQUIRE(
        sailroute::to_string(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival) ==
        "weighted_mean_elapsed_arrival");
    REQUIRE(
        sailroute::to_string(
            EnsembleObjectiveKind::weighted_p75_elapsed_arrival) ==
        "weighted_p75_elapsed_arrival");
    REQUIRE(
        sailroute::to_string(
            EnsembleObjectiveKind::weighted_p90_elapsed_arrival) ==
        "weighted_p90_elapsed_arrival");
    REQUIRE(
        sailroute::to_string(
            EnsembleObjectiveKind::probability_before_target) ==
        "probability_before_target");
    REQUIRE(
        sailroute::to_string(
            EnsembleObjectiveKind::probability_beating_rival) ==
        "probability_beating_rival");
    REQUIRE(
        sailroute::objective_direction(
            EnsembleObjectiveKind::weighted_p90_elapsed_arrival) ==
        EnsembleObjectiveDirection::minimize);
    REQUIRE(
        sailroute::objective_direction(
            EnsembleObjectiveKind::probability_before_target) ==
        EnsembleObjectiveDirection::maximize);
    REQUIRE(
        sailroute::to_string(EnsembleObjectiveDirection::maximize) ==
        "maximize");
    REQUIRE(
        sailroute::to_string(
            EnsembleObjectiveValueClass::positive_infinity) ==
        "positive_infinity");
    REQUIRE(
        sailroute::to_string(EnsembleMemberOutcomeClass::infeasible_no_route) ==
        "infeasible_no_route");
    REQUIRE(
        sailroute::to_string(EnsembleMemberOutcomeClass::provider_failure) ==
        "provider_failure");
}

TEST_CASE("weighted mean is canonical and zero-weight incompletion is ignored") {
    const EnsembleDataset dataset = dataset_from_weights(
        {{"z-zero", 0.0}, {"charlie", 1.0}, {"bravo", 2.0}, {"alpha", 1.0}});
    const std::vector<EnsembleMemberOutcome> shuffled{
        reached("charlie", 30.0),
        incomplete("z-zero", EnsembleMemberOutcomeClass::provider_failure),
        reached("alpha", 10.0),
        reached("bravo", 20.0),
    };
    const auto evaluation = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
        shuffled,
        EnsembleObjectiveTieBreakInputs{91U, 37U, "action/002"});
    REQUIRE(evaluation.has_value());
    REQUIRE(evaluation.value().value.is_finite());
    REQUIRE_NEAR(evaluation.value().value.finite_value, 20.0, 1e-12);
    REQUIRE(evaluation.value().diagnostics.incomplete_member_weight == 0.0);
    REQUIRE(
        evaluation.value().diagnostics.members[0].candidate.member_identifier ==
        "alpha");
    REQUIRE(
        evaluation.value().diagnostics.members[3].candidate.member_identifier ==
        "z-zero");
    REQUIRE(
        evaluation.value().diagnostics.members[3].elapsed_arrival.
            is_positive_infinity());
    REQUIRE(evaluation.value().diagnostics.generated_states == 91U);
    REQUIRE(evaluation.value().diagnostics.settled_states == 37U);
    REQUIRE(
        evaluation.value().diagnostics.canonical_action_sequence_identity ==
        "action/002");
}

TEST_CASE("positive-weight incompletion makes weighted mean explicit infinity") {
    const EnsembleDataset dataset =
        dataset_from_weights({{"alpha", 1.0}, {"bravo", 2.0}, {"charlie", 1.0}});
    const std::vector<EnsembleMemberOutcome> outcomes{
        reached("bravo", 20.0),
        reached("alpha", 10.0),
        incomplete(
            "charlie",
            EnsembleMemberOutcomeClass::forecast_exhausted,
            Error{ErrorCode::forecast_exhausted, "member horizon ended"}),
    };
    const auto evaluation = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
        outcomes);
    REQUIRE(evaluation.has_value());
    REQUIRE(evaluation.value().value.is_positive_infinity());
    REQUIRE_NEAR(
        evaluation.value().diagnostics.incomplete_member_weight,
        0.25,
        1e-15);
    REQUIRE_NEAR(
        evaluation.value().diagnostics.weighted_finite_mean_arrival.finite_value,
        50.0 / 3.0,
        1e-12);
    REQUIRE(
        evaluation.value().diagnostics.worst_finite_arrival.finite_value ==
        20.0);
    REQUIRE(
        evaluation.value().diagnostics.members[2].candidate.error->message ==
        "member horizon ended");
    REQUIRE_NEAR(
        evaluation.value().diagnostics.members[0].weighted_contribution,
        2.5,
        1e-15);
    REQUIRE_NEAR(
        evaluation.value().diagnostics.members[1].weighted_contribution,
        10.0,
        1e-15);
}

TEST_CASE("weighted nearest-rank P75 and P90 use canonical infinity ordering") {
    const EnsembleDataset dataset = dataset_from_weights(
        {{"zero", 0.0}, {"charlie", 1.0}, {"alpha", 1.0}, {"bravo", 2.0}});
    const std::vector<EnsembleMemberOutcome> outcomes{
        reached("charlie", 30.0),
        incomplete("zero", EnsembleMemberOutcomeClass::other_error),
        reached("bravo", 20.0),
        reached("alpha", 10.0),
    };
    const auto p75 = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_p75_elapsed_arrival),
        outcomes);
    const auto p90 = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_p90_elapsed_arrival),
        outcomes);
    REQUIRE(p75.has_value());
    REQUIRE(p90.has_value());
    REQUIRE(p75.value().value.finite_value == 20.0);
    REQUIRE(p90.value().value.finite_value == 30.0);
    REQUIRE(p75.value().diagnostics.members[1].selected_quantile_member);
    REQUIRE(p90.value().diagnostics.members[2].selected_quantile_member);

    std::vector<EnsembleMemberOutcome> incomplete_tail = outcomes;
    incomplete_tail[0] = incomplete(
        "charlie",
        EnsembleMemberOutcomeClass::duration_exhausted);
    const auto boundary = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_p75_elapsed_arrival),
        incomplete_tail);
    const auto infinite_tail = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_p90_elapsed_arrival),
        incomplete_tail);
    REQUIRE(boundary.has_value());
    REQUIRE(infinite_tail.has_value());
    REQUIRE(boundary.value().value.finite_value == 20.0);
    REQUIRE(infinite_tail.value().value.is_positive_infinity());
}

TEST_CASE("objectives preserve exact boundaries with uneven weights") {
    const EnsembleDataset dataset = dataset_from_weights(
        {{"alpha", 581.0}, {"bravo", 1.0}, {"charlie", 194.0}});
    const std::vector<EnsembleMemberOutcome> all_reached{
        reached("alpha", 10.0),
        reached("bravo", 10.0),
        reached("charlie", 10.0),
    };
    const auto mean = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
        all_reached);
    REQUIRE(mean.has_value());
    REQUIRE(mean.value().value.finite_value == 10.0);

    EnsembleObjective target =
        objective(EnsembleObjectiveKind::probability_before_target);
    target.target = sailroute::EnsembleArrivalTarget{11.0};
    const auto probability =
        evaluate_ensemble_objective(dataset, target, all_reached);
    REQUIRE(probability.has_value());
    REQUIRE(probability.value().value.finite_value == 1.0);

    const auto p75 = evaluate_ensemble_objective(
        dataset,
        objective(EnsembleObjectiveKind::weighted_p75_elapsed_arrival),
        std::vector<EnsembleMemberOutcome>{
            reached("alpha", 10.0),
            reached("bravo", 20.0),
            incomplete(
                "charlie",
                EnsembleMemberOutcomeClass::forecast_exhausted),
        });
    REQUIRE(p75.has_value());
    REQUIRE(p75.value().value.is_finite());
    REQUIRE(p75.value().value.finite_value == 20.0);
}

TEST_CASE("objective means and quantiles avoid intermediate rounding overflow") {
    const double maximum = std::numeric_limits<double>::max();
    const EnsembleDataset mean_dataset =
        dataset_from_weights({{"alpha", maximum}});
    const auto mean = evaluate_ensemble_objective(
        mean_dataset,
        objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
        std::vector<EnsembleMemberOutcome>{reached("alpha", maximum)});
    REQUIRE(mean.has_value());
    REQUIRE(mean.value().value.is_finite());
    REQUIRE(mean.value().value.finite_value == maximum);

    const double unit = std::ldexp(1.0, -51);
    const double tiny = std::ldexp(1.0, -53);
    const EnsembleDataset quantile_dataset = dataset_from_weights(
        {{"alpha", 3.0 - unit},
         {"bravo", tiny},
         {"charlie", tiny},
         {"delta", tiny},
         {"echo", tiny},
         {"zulu", 1.0}});
    const auto p75 = evaluate_ensemble_objective(
        quantile_dataset,
        objective(EnsembleObjectiveKind::weighted_p75_elapsed_arrival),
        std::vector<EnsembleMemberOutcome>{
            reached("alpha", 10.0),
            reached("bravo", 10.0),
            reached("charlie", 10.0),
            reached("delta", 10.0),
            reached("echo", 10.0),
            incomplete(
                "zulu",
                EnsembleMemberOutcomeClass::forecast_exhausted),
        });
    REQUIRE(p75.has_value());
    REQUIRE(p75.value().value.is_finite());
    REQUIRE(p75.value().value.finite_value == 10.0);
}

TEST_CASE("target probability is strict at the target and incomplete is a miss") {
    const EnsembleDataset dataset =
        dataset_from_weights({{"alpha", 1.0}, {"bravo", 2.0}, {"charlie", 1.0}});
    EnsembleObjective target =
        objective(EnsembleObjectiveKind::probability_before_target);
    target.target = sailroute::EnsembleArrivalTarget{20.0};
    const auto evaluation = evaluate_ensemble_objective(
        dataset,
        target,
        std::vector<EnsembleMemberOutcome>{
            incomplete("charlie", EnsembleMemberOutcomeClass::missing_data),
            reached("bravo", 20.0),
            reached("alpha", 19.999),
        });
    REQUIRE(evaluation.has_value());
    REQUIRE_NEAR(evaluation.value().value.finite_value, 0.25, 1e-15);
    REQUIRE(
        evaluation.value().diagnostics.members[0].probability_score == 1.0);
    REQUIRE(
        evaluation.value().diagnostics.members[1].probability_score == 0.0);
    REQUIRE(
        evaluation.value().diagnostics.members[2].probability_score == 0.0);
}

TEST_CASE("rival probability covers arrival and incomplete class ordering") {
    const EnsembleDataset dataset = dataset_from_weights(
        {{"h", 1.0}, {"f", 1.0}, {"d", 1.0}, {"b", 1.0},
         {"g", 1.0}, {"e", 1.0}, {"c", 1.0}, {"a", 1.0}});
    EnsembleObjective rival =
        objective(EnsembleObjectiveKind::probability_beating_rival);
    rival.rival_outcomes = {
        reached("h", 10.0),
        incomplete("g", EnsembleMemberOutcomeClass::forecast_exhausted),
        incomplete("f", EnsembleMemberOutcomeClass::duration_exhausted),
        incomplete("e", EnsembleMemberOutcomeClass::forecast_exhausted),
        incomplete("d", EnsembleMemberOutcomeClass::provider_failure),
        reached("c", 10.0),
        reached("b", 10.0),
        reached("a", 20.0),
    };
    const std::vector<EnsembleMemberOutcome> candidate{
        incomplete("h", EnsembleMemberOutcomeClass::missing_data),
        incomplete("g", EnsembleMemberOutcomeClass::duration_exhausted),
        incomplete(
            "f",
            EnsembleMemberOutcomeClass::forecast_exhausted,
            Error{ErrorCode::forecast_exhausted, "candidate forecast ended"}),
        incomplete("e", EnsembleMemberOutcomeClass::forecast_exhausted),
        reached("d", 10.0),
        reached("c", 20.0),
        reached("b", 10.0),
        reached("a", 10.0),
    };
    const auto evaluation =
        evaluate_ensemble_objective(dataset, rival, candidate);
    REQUIRE(evaluation.has_value());
    REQUIRE_NEAR(evaluation.value().value.finite_value, 0.5, 1e-15);
    REQUIRE(evaluation.value().diagnostics.members[0].probability_score == 1.0);
    REQUIRE(evaluation.value().diagnostics.members[1].probability_score == 0.5);
    REQUIRE(evaluation.value().diagnostics.members[2].probability_score == 0.0);
    REQUIRE(evaluation.value().diagnostics.members[3].probability_score == 1.0);
    REQUIRE(evaluation.value().diagnostics.members[4].probability_score == 0.5);
    REQUIRE(evaluation.value().diagnostics.members[5].probability_score == 1.0);
    REQUIRE(evaluation.value().diagnostics.members[6].probability_score == 0.0);
    REQUIRE(evaluation.value().diagnostics.members[7].probability_score == 0.0);
    REQUIRE(
        evaluation.value().diagnostics.members[5].candidate.error->message ==
        "candidate forecast ended");
}

TEST_CASE("all raw member completion and failure classes remain auditable") {
    const EnsembleDataset dataset = dataset_from_weights(
        {{"reached", 1.0}, {"forecast", 1.0}, {"duration", 1.0},
         {"infeasible", 1.0}, {"missing", 1.0}, {"provider", 1.0},
         {"cancelled", 1.0}, {"other", 1.0}});
    EnsembleObjective target =
        objective(EnsembleObjectiveKind::probability_before_target);
    target.target = sailroute::EnsembleArrivalTarget{100.0};
    const std::vector<EnsembleMemberOutcome> outcomes{
        incomplete(
            "other",
            EnsembleMemberOutcomeClass::other_error,
            Error{ErrorCode::invalid_argument, "other detail"}),
        incomplete(
            "provider",
            EnsembleMemberOutcomeClass::provider_failure,
            Error{ErrorCode::environment_data_unavailable, "provider detail"}),
        reached("reached", 50.0),
        incomplete(
            "cancelled",
            EnsembleMemberOutcomeClass::cancelled,
            Error{ErrorCode::cancelled, "cancel detail"}),
        incomplete(
            "missing",
            EnsembleMemberOutcomeClass::missing_data,
            Error{ErrorCode::environment_data_unavailable, "missing detail"}),
        incomplete(
            "infeasible",
            EnsembleMemberOutcomeClass::infeasible_no_route,
            Error{ErrorCode::no_route, "no route detail"}),
        incomplete("duration", EnsembleMemberOutcomeClass::duration_exhausted),
        incomplete("forecast", EnsembleMemberOutcomeClass::forecast_exhausted),
    };
    const auto evaluation =
        evaluate_ensemble_objective(dataset, target, outcomes);
    REQUIRE(evaluation.has_value());
    REQUIRE(evaluation.value().diagnostics.members.size() == 8U);
    const std::vector<EnsembleMemberOutcomeClass> expected{
        EnsembleMemberOutcomeClass::cancelled,
        EnsembleMemberOutcomeClass::duration_exhausted,
        EnsembleMemberOutcomeClass::forecast_exhausted,
        EnsembleMemberOutcomeClass::infeasible_no_route,
        EnsembleMemberOutcomeClass::missing_data,
        EnsembleMemberOutcomeClass::other_error,
        EnsembleMemberOutcomeClass::provider_failure,
        EnsembleMemberOutcomeClass::reached,
    };
    for (std::size_t index = 0U; index < expected.size(); ++index) {
        REQUIRE(
            evaluation.value().diagnostics.members[index].
                candidate.outcome_class == expected[index]);
    }
    REQUIRE(
        evaluation.value().diagnostics.members[6].candidate.error->message ==
        "provider detail");
}

TEST_CASE("objective evaluation accepts boundary normalized weights") {
    const double maximum = std::numeric_limits<double>::max();
    const EnsembleDataset dataset = dataset_from_weights(
        {{"zero", 0.0}, {"bravo", maximum}, {"alpha", maximum}});
    EnsembleObjective target =
        objective(EnsembleObjectiveKind::probability_before_target);
    target.target = sailroute::EnsembleArrivalTarget{20.0};
    const auto evaluation = evaluate_ensemble_objective(
        dataset,
        target,
        std::vector<EnsembleMemberOutcome>{
            incomplete("zero", EnsembleMemberOutcomeClass::other_error),
            reached("bravo", 20.0),
            reached("alpha", 10.0),
        });
    REQUIRE(evaluation.has_value());
    REQUIRE(evaluation.value().value.finite_value == 0.5);
}

TEST_CASE("objective validation rejects invalid combinations and outcomes") {
    const EnsembleDataset dataset =
        dataset_from_weights({{"alpha", 1.0}, {"bravo", 1.0}});
    const std::vector<EnsembleMemberOutcome> valid{
        reached("alpha", 10.0),
        reached("bravo", 20.0),
    };

    EnsembleObjective target =
        objective(EnsembleObjectiveKind::probability_before_target);
    REQUIRE(!evaluate_ensemble_objective(dataset, target, valid).has_value());
    target.target = sailroute::EnsembleArrivalTarget{
        std::numeric_limits<double>::infinity()};
    REQUIRE(!evaluate_ensemble_objective(dataset, target, valid).has_value());
    target.target = sailroute::EnsembleArrivalTarget{
        std::numeric_limits<double>::quiet_NaN()};
    REQUIRE(!evaluate_ensemble_objective(dataset, target, valid).has_value());
    target.target = sailroute::EnsembleArrivalTarget{-1.0};
    REQUIRE(!evaluate_ensemble_objective(dataset, target, valid).has_value());

    EnsembleObjective mean =
        objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival);
    mean.target = sailroute::EnsembleArrivalTarget{10.0};
    REQUIRE(!evaluate_ensemble_objective(dataset, mean, valid).has_value());
    mean.target.reset();
    mean.rival_outcomes = valid;
    REQUIRE(!evaluate_ensemble_objective(dataset, mean, valid).has_value());

    EnsembleObjective rival =
        objective(EnsembleObjectiveKind::probability_beating_rival);
    REQUIRE(!evaluate_ensemble_objective(dataset, rival, valid).has_value());
    rival.target = sailroute::EnsembleArrivalTarget{10.0};
    rival.rival_outcomes = valid;
    REQUIRE(!evaluate_ensemble_objective(dataset, rival, valid).has_value());
    rival.target.reset();
    rival.rival_outcomes = {reached("alpha", 10.0)};
    REQUIRE(!evaluate_ensemble_objective(dataset, rival, valid).has_value());
    rival.rival_outcomes = {
        reached("alpha", 10.0),
        reached("alpha", 11.0),
    };
    REQUIRE(!evaluate_ensemble_objective(dataset, rival, valid).has_value());

    const std::vector<EnsembleMemberOutcome> missing{
        reached("alpha", 10.0)};
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_p75_elapsed_arrival),
             missing)
             .has_value());
    const std::vector<EnsembleMemberOutcome> duplicate{
        reached("alpha", 10.0),
        reached("alpha", 11.0),
    };
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_p90_elapsed_arrival),
             duplicate)
             .has_value());
    const std::vector<EnsembleMemberOutcome> unknown{
        reached("alpha", 10.0),
        reached("unknown", 20.0),
    };
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_p90_elapsed_arrival),
             unknown)
             .has_value());
    const std::vector<EnsembleMemberOutcome> empty_identifier{
        reached("", 10.0),
        reached("bravo", 20.0),
    };
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
             empty_identifier)
             .has_value());
    const std::vector<EnsembleMemberOutcome> non_finite{
        reached("alpha", std::numeric_limits<double>::infinity()),
        reached("bravo", 20.0),
    };
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
             non_finite)
             .has_value());
    const std::vector<EnsembleMemberOutcome> incomplete_with_arrival{
        EnsembleMemberOutcome{
            "alpha",
            EnsembleMemberOutcomeClass::forecast_exhausted,
            10.0,
            std::nullopt},
        reached("bravo", 20.0),
    };
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
             incomplete_with_arrival)
             .has_value());
    const std::vector<EnsembleMemberOutcome> reached_without_arrival{
        incomplete("alpha", EnsembleMemberOutcomeClass::reached),
        reached("bravo", 20.0),
    };
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
             reached_without_arrival)
             .has_value());
    const std::vector<EnsembleMemberOutcome> unknown_class{
        EnsembleMemberOutcome{
            "alpha",
            static_cast<EnsembleMemberOutcomeClass>(255U),
            std::nullopt,
            std::nullopt},
        reached("bravo", 20.0),
    };
    REQUIRE(
        !evaluate_ensemble_objective(
             dataset,
             objective(EnsembleObjectiveKind::weighted_mean_elapsed_arrival),
             unknown_class)
             .has_value());
}

TEST_CASE("objective comparator applies primary directions and every tie break") {
    const auto baseline = comparison_evaluation(
        finite(10.0), 0.1, finite(12.0), finite(20.0), 100U, 50U, "b");
    auto other = baseline;
    other.value = finite(20.0);
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            other) < 0);
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::probability_before_target,
            baseline,
            other) > 0);

    other = baseline;
    other.value = infinity();
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_p90_elapsed_arrival,
            baseline,
            other) < 0);

    other = baseline;
    other.diagnostics.incomplete_member_weight = 0.2;
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            other) < 0);

    other = baseline;
    other.diagnostics.weighted_finite_mean_arrival = finite(13.0);
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            other) < 0);

    other = baseline;
    other.diagnostics.worst_finite_arrival = finite(21.0);
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            other) < 0);

    other = baseline;
    other.diagnostics.generated_states = 101U;
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            other) < 0);

    other = baseline;
    other.diagnostics.settled_states = 51U;
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            other) < 0);

    other = baseline;
    other.diagnostics.canonical_action_sequence_identity = "c";
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            other) < 0);
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            baseline,
            baseline) == 0);

    const auto no_completions = comparison_evaluation(
        infinity(), 1.0, infinity(), infinity(), 10U, 10U, "a");
    other = no_completions;
    other.diagnostics.generated_states = 11U;
    REQUIRE(
        compare_ensemble_objective_evaluations(
            EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
            no_completions,
            other) < 0);
}
