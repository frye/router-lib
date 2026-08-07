#include "sailroute/ensemble.hpp"

#include "ensemble/policy.hpp"
#include "ensemble/search_state.hpp"
#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace {

using namespace std::chrono_literals;
using sailroute::Coordinate;
using sailroute::EnsembleDataset;
using sailroute::EnsembleMemberInput;
using sailroute::EnsembleMemberOutcomeClass;
using sailroute::EnsembleObjective;
using sailroute::EnsembleObjectiveKind;
using sailroute::EnsemblePolicyOptions;
using sailroute::EnsembleRunMetadata;
using sailroute::RoutePoint;
using sailroute::TimePoint;
using sailroute::detail::EnsembleCommonAction;
using sailroute::detail::EnsembleMemberSearchStatus;
using sailroute::detail::EnsembleSearchLabel;
using sailroute::detail::OperationalConfiguration;
using sailroute::test::ConstantWindGribFixture;

constexpr std::chrono::seconds fixture_epoch{1'784'035'200};

EnsembleRunMetadata metadata(std::string run) {
    return EnsembleRunMetadata{
        std::move(run),
        "policy-model",
        TimePoint{fixture_epoch},
        "generated policy fixture",
        1U};
}

EnsembleMemberInput input(
    std::string identifier,
    const ConstantWindGribFixture& fixture,
    double weight = 1.0) {
    EnsembleMemberInput result;
    result.identifier = std::move(identifier);
    result.weight = weight;
    result.grib_path = fixture.path();
    return result;
}

EnsembleSearchLabel child(
    const EnsembleSearchLabel& parent,
    std::size_t parent_index,
    EnsembleCommonAction action,
    Coordinate position,
    std::chrono::seconds alpha_elapsed,
    std::chrono::seconds zulu_elapsed,
    bool terminal) {
    EnsembleSearchLabel result = parent;
    result.parent_label = parent_index;
    result.incoming_action = action;
    result.common_action_history.push_back(action);
    const std::chrono::seconds elapsed[] = {alpha_elapsed, zulu_elapsed};
    for (std::size_t index = 0U; index < result.members.size(); ++index) {
        result.members[index].point.position = position;
        result.members[index].point.time =
            TimePoint{fixture_epoch} + elapsed[index];
        result.members[index].point.heading_degrees =
            action.heading_degrees;
        result.members[index].status = terminal
            ? EnsembleMemberSearchStatus::completed
            : EnsembleMemberSearchStatus::active;
        result.members[index].outcome_class = terminal
            ? std::optional<EnsembleMemberOutcomeClass>{
                  EnsembleMemberOutcomeClass::reached}
            : std::nullopt;
    }
    sailroute::detail::canonicalize_ensemble_label(result);
    return result;
}

}  // namespace

TEST_CASE("ensemble policy builds stable divergent and reconvergent decisions") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata("cycle-one"),
        {input("zulu", fixture), input("alpha", fixture)});
    REQUIRE(dataset.has_value());

    std::vector<RoutePoint> roots(2U);
    for (RoutePoint& point : roots) {
        point.position = Coordinate{1.0, 0.5};
        point.time = TimePoint{fixture_epoch};
    }
    std::vector<OperationalConfiguration> configurations(2U);
    auto root = sailroute::detail::make_initial_ensemble_label(
        dataset.value(), roots, configurations);
    REQUIRE(root.has_value());

    auto east = sailroute::detail::make_common_heading_action(90.0, 30min);
    auto west = sailroute::detail::make_common_heading_action(270.0, 30min);
    auto finish = sailroute::detail::make_common_heading_action(0.0, 30min);
    REQUIRE(east.has_value());
    REQUIRE(west.has_value());
    REQUIRE(finish.has_value());

    std::vector<EnsembleSearchLabel> labels;
    labels.push_back(root.value());
    labels.push_back(child(
        labels[0],
        0U,
        east.value(),
        Coordinate{1.0, 0.6},
        30min,
        30min,
        false));
    labels.push_back(child(
        labels[0],
        0U,
        west.value(),
        Coordinate{1.0, 0.4},
        30min,
        30min,
        false));
    labels.push_back(child(
        labels[1],
        1U,
        finish.value(),
        Coordinate{1.1, 0.5},
        1h,
        2h,
        true));
    labels.push_back(child(
        labels[2],
        2U,
        finish.value(),
        Coordinate{1.1, 0.5},
        1h,
        2h,
        true));

    const EnsembleObjective objective{
        EnsembleObjectiveKind::weighted_mean_elapsed_arrival, {}, {}};
    REQUIRE(sailroute::detail::evaluate_ensemble_label_objective(
                dataset.value(),
                objective,
                labels[3],
                TimePoint{fixture_epoch})
                .has_value());
    REQUIRE(sailroute::detail::evaluate_ensemble_label_objective(
                dataset.value(),
                objective,
                labels[4],
                TimePoint{fixture_epoch})
                .has_value());

    const std::vector<std::size_t> terminals{4U, 3U};
    auto policy = sailroute::detail::build_ensemble_policy(
        dataset.value(),
        objective,
        EnsemblePolicyOptions{},
        labels,
        terminals,
        3U,
        TimePoint{fixture_epoch});
    REQUIRE(policy.has_value());
    REQUIRE(policy.value().graph.alternatives.size() == 2U);
    REQUIRE(policy.value().graph.nodes.size() == 4U);
    REQUIRE(policy.value().graph.branches.size() == 4U);
    REQUIRE(policy.value().decision_points.size() == 1U);
    REQUIRE(policy.value().decision_points[0].branches.size() == 2U);
    REQUIRE(
        policy.value().re_evaluation.prior_run_identifier == "cycle-one");
    REQUIRE(
        policy.value().re_evaluation.selected_branch_identity ==
        policy.value().graph.alternatives[0].branch_identity);

    double support = 0.0;
    for (const auto& alternative : policy.value().graph.alternatives) {
        support += alternative.supporting_member_weight;
        REQUIRE(alternative.wrong_choice_cost.is_finite());
        REQUIRE(alternative.wrong_choice_cost.finite_value == 0.0);
        REQUIRE(alternative.selected != alternative.requires_re_evaluation);
    }
    REQUIRE(support == 1.0);

    auto second_dataset = EnsembleDataset::load(
        metadata("cycle-two"),
        {input("alpha", fixture), input("zulu", fixture)});
    REQUIRE(second_dataset.has_value());
    auto repeated = sailroute::detail::build_ensemble_policy(
        second_dataset.value(),
        objective,
        EnsemblePolicyOptions{},
        labels,
        terminals,
        3U,
        TimePoint{fixture_epoch});
    REQUIRE(repeated.has_value());
    REQUIRE(
        repeated.value().re_evaluation.canonical_branch_identities ==
        policy.value().re_evaluation.canonical_branch_identities);
    REQUIRE(
        repeated.value().re_evaluation.prior_run_identifier == "cycle-two");
}

TEST_CASE("zero policy alternatives produces no false decision") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata("single-cycle"),
        {input("alpha", fixture), input("zulu", fixture)});
    REQUIRE(dataset.has_value());

    std::vector<RoutePoint> roots(2U);
    for (RoutePoint& point : roots) {
        point.position = Coordinate{1.0, 0.5};
        point.time = TimePoint{fixture_epoch};
    }
    std::vector<OperationalConfiguration> configurations(2U);
    auto root = sailroute::detail::make_initial_ensemble_label(
        dataset.value(), roots, configurations);
    REQUIRE(root.has_value());
    auto action = sailroute::detail::make_common_heading_action(90.0, 30min);
    REQUIRE(action.has_value());
    std::vector<EnsembleSearchLabel> labels;
    labels.push_back(root.value());
    labels.push_back(child(
        labels[0],
        0U,
        action.value(),
        Coordinate{1.0, 0.6},
        30min,
        30min,
        true));
    const EnsembleObjective objective{
        EnsembleObjectiveKind::weighted_mean_elapsed_arrival, {}, {}};
    REQUIRE(sailroute::detail::evaluate_ensemble_label_objective(
                dataset.value(),
                objective,
                labels[1],
                TimePoint{fixture_epoch})
                .has_value());
    const std::vector<std::size_t> terminals{1U};
    EnsemblePolicyOptions options;
    options.max_alternatives = 0U;
    auto policy = sailroute::detail::build_ensemble_policy(
        dataset.value(),
        objective,
        options,
        labels,
        terminals,
        1U,
        TimePoint{fixture_epoch});
    REQUIRE(policy.has_value());
    REQUIRE(policy.value().graph.alternatives.size() == 1U);
    REQUIRE(policy.value().decision_points.empty());
}

TEST_CASE("policy dominance ignores zero-weight members") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata("zero-weight"),
        {input("alpha", fixture, 1.0), input("zulu", fixture, 0.0)});
    REQUIRE(dataset.has_value());

    std::vector<RoutePoint> roots(2U);
    for (RoutePoint& point : roots) {
        point.position = Coordinate{1.0, 0.5};
        point.time = TimePoint{fixture_epoch};
    }
    std::vector<OperationalConfiguration> configurations(2U);
    auto root = sailroute::detail::make_initial_ensemble_label(
        dataset.value(), roots, configurations);
    REQUIRE(root.has_value());
    auto east = sailroute::detail::make_common_heading_action(90.0, 30min);
    auto west = sailroute::detail::make_common_heading_action(270.0, 30min);
    REQUIRE(east.has_value());
    REQUIRE(west.has_value());
    const EnsembleSearchLabel first = child(
        root.value(),
        0U,
        east.value(),
        Coordinate{1.0, 0.6},
        1h,
        2h,
        true);
    const EnsembleSearchLabel second = child(
        root.value(),
        0U,
        west.value(),
        Coordinate{1.0, 0.4},
        1h,
        1h,
        true);
    REQUIRE(!sailroute::detail::policy_terminal_dominates(
        dataset.value(), first, second));
    REQUIRE(!sailroute::detail::policy_terminal_dominates(
        dataset.value(), second, first));
}

TEST_CASE("probability policy support follows member scores") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata("probability-support"),
        {input("alpha", fixture), input("zulu", fixture)});
    REQUIRE(dataset.has_value());

    std::vector<RoutePoint> roots(2U);
    for (RoutePoint& point : roots) {
        point.position = Coordinate{1.0, 0.5};
        point.time = TimePoint{fixture_epoch};
    }
    std::vector<OperationalConfiguration> configurations(2U);
    auto root = sailroute::detail::make_initial_ensemble_label(
        dataset.value(), roots, configurations);
    REQUIRE(root.has_value());
    auto east = sailroute::detail::make_common_heading_action(90.0, 30min);
    auto west = sailroute::detail::make_common_heading_action(270.0, 30min);
    REQUIRE(east.has_value());
    REQUIRE(west.has_value());
    std::vector<EnsembleSearchLabel> labels;
    labels.push_back(root.value());
    labels.push_back(child(
        labels[0],
        0U,
        east.value(),
        Coordinate{1.0, 0.6},
        60min,
        80min,
        true));
    labels.push_back(child(
        labels[0],
        0U,
        west.value(),
        Coordinate{1.0, 0.4},
        70min,
        85min,
        true));
    EnsembleObjective objective;
    objective.kind = EnsembleObjectiveKind::probability_before_target;
    objective.target = sailroute::EnsembleArrivalTarget{5'400.0};
    REQUIRE(sailroute::detail::evaluate_ensemble_label_objective(
                dataset.value(),
                objective,
                labels[1],
                TimePoint{fixture_epoch})
                .has_value());
    REQUIRE(sailroute::detail::evaluate_ensemble_label_objective(
                dataset.value(),
                objective,
                labels[2],
                TimePoint{fixture_epoch})
                .has_value());
    const std::vector<std::size_t> terminals{1U, 2U};
    auto policy = sailroute::detail::build_ensemble_policy(
        dataset.value(),
        objective,
        EnsemblePolicyOptions{},
        labels,
        terminals,
        1U,
        TimePoint{fixture_epoch});
    REQUIRE(policy.has_value());
    REQUIRE(policy.value().decision_points.size() == 1U);
    for (const auto& branch : policy.value().decision_points[0].branches) {
        REQUIRE(branch.supporting_member_weight == 0.5);
    }
    for (const auto& alternative : policy.value().graph.alternatives) {
        REQUIRE(alternative.supporting_member_weight == 0.5);
    }
}
