#include "sailroute/ensemble.hpp"
#include "sailroute/polar.hpp"

#include "ensemble/search_state.hpp"
#include "ensemble/transition.hpp"
#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;
using sailroute::Coordinate;
using sailroute::CurrentProvider;
using sailroute::CurrentVector;
using sailroute::EnsembleDataset;
using sailroute::EnsembleMemberInput;
using sailroute::EnsembleMemberOutcomeClass;
using sailroute::EnsembleObjective;
using sailroute::EnsembleObjectiveKind;
using sailroute::EnsembleRunMetadata;
using sailroute::EnvironmentCoverage;
using sailroute::EnvironmentSample;
using sailroute::EnvironmentSampleStatus;
using sailroute::Error;
using sailroute::ErrorCode;
using sailroute::MissingDataPolicy;
using sailroute::ProviderMetadata;
using sailroute::RoutePoint;
using sailroute::RoutingEnvironment;
using sailroute::RoutingOptions;
using sailroute::TimePoint;
using sailroute::detail::EnsembleCommonAction;
using sailroute::detail::EnsembleMemberSearchStatus;
using sailroute::detail::EnsembleMemberTransitionStatus;
using sailroute::detail::EnsembleSearchLabel;
using sailroute::detail::EnsembleTransitionParameters;
using sailroute::detail::OperationalConfiguration;
using sailroute::test::ConstantWindGribFixture;

constexpr std::chrono::seconds fixture_epoch{1'784'035'200};

EnsembleRunMetadata metadata() {
    return EnsembleRunMetadata{
        "search-cycle",
        "search-model",
        TimePoint{fixture_epoch},
        "generated search fixture",
        1U};
}

ProviderMetadata provider_metadata(std::string name) {
    return ProviderMetadata{std::move(name), "search state test", "1"};
}

class MissingCurrentProvider final : public CurrentProvider {
public:
    MissingCurrentProvider()
        : metadata_(provider_metadata("missing-current")) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }
    [[nodiscard]] EnvironmentCoverage coverage() const override {
        return {};
    }
    [[nodiscard]] EnvironmentSample<CurrentVector> sample(
        Coordinate,
        TimePoint) const override {
        return EnvironmentSample<CurrentVector>::without_value(
            EnvironmentSampleStatus::unavailable);
    }

private:
    ProviderMetadata metadata_;
};

class InteriorMissingCurrentProvider final : public CurrentProvider {
public:
    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }
    [[nodiscard]] EnvironmentCoverage coverage() const override { return {}; }
    [[nodiscard]] EnvironmentSample<CurrentVector> sample(
        Coordinate, TimePoint time) const override {
        if (time > TimePoint{fixture_epoch} &&
            time < TimePoint{fixture_epoch + 30min}) {
            return EnvironmentSample<CurrentVector>::without_value(
                EnvironmentSampleStatus::unavailable);
        }
        return EnvironmentSample<CurrentVector>::available(CurrentVector{});
    }
private:
    ProviderMetadata metadata_{provider_metadata("interior-missing-current")};
};

RoutingEnvironment current_environment(CurrentVector current) {
    RoutingEnvironment environment;
    environment.currents.provider =
        sailroute::make_uniform_current_provider(
            current,
            provider_metadata("uniform-current"))
            .value();
    return environment;
}

RoutingEnvironment missing_current_environment() {
    RoutingEnvironment environment;
    environment.currents.provider =
        std::make_shared<const MissingCurrentProvider>();
    environment.currents.missing_data_policy = MissingDataPolicy::fail_route;
    return environment;
}

EnsembleMemberInput input(
    std::string identifier,
    const ConstantWindGribFixture& fixture,
    RoutingEnvironment environment = {},
    double weight = 1.0) {
    EnsembleMemberInput result;
    result.identifier = std::move(identifier);
    result.weight = weight;
    result.grib_path = fixture.path();
    result.environment = std::move(environment);
    return result;
}

RoutePoint point(
    Coordinate position = Coordinate{1.0, 0.5},
    TimePoint time = TimePoint{fixture_epoch}) {
    return RoutePoint{position, time, 0.0, 0.0, 0.0, 0.0, 0.0, std::nullopt};
}

EnsembleSearchLabel initial_label(
    const EnsembleDataset& dataset,
    Coordinate position = Coordinate{1.0, 0.5}) {
    const std::vector<RoutePoint> points(dataset.member_count(), point(position));
    const std::vector<OperationalConfiguration> configurations(
        dataset.member_count());
    auto label = sailroute::detail::make_initial_ensemble_label(
        dataset, points, configurations);
    REQUIRE(label.has_value());
    return std::move(label.value());
}

EnsembleCommonAction action(Coordinate target) {
    auto result = sailroute::detail::make_common_target_action(target);
    REQUIRE(result.has_value());
    return result.value();
}

EnsembleObjective mean_objective() {
    EnsembleObjective objective;
    objective.kind =
        EnsembleObjectiveKind::weighted_mean_elapsed_arrival;
    return objective;
}

}  // namespace

TEST_CASE("shared search canonicalizes members actions and root identity") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata(),
        {input("zulu", fixture), input("alpha", fixture)});
    REQUIRE(dataset.has_value());

    auto label = initial_label(dataset.value(), Coordinate{-0.0, 0.0});
    REQUIRE(label.members.size() == 2U);
    REQUIRE(label.members[0].member_identifier == "alpha");
    REQUIRE(label.members[1].member_identifier == "zulu");
    REQUIRE(!std::signbit(label.members[0].point.position.latitude_degrees));
    REQUIRE(label.canonical_action_sequence_identity == "root");
    REQUIRE(!label.canonical_label_identity.empty());

    const auto seam_positive =
        sailroute::detail::make_common_target_action(Coordinate{-0.0, 180.0});
    const auto seam_negative =
        sailroute::detail::make_common_target_action(Coordinate{0.0, -180.0});
    REQUIRE(seam_positive.has_value());
    REQUIRE(seam_negative.has_value());
    REQUIRE(seam_positive.value() == seam_negative.value());
    REQUIRE(
        sailroute::detail::common_action_identity(seam_positive.value()) ==
        sailroute::detail::common_action_identity(seam_negative.value()));
    REQUIRE(
        !sailroute::detail::make_common_target_action(
             Coordinate{91.0, 0.0})
             .has_value());
    const auto heading =
        sailroute::detail::make_common_heading_action(450.0, 30min);
    const auto canonical_heading =
        sailroute::detail::make_common_heading_action(90.0, 30min);
    REQUIRE(heading.has_value());
    REQUIRE(canonical_heading.has_value());
    REQUIRE(heading.value() == canonical_heading.value());
    REQUIRE(
        sailroute::detail::common_action_identity(heading.value()) ==
        sailroute::detail::common_action_identity(canonical_heading.value()));
    REQUIRE(
        !sailroute::detail::make_common_heading_action(
             std::numeric_limits<double>::infinity(), 30min)
             .has_value());
    REQUIRE(
        !sailroute::detail::make_common_wait_action(0min).has_value());
}

TEST_CASE("one common target preserves divergent canonical member arrivals") {
    const ConstantWindGribFixture fast(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -10.0});
    const ConstantWindGribFixture slow(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -5.0});
    auto dataset = EnsembleDataset::load(
        metadata(),
        {input("slow", slow), input("fast", fast)});
    REQUIRE(dataset.has_value());
    const EnsembleSearchLabel root = initial_label(dataset.value());
    const Coordinate target{1.0, 0.6};
    const EnsembleObjective objective = mean_objective();
    EnsembleTransitionParameters parameters{
        target,
        TimePoint{fixture_epoch},
        TimePoint{fixture_epoch + 24h},
        &objective,
        {}};

    const auto evaluated = sailroute::detail::evaluate_common_target_transition(
        dataset.value(),
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        RoutingOptions{},
        root,
        0U,
        action(target),
        parameters);
    REQUIRE(evaluated.has_value());
    REQUIRE(evaluated.value().diagnostics.members[0].member_identifier == "fast");
    REQUIRE(evaluated.value().diagnostics.members[1].member_identifier == "slow");
    REQUIRE(
        evaluated.value().diagnostics.members[0].status ==
        EnsembleMemberTransitionStatus::reached);
    REQUIRE(
        evaluated.value().diagnostics.members[1].status ==
        EnsembleMemberTransitionStatus::reached);
    REQUIRE(
        evaluated.value().label.members[0].point.time <
        evaluated.value().label.members[1].point.time);
    REQUIRE(evaluated.value().label.aggregate_objective.has_value());
    REQUIRE(
        evaluated.value().label.common_action_history ==
        std::vector<EnsembleCommonAction>{action(target)});
    REQUIRE(
        evaluated.value().label.aggregate_objective->diagnostics.
            canonical_action_sequence_identity ==
        evaluated.value().label.canonical_action_sequence_identity);
}

TEST_CASE("member infeasibility and provider errors do not abort later members") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata(),
        {
            input("charlie", fixture, current_environment(CurrentVector{})),
            input("bravo", fixture, missing_current_environment()),
            input(
                "alpha",
                fixture,
                current_environment(CurrentVector{0.0, 100.0})),
        });
    REQUIRE(dataset.has_value());
    const Coordinate target{1.0, 0.6};
    const EnsembleObjective objective = mean_objective();
    const auto evaluated = sailroute::detail::evaluate_common_target_transition(
        dataset.value(),
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        RoutingOptions{},
        initial_label(dataset.value()),
        0U,
        action(target),
        EnsembleTransitionParameters{
            target,
            TimePoint{fixture_epoch},
            TimePoint{fixture_epoch + 24h},
            &objective,
            {}});
    REQUIRE(evaluated.has_value());
    const auto& diagnostics = evaluated.value().diagnostics;
    REQUIRE(diagnostics.members[0].member_identifier == "alpha");
    REQUIRE(diagnostics.members[1].member_identifier == "bravo");
    REQUIRE(diagnostics.members[2].member_identifier == "charlie");
    REQUIRE(
        diagnostics.members[0].status ==
        EnsembleMemberTransitionStatus::infeasible);
    REQUIRE(
        diagnostics.members[0].outcome_class ==
        EnsembleMemberOutcomeClass::infeasible_no_route);
    REQUIRE(
        diagnostics.members[1].status ==
        EnsembleMemberTransitionStatus::error);
    REQUIRE(
        diagnostics.members[1].outcome_class ==
        EnsembleMemberOutcomeClass::missing_data);
    REQUIRE(diagnostics.members[1].error.has_value());
    REQUIRE(
        diagnostics.members[1].error->code ==
        ErrorCode::environment_data_unavailable);
    REQUIRE(
        diagnostics.members[2].status ==
        EnsembleMemberTransitionStatus::reached);
    REQUIRE(diagnostics.merged_environment.current_samples >= 3U);
    REQUIRE(diagnostics.merged_environment.current_rejections == 1U);
    REQUIRE(!evaluated.value().common_action_legal);
    REQUIRE(!evaluated.value().label.aggregate_objective.has_value());
}

TEST_CASE("transition errors map to explicit member outcome classes") {
    using sailroute::detail::classify_member_transition_error;
    REQUIRE(
        classify_member_transition_error(
            Error{ErrorCode::forecast_exhausted, ""}) ==
        EnsembleMemberOutcomeClass::forecast_exhausted);
    REQUIRE(
        classify_member_transition_error(
            Error{ErrorCode::incomplete_forecast, ""}) ==
        EnsembleMemberOutcomeClass::missing_data);
    REQUIRE(
        classify_member_transition_error(
            Error{ErrorCode::environment_data_unavailable, ""}) ==
        EnsembleMemberOutcomeClass::missing_data);
    REQUIRE(
        classify_member_transition_error(
            Error{ErrorCode::invalid_environment, ""}) ==
        EnsembleMemberOutcomeClass::provider_failure);
    REQUIRE(
        classify_member_transition_error(
            Error{ErrorCode::cancelled, ""}) ==
        EnsembleMemberOutcomeClass::cancelled);
    REQUIRE(
        classify_member_transition_error(Error{ErrorCode::no_route, ""}) ==
        EnsembleMemberOutcomeClass::infeasible_no_route);
    REQUIRE(
        classify_member_transition_error(Error{ErrorCode::resource_limit, ""}) ==
        EnsembleMemberOutcomeClass::other_error);
}

TEST_CASE("ensemble eligibility keeps member diagnostics but rejects the common action") {
    const ConstantWindGribFixture high(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -10.0});
    const ConstantWindGribFixture low(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -5.0});
    auto dataset = EnsembleDataset::load(
        metadata(),
        {input("high", high), input("low", low)});
    REQUIRE(dataset.has_value());
    RoutingOptions options;
    options.segment_eligibility = [](const sailroute::RouteSegmentView& segment) {
        return segment.candidate.true_wind_speed_knots < 15.0;
    };
    const Coordinate target{1.0, 0.6};
    const auto evaluated = sailroute::detail::evaluate_common_target_transition(
        dataset.value(),
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        options,
        initial_label(dataset.value()),
        0U,
        action(target),
        EnsembleTransitionParameters{
            Coordinate{1.0, 1.0},
            TimePoint{fixture_epoch},
            TimePoint{fixture_epoch + 24h},
            nullptr,
            {}});
    REQUIRE(evaluated.has_value());
    REQUIRE(!evaluated.value().common_action_legal);
    REQUIRE(
        evaluated.value().diagnostics.members[0].status ==
        EnsembleMemberTransitionStatus::infeasible);
    REQUIRE(
        evaluated.value().diagnostics.members[1].status ==
        EnsembleMemberTransitionStatus::legal);
    REQUIRE(
        evaluated.value().label.members[1].status ==
        EnsembleMemberSearchStatus::active);
}

TEST_CASE("common heading and wait actions support divergent member states") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata(),
        {
            input(
                "west-current",
                fixture,
                current_environment(CurrentVector{-1.0, 0.0})),
            input(
                "east-current",
                fixture,
                current_environment(CurrentVector{1.0, 0.0})),
        });
    REQUIRE(dataset.has_value());
    const EnsembleSearchLabel root = initial_label(dataset.value());
    const auto heading =
        sailroute::detail::make_common_heading_action(90.0, 30min);
    REQUIRE(heading.has_value());
    const EnsembleTransitionParameters parameters{
        Coordinate{1.0, 2.0},
        TimePoint{fixture_epoch},
        TimePoint{fixture_epoch + 24h},
        nullptr,
        {}};
    const auto sailed = sailroute::detail::evaluate_common_transition(
        dataset.value(),
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        RoutingOptions{},
        root,
        0U,
        heading.value(),
        parameters);
    REQUIRE(sailed.has_value());
    REQUIRE(
        sailed.value().diagnostics.members[0].status ==
        EnsembleMemberTransitionStatus::legal);
    REQUIRE(
        sailed.value().diagnostics.members[1].status ==
        EnsembleMemberTransitionStatus::legal);
    REQUIRE(
        sailed.value().label.members[0].point.position.longitude_degrees !=
        sailed.value().label.members[1].point.position.longitude_degrees);

    const auto wait = sailroute::detail::make_common_wait_action(10min);
    REQUIRE(wait.has_value());
    RoutingOptions holding_options;
    holding_options.holding_eligibility =
        [](const sailroute::RouteSegmentView&) { return true; };
    const auto waited = sailroute::detail::evaluate_common_transition(
        dataset.value(),
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        holding_options,
        root,
        0U,
        wait.value(),
        parameters);
    REQUIRE(waited.has_value());
    for (std::size_t index = 0U; index < dataset.value().member_count(); ++index) {
        REQUIRE(
            waited.value().label.members[index].point.position.latitude_degrees ==
            root.members[index].point.position.latitude_degrees);
        REQUIRE(
            waited.value().label.members[index].point.position.longitude_degrees ==
            root.members[index].point.position.longitude_degrees);
        REQUIRE(
            waited.value().label.members[index].point.time ==
            root.members[index].point.time + 10min);
    }

    const auto missing = sailroute::detail::evaluate_common_transition(
        dataset.value(),
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        RoutingOptions{},
        initial_label(dataset.value(), Coordinate{1.0, 3.0}),
        0U,
        heading.value(),
        parameters);
    REQUIRE(missing.has_value());
    REQUIRE(
        missing.value().diagnostics.members[0].outcome_class ==
        EnsembleMemberOutcomeClass::missing_data);
    REQUIRE(
        missing.value().diagnostics.members[1].outcome_class ==
        EnsembleMemberOutcomeClass::missing_data);
}

TEST_CASE("ensemble dominance is member-wise and state compatible") {
    const ConstantWindGribFixture fixture;
    auto dataset =
        EnsembleDataset::load(metadata(), {input("a", fixture), input("b", fixture)});
    REQUIRE(dataset.has_value());
    EnsembleSearchLabel earlier = initial_label(dataset.value());
    earlier.common_action_history.push_back(action(Coordinate{1.0, 0.6}));
    earlier.incoming_action = earlier.common_action_history.back();
    for (auto& member : earlier.members) {
        member.point.position = Coordinate{1.0, 0.6};
        member.point.time += 10min;
        member.status = EnsembleMemberSearchStatus::completed;
        member.outcome_class = EnsembleMemberOutcomeClass::reached;
    }
    sailroute::detail::canonicalize_ensemble_label(earlier);

    EnsembleSearchLabel later = earlier;
    later.members[0].point.time += 1min;
    later.members[1].point.time += 2min;
    sailroute::detail::canonicalize_ensemble_label(later);
    REQUIRE(sailroute::detail::dominates(earlier, later));
    REQUIRE(!sailroute::detail::dominates(later, earlier));
    REQUIRE(!sailroute::detail::dominates(earlier, earlier));

    EnsembleSearchLabel active_earlier = earlier;
    EnsembleSearchLabel active_later = later;
    for (auto& member : active_earlier.members) {
        member.status = EnsembleMemberSearchStatus::active;
        member.outcome_class.reset();
    }
    for (auto& member : active_later.members) {
        member.status = EnsembleMemberSearchStatus::active;
        member.outcome_class.reset();
    }
    sailroute::detail::canonicalize_ensemble_label(active_earlier);
    sailroute::detail::canonicalize_ensemble_label(active_later);
    REQUIRE(!sailroute::detail::dominates(active_earlier, active_later));

    EnsembleSearchLabel tradeoff = later;
    tradeoff.members[0].point.time -= 3min;
    sailroute::detail::canonicalize_ensemble_label(tradeoff);
    REQUIRE(!sailroute::detail::dominates(earlier, tradeoff));
    REQUIRE(!sailroute::detail::dominates(tradeoff, earlier));

    EnsembleSearchLabel other_action = later;
    other_action.common_action_history.back() =
        action(Coordinate{1.0, 0.7});
    sailroute::detail::canonicalize_ensemble_label(other_action);
    REQUIRE(!sailroute::detail::dominates(earlier, other_action));

    EnsembleSearchLabel other_configuration = later;
    other_configuration.members[0].configuration.sail = 1U;
    sailroute::detail::canonicalize_ensemble_label(other_configuration);
    REQUIRE(!sailroute::detail::dominates(earlier, other_configuration));

    earlier.aggregate_objective.emplace();
    earlier.aggregate_objective->value.finite_value =
        std::numeric_limits<double>::max();
    later.aggregate_objective.emplace();
    later.aggregate_objective->value.finite_value = 0.0;
    REQUIRE(sailroute::detail::dominates(earlier, later));
}

TEST_CASE("ensemble waits require explicit holding and segment permission") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    const auto wait = sailroute::detail::make_common_wait_action(30min);
    REQUIRE(wait.has_value());
    const EnsembleTransitionParameters parameters{
        Coordinate{1.0, 1.5}, TimePoint{fixture_epoch},
        TimePoint{fixture_epoch + 2h}, nullptr, {}};
    const auto evaluate = [&](const RoutingOptions& options) {
        return sailroute::detail::evaluate_common_transition(
            dataset.value(), sailroute::VesselPolar::default_racer_cruiser_45ft(),
            options, initial_label(dataset.value()), 0U, wait.value(), parameters);
    };
    RoutingOptions options;
    auto denied = evaluate(options);
    REQUIRE(denied.has_value());
    REQUIRE(!denied.value().common_action_legal);
    REQUIRE(denied.value().label.members[0].point.time == TimePoint{fixture_epoch});

    std::size_t holding_checks = 0U;
    std::size_t segment_checks = 0U;
    options.holding_eligibility = [&](const sailroute::RouteSegmentView&) {
        ++holding_checks;
        return true;
    };
    options.segment_eligibility = [&](const sailroute::RouteSegmentView&) {
        ++segment_checks;
        return false;
    };
    denied = evaluate(options);
    REQUIRE(denied.has_value());
    REQUIRE(!denied.value().common_action_legal);
    REQUIRE(holding_checks > 0U);
    REQUIRE(segment_checks > 0U);

    options.segment_eligibility = {};
    const auto allowed = evaluate(options);
    REQUIRE(allowed.has_value());
    REQUIRE(allowed.value().common_action_legal);
    REQUIRE(allowed.value().label.members[0].point.time ==
            TimePoint{fixture_epoch + 30min});
}

TEST_CASE("ensemble waits sample environment inside the holding interval") {
    const ConstantWindGribFixture fixture;
    RoutingEnvironment environment;
    environment.currents.provider =
        std::make_shared<const InteriorMissingCurrentProvider>();
    environment.currents.missing_data_policy = MissingDataPolicy::fail_route;
    auto dataset = EnsembleDataset::load(
        metadata(), {input("member", fixture, environment)});
    REQUIRE(dataset.has_value());
    RoutingOptions options;
    options.maximum_integration_step = 5min;
    options.holding_eligibility =
        [](const sailroute::RouteSegmentView&) { return true; };
    const auto wait = sailroute::detail::make_common_wait_action(30min);
    const auto result = sailroute::detail::evaluate_common_transition(
        dataset.value(), sailroute::VesselPolar::default_racer_cruiser_45ft(),
        options, initial_label(dataset.value()), 0U, wait.value(),
        EnsembleTransitionParameters{
            Coordinate{1.0, 1.5}, TimePoint{fixture_epoch},
            TimePoint{fixture_epoch + 2h}, nullptr, {}});
    REQUIRE(result.has_value());
    REQUIRE(result.value().common_action_legal);
    REQUIRE(result.value().label.members[0].outcome_class ==
            EnsembleMemberOutcomeClass::missing_data);
    REQUIRE(result.value().diagnostics.merged_environment.current_samples >= 2U);
    REQUIRE(result.value().label.members[0].point.time == TimePoint{fixture_epoch});
}

TEST_CASE("ensemble waits enforce wind limits and time-active exclusions") {
    const ConstantWindGribFixture fixture(
        ConstantWindGribFixture::Options{
            .north_metres_per_second = -5.0,
            .final_north_metres_per_second = -20.0,
            .final_forecast_hour = 3});
    sailroute::ExclusionZone zone;
    zone.identifier = "interior-holding-restriction";
    zone.active_from = TimePoint{fixture_epoch + 5min};
    zone.active_until = TimePoint{fixture_epoch + 25min};
    zone.polygons.push_back(sailroute::ExclusionPolygon{
        sailroute::ExclusionRing{
            {Coordinate{0.9, 0.4}, Coordinate{0.9, 0.6},
             Coordinate{1.1, 0.6}, Coordinate{1.1, 0.4}}},
        {}});
    const auto zones = sailroute::ExclusionZoneSet::create(
        {zone}, provider_metadata("holding-restriction"));
    REQUIRE(zones.has_value());
    for (const bool use_exclusion : {false, true}) {
        RoutingEnvironment environment;
        if (use_exclusion) environment.exclusions.zones = zones.value();
        const auto dataset = EnsembleDataset::load(
            metadata(), {input("member", fixture, environment)});
        REQUIRE(dataset.has_value());
        RoutingOptions options;
        options.maximum_integration_step = 5min;
        options.holding_eligibility =
            [](const sailroute::RouteSegmentView&) { return true; };
        if (!use_exclusion) options.maximum_true_wind_speed_knots = 25.0;
        const auto wait = sailroute::detail::make_common_wait_action(30min);
        const auto result = sailroute::detail::evaluate_common_transition(
            dataset.value(), sailroute::VesselPolar::default_racer_cruiser_45ft(),
            options, initial_label(dataset.value()), 0U, wait.value(),
            EnsembleTransitionParameters{
                Coordinate{1.0, 1.5}, TimePoint{fixture_epoch},
                TimePoint{fixture_epoch + 1h}, nullptr, {}});
        REQUIRE(result.has_value());
        if (result.value().common_action_legal) {
            throw std::runtime_error(use_exclusion
                ? "time-active holding exclusion was accepted"
                : "holding wind limit was accepted");
        }
        REQUIRE(!result.value().common_action_legal);
        REQUIRE(result.value().label.members[0].outcome_class ==
                EnsembleMemberOutcomeClass::infeasible_no_route);
        REQUIRE(result.value().label.members[0].point.time == TimePoint{fixture_epoch});
        if (use_exclusion) {
            REQUIRE(result.value().diagnostics.merged_environment.exclusion_rejections > 0U);
        }
    }
}

TEST_CASE("ensemble diagnostic members do not prolong completed participating plans") {
    const ConstantWindGribFixture fixture;
    const auto dataset = EnsembleDataset::load(
        metadata(), {input("active", fixture), input("diagnostic", fixture, {}, 0.0)});
    REQUIRE(dataset.has_value());
    auto label = initial_label(dataset.value());
    label.members[0].status = EnsembleMemberSearchStatus::completed;
    label.members[0].outcome_class = EnsembleMemberOutcomeClass::reached;
    label.members[0].point.time += 15min;
    sailroute::detail::finalize_diagnostic_members(dataset.value(), label);
    REQUIRE(label.members[1].status == EnsembleMemberSearchStatus::failed);
    REQUIRE(label.members[1].error.has_value());
    REQUIRE(label.members[1].error->message.find("zero-weight diagnostic") !=
            std::string::npos);
    const auto objective = sailroute::detail::evaluate_ensemble_label_objective(
        dataset.value(), mean_objective(), label, TimePoint{fixture_epoch});
    REQUIRE(objective.has_value());
    REQUIRE(objective.value().value.finite_value == 900.0);
    REQUIRE(objective.value().diagnostics.incomplete_member_weight == 0.0);
}

TEST_CASE("ensemble reconstruction retains bounded-integration physical vertices") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    std::vector<EnsembleSearchLabel> labels{initial_label(dataset.value())};
    RoutingOptions options;
    options.maximum_integration_step = 5min;
    const auto heading = sailroute::detail::make_common_heading_action(90.0, 30min);
    const auto result = sailroute::detail::evaluate_common_transition(
        dataset.value(), sailroute::VesselPolar::default_racer_cruiser_45ft(),
        options, labels.front(), 0U, heading.value(),
        EnsembleTransitionParameters{
            Coordinate{1.0, 1.5}, TimePoint{fixture_epoch},
            TimePoint{fixture_epoch + 2h}, nullptr, {}});
    REQUIRE(result.has_value());
    REQUIRE(result.value().common_action_legal);
    REQUIRE(!result.value().label.members[0].intermediate_points.empty());
    labels.push_back(result.value().label);
    const auto route = sailroute::detail::reconstruct_member_route(labels, 1U, 0U);
    REQUIRE(route.has_value());
    REQUIRE(route.value().size() > 2U);
    for (std::size_t index = 1U; index < route.value().size(); ++index) {
        REQUIRE(route.value()[index].time - route.value()[index - 1U].time <= 5min);
    }
}

TEST_CASE("ensemble bounded heading reaches the arrival circle with eligibility enabled") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    auto label = initial_label(dataset.value());
    RoutingOptions options;
    std::chrono::seconds last_checked_duration{};
    options.segment_eligibility = [&](const sailroute::RouteSegmentView& segment) {
        last_checked_duration = segment.candidate.time - segment.parent.time;
        return std::abs(segment.candidate.heading_degrees - 90.0) < 1.0e-12;
    };
    const auto heading = sailroute::detail::make_common_heading_action(90.0, 30min);
    for (std::size_t step = 0U; step < 6U; ++step) {
        const auto result = sailroute::detail::evaluate_common_transition(
            dataset.value(), sailroute::VesselPolar::default_racer_cruiser_45ft(),
            options, label, step, heading.value(),
            EnsembleTransitionParameters{
                Coordinate{1.0, 0.6}, TimePoint{fixture_epoch},
                TimePoint{fixture_epoch + 3h}, nullptr, {}});
        REQUIRE(result.has_value());
        if (!result.value().common_action_legal) {
            throw std::runtime_error(
                "due-east transition rejected at step " + std::to_string(step) +
                " after " + std::to_string(
                    (label.members[0].point.time - TimePoint{fixture_epoch}).count()) +
                " seconds; last eligibility-checked substep duration " +
                std::to_string(last_checked_duration.count()) + " seconds");
        }
        label = result.value().label;
        if (label.members[0].status == EnsembleMemberSearchStatus::completed) {
            return;
        }
    }
    REQUIRE(label.members[0].status == EnsembleMemberSearchStatus::completed);
}

TEST_CASE("shared search rejects member-local alignment") {
    const ConstantWindGribFixture long_fixture;
    const ConstantWindGribFixture short_fixture(
        ConstantWindGribFixture::Options{.final_forecast_hour = 12});
    auto dataset = EnsembleDataset::load(
        metadata(),
        {input("long", long_fixture), input("short", short_fixture)},
        sailroute::EnsembleLoadOptions{
            sailroute::EnsembleAlignmentMode::permissive_member_local});
    REQUIRE(dataset.has_value());
    REQUIRE(!dataset.value().alignment().shared_search_compatible());
    const std::vector<RoutePoint> points(
        dataset.value().member_count(), point());
    const std::vector<OperationalConfiguration> configurations(
        dataset.value().member_count());
    REQUIRE(
        !sailroute::detail::make_initial_ensemble_label(
             dataset.value(), points, configurations)
             .has_value());
}

TEST_CASE("shared transitions preserve forecast and duration exhaustion") {
    const ConstantWindGribFixture short_fixture(
        ConstantWindGribFixture::Options{.final_forecast_hour = 1});
    auto forecast_dataset =
        EnsembleDataset::load(metadata(), {input("member", short_fixture)});
    REQUIRE(forecast_dataset.has_value());
    RoutingOptions midpoint_options;
    midpoint_options.wind_sampling = sailroute::WindSampling::midpoint;
    midpoint_options.midpoint_wind_sampling_threshold = 0min;
    midpoint_options.maneuver.tack_penalty = 2h;
    const Coordinate far_target{1.0, 0.6};
    EnsembleSearchLabel forecast_root = initial_label(forecast_dataset.value());
    forecast_root.members[0].configuration.board = -1;
    sailroute::detail::canonicalize_ensemble_label(forecast_root);
    const auto forecast =
        sailroute::detail::evaluate_common_target_transition(
            forecast_dataset.value(),
            sailroute::VesselPolar::default_racer_cruiser_45ft(),
            midpoint_options,
            forecast_root,
            0U,
            action(far_target),
            EnsembleTransitionParameters{
                far_target,
                TimePoint{fixture_epoch},
                TimePoint{fixture_epoch + 24h},
                nullptr,
                {}});
    REQUIRE(forecast.has_value());
    REQUIRE(
        forecast.value().diagnostics.members[0].outcome_class ==
        EnsembleMemberOutcomeClass::forecast_exhausted);

    const ConstantWindGribFixture fixture;
    auto duration_dataset =
        EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(duration_dataset.has_value());
    const Coordinate near_target{1.0, 0.6};
    const auto duration =
        sailroute::detail::evaluate_common_target_transition(
            duration_dataset.value(),
            sailroute::VesselPolar::default_racer_cruiser_45ft(),
            RoutingOptions{},
            initial_label(duration_dataset.value()),
            0U,
            action(near_target),
            EnsembleTransitionParameters{
                near_target,
                TimePoint{fixture_epoch},
                TimePoint{fixture_epoch + 1min},
                nullptr,
                {}});
    REQUIRE(duration.has_value());
    REQUIRE(
        duration.value().diagnostics.members[0].outcome_class ==
        EnsembleMemberOutcomeClass::duration_exhausted);

    const auto wait = sailroute::detail::make_common_wait_action(10min);
    REQUIRE(wait.has_value());
    const auto forecast_boundary =
        sailroute::detail::evaluate_common_transition(
            duration_dataset.value(),
            sailroute::VesselPolar::default_racer_cruiser_45ft(),
            RoutingOptions{},
            initial_label(duration_dataset.value()),
            0U,
            wait.value(),
            EnsembleTransitionParameters{
                near_target,
                TimePoint{fixture_epoch},
                TimePoint{fixture_epoch + 1min},
                nullptr,
                {},
                sailroute::detail::EnsembleRouteEndCause::forecast_horizon});
    REQUIRE(forecast_boundary.has_value());
    REQUIRE(
        forecast_boundary.value().diagnostics.members[0].outcome_class ==
        EnsembleMemberOutcomeClass::forecast_exhausted);
}

TEST_CASE("label identities ordering and reconstruction are deterministic") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    const RoutingOptions options;
    const EnsembleSearchLabel root = initial_label(dataset.value());
    const Coordinate first_target{1.0, 0.6};
    const Coordinate second_target{1.0, 0.7};
    const EnsembleTransitionParameters parameters{
        Coordinate{1.0, 1.0},
        TimePoint{fixture_epoch},
        TimePoint{fixture_epoch + 24h},
        nullptr,
        {}};
    const auto first = sailroute::detail::evaluate_common_target_transition(
        dataset.value(),
        polar,
        options,
        root,
        0U,
        action(first_target),
        parameters);
    REQUIRE(first.has_value());
    const auto second = sailroute::detail::evaluate_common_target_transition(
        dataset.value(),
        polar,
        options,
        first.value().label,
        1U,
        action(second_target),
        parameters);
    REQUIRE(second.has_value());

    const std::vector<EnsembleSearchLabel> arena{
        root, first.value().label, second.value().label};
    const auto actions =
        sailroute::detail::reconstruct_common_actions(arena, 2U);
    REQUIRE(actions.has_value());
    const std::vector<EnsembleCommonAction> expected_actions{
        action(first_target), action(second_target)};
    REQUIRE(actions.value() == expected_actions);
    const auto route =
        sailroute::detail::reconstruct_member_route(arena, 2U, 0U);
    REQUIRE(route.has_value());
    REQUIRE(route.value().size() >= 3U);
    REQUIRE(route.value()[0].position.longitude_degrees == 0.5);
    REQUIRE(route.value().back().position.longitude_degrees == 0.7);
    REQUIRE(
        second.value().label.canonical_action_sequence_identity ==
        "root/" +
            sailroute::detail::common_action_identity(action(first_target)) +
            "/" +
            sailroute::detail::common_action_identity(action(second_target)));

    std::vector<EnsembleSearchLabel> labels{
        second.value().label, root, first.value().label};
    std::mt19937 generator{42U};
    std::shuffle(labels.begin(), labels.end(), generator);
    std::sort(
        labels.begin(),
        labels.end(),
        sailroute::detail::ensemble_label_less);
    const std::vector<std::string> first_order{
        labels[0].canonical_label_identity,
        labels[1].canonical_label_identity,
        labels[2].canonical_label_identity};
    std::shuffle(labels.begin(), labels.end(), generator);
    std::sort(
        labels.begin(),
        labels.end(),
        sailroute::detail::ensemble_label_less);
    REQUIRE(labels[0].canonical_label_identity == first_order[0]);
    REQUIRE(labels[1].canonical_label_identity == first_order[1]);
    REQUIRE(labels[2].canonical_label_identity == first_order[2]);
}
