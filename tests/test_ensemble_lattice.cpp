#include "sailroute/ensemble.hpp"

#include "ensemble/lattice_solver.hpp"
#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <chrono>
#include <memory>
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
using sailroute::EnsembleObjectiveKind;
using sailroute::EnsembleRouteActionKind;
using sailroute::EnsembleRouteRequest;
using sailroute::EnsembleRouteResult;
using sailroute::EnsembleRouter;
using sailroute::EnsembleRunMetadata;
using sailroute::EnsembleSolver;
using sailroute::EnvironmentCoverage;
using sailroute::EnvironmentSample;
using sailroute::EnvironmentSampleStatus;
using sailroute::ErrorCode;
using sailroute::MissingDataPolicy;
using sailroute::ProviderMetadata;
using sailroute::RoutingEnvironment;
using sailroute::TimePoint;
using sailroute::test::ConstantWindGribFixture;

constexpr std::chrono::seconds fixture_epoch{1'784'035'200};

EnsembleRunMetadata metadata(std::string run = "lattice-cycle") {
    return EnsembleRunMetadata{
        std::move(run),
        "lattice-model",
        TimePoint{fixture_epoch},
        "generated lattice fixture",
        1U};
}

ProviderMetadata provider_metadata(std::string name) {
    return ProviderMetadata{std::move(name), "ensemble lattice test", "1"};
}

class MissingCurrentProvider final : public CurrentProvider {
public:
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
    ProviderMetadata metadata_{provider_metadata("missing-current")};
};

RoutingEnvironment uniform_current(CurrentVector value) {
    RoutingEnvironment environment;
    environment.currents.provider =
        sailroute::make_uniform_current_provider(
            value, provider_metadata("uniform-current"))
            .value();
    return environment;
}

RoutingEnvironment missing_current() {
    RoutingEnvironment environment;
    environment.currents.provider =
        std::make_shared<const MissingCurrentProvider>();
    environment.currents.missing_data_policy = MissingDataPolicy::fail_route;
    return environment;
}

EnsembleMemberInput input(
    std::string identifier,
    const ConstantWindGribFixture& fixture,
    double weight = 1.0,
    RoutingEnvironment environment = {}) {
    EnsembleMemberInput result;
    result.identifier = std::move(identifier);
    result.weight = weight;
    result.grib_path = fixture.path();
    result.environment = std::move(environment);
    return result;
}

EnsembleRouteRequest request(
    Coordinate start = Coordinate{1.0, 0.5},
    Coordinate destination = Coordinate{1.0, 0.6}) {
    EnsembleRouteRequest result;
    result.start = start;
    result.destination = destination;
    result.departure_time = TimePoint{fixture_epoch};
    result.options.maximum_route_duration = 2h;
    result.lattice.subdivision_level = 0U;
    result.lattice.time_bucket = 30min;
    result.lattice.max_labels_per_state = 512U;
    result.lattice.max_total_labels = 5'000U;
    return result;
}

const sailroute::EnsembleMemberRouteResult& member(
    const EnsembleRouteResult& result,
    std::string_view identifier) {
    for (const auto& candidate : result.members) {
        if (candidate.outcome.member_identifier == identifier) {
            return candidate;
        }
    }
    throw std::runtime_error("missing ensemble member result");
}

void require_same_route(
    const sailroute::EnsembleMemberRouteResult& left,
    const sailroute::EnsembleMemberRouteResult& right) {
    REQUIRE(left.outcome.outcome_class == right.outcome.outcome_class);
    REQUIRE(
        left.outcome.elapsed_arrival_seconds ==
        right.outcome.elapsed_arrival_seconds);
    REQUIRE(left.points.size() == right.points.size());
    for (std::size_t index = 0U; index < left.points.size(); ++index) {
        REQUIRE(left.points[index].position.latitude_degrees ==
                right.points[index].position.latitude_degrees);
        REQUIRE(left.points[index].position.longitude_degrees ==
                right.points[index].position.longitude_degrees);
        REQUIRE(left.points[index].time == right.points[index].time);
        REQUIRE(left.points[index].heading_degrees ==
                right.points[index].heading_degrees);
        REQUIRE(left.points[index].boat_speed_knots ==
                right.points[index].boat_speed_knots);
    }
}

}  // namespace

TEST_CASE("ensemble lattice is the default and identical members stay identical") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata(), {input("zulu", fixture), input("alpha", fixture)});
    REQUIRE(dataset.has_value());
    EnsembleRouter router{dataset.value()};
    const EnsembleRouteRequest route_request = request();
    REQUIRE(route_request.solver == EnsembleSolver::time_dependent_lattice);
    REQUIRE(
        sailroute::to_string(route_request.solver) ==
        "time_dependent_lattice");

    const auto result = router.optimize(route_request);
    REQUIRE(result.has_value());
    REQUIRE(result.value().solver == EnsembleSolver::time_dependent_lattice);
    REQUIRE(result.value().members.size() == 2U);
    REQUIRE(result.value().members[0].outcome.member_identifier == "alpha");
    REQUIRE(result.value().members[1].outcome.member_identifier == "zulu");
    require_same_route(result.value().members[0], result.value().members[1]);
    REQUIRE(result.value().members[0].outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(result.value().objective.value.is_finite());
    REQUIRE(!result.value().common_actions.empty());
    REQUIRE(result.value().common_actions.back().kind ==
            EnsembleRouteActionKind::target);
    REQUIRE(result.value().lattice_diagnostics.max_labels_per_state == 512U);
    REQUIRE(result.value().lattice_diagnostics.max_total_labels == 5'000U);
    REQUIRE(!result.value().lattice_diagnostics.refinement_performed);
    REQUIRE(!result.value().policy.root_node_identity.empty());
    REQUIRE(!result.value().policy.alternatives.empty());
    REQUIRE(
        result.value().re_evaluation.prior_run_identifier ==
        dataset.value().metadata().run_identifier);
}

TEST_CASE("ensemble lattice retains divergent member arrivals and complete routes") {
    const ConstantWindGribFixture fast(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -10.0});
    const ConstantWindGribFixture slow(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -5.0});
    auto dataset = EnsembleDataset::load(
        metadata(), {input("slow", slow), input("fast", fast)});
    REQUIRE(dataset.has_value());

    const auto result = EnsembleRouter{dataset.value()}.optimize(request());
    REQUIRE(result.has_value());
    const auto& fast_result = member(result.value(), "fast");
    const auto& slow_result = member(result.value(), "slow");
    REQUIRE(fast_result.outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(slow_result.outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(
        *fast_result.outcome.elapsed_arrival_seconds <
        *slow_result.outcome.elapsed_arrival_seconds);
    REQUIRE(fast_result.points.size() >= 2U);
    REQUIRE(slow_result.points.size() >= 2U);
}

TEST_CASE("probability and quantile objectives tolerate member local failures") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata(),
        {
            input("good", fixture, 0.9, uniform_current(CurrentVector{})),
            input("bad", fixture, 0.1, missing_current()),
        });
    REQUIRE(dataset.has_value());
    EnsembleRouter router{dataset.value()};

    EnsembleRouteRequest probability = request();
    probability.objective.kind =
        EnsembleObjectiveKind::probability_before_target;
    probability.objective.target = sailroute::EnsembleArrivalTarget{7'200.0};
    auto probability_result = router.optimize(probability);
    REQUIRE(probability_result.has_value());
    REQUIRE(member(probability_result.value(), "bad").outcome.outcome_class ==
            EnsembleMemberOutcomeClass::missing_data);
    REQUIRE(member(probability_result.value(), "good").outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE_NEAR(
        probability_result.value().objective.value.finite_value, 0.9, 1.0e-12);
    REQUIRE(probability_result.value().lattice_diagnostics.zero_heuristic);

    EnsembleRouteRequest quantile = request();
    quantile.objective.kind =
        EnsembleObjectiveKind::weighted_p75_elapsed_arrival;
    auto quantile_result = router.optimize(quantile);
    REQUIRE(quantile_result.has_value());
    REQUIRE(quantile_result.value().objective.value.is_finite());
    REQUIRE(member(quantile_result.value(), "bad").outcome.outcome_class ==
            EnsembleMemberOutcomeClass::missing_data);
}

TEST_CASE("Stage 3 eligibility failures remain member local in lattice search") {
    const ConstantWindGribFixture high(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -10.0});
    const ConstantWindGribFixture low(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -5.0});
    auto dataset = EnsembleDataset::load(
        metadata(), {input("high", high), input("low", low)});
    REQUIRE(dataset.has_value());
    EnsembleRouteRequest route_request = request();
    route_request.objective.kind =
        EnsembleObjectiveKind::probability_before_target;
    route_request.objective.target =
        sailroute::EnsembleArrivalTarget{7'200.0};
    route_request.options.segment_eligibility =
        [](const sailroute::RouteSegmentView& segment) {
            return segment.candidate.true_wind_speed_knots < 15.0;
        };

    const auto result =
        EnsembleRouter{dataset.value()}.optimize(route_request);
    REQUIRE(result.has_value());
    REQUIRE(member(result.value(), "high").outcome.outcome_class ==
            EnsembleMemberOutcomeClass::infeasible_no_route);
    REQUIRE(member(result.value(), "low").outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
}

TEST_CASE("ensemble lattice handles high latitude seam routes canonically") {
    const ConstantWindGribFixture fixture(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 10.0,
            .north_metres_per_second = 0.0,
            .south_latitude_degrees = 79.0,
            .west_longitude_degrees = 179.0});
    auto dataset =
        EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    EnsembleRouteRequest seam =
        request(Coordinate{80.0, 179.5}, Coordinate{80.0, -179.5});
    seam.options.maximum_route_duration = 6h;
    seam.policy.max_alternatives = 0U;

    const auto result = EnsembleRouter{dataset.value()}.optimize(seam);
    REQUIRE(result.has_value());
    REQUIRE(result.value().members[0].outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(
        result.value().members[0].points.back().position.longitude_degrees ==
        -179.5);
}

TEST_CASE("ensemble lattice label limits are explicit hard errors") {
    const ConstantWindGribFixture fixture;
    auto dataset =
        EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    EnsembleRouter router{dataset.value()};

    EnsembleRouteRequest invalid = request();
    invalid.lattice.max_labels_per_state = 0U;
    auto invalid_result = router.optimize(invalid);
    REQUIRE(!invalid_result.has_value());
    REQUIRE(invalid_result.error().code == ErrorCode::invalid_argument);
    REQUIRE(invalid_result.error().message.find("label limits") !=
            std::string::npos);

    EnsembleRouteRequest bounded = request();
    bounded.lattice.max_total_labels = 1U;
    auto bounded_result = router.optimize(bounded);
    REQUIRE(!bounded_result.has_value());
    REQUIRE(bounded_result.error().code == ErrorCode::no_route);
    REQUIRE(bounded_result.error().message.find("max_total_labels") !=
            std::string::npos);
    REQUIRE(bounded_result.error().message.find("hard limit") !=
            std::string::npos);
}

TEST_CASE("ensemble lattice is repeatable and independent of member input order") {
    const ConstantWindGribFixture fast(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -10.0});
    const ConstantWindGribFixture slow(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 0.0,
            .north_metres_per_second = -5.0});
    auto first_dataset = EnsembleDataset::load(
        metadata("first"),
        {input("slow", slow), input("fast", fast)});
    auto second_dataset = EnsembleDataset::load(
        metadata("second"),
        {input("fast", fast), input("slow", slow)});
    REQUIRE(first_dataset.has_value());
    REQUIRE(second_dataset.has_value());
    const EnsembleRouteRequest route_request = request();
    EnsembleRouter first_router{first_dataset.value()};

    const auto first = first_router.optimize(route_request);
    const auto repeated = first_router.optimize(route_request);
    const auto reordered =
        EnsembleRouter{second_dataset.value()}.optimize(route_request);
    REQUIRE(first.has_value());
    REQUIRE(repeated.has_value());
    REQUIRE(reordered.has_value());
    REQUIRE(
        first.value().canonical_action_sequence_identity ==
        repeated.value().canonical_action_sequence_identity);
    REQUIRE(
        first.value().canonical_action_sequence_identity ==
        reordered.value().canonical_action_sequence_identity);
    REQUIRE(
        first.value().lattice_diagnostics.generated_labels ==
        repeated.value().lattice_diagnostics.generated_labels);
    REQUIRE(
        first.value().lattice_diagnostics.settled_labels ==
        reordered.value().lattice_diagnostics.settled_labels);
    require_same_route(
        member(first.value(), "fast"), member(reordered.value(), "fast"));
    require_same_route(
        member(first.value(), "slow"), member(reordered.value(), "slow"));
}

TEST_CASE("internal ensemble lattice cancellation is deterministic") {
    const ConstantWindGribFixture fixture;
    auto dataset =
        EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    const auto cancelled =
        sailroute::detail::optimize_ensemble_lattice_route(
            dataset.value(),
            sailroute::VesselPolar::default_racer_cruiser_45ft(),
            request(),
            [] { return true; });
    REQUIRE(!cancelled.has_value());
    REQUIRE(cancelled.error().code == ErrorCode::cancelled);
    REQUIRE(cancelled.error().message.find("0 settled labels") !=
            std::string::npos);
}
