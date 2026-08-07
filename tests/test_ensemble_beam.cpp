#include "sailroute/ensemble.hpp"

#include "ensemble/beam_solver.hpp"
#include "grib_fixture.hpp"
#include "routing/geodesy.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <utility>

namespace {

using namespace std::chrono_literals;
using sailroute::Coordinate;
using sailroute::EnsembleDataset;
using sailroute::EnsembleMemberInput;
using sailroute::EnsembleMemberOutcomeClass;
using sailroute::EnsembleRouteActionKind;
using sailroute::EnsembleRouteRequest;
using sailroute::EnsembleRouteResult;
using sailroute::EnsembleRouter;
using sailroute::EnsembleRunMetadata;
using sailroute::EnsembleSolver;
using sailroute::ErrorCode;
using sailroute::TimePoint;
using sailroute::test::ConstantWindGribFixture;

constexpr std::chrono::seconds fixture_epoch{1'784'035'200};

EnsembleRunMetadata metadata(std::string run = "beam-cycle") {
    return EnsembleRunMetadata{
        std::move(run),
        "beam-model",
        TimePoint{fixture_epoch},
        "generated beam fixture",
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

EnsembleRouteRequest beam_request() {
    EnsembleRouteRequest request;
    request.start = Coordinate{1.0, 0.5};
    request.destination = Coordinate{1.0, 0.6};
    request.departure_time = TimePoint{fixture_epoch};
    request.options.maximum_route_duration = 3h;
    request.solver = EnsembleSolver::experimental_isochrone_beam;
    request.enable_experimental_beam = true;
    request.beam.time_step = 30min;
    request.beam.heading_step_degrees = 45.0;
    request.beam.centroid_bucket_nautical_miles = 2.0;
    request.beam.max_nodes_per_bucket = 4U;
    request.beam.beam_width = 64U;
    request.beam.max_steps = 16U;
    request.beam.max_total_nodes = 10'000U;
    return request;
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
    }
}

}  // namespace

TEST_CASE("experimental ensemble beam requires an exact explicit opt in") {
    const ConstantWindGribFixture fixture;
    auto dataset =
        EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());
    EnsembleRouter router{dataset.value()};

    EnsembleRouteRequest unenabled = beam_request();
    unenabled.enable_experimental_beam = false;
    auto unenabled_result = router.optimize(unenabled);
    REQUIRE(!unenabled_result.has_value());
    REQUIRE(unenabled_result.error().code == ErrorCode::invalid_argument);
    REQUIRE(unenabled_result.error().message.find("requires") !=
            std::string::npos);

    EnsembleRouteRequest contradictory = beam_request();
    contradictory.solver = EnsembleSolver::time_dependent_lattice;
    auto contradictory_result = router.optimize(contradictory);
    REQUIRE(!contradictory_result.has_value());
    REQUIRE(contradictory_result.error().code == ErrorCode::invalid_argument);
    REQUIRE(contradictory_result.error().message.find("requires") !=
            std::string::npos);

    EnsembleRouteRequest defaults;
    REQUIRE(defaults.solver == EnsembleSolver::time_dependent_lattice);
    REQUIRE(!defaults.enable_experimental_beam);
    REQUIRE(!EnsembleRouteResult{}.experimental);
}

TEST_CASE("experimental beam keeps canonical identical member states") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata(), {input("zulu", fixture), input("alpha", fixture)});
    REQUIRE(dataset.has_value());

    const auto result =
        EnsembleRouter{dataset.value()}.optimize(beam_request());
    REQUIRE(result.has_value());
    REQUIRE(
        result.value().solver ==
        EnsembleSolver::experimental_isochrone_beam);
    REQUIRE(result.value().experimental);
    REQUIRE(
        sailroute::to_string(result.value().solver) ==
        "experimental_isochrone_beam");
    REQUIRE(result.value().members[0].outcome.member_identifier == "alpha");
    REQUIRE(result.value().members[1].outcome.member_identifier == "zulu");
    require_same_route(result.value().members[0], result.value().members[1]);
    REQUIRE(result.value().members[0].outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(!result.value().common_actions.empty());
    for (const auto& action : result.value().common_actions) {
        REQUIRE(action.kind == EnsembleRouteActionKind::heading_for_duration);
        REQUIRE(action.duration == 30min);
    }
    REQUIRE(result.value().beam_diagnostics.beam_width == 64U);
    REQUIRE(result.value().beam_diagnostics.max_total_nodes == 10'000U);
    REQUIRE(!result.value().policy.root_node_identity.empty());
    REQUIRE(!result.value().policy.alternatives.empty());
    REQUIRE(!result.value().re_evaluation.selected_branch_identity.empty());
}

TEST_CASE("experimental beam retains divergent member positions") {
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

    const auto result =
        EnsembleRouter{dataset.value()}.optimize(beam_request());
    REQUIRE(result.has_value());
    const auto& fast_result = member(result.value(), "fast");
    const auto& slow_result = member(result.value(), "slow");
    REQUIRE(fast_result.outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(slow_result.outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(fast_result.points.size() >= 2U);
    REQUIRE(slow_result.points.size() >= 2U);
    REQUIRE(
        fast_result.points[1].position.longitude_degrees !=
        slow_result.points[1].position.longitude_degrees);
}

TEST_CASE("experimental beam accepts only common Stage 3 legal headings") {
    const ConstantWindGribFixture fixture;
    auto dataset = EnsembleDataset::load(
        metadata(), {input("alpha", fixture), input("zulu", fixture)});
    REQUIRE(dataset.has_value());
    EnsembleRouteRequest request = beam_request();
    request.options.segment_eligibility =
        [](const sailroute::RouteSegmentView& segment) {
            return std::abs(segment.candidate.heading_degrees - 90.0) <
                1.0e-12;
        };

    const auto result = EnsembleRouter{dataset.value()}.optimize(request);
    REQUIRE(result.has_value());
    REQUIRE(result.value().members[0].outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(result.value().members[1].outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached);
    REQUIRE(result.value().beam_diagnostics.rejected_common_actions > 0U);
    for (const auto& action : result.value().common_actions) {
        REQUIRE_NEAR(action.heading_degrees, 90.0, 1.0e-12);
    }
}

TEST_CASE("experimental beam clips the first arrival-radius crossing") {
    const ConstantWindGribFixture fixture;
    auto dataset =
        EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());

    EnsembleRouteRequest request = beam_request();
    request.destination = Coordinate{1.0, 0.55};
    request.options.arrival_radius_nautical_miles = 0.1;
    request.beam.time_step = 2h;
    request.options.maximum_route_duration = 1h;
    request.options.segment_eligibility =
        [](const sailroute::RouteSegmentView& segment) {
            return std::abs(segment.candidate.heading_degrees - 90.0) <
                1.0e-12;
        };

    const auto result = EnsembleRouter{dataset.value()}.optimize(request);
    REQUIRE(result.has_value());
    const auto& route = member(result.value(), "member");
    REQUIRE(route.outcome.outcome_class == EnsembleMemberOutcomeClass::reached);
    REQUIRE(route.outcome.elapsed_arrival_seconds.has_value());
    REQUIRE(*route.outcome.elapsed_arrival_seconds < 3600.0);
    REQUIRE(route.points.back().time <
            *request.departure_time + request.options.maximum_route_duration);
    REQUIRE(
        sailroute::detail::great_circle_distance_nautical_miles(
            route.points.back().position, request.destination) <=
        request.options.arrival_radius_nautical_miles + 1.0e-9);
}

TEST_CASE("experimental beam is repeatable and member order independent") {
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
        {input("slow", slow, 0.25), input("fast", fast, 0.75)});
    auto reordered_dataset = EnsembleDataset::load(
        metadata("reordered"),
        {input("fast", fast, 0.75), input("slow", slow, 0.25)});
    REQUIRE(first_dataset.has_value());
    REQUIRE(reordered_dataset.has_value());
    EnsembleRouter router{first_dataset.value()};

    const auto first = router.optimize(beam_request());
    const auto repeated = router.optimize(beam_request());
    const auto reordered =
        EnsembleRouter{reordered_dataset.value()}.optimize(beam_request());
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
        first.value().beam_diagnostics.generated_nodes ==
        reordered.value().beam_diagnostics.generated_nodes);
    require_same_route(
        member(first.value(), "fast"), member(reordered.value(), "fast"));
    require_same_route(
        member(first.value(), "slow"), member(reordered.value(), "slow"));
}

TEST_CASE("experimental beam hard limits and cancellation are explicit") {
    const ConstantWindGribFixture fixture;
    auto dataset =
        EnsembleDataset::load(metadata(), {input("member", fixture)});
    REQUIRE(dataset.has_value());

    EnsembleRouteRequest bounded = beam_request();
    bounded.beam.max_total_nodes = 1U;
    auto bounded_result =
        EnsembleRouter{dataset.value()}.optimize(bounded);
    REQUIRE(!bounded_result.has_value());
    REQUIRE(bounded_result.error().code == ErrorCode::no_route);
    REQUIRE(bounded_result.error().message.find("hard limit") !=
            std::string::npos);

    EnsembleRouteRequest rejected = beam_request();
    rejected.beam.max_total_nodes = 3U;
    rejected.options.segment_eligibility =
        [](const sailroute::RouteSegmentView&) { return false; };
    auto rejected_result =
        EnsembleRouter{dataset.value()}.optimize(rejected);
    REQUIRE(!rejected_result.has_value());
    REQUIRE(rejected_result.error().code == ErrorCode::no_route);
    REQUIRE(rejected_result.error().message.find(
                "3 generated and 0 accepted") != std::string::npos);

    const auto cancelled =
        sailroute::detail::optimize_ensemble_beam_route(
            dataset.value(),
            sailroute::VesselPolar::default_racer_cruiser_45ft(),
            beam_request(),
            [] { return true; });
    REQUIRE(!cancelled.has_value());
    REQUIRE(cancelled.error().code == ErrorCode::cancelled);
    REQUIRE(cancelled.error().message.find("0 expanded nodes") !=
            std::string::npos);
}
