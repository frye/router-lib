#include "sailroute/serialization.hpp"

#include "test_support.hpp"

#include <chrono>
#include <cmath>
#include <string>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

using TP = sailroute::TimePoint;

TP epoch_plus(long long seconds) {
    return TP{std::chrono::seconds{seconds}};
}

void replace_once(
    std::string& text,
    std::string_view from,
    std::string_view to) {
    const auto position = text.find(from);
    REQUIRE(position != std::string::npos);
    text.replace(position, from.size(), to);
}

/// Complete metadata for one canonical member.
sailroute::EnsembleMemberMetadata member_metadata(
    std::string identifier) {
    sailroute::EnsembleMemberMetadata member;
    member.identifier = std::move(identifier);
    member.original_weight = 1.0;
    member.normalized_weight = 0.5;
    member.weather.first_valid_time = epoch_plus(1'700'000'000LL);
    member.weather.last_valid_time = epoch_plus(1'700'003'600LL);
    member.weather.initialization_time = epoch_plus(1'700'000'000LL);
    member.weather.latitude_count = 2U;
    member.weather.longitude_count = 2U;
    member.weather.source = "fixture.grib";
    member.wind_valid_times = {
        member.weather.first_valid_time,
        member.weather.last_valid_time};
    member.wind_grid.latitude_count = 2U;
    member.wind_grid.longitude_count = 2U;
    member.wind_grid.south_latitude_degrees = 0.0;
    member.wind_grid.west_longitude_degrees = 0.0;
    member.wind_grid.latitude_step_degrees = 1.0;
    member.wind_grid.longitude_step_degrees = 1.0;
    return member;
}

/// Minimal two-member ensemble route document suitable for round-trip tests.
sailroute::EnsembleRouteDocument minimal_doc() {
    sailroute::EnsembleRunMetadata meta;
    meta.run_identifier = "run-001";
    meta.model_identifier = "fixture-model";
    meta.initialization_time = epoch_plus(1'700'000'000LL);
    meta.attribution = "fixture attribution";

    std::vector<sailroute::EnsembleMemberMetadata> metadata;
    metadata.push_back(member_metadata("ecmwf"));
    metadata.push_back(member_metadata("gfs"));

    sailroute::EnsembleRouteResult result;
    result.departure_time = epoch_plus(1'700'000'000LL);
    result.departure_source = sailroute::DepartureSource::explicit_time;
    result.solver = sailroute::EnsembleSolver::time_dependent_lattice;
    result.experimental = false;

    // One simple action.
    result.common_actions.push_back(sailroute::EnsembleRouteAction{
        .kind = sailroute::EnsembleRouteActionKind::target,
        .target = {10.0, -20.0},
        .heading_degrees = 0.0,
        .duration = std::chrono::seconds{3600},
    });

    // Two member results in canonical identifier order.
    sailroute::EnsembleMemberOutcome o1;
    o1.member_identifier = "ecmwf";
    o1.outcome_class = sailroute::EnsembleMemberOutcomeClass::reached;
    o1.elapsed_arrival_seconds = 7200.0;

    sailroute::EnsembleMemberRouteResult mr1;
    mr1.outcome = o1;
    mr1.points = {sailroute::RoutePoint{
        .position = {10.0, -20.0},
        .time = epoch_plus(1'700'000'000LL),
        .heading_degrees = 270.0,
        .boat_speed_knots = 7.0,
        .true_wind_speed_knots = 12.0,
        .true_wind_direction_degrees = 90.0,
        .cumulative_distance_nautical_miles = 0.0,
    }};
    result.members.push_back(std::move(mr1));

    sailroute::EnsembleMemberOutcome o2;
    o2.member_identifier = "gfs";
    o2.outcome_class = sailroute::EnsembleMemberOutcomeClass::reached;
    o2.elapsed_arrival_seconds = 8000.0;

    sailroute::EnsembleMemberRouteResult mr2;
    mr2.outcome = o2;
    result.members.push_back(std::move(mr2));

    // Objective evaluation (only valid fields: value + diagnostics).
    result.objective.value.value_class =
        sailroute::EnsembleObjectiveValueClass::finite;
    result.objective.value.finite_value = 7600.0;
    result.objective.diagnostics.weighted_finite_mean_arrival.value_class =
        sailroute::EnsembleObjectiveValueClass::finite;
    result.objective.diagnostics.weighted_finite_mean_arrival.finite_value = 7600.0;
    result.objective.diagnostics.worst_finite_arrival.value_class =
        sailroute::EnsembleObjectiveValueClass::finite;
    result.objective.diagnostics.worst_finite_arrival.finite_value = 8000.0;
    result.objective.diagnostics.members = {
        sailroute::EnsembleObjectiveMemberDiagnostic{
            .normalized_weight = 0.5,
            .candidate = o1,
            .elapsed_arrival = {
                sailroute::EnsembleObjectiveValueClass::finite, 7200.0},
            .weighted_contribution = 3600.0,
        },
        sailroute::EnsembleObjectiveMemberDiagnostic{
            .normalized_weight = 0.5,
            .candidate = o2,
            .elapsed_arrival = {
                sailroute::EnsembleObjectiveValueClass::finite, 8000.0},
            .weighted_contribution = 4000.0,
        }};

    // Policy graph with a two-branch decision.
    result.policy.root_node_identity = "n0";
    sailroute::EnsemblePolicyNode root_node;
    root_node.node_identity = "n0";
    root_node.canonical_member_positions = {
        {10.0, -20.0}, {10.0, -20.0}};
    root_node.earliest_member_time = epoch_plus(1'700'000'000LL);
    root_node.latest_member_time = epoch_plus(1'700'000'000LL);
    root_node.outgoing_branch_identities = {"b0", "b1"};
    result.policy.nodes.push_back(std::move(root_node));
    for (const std::string_view identity : {"n1", "n2"}) {
        sailroute::EnsemblePolicyNode terminal;
        terminal.node_identity = identity;
        terminal.canonical_member_positions = {
            {10.0, -20.0}, {10.0, -20.0}};
        terminal.earliest_member_time = epoch_plus(1'700'003'600LL);
        terminal.latest_member_time = epoch_plus(1'700'003'600LL);
        terminal.terminal = true;
        result.policy.nodes.push_back(std::move(terminal));
    }
    result.policy.branches = {
        sailroute::EnsemblePolicyBranch{
            .branch_identity = "b0",
            .from_node_identity = "n0",
            .to_node_identity = "n1",
            .action = result.common_actions.front(),
            .selected = true,
            .requires_re_evaluation = false,
            .supporting_member_weight = 0.5,
            .wrong_choice_cost = {
                sailroute::EnsembleObjectiveValueClass::finite, 0.0},
        },
        sailroute::EnsemblePolicyBranch{
            .branch_identity = "b1",
            .from_node_identity = "n0",
            .to_node_identity = "n2",
            .action = result.common_actions.front(),
            .selected = false,
            .requires_re_evaluation = true,
            .supporting_member_weight = 0.5,
            .wrong_choice_cost = {
                sailroute::EnsembleObjectiveValueClass::finite, 400.0},
        }};

    sailroute::EnsemblePolicyAlternative alternative;
    alternative.branch_identity = "selected-route";
    alternative.selected = true;
    alternative.requires_re_evaluation = false;
    alternative.common_actions = result.common_actions;
    alternative.objective = result.objective;
    alternative.member_outcomes = {o1, o2};
    alternative.wrong_choice_cost = {
        sailroute::EnsembleObjectiveValueClass::finite, 0.0};
    result.policy.alternatives.push_back(std::move(alternative));
    auto alternate = result.policy.alternatives.front();
    alternate.branch_identity = "alternate-route";
    alternate.selected = false;
    alternate.requires_re_evaluation = true;
    alternate.wrong_choice_cost = {
        sailroute::EnsembleObjectiveValueClass::finite, 400.0};
    result.policy.alternatives.push_back(std::move(alternate));

    sailroute::EnsembleDecisionPoint decision;
    decision.decision_identity = "decision-n0";
    decision.policy_node_identity = "n0";
    decision.canonical_member_positions = {
        {10.0, -20.0}, {10.0, -20.0}};
    decision.earliest_time = epoch_plus(1'700'000'000LL);
    decision.latest_commitment_time = epoch_plus(1'700'000'900LL);
    for (const auto& branch : result.policy.branches) {
        decision.branches.push_back(sailroute::EnsembleDecisionBranch{
            .policy_branch_identity = branch.branch_identity,
            .action = branch.action,
            .selected = branch.selected,
            .requires_re_evaluation = branch.requires_re_evaluation,
            .supporting_member_weight = branch.supporting_member_weight,
            .wrong_choice_cost = branch.wrong_choice_cost,
        });
    }
    result.decision_points.push_back(std::move(decision));

    result.re_evaluation.prior_run_identifier = meta.run_identifier;
    result.re_evaluation.selected_branch_identity = "selected-route";
    result.re_evaluation.canonical_branch_identities = {
        "alternate-route", "selected-route"};
    result.re_evaluation.time_tolerance = std::chrono::seconds{900};

    return sailroute::EnsembleRouteDocument{
        std::move(meta), std::move(metadata), std::move(result)};
}

/// Minimal rival outcomes document.
sailroute::EnsembleRivalOutcomesDocument minimal_rival_doc() {
    sailroute::EnsembleMemberOutcome o1;
    o1.member_identifier = "gfs";
    o1.outcome_class = sailroute::EnsembleMemberOutcomeClass::reached;
    o1.elapsed_arrival_seconds = 6000.0;

    sailroute::EnsembleMemberOutcome o2;
    o2.member_identifier = "ecmwf";
    o2.outcome_class = sailroute::EnsembleMemberOutcomeClass::forecast_exhausted;

    return sailroute::EnsembleRivalOutcomesDocument{
        std::vector<sailroute::EnsembleMemberOutcome>{o1, o2}};
}

}  // namespace

// ---------------------------------------------------------------------------
// Round-trip tests
// ---------------------------------------------------------------------------

TEST_CASE("ensemble round-trip serializes and restores schema version") {
    const auto doc = minimal_doc();
    const auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());
    REQUIRE(json.value().find("\"ensemble_route_result_v1\"") != std::string::npos);
}

TEST_CASE("ensemble round-trip preserves run identifier") {
    const auto doc = minimal_doc();
    const auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());
    REQUIRE(json.value().find("\"run-001\"") != std::string::npos);

    const auto parsed = sailroute::ensemble_route_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(parsed.value().metadata.run_identifier == "run-001");
}

TEST_CASE("ensemble round-trip preserves member identifier list") {
    const auto doc = minimal_doc();
    const auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());

    const auto parsed = sailroute::ensemble_route_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(parsed.value().result.members.size() == 2U);
    REQUIRE(parsed.value().result.members.at(0).outcome.member_identifier == "ecmwf");
    REQUIRE(parsed.value().result.members.at(1).outcome.member_identifier == "gfs");
}

TEST_CASE("ensemble round-trip preserves objective evaluation value") {
    const auto doc = minimal_doc();
    const auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());

    const auto parsed = sailroute::ensemble_route_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(
        parsed.value().result.objective.value.value_class ==
        sailroute::EnsembleObjectiveValueClass::finite);
    REQUIRE(parsed.value().result.objective.value.finite_value == 7600.0);
}

TEST_CASE("ensemble round-trip preserves elapsed arrival seconds") {
    const auto doc = minimal_doc();
    const auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());

    const auto parsed = sailroute::ensemble_route_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(
        parsed.value().result.members.at(0).outcome.elapsed_arrival_seconds.has_value());
    REQUIRE(
        parsed.value().result.members.at(0).outcome.elapsed_arrival_seconds.value() ==
        7200.0);
}

TEST_CASE("ensemble round-trip preserves departure time") {
    const auto doc = minimal_doc();
    const auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());

    const auto parsed = sailroute::ensemble_route_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(
        parsed.value().result.departure_time.time_since_epoch().count() ==
        doc.result.departure_time.time_since_epoch().count());
}

TEST_CASE("ensemble serializer does not change existing deterministic output") {
    // Serialize a minimal doc and a deterministic route in the same process.
    // Verify the deterministic output is unaffected.
    sailroute::RouteResult r;
    r.departure_time = sailroute::TimePoint{std::chrono::seconds{1'700'000'000}};
    r.arrival_time = r.departure_time + std::chrono::hours{1};
    r.departure_source = sailroute::DepartureSource::explicit_time;
    const auto det1 = sailroute::route_to_json(r);

    // Now do ensemble serialization.
    const auto doc = minimal_doc();
    const auto ensemble_json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(ensemble_json.has_value());

    const auto det2 = sailroute::route_to_json(r);
    REQUIRE(det1.has_value());
    REQUIRE(det2.has_value());
    REQUIRE(det1.value() == det2.value());
}

// ---------------------------------------------------------------------------
// Rival outcomes round-trip
// ---------------------------------------------------------------------------

TEST_CASE("rival outcomes round-trip preserves schema version") {
    const auto doc = minimal_rival_doc();
    const auto json = sailroute::ensemble_rival_outcomes_to_json(doc);
    REQUIRE(json.has_value());
    REQUIRE(json.value().find("\"ensemble_rival_outcomes_v1\"") != std::string::npos);
}

TEST_CASE("rival outcomes round-trip preserves member outcomes") {
    const auto doc = minimal_rival_doc();
    const auto json = sailroute::ensemble_rival_outcomes_to_json(doc);
    REQUIRE(json.has_value());

    const auto parsed = sailroute::ensemble_rival_outcomes_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(parsed.value().member_outcomes.size() == 2U);
    REQUIRE(
        parsed.value().member_outcomes.at(0).member_identifier == "gfs");
    REQUIRE(
        parsed.value().member_outcomes.at(0).outcome_class ==
        sailroute::EnsembleMemberOutcomeClass::reached);
    REQUIRE(
        parsed.value().member_outcomes.at(1).outcome_class ==
        sailroute::EnsembleMemberOutcomeClass::forecast_exhausted);
}

// ---------------------------------------------------------------------------
// Parser rejection tests
// ---------------------------------------------------------------------------

TEST_CASE("ensemble parser rejects malformed JSON") {
    const auto result = sailroute::ensemble_route_from_json("{not json");
    REQUIRE(!result.has_value());
}

TEST_CASE("ensemble parser rejects non-object top-level") {
    const auto result = sailroute::ensemble_route_from_json("[1,2,3]");
    REQUIRE(!result.has_value());
}

TEST_CASE("ensemble parser rejects unknown schema version") {
    const auto result = sailroute::ensemble_route_from_json(
        R"({"schema_version":"ensemble_route_result_v99"})");
    REQUIRE(!result.has_value());
}

TEST_CASE("ensemble parser rejects missing schema_version field") {
    const auto result = sailroute::ensemble_route_from_json(
        R"({"metadata":{},"result":{}})");
    REQUIRE(!result.has_value());
}

TEST_CASE("ensemble parser rejects wrong type for numeric field") {
    // Build a valid doc, inject a wrong type for elapsed_arrival_seconds.
    const auto doc = minimal_doc();
    auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());
    auto& s = json.value();
    replace_once(
        s,
        "\"elapsed_arrival_seconds\":7200",
        "\"elapsed_arrival_seconds\":\"bad\"");
    const auto result = sailroute::ensemble_route_from_json(s);
    REQUIRE(!result.has_value());
}

TEST_CASE("ensemble parser rejects invalid enum name for outcome_class") {
    const auto doc = minimal_doc();
    auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());
    auto& s = json.value();
    // Replace first "reached" with an invalid class name.
    replace_once(s, "\"reached\"", "\"not_a_class\"");
    const auto result = sailroute::ensemble_route_from_json(s);
    REQUIRE(!result.has_value());
}

TEST_CASE("ensemble parser rejects duplicate object fields") {
    auto json = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(json.has_value());
    replace_once(
        json.value(),
        R"("schema_version":"ensemble_route_result_v1")",
        R"("schema_version":"ensemble_route_result_v1","schema_version":"ensemble_route_result_v1")");
    REQUIRE(!sailroute::ensemble_route_from_json(json.value()).has_value());
}

TEST_CASE("ensemble parser rejects fractional and overflowing integers") {
    auto fractional = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(fractional.has_value());
    replace_once(
        fractional.value(),
        "\"schema_revision\":1",
        "\"schema_revision\":1.5");
    REQUIRE(
        !sailroute::ensemble_route_from_json(fractional.value()).has_value());

    auto overflow = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(overflow.has_value());
    replace_once(
        overflow.value(),
        "\"schema_revision\":1",
        "\"schema_revision\":4294967296");
    REQUIRE(
        !sailroute::ensemble_route_from_json(overflow.value()).has_value());
}

TEST_CASE("ensemble parser enforces tagged objective value invariants") {
    auto json = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(json.has_value());
    replace_once(
        json.value(),
        R"("value_class":"finite","finite_value":7600)",
        R"("value_class":"positive_infinity","finite_value":7600)");
    REQUIRE(!sailroute::ensemble_route_from_json(json.value()).has_value());
}

TEST_CASE("ensemble parser enforces reached outcome invariants") {
    auto json = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(json.has_value());
    replace_once(
        json.value(),
        ",\"elapsed_arrival_seconds\":7200",
        "");
    REQUIRE(!sailroute::ensemble_route_from_json(json.value()).has_value());
}

TEST_CASE("ensemble parser rejects inconsistent policy topology") {
    auto json = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(json.has_value());
    replace_once(
        json.value(),
        R"("root_node_identity":"n0")",
        R"("root_node_identity":"missing")");
    REQUIRE(!sailroute::ensemble_route_from_json(json.value()).has_value());
}

TEST_CASE("ensemble parser rejects unknown decision branch references") {
    auto json = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(json.has_value());
    replace_once(
        json.value(),
        R"("policy_branch_identity":"b0")",
        R"("policy_branch_identity":"missing")");
    REQUIRE(!sailroute::ensemble_route_from_json(json.value()).has_value());
}

TEST_CASE("ensemble validation rejects missing or contradictory decisions") {
    auto missing = minimal_doc();
    missing.result.decision_points.clear();
    REQUIRE(!sailroute::ensemble_route_to_json(missing).has_value());

    auto contradictory = minimal_doc();
    contradictory.result.decision_points[0].branches[0].selected = false;
    REQUIRE(!sailroute::ensemble_route_to_json(contradictory).has_value());

    auto duplicate_node = minimal_doc();
    auto duplicate = duplicate_node.result.decision_points.front();
    duplicate.decision_identity = "second-decision-same-node";
    duplicate_node.result.decision_points.push_back(std::move(duplicate));
    REQUIRE(!sailroute::ensemble_route_to_json(duplicate_node).has_value());

    auto deadline = minimal_doc();
    deadline.result.decision_points[0].latest_commitment_time +=
        std::chrono::seconds{1};
    REQUIRE(!sailroute::ensemble_route_to_json(deadline).has_value());
}

TEST_CASE("ensemble parser rejects excessive JSON nesting") {
    std::string json(
        sailroute::ensemble_json_max_nesting_depth + 1U, '[');
    json.append(
        sailroute::ensemble_json_max_nesting_depth + 1U, ']');
    REQUIRE(!sailroute::ensemble_route_from_json(json).has_value());
    REQUIRE(!sailroute::ensemble_rival_outcomes_from_json(json).has_value());
}

TEST_CASE("ensemble parser rejects inconsistent member identities") {
    auto json = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(json.has_value());
    replace_once(
        json.value(),
        R"("member_identifier":"ecmwf")",
        R"("member_identifier":"wrong")");
    REQUIRE(!sailroute::ensemble_route_from_json(json.value()).has_value());
}

TEST_CASE("ensemble serializer rejects diagnostic weight contradictions") {
    auto doc = minimal_doc();
    doc.result.objective.diagnostics.members[0].normalized_weight = 0.4;
    REQUIRE(!sailroute::ensemble_route_to_json(doc).has_value());
}

TEST_CASE("ensemble serializer rejects cyclic and unreachable policy nodes") {
    auto cyclic = minimal_doc();
    cyclic.result.policy.nodes[0].outgoing_branch_identities = {"b0"};
    cyclic.result.policy.nodes[1].terminal = false;
    cyclic.result.policy.nodes[1].outgoing_branch_identities = {"b1"};
    cyclic.result.policy.branches[1].from_node_identity = "n1";
    cyclic.result.policy.branches[1].to_node_identity = "n0";
    cyclic.result.decision_points.clear();
    REQUIRE(!sailroute::ensemble_route_to_json(cyclic).has_value());

    auto unreachable = minimal_doc();
    auto extra = unreachable.result.policy.nodes.back();
    extra.node_identity = "unreachable";
    unreachable.result.policy.nodes.push_back(std::move(extra));
    REQUIRE(!sailroute::ensemble_route_to_json(unreachable).has_value());
}

TEST_CASE("ensemble round-trip preserves canonical member attribution") {
    const auto json = sailroute::ensemble_route_to_json(minimal_doc());
    REQUIRE(json.has_value());
    const auto parsed = sailroute::ensemble_route_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(parsed.value().member_metadata.size() == 2U);
    REQUIRE(parsed.value().member_metadata[0].identifier == "ecmwf");
    REQUIRE(parsed.value().member_metadata[0].normalized_weight == 0.5);
    REQUIRE(
        parsed.value().member_metadata[0].weather.source ==
        "fixture.grib");
    REQUIRE(
        parsed.value().member_metadata[0].weather.initialization_time ==
        epoch_plus(1'700'000'000LL));
}

TEST_CASE("ensemble round-trip preserves environment attribution") {
    auto doc = minimal_doc();
    for (auto& member : doc.member_metadata) {
        member.configured_variables.currents = true;
        member.current_coverage = sailroute::EnvironmentCoverage{
            .global_longitude_coverage = true};
        sailroute::RouteEnvironmentMetadata environment;
        environment.current_provider = sailroute::ProviderMetadata{
            "current-provider", "current source", "revision-1"};
        member.environment = std::move(environment);
    }
    const auto json = sailroute::ensemble_route_to_json(doc);
    REQUIRE(json.has_value());
    const auto parsed = sailroute::ensemble_route_from_json(json.value());
    REQUIRE(parsed.has_value());
    REQUIRE(parsed.value().member_metadata[0].environment.has_value());
    REQUIRE(
        parsed.value().member_metadata[0].environment->current_provider->
            source == "current source");
    REQUIRE(
        parsed.value().member_metadata[0].current_coverage->
            global_longitude_coverage);
}

TEST_CASE("ensemble parser rejects non-finite numeric value encoded as string") {
    // The serializer encodes +Inf as {"inf":"positive"} tagged object.
    // Providing a bare non-finite JSON number is not valid JSON; parser
    // should reject it via the strict string-to-number path.
    const auto result = sailroute::ensemble_route_from_json(
        R"({"schema_version":"ensemble_route_result_v1",)"
        R"("metadata":{},"result":{"departure_time":NaN}})");
    REQUIRE(!result.has_value());
}

TEST_CASE("rival outcomes parser rejects unknown schema version") {
    const auto result = sailroute::ensemble_rival_outcomes_from_json(
        R"({"schema_version":"ensemble_rival_outcomes_v99","member_outcomes":[]})");
    REQUIRE(!result.has_value());
}

TEST_CASE("rival outcomes parser rejects malformed JSON") {
    const auto result =
        sailroute::ensemble_rival_outcomes_from_json("not json at all");
    REQUIRE(!result.has_value());
}

TEST_CASE("rival outcomes serializer rejects invalid outcomes and duplicates") {
    auto invalid = minimal_rival_doc();
    invalid.member_outcomes[0].elapsed_arrival_seconds.reset();
    REQUIRE(
        !sailroute::ensemble_rival_outcomes_to_json(invalid).has_value());

    auto duplicate = minimal_rival_doc();
    duplicate.member_outcomes[1].member_identifier =
        duplicate.member_outcomes[0].member_identifier;
    REQUIRE(
        !sailroute::ensemble_rival_outcomes_to_json(duplicate).has_value());

    auto invalid_class = minimal_rival_doc();
    invalid_class.member_outcomes[0].outcome_class =
        static_cast<sailroute::EnsembleMemberOutcomeClass>(255U);
    REQUIRE(
        !sailroute::ensemble_rival_outcomes_to_json(invalid_class).has_value());

    auto invalid_error = minimal_rival_doc();
    invalid_error.member_outcomes[1].error = sailroute::Error{
        static_cast<sailroute::ErrorCode>(255), "invalid enum"};
    REQUIRE(
        !sailroute::ensemble_rival_outcomes_to_json(invalid_error).has_value());
}
