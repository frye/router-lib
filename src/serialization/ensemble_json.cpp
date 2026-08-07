// Ensemble JSON serialization and deserialization.
//
// Isolated from the deterministic json.cpp serializer; uses the same
// shared helper headers (text_encoding, numeric_encoding) but introduces
// no new library dependencies. Existing deterministic bytes are unaffected.

#include "sailroute/serialization.hpp"

#include "sailroute/time.hpp"
#include "serialization/numeric_encoding.hpp"
#include "serialization/text_encoding.hpp"

#include <algorithm>
#include <cassert>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

namespace sailroute {

// ---------------------------------------------------------------------------
// Failable-void alias used by all serializer (append_*) helpers.
// std::nullopt = success; Error = failure (e.g. non-finite double).
// ---------------------------------------------------------------------------
using Err = std::optional<Error>;
static const Err kOk{};  // NOLINT(fuchsia-statically-constructed-objects)

// ---------------------------------------------------------------------------
// Serialization helpers
// ---------------------------------------------------------------------------

static void astr(std::string& out, std::string_view s) {
    serialization_detail::append_json_string(out, s);
}

static Err anum(std::string& out, double v) {
    if (!serialization_detail::append_number(out, v)) {
        return Error{ErrorCode::output_error, "non-finite value in ensemble result"};
    }
    return kOk;
}

static void ausize(std::string& out, std::size_t v) {
    // std::size_t never fails for JSON integer output.
    char buf[32]{};
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    out.append(buf, r.ptr);
}

static void au32(std::string& out, std::uint32_t v) {
    char buf[16]{};
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    out.append(buf, r.ptr);
}

static void ai64(std::string& out, std::int64_t v) {
    char buf[24]{};
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    out.append(buf, r.ptr);
}

static void au64(std::string& out, std::uint64_t v) {
    char buf[24]{};
    const auto r = std::to_chars(buf, buf + sizeof(buf), v);
    out.append(buf, r.ptr);
}

static void akey(std::string& out, std::string_view k) {
    serialization_detail::append_json_string(out, k);
    out.push_back(':');
}

// Append a JSON string key + colon + quoted string value.
static void afield(std::string& out, std::string_view k, std::string_view v) {
    akey(out, k);
    astr(out, v);
}

// Append a JSON key + colon + integer (size_t).
static void afield_size(std::string& out, std::string_view k, std::size_t v) {
    akey(out, k);
    ausize(out, v);
}

// Append a JSON key + colon + uint32.
static void afield_u32(std::string& out, std::string_view k, std::uint32_t v) {
    akey(out, k);
    au32(out, v);
}

static void afield_u64(std::string& out, std::string_view k, std::uint64_t v) {
    akey(out, k);
    au64(out, v);
}

// Append a JSON key + colon + bool.
static void afield_bool(std::string& out, std::string_view k, bool v) {
    akey(out, k);
    out.append(v ? "true" : "false");
}

// Append a JSON key + colon + double (fallible).
static Err afield_dbl(std::string& out, std::string_view k, double v) {
    akey(out, k);
    return anum(out, v);
}

// Append a JSON key + colon + double + trailing comma (fallible).
static Err afield_dbl_comma(std::string& out, std::string_view k, double v) {
    if (auto e = afield_dbl(out, k, v)) return e;
    out.push_back(',');
    return kOk;
}

// Write Coordinate as {"latitude_degrees":..., "longitude_degrees":...}
static Err append_coordinate(std::string& out, Coordinate coord) {
    out.push_back('{');
    akey(out, "latitude_degrees");
    if (auto e = anum(out, coord.latitude_degrees)) return e;
    out.push_back(',');
    akey(out, "longitude_degrees");
    if (auto e = anum(out, coord.longitude_degrees)) return e;
    out.push_back('}');
    return kOk;
}

static Err append_objective_value(std::string& out,
                                   const EnsembleObjectiveValue& v) {
    out.push_back('{');
    afield(out, "value_class", to_string(v.value_class));
    if (v.is_finite()) {
        out.push_back(',');
        akey(out, "finite_value");
        if (auto e = anum(out, v.finite_value)) return e;
    }
    out.push_back('}');
    return kOk;
}

static Err append_member_outcome(std::string& out,
                                  const EnsembleMemberOutcome& mo) {
    out.push_back('{');
    afield(out, "member_identifier", mo.member_identifier);
    out.push_back(',');
    afield(out, "outcome_class", to_string(mo.outcome_class));
    if (mo.elapsed_arrival_seconds) {
        out.push_back(',');
        akey(out, "elapsed_arrival_seconds");
        if (auto e = anum(out, *mo.elapsed_arrival_seconds)) return e;
    }
    if (mo.error) {
        out.push_back(',');
        out.append("\"error\":{");
        afield(out, "code", to_string(mo.error->code));
        out.push_back(',');
        afield(out, "message", mo.error->message);
        out.push_back('}');
    }
    out.push_back('}');
    return kOk;
}

static Err append_route_action(std::string& out,
                                const EnsembleRouteAction& a) {
    const char* kind_str = nullptr;
    switch (a.kind) {
        case EnsembleRouteActionKind::target:
            kind_str = "target"; break;
        case EnsembleRouteActionKind::heading_for_duration:
            kind_str = "heading_for_duration"; break;
        case EnsembleRouteActionKind::wait_for_duration:
            kind_str = "wait_for_duration"; break;
    }
    out.push_back('{');
    afield(out, "kind", kind_str);
    out.push_back(',');
    akey(out, "target");
    if (auto e = append_coordinate(out, a.target)) return e;
    out.push_back(',');
    akey(out, "heading_degrees");
    if (auto e = anum(out, a.heading_degrees)) return e;
    out.push_back(',');
    akey(out, "duration_seconds");
    ai64(out, static_cast<std::int64_t>(a.duration.count()));
    out.push_back('}');
    return kOk;
}

static Err append_route_action_array(std::string& out,
                                      const std::vector<EnsembleRouteAction>& v) {
    out.push_back('[');
    for (std::size_t i = 0; i < v.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_route_action(out, v[i])) return e;
    }
    out.push_back(']');
    return kOk;
}

static Err append_route_point(std::string& out, const RoutePoint& p) {
    out.push_back('{');
    akey(out, "position");
    if (auto e = append_coordinate(out, p.position)) return e;
    out.push_back(',');
    afield(out, "time", format_utc_time(p.time));
    out.push_back(',');
    akey(out, "heading_degrees");
    if (auto e = anum(out, p.heading_degrees)) return e;
    out.push_back(',');
    akey(out, "boat_speed_knots");
    if (auto e = anum(out, p.boat_speed_knots)) return e;
    out.push_back(',');
    akey(out, "true_wind_speed_knots");
    if (auto e = anum(out, p.true_wind_speed_knots)) return e;
    out.push_back(',');
    akey(out, "true_wind_direction_degrees");
    if (auto e = anum(out, p.true_wind_direction_degrees)) return e;
    out.push_back(',');
    akey(out, "cumulative_distance_nautical_miles");
    if (auto e = anum(out, p.cumulative_distance_nautical_miles)) return e;
    if (p.environment) {
        const auto& env = *p.environment;
        out.push_back(',');
        out.append("\"environment\":{");
        akey(out, "speed_over_ground_knots");
        if (auto e = anum(out, env.speed_over_ground_knots)) return e;
        out.push_back(',');
        akey(out, "course_over_ground_degrees");
        if (auto e = anum(out, env.course_over_ground_degrees)) return e;
        out.push_back(',');
        akey(out, "current_east_knots");
        if (auto e = anum(out, env.current_east_knots)) return e;
        out.push_back(',');
        akey(out, "current_north_knots");
        if (auto e = anum(out, env.current_north_knots)) return e;
        out.push_back(',');
        akey(out, "flat_water_speed_knots");
        if (auto e = anum(out, env.flat_water_speed_knots)) return e;
        out.push_back(',');
        akey(out, "significant_wave_height_metres");
        if (auto e = anum(out, env.significant_wave_height_metres)) return e;
        out.push_back(',');
        akey(out, "wave_period_seconds");
        if (auto e = anum(out, env.wave_period_seconds)) return e;
        out.push_back(',');
        akey(out, "relative_wave_angle_degrees");
        if (auto e = anum(out, env.relative_wave_angle_degrees)) return e;
        out.push_back(',');
        afield_bool(out, "current_applied", env.current_applied);
        out.push_back(',');
        afield_bool(out, "wave_applied", env.wave_applied);
        out.push_back('}');
    }
    out.push_back('}');
    return kOk;
}

static Err append_env_diagnostics(std::string& out,
                                   const EnvironmentDiagnostics& d) {
    out.push_back('{');
    afield_size(out, "current_samples", d.current_samples);
    out.push_back(',');
    afield_size(out, "current_rejections", d.current_rejections);
    out.push_back(',');
    afield_size(out, "wave_samples", d.wave_samples);
    out.push_back(',');
    afield_size(out, "wave_rejections", d.wave_rejections);
    out.push_back(',');
    afield_size(out, "sea_state_evaluations", d.sea_state_evaluations);
    out.push_back(',');
    afield_size(out, "land_checks", d.land_checks);
    out.push_back(',');
    afield_size(out, "land_distance_queries", d.land_distance_queries);
    out.push_back(',');
    afield_size(out, "land_rejections", d.land_rejections);
    out.push_back(',');
    afield_size(out, "exclusion_checks", d.exclusion_checks);
    out.push_back(',');
    afield_size(out, "exclusion_geometry_tests", d.exclusion_geometry_tests);
    out.push_back(',');
    afield_size(out, "exclusion_rejections", d.exclusion_rejections);
    out.push_back('}');
    return kOk;
}

static Err append_objective_member_diagnostic(
    std::string& out, const EnsembleObjectiveMemberDiagnostic& d) {
    out.push_back('{');
    akey(out, "normalized_weight");
    if (auto e = anum(out, d.normalized_weight)) return e;
    out.push_back(',');
    akey(out, "candidate");
    if (auto e = append_member_outcome(out, d.candidate)) return e;
    if (d.rival) {
        out.push_back(',');
        akey(out, "rival");
        if (auto e = append_member_outcome(out, *d.rival)) return e;
    }
    out.push_back(',');
    akey(out, "elapsed_arrival");
    if (auto e = append_objective_value(out, d.elapsed_arrival)) return e;
    out.push_back(',');
    akey(out, "probability_score");
    if (auto e = anum(out, d.probability_score)) return e;
    out.push_back(',');
    akey(out, "weighted_contribution");
    if (auto e = anum(out, d.weighted_contribution)) return e;
    out.push_back(',');
    afield_bool(out, "selected_quantile_member", d.selected_quantile_member);
    out.push_back('}');
    return kOk;
}

static Err append_objective_diagnostics(std::string& out,
                                         const EnsembleObjectiveDiagnostics& d) {
    out.push_back('{');
    out.append("\"members\":[");
    for (std::size_t i = 0; i < d.members.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_objective_member_diagnostic(out, d.members[i]))
            return e;
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "incomplete_member_weight");
    if (auto e = anum(out, d.incomplete_member_weight)) return e;
    out.push_back(',');
    akey(out, "weighted_finite_mean_arrival");
    if (auto e = append_objective_value(out, d.weighted_finite_mean_arrival))
        return e;
    out.push_back(',');
    akey(out, "worst_finite_arrival");
    if (auto e = append_objective_value(out, d.worst_finite_arrival)) return e;
    out.push_back(',');
    afield_size(out, "generated_states", d.generated_states);
    out.push_back(',');
    afield_size(out, "settled_states", d.settled_states);
    out.push_back(',');
    afield(out, "canonical_action_sequence_identity",
           d.canonical_action_sequence_identity);
    out.push_back('}');
    return kOk;
}

static Err append_objective_evaluation(std::string& out,
                                        const EnsembleObjectiveEvaluation& ev) {
    out.push_back('{');
    akey(out, "value");
    if (auto e = append_objective_value(out, ev.value)) return e;
    out.push_back(',');
    akey(out, "diagnostics");
    if (auto e = append_objective_diagnostics(out, ev.diagnostics)) return e;
    out.push_back('}');
    return kOk;
}

static Err append_lattice_diagnostics(std::string& out,
                                       const EnsembleLatticeDiagnostics& d) {
    out.push_back('{');
    afield_size(out, "settled_labels", d.settled_labels);
    out.push_back(',');
    afield_size(out, "queued_labels", d.queued_labels);
    out.push_back(',');
    afield_size(out, "generated_labels", d.generated_labels);
    out.push_back(',');
    afield_size(out, "retained_labels", d.retained_labels);
    out.push_back(',');
    afield_size(out, "dominated_labels", d.dominated_labels);
    out.push_back(',');
    afield_size(out, "stale_queue_entries", d.stale_queue_entries);
    out.push_back(',');
    afield_size(out, "peak_labels_per_state", d.peak_labels_per_state);
    out.push_back(',');
    afield_size(out, "max_labels_per_state", d.max_labels_per_state);
    out.push_back(',');
    afield_size(out, "max_total_labels", d.max_total_labels);
    out.push_back(',');
    afield_size(out, "subdivision_level", d.subdivision_level);
    out.push_back(',');
    afield_bool(out, "zero_heuristic", d.zero_heuristic);
    out.push_back(',');
    afield_bool(out, "refinement_performed", d.refinement_performed);
    out.push_back('}');
    return kOk;
}

static Err append_beam_diagnostics(std::string& out,
                                    const EnsembleBeamDiagnostics& d) {
    out.push_back('{');
    afield_size(out, "expanded_nodes", d.expanded_nodes);
    out.push_back(',');
    afield_size(out, "generated_nodes", d.generated_nodes);
    out.push_back(',');
    afield_size(out, "accepted_nodes", d.accepted_nodes);
    out.push_back(',');
    afield_size(out, "retained_nodes", d.retained_nodes);
    out.push_back(',');
    afield_size(out, "rejected_common_actions", d.rejected_common_actions);
    out.push_back(',');
    afield_size(out, "pruned_by_bucket", d.pruned_by_bucket);
    out.push_back(',');
    afield_size(out, "pruned_by_beam", d.pruned_by_beam);
    out.push_back(',');
    afield_size(out, "peak_frontier", d.peak_frontier);
    out.push_back(',');
    afield_size(out, "completed_nodes", d.completed_nodes);
    out.push_back(',');
    afield_size(out, "beam_width", d.beam_width);
    out.push_back(',');
    afield_size(out, "max_nodes_per_bucket", d.max_nodes_per_bucket);
    out.push_back(',');
    afield_size(out, "max_steps", d.max_steps);
    out.push_back(',');
    afield_size(out, "max_total_nodes", d.max_total_nodes);
    out.push_back('}');
    return kOk;
}

static Err append_member_route_result(std::string& out,
                                       const EnsembleMemberRouteResult& m) {
    out.push_back('{');
    akey(out, "outcome");
    if (auto e = append_member_outcome(out, m.outcome)) return e;
    out.push_back(',');
    out.append("\"points\":[");
    for (std::size_t i = 0; i < m.points.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_route_point(out, m.points[i])) return e;
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "environment_diagnostics");
    if (auto e = append_env_diagnostics(out, m.environment_diagnostics))
        return e;
    out.push_back('}');
    return kOk;
}

static Err append_policy_alternative(std::string& out,
                                      const EnsemblePolicyAlternative& a) {
    out.push_back('{');
    afield(out, "branch_identity", a.branch_identity);
    out.push_back(',');
    afield_bool(out, "selected", a.selected);
    out.push_back(',');
    afield_bool(out, "requires_re_evaluation", a.requires_re_evaluation);
    out.push_back(',');
    akey(out, "common_actions");
    if (auto e = append_route_action_array(out, a.common_actions)) return e;
    out.push_back(',');
    akey(out, "objective");
    if (auto e = append_objective_evaluation(out, a.objective)) return e;
    out.push_back(',');
    out.append("\"member_outcomes\":[");
    for (std::size_t i = 0; i < a.member_outcomes.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_member_outcome(out, a.member_outcomes[i])) return e;
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "supporting_member_weight");
    if (auto e = anum(out, a.supporting_member_weight)) return e;
    out.push_back(',');
    akey(out, "wrong_choice_cost");
    if (auto e = append_objective_value(out, a.wrong_choice_cost)) return e;
    out.push_back('}');
    return kOk;
}

static Err append_policy_node(std::string& out,
                               const EnsemblePolicyNode& n) {
    out.push_back('{');
    afield(out, "node_identity", n.node_identity);
    out.push_back(',');
    out.append("\"canonical_member_positions\":[");
    for (std::size_t i = 0; i < n.canonical_member_positions.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_coordinate(out, n.canonical_member_positions[i]))
            return e;
    }
    out.push_back(']');
    out.push_back(',');
    afield(out, "earliest_member_time", format_utc_time(n.earliest_member_time));
    out.push_back(',');
    afield(out, "latest_member_time", format_utc_time(n.latest_member_time));
    out.push_back(',');
    afield_bool(out, "terminal", n.terminal);
    out.push_back(',');
    out.append("\"outgoing_branch_identities\":[");
    for (std::size_t i = 0; i < n.outgoing_branch_identities.size(); ++i) {
        if (i > 0) out.push_back(',');
        astr(out, n.outgoing_branch_identities[i]);
    }
    out.push_back(']');
    out.push_back('}');
    return kOk;
}

static Err append_policy_branch(std::string& out,
                                 const EnsemblePolicyBranch& b) {
    out.push_back('{');
    afield(out, "branch_identity", b.branch_identity);
    out.push_back(',');
    afield(out, "from_node_identity", b.from_node_identity);
    out.push_back(',');
    afield(out, "to_node_identity", b.to_node_identity);
    out.push_back(',');
    akey(out, "action");
    if (auto e = append_route_action(out, b.action)) return e;
    out.push_back(',');
    afield_bool(out, "selected", b.selected);
    out.push_back(',');
    afield_bool(out, "requires_re_evaluation", b.requires_re_evaluation);
    out.push_back(',');
    akey(out, "supporting_member_weight");
    if (auto e = anum(out, b.supporting_member_weight)) return e;
    out.push_back(',');
    akey(out, "wrong_choice_cost");
    if (auto e = append_objective_value(out, b.wrong_choice_cost)) return e;
    out.push_back('}');
    return kOk;
}

static Err append_policy_graph(std::string& out,
                                const EnsemblePolicyGraph& g) {
    out.push_back('{');
    afield_u32(out, "schema_revision", g.schema_revision);
    out.push_back(',');
    afield(out, "root_node_identity", g.root_node_identity);
    out.push_back(',');
    out.append("\"nodes\":[");
    for (std::size_t i = 0; i < g.nodes.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_policy_node(out, g.nodes[i])) return e;
    }
    out.push_back(']');
    out.push_back(',');
    out.append("\"branches\":[");
    for (std::size_t i = 0; i < g.branches.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_policy_branch(out, g.branches[i])) return e;
    }
    out.push_back(']');
    out.push_back(',');
    out.append("\"alternatives\":[");
    for (std::size_t i = 0; i < g.alternatives.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_policy_alternative(out, g.alternatives[i])) return e;
    }
    out.push_back(']');
    out.push_back('}');
    return kOk;
}

static Err append_decision_branch(std::string& out,
                                   const EnsembleDecisionBranch& b) {
    out.push_back('{');
    afield(out, "policy_branch_identity", b.policy_branch_identity);
    out.push_back(',');
    akey(out, "action");
    if (auto e = append_route_action(out, b.action)) return e;
    out.push_back(',');
    afield_bool(out, "selected", b.selected);
    out.push_back(',');
    afield_bool(out, "requires_re_evaluation", b.requires_re_evaluation);
    out.push_back(',');
    akey(out, "supporting_member_weight");
    if (auto e = anum(out, b.supporting_member_weight)) return e;
    out.push_back(',');
    akey(out, "wrong_choice_cost");
    if (auto e = append_objective_value(out, b.wrong_choice_cost)) return e;
    out.push_back('}');
    return kOk;
}

static Err append_decision_point(std::string& out,
                                  const EnsembleDecisionPoint& dp) {
    out.push_back('{');
    afield(out, "decision_identity", dp.decision_identity);
    out.push_back(',');
    afield(out, "policy_node_identity", dp.policy_node_identity);
    out.push_back(',');
    out.append("\"canonical_member_positions\":[");
    for (std::size_t i = 0; i < dp.canonical_member_positions.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_coordinate(out, dp.canonical_member_positions[i]))
            return e;
    }
    out.push_back(']');
    out.push_back(',');
    afield(out, "earliest_time", format_utc_time(dp.earliest_time));
    out.push_back(',');
    afield(out, "latest_commitment_time",
           format_utc_time(dp.latest_commitment_time));
    out.push_back(',');
    out.append("\"branches\":[");
    for (std::size_t i = 0; i < dp.branches.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_decision_branch(out, dp.branches[i])) return e;
    }
    out.push_back(']');
    out.push_back('}');
    return kOk;
}

static Err append_re_evaluation_objective(std::string& out,
                                           const EnsembleObjective& obj) {
    out.push_back('{');
    afield(out, "kind", to_string(obj.kind));
    if (obj.target) {
        out.push_back(',');
        out.append("\"target\":{");
        akey(out, "elapsed_seconds");
        if (auto e = anum(out, obj.target->elapsed_seconds)) return e;
        out.push_back('}');
    }
    if (!obj.rival_outcomes.empty()) {
        out.push_back(',');
        out.append("\"rival_outcomes\":[");
        for (std::size_t i = 0; i < obj.rival_outcomes.size(); ++i) {
            if (i > 0) out.push_back(',');
            if (auto e = append_member_outcome(out, obj.rival_outcomes[i]))
                return e;
        }
        out.push_back(']');
    }
    out.push_back('}');
    return kOk;
}

static Err append_re_evaluation_state(std::string& out,
                                       const EnsembleReevaluationState& s) {
    out.push_back('{');
    afield_u32(out, "schema_revision", s.schema_revision);
    out.push_back(',');
    afield(out, "prior_run_identifier", s.prior_run_identifier);
    out.push_back(',');
    afield(out, "selected_branch_identity", s.selected_branch_identity);
    out.push_back(',');
    out.append("\"canonical_branch_identities\":[");
    for (std::size_t i = 0; i < s.canonical_branch_identities.size(); ++i) {
        if (i > 0) out.push_back(',');
        astr(out, s.canonical_branch_identities[i]);
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "objective");
    if (auto e = append_re_evaluation_objective(out, s.objective)) return e;
    out.push_back(',');
    akey(out, "spatial_tolerance_nautical_miles");
    if (auto e = anum(out, s.spatial_tolerance_nautical_miles)) return e;
    out.push_back(',');
    akey(out, "time_tolerance_seconds");
    ai64(out, static_cast<std::int64_t>(s.time_tolerance.count()));
    out.push_back('}');
    return kOk;
}

static Err append_run_metadata(std::string& out,
                                const EnsembleRunMetadata& m) {
    out.push_back('{');
    afield(out, "run_identifier", m.run_identifier);
    out.push_back(',');
    afield(out, "model_identifier", m.model_identifier);
    out.push_back(',');
    afield(out, "initialization_time", format_utc_time(m.initialization_time));
    out.push_back(',');
    afield(out, "attribution", m.attribution);
    out.push_back(',');
    afield_u32(out, "schema_revision", m.schema_revision);
    out.push_back('}');
    return kOk;
}

static Err append_geographic_bounds(
    std::string& out,
    const GeographicBounds& bounds) {
    out.push_back('{');
    if (auto e = afield_dbl_comma(
            out, "south_latitude_degrees",
            bounds.south_latitude_degrees)) return e;
    if (auto e = afield_dbl_comma(
            out, "west_longitude_degrees",
            bounds.west_longitude_degrees)) return e;
    if (auto e = afield_dbl_comma(
            out, "north_latitude_degrees",
            bounds.north_latitude_degrees)) return e;
    if (auto e = afield_dbl(
            out, "east_longitude_degrees",
            bounds.east_longitude_degrees)) return e;
    out.push_back('}');
    return kOk;
}

static Err append_forecast_metadata(
    std::string& out,
    const ForecastMetadata& metadata) {
    out.push_back('{');
    afield(
        out,
        "initialization_time",
        format_utc_time(metadata.initialization_time));
    out.push_back(',');
    afield(out, "first_valid_time", format_utc_time(metadata.first_valid_time));
    out.push_back(',');
    afield(out, "last_valid_time", format_utc_time(metadata.last_valid_time));
    out.push_back(',');
    afield_size(out, "latitude_count", metadata.latitude_count);
    out.push_back(',');
    afield_size(out, "longitude_count", metadata.longitude_count);
    out.push_back(',');
    afield_bool(
        out, "global_longitude_coverage",
        metadata.global_longitude_coverage);
    out.push_back(',');
    afield(out, "source", metadata.source);
    out.push_back('}');
    return kOk;
}

static Err append_forecast_grid_identity(
    std::string& out,
    const ForecastGridIdentity& grid) {
    out.push_back('{');
    afield_size(out, "latitude_count", grid.latitude_count);
    out.push_back(',');
    afield_size(out, "longitude_count", grid.longitude_count);
    out.push_back(',');
    if (auto e = afield_dbl_comma(
            out, "south_latitude_degrees",
            grid.south_latitude_degrees)) return e;
    if (auto e = afield_dbl_comma(
            out, "west_longitude_degrees",
            grid.west_longitude_degrees)) return e;
    if (auto e = afield_dbl_comma(
            out, "latitude_step_degrees",
            grid.latitude_step_degrees)) return e;
    if (auto e = afield_dbl_comma(
            out, "longitude_step_degrees",
            grid.longitude_step_degrees)) return e;
    afield_bool(
        out, "global_longitude_coverage",
        grid.global_longitude_coverage);
    out.push_back(',');
    afield_bool(
        out, "duplicate_longitude_endpoint",
        grid.duplicate_longitude_endpoint);
    if (grid.interpolation_bounds) {
        out.push_back(',');
        akey(out, "interpolation_bounds");
        if (auto e = append_geographic_bounds(
                out, *grid.interpolation_bounds)) return e;
    }
    out.push_back('}');
    return kOk;
}

static void append_provider_metadata(
    std::string& out,
    const ProviderMetadata& provider) {
    out.push_back('{');
    afield(out, "name", provider.name);
    out.push_back(',');
    afield(out, "source", provider.source);
    out.push_back(',');
    afield(out, "revision", provider.revision);
    out.push_back('}');
}

static Err append_route_environment_metadata(
    std::string& out,
    const RouteEnvironmentMetadata& environment) {
    out.push_back('{');
    bool first = true;
    const auto append_provider = [&](std::string_view key,
                                     const std::optional<ProviderMetadata>& value) {
        if (!value) return;
        if (!first) out.push_back(',');
        first = false;
        akey(out, key);
        append_provider_metadata(out, *value);
    };
    append_provider("current_provider", environment.current_provider);
    append_provider("wave_provider", environment.wave_provider);
    append_provider("sea_state_model", environment.sea_state_model);
    append_provider("landmask", environment.landmask);
    append_provider("exclusions", environment.exclusions);
    const auto comma = [&]() {
        if (!first) out.push_back(',');
        first = false;
    };
    comma();
    afield(out, "current_policy", to_string(environment.current_policy));
    comma();
    afield(out, "wave_policy", to_string(environment.wave_policy));
    comma();
    afield(out, "land_policy", to_string(environment.land_policy));
    comma();
    afield(out, "sampling", to_string(environment.sampling));
    comma();
    if (auto e = afield_dbl(
            out, "land_resolution_nautical_miles",
            environment.land_resolution_nautical_miles)) return e;
    comma();
    if (auto e = afield_dbl(
            out, "land_interpolation_error_nautical_miles",
            environment.land_interpolation_error_nautical_miles)) return e;
    comma();
    if (auto e = afield_dbl(
            out, "land_clearance_nautical_miles",
            environment.land_clearance_nautical_miles)) return e;
    comma();
    afield(
        out, "exclusion_boundary_policy",
        to_string(environment.exclusion_boundary_policy));
    comma();
    afield_size(
        out, "exclusion_zone_count",
        environment.exclusion_zone_count);
    comma();
    afield_u64(
        out, "exclusion_revision",
        environment.exclusion_revision);
    out.push_back('}');
    return kOk;
}

static Err append_environment_coverage(
    std::string& out,
    const EnvironmentCoverage& coverage) {
    out.push_back('{');
    bool first = true;
    const auto comma = [&]() {
        if (!first) out.push_back(',');
        first = false;
    };
    const auto append_optional_double =
        [&](std::string_view key, const std::optional<double>& value) -> Err {
            if (!value) return kOk;
            comma();
            return afield_dbl(out, key, *value);
        };
    if (auto e = append_optional_double(
            "south_latitude_degrees",
            coverage.south_latitude_degrees)) return e;
    if (auto e = append_optional_double(
            "north_latitude_degrees",
            coverage.north_latitude_degrees)) return e;
    if (auto e = append_optional_double(
            "west_longitude_degrees",
            coverage.west_longitude_degrees)) return e;
    if (auto e = append_optional_double(
            "east_longitude_degrees",
            coverage.east_longitude_degrees)) return e;
    if (coverage.first_valid_time) {
        comma();
        afield(
            out, "first_valid_time",
            format_utc_time(*coverage.first_valid_time));
    }
    if (coverage.last_valid_time) {
        comma();
        afield(
            out, "last_valid_time",
            format_utc_time(*coverage.last_valid_time));
    }
    comma();
    afield_bool(
        out, "global_longitude_coverage",
        coverage.global_longitude_coverage);
    out.push_back('}');
    return kOk;
}

static Err append_member_metadata(
    std::string& out,
    const EnsembleMemberMetadata& member) {
    out.push_back('{');
    afield(out, "identifier", member.identifier);
    out.push_back(',');
    if (auto e = afield_dbl_comma(
            out, "original_weight", member.original_weight)) return e;
    if (auto e = afield_dbl_comma(
            out, "normalized_weight", member.normalized_weight)) return e;
    akey(out, "weather");
    if (auto e = append_forecast_metadata(out, member.weather)) return e;
    out.push_back(',');
    out.append("\"wind_valid_times\":[");
    for (std::size_t i = 0; i < member.wind_valid_times.size(); ++i) {
        if (i > 0U) out.push_back(',');
        astr(out, format_utc_time(member.wind_valid_times[i]));
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "wind_grid");
    if (auto e = append_forecast_grid_identity(out, member.wind_grid)) return e;
    out.push_back(',');
    out.append("\"configured_variables\":{");
    afield_bool(
        out, "wind", member.configured_variables.wind);
    out.push_back(',');
    afield_bool(
        out, "currents", member.configured_variables.currents);
    out.push_back(',');
    afield_bool(
        out, "waves", member.configured_variables.waves);
    out.push_back(',');
    afield_bool(
        out, "land", member.configured_variables.land);
    out.push_back(',');
    afield_bool(
        out, "exclusions", member.configured_variables.exclusions);
    out.push_back('}');
    if (member.environment) {
        out.push_back(',');
        akey(out, "environment");
        if (auto e = append_route_environment_metadata(
                out, *member.environment)) return e;
    }
    if (member.current_coverage) {
        out.push_back(',');
        akey(out, "current_coverage");
        if (auto e = append_environment_coverage(
                out, *member.current_coverage)) return e;
    }
    if (member.wave_coverage) {
        out.push_back(',');
        akey(out, "wave_coverage");
        if (auto e = append_environment_coverage(
                out, *member.wave_coverage)) return e;
    }
    out.push_back('}');
    return kOk;
}

static Err append_solver_name(std::string& out, EnsembleSolver s) {
    std::string_view sv;
    switch (s) {
        case EnsembleSolver::time_dependent_lattice:
            sv = "time_dependent_lattice"; break;
        case EnsembleSolver::experimental_isochrone_beam:
            sv = "experimental_isochrone_beam"; break;
    }
    astr(out, sv);
    return kOk;
}

static Err append_departure_source(std::string& out, DepartureSource ds) {
    std::string_view sv = to_string(ds);
    astr(out, sv);
    return kOk;
}

static Err append_result(std::string& out, const EnsembleRouteResult& r) {
    out.push_back('{');
    afield(out, "departure_time", format_utc_time(r.departure_time));
    out.push_back(',');
    akey(out, "departure_source");
    if (auto e = append_departure_source(out, r.departure_source)) return e;
    out.push_back(',');
    akey(out, "solver");
    if (auto e = append_solver_name(out, r.solver)) return e;
    out.push_back(',');
    akey(out, "common_actions");
    if (auto e = append_route_action_array(out, r.common_actions)) return e;
    out.push_back(',');
    afield(out, "canonical_action_sequence_identity",
           r.canonical_action_sequence_identity);
    out.push_back(',');
    out.append("\"members\":[");
    for (std::size_t i = 0; i < r.members.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_member_route_result(out, r.members[i])) return e;
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "objective");
    if (auto e = append_objective_evaluation(out, r.objective)) return e;
    out.push_back(',');
    akey(out, "lattice_diagnostics");
    if (auto e = append_lattice_diagnostics(out, r.lattice_diagnostics))
        return e;
    out.push_back(',');
    akey(out, "beam_diagnostics");
    if (auto e = append_beam_diagnostics(out, r.beam_diagnostics)) return e;
    out.push_back(',');
    akey(out, "policy");
    if (auto e = append_policy_graph(out, r.policy)) return e;
    out.push_back(',');
    out.append("\"decision_points\":[");
    for (std::size_t i = 0; i < r.decision_points.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_decision_point(out, r.decision_points[i])) return e;
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "re_evaluation");
    if (auto e = append_re_evaluation_state(out, r.re_evaluation)) return e;
    out.push_back(',');
    afield_bool(out, "experimental", r.experimental);
    out.push_back('}');
    return kOk;
}

// ---------------------------------------------------------------------------
// Public: ensemble_route_to_json
// ---------------------------------------------------------------------------

static std::optional<Error> validate_document(
    const EnsembleRouteDocument& doc);

Result<std::string> ensemble_route_to_json(const EnsembleRouteDocument& doc) {
    if (const auto validation = validate_document(doc)) {
        return *validation;
    }
    std::string out;
    out.reserve(65536);
    out.push_back('{');
    afield(out, "schema_version", "ensemble_route_result_v1");
    out.push_back(',');
    akey(out, "run_metadata");
    if (auto e = append_run_metadata(out, doc.metadata)) return *e;
    out.push_back(',');
    out.append("\"member_metadata\":[");
    for (std::size_t i = 0; i < doc.member_metadata.size(); ++i) {
        if (i > 0U) out.push_back(',');
        if (auto e = append_member_metadata(out, doc.member_metadata[i])) {
            return *e;
        }
    }
    out.push_back(']');
    out.push_back(',');
    akey(out, "result");
    if (auto e = append_result(out, doc.result)) return *e;
    out.push_back('}');
    return out;
}

// ---------------------------------------------------------------------------
// Minimal recursive-descent JSON parser
// ---------------------------------------------------------------------------

struct JsonNull {};

struct JsonValue;
using JsonArray  = std::vector<JsonValue>;
using JsonObject = std::vector<std::pair<std::string, JsonValue>>;

struct JsonValue {
    enum class Tag { Null, Bool, Number, String, Array, Object } tag;
    bool        b{};
    double      n{};
    std::string number_lexeme;
    std::string s;
    JsonArray   arr;
    JsonObject  obj;

    JsonValue() : tag(Tag::Null) {}

    static JsonValue make_null()              { JsonValue v; v.tag = Tag::Null;   return v; }
    static JsonValue make_bool(bool b)        { JsonValue v; v.tag = Tag::Bool;   v.b = b; return v; }
    static JsonValue make_number(double n, std::string lexeme) {
        JsonValue v;
        v.tag = Tag::Number;
        v.n = n;
        v.number_lexeme = std::move(lexeme);
        return v;
    }
    static JsonValue make_string(std::string s) {
        JsonValue v; v.tag = Tag::String; v.s = std::move(s); return v;
    }
    static JsonValue make_array(JsonArray a)  { JsonValue v; v.tag = Tag::Array;  v.arr = std::move(a); return v; }
    static JsonValue make_object(JsonObject o){ JsonValue v; v.tag = Tag::Object; v.obj = std::move(o); return v; }

    [[nodiscard]] bool is_null()   const { return tag == Tag::Null; }
    [[nodiscard]] bool is_bool()   const { return tag == Tag::Bool; }
    [[nodiscard]] bool is_number() const { return tag == Tag::Number; }
    [[nodiscard]] bool is_string() const { return tag == Tag::String; }
    [[nodiscard]] bool is_array()  const { return tag == Tag::Array; }
    [[nodiscard]] bool is_object() const { return tag == Tag::Object; }

    [[nodiscard]] const JsonValue* find(std::string_view key) const {
        if (!is_object()) return nullptr;
        for (const auto& [k, v] : obj) {
            if (k == key) return &v;
        }
        return nullptr;
    }
};

struct JsonParser {
    std::string_view src;
    std::size_t pos{};

    static Error make_error(std::string msg) {
        return Error{ErrorCode::invalid_argument, std::move(msg)};
    }

    void skip_ws() {
        while (pos < src.size() &&
               (src[pos] == ' ' || src[pos] == '\t' ||
                src[pos] == '\r' || src[pos] == '\n')) {
            ++pos;
        }
    }

    bool at_end() const { return pos >= src.size(); }
    char peek()   const { return pos < src.size() ? src[pos] : '\0'; }
    char advance(){ return pos < src.size() ? src[pos++] : '\0'; }

    bool try_consume(char c) {
        skip_ws();
        if (!at_end() && src[pos] == c) { ++pos; return true; }
        return false;
    }

    Result<JsonValue> parse_value(std::size_t depth = 0U) {
        skip_ws();
        if (at_end()) return make_error("unexpected end of JSON input");
        switch (peek()) {
            case '"': return parse_string_value();
            case '{':
                if (depth >= ensemble_json_max_nesting_depth)
                    return make_error("JSON nesting depth limit exceeded");
                return parse_object(depth + 1U);
            case '[':
                if (depth >= ensemble_json_max_nesting_depth)
                    return make_error("JSON nesting depth limit exceeded");
                return parse_array(depth + 1U);
            case 't': return parse_literal("true",  JsonValue::make_bool(true));
            case 'f': return parse_literal("false", JsonValue::make_bool(false));
            case 'n': return parse_literal("null",  JsonValue::make_null());
            default:  return parse_number();
        }
    }

    Result<JsonValue> parse_literal(std::string_view expected, JsonValue v) {
        if (src.substr(pos, expected.size()) != expected)
            return make_error(std::string("expected ") + std::string(expected));
        pos += expected.size();
        return v;
    }

    Result<JsonValue> parse_number() {
        const std::size_t start = pos;
        if (!at_end() && peek() == '-') ++pos;
        if (at_end()) return make_error("invalid JSON number");
        if (peek() == '0') {
            ++pos;
            if (!at_end() && src[pos] >= '0' && src[pos] <= '9') {
                return make_error("JSON numbers must not contain leading zeroes");
            }
        } else if (peek() >= '1' && peek() <= '9') {
            while (!at_end() && src[pos] >= '0' && src[pos] <= '9') ++pos;
        } else {
            return make_error("expected JSON value");
        }
        if (!at_end() && src[pos] == '.') {
            ++pos;
            const std::size_t fraction_start = pos;
            while (!at_end() && (src[pos] >= '0' && src[pos] <= '9')) ++pos;
            if (pos == fraction_start) {
                return make_error("JSON fraction requires a digit");
            }
        }
        if (!at_end() && (src[pos] == 'e' || src[pos] == 'E')) {
            ++pos;
            if (!at_end() && (src[pos] == '+' || src[pos] == '-')) ++pos;
            const std::size_t exponent_start = pos;
            while (!at_end() && (src[pos] >= '0' && src[pos] <= '9')) ++pos;
            if (pos == exponent_start) {
                return make_error("JSON exponent requires a digit");
            }
        }
        const std::string_view num_str = src.substr(start, pos - start);
        double result{};
        auto [end, ec] = std::from_chars(num_str.data(),
                                          num_str.data() + num_str.size(),
                                          result);
        if (ec != std::errc{} || end != num_str.data() + num_str.size())
            return make_error(std::string("invalid number: ") + std::string(num_str));
        if (!std::isfinite(result)) {
            return make_error("JSON number is outside the finite double range");
        }
        return JsonValue::make_number(result, std::string{num_str});
    }

    Result<std::string> parse_raw_string() {
        if (advance() != '"')
            return make_error("expected '\"'");
        std::string result;
        while (true) {
            if (at_end()) return make_error("unterminated string");
            const char c = advance();
            if (c == '"') break;
            if (c == '\\') {
                if (at_end()) return make_error("unterminated escape");
                const char esc = advance();
                switch (esc) {
                    case '"':  result.push_back('"');  break;
                    case '\\': result.push_back('\\'); break;
                    case '/':  result.push_back('/');  break;
                    case 'b':  result.push_back('\b'); break;
                    case 'f':  result.push_back('\f'); break;
                    case 'n':  result.push_back('\n'); break;
                    case 'r':  result.push_back('\r'); break;
                    case 't':  result.push_back('\t'); break;
                    case 'u': {
                        if (pos + 4 > src.size())
                            return make_error("incomplete \\u escape");
                        std::uint32_t cp{};
                        for (int i = 0; i < 4; ++i) {
                            const char h = advance();
                            cp <<= 4U;
                            if (h >= '0' && h <= '9') cp |= static_cast<std::uint32_t>(h - '0');
                            else if (h >= 'a' && h <= 'f') cp |= static_cast<std::uint32_t>(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') cp |= static_cast<std::uint32_t>(h - 'A' + 10);
                            else return make_error("invalid hex digit in \\u escape");
                        }
                        // surrogate pair
                        if (cp >= 0xD800U && cp <= 0xDBFFU) {
                            if (pos + 6 > src.size() || src[pos] != '\\' || src[pos+1] != 'u')
                                return make_error("expected low surrogate");
                            pos += 2;
                            std::uint32_t low{};
                            for (int i = 0; i < 4; ++i) {
                                const char h = advance();
                                low <<= 4U;
                                if (h >= '0' && h <= '9') low |= static_cast<std::uint32_t>(h - '0');
                                else if (h >= 'a' && h <= 'f') low |= static_cast<std::uint32_t>(h - 'a' + 10);
                                else if (h >= 'A' && h <= 'F') low |= static_cast<std::uint32_t>(h - 'A' + 10);
                                else return make_error("invalid hex digit in surrogate");
                            }
                            if (low < 0xDC00U || low > 0xDFFFU)
                                return make_error("invalid low surrogate");
                            cp = 0x10000U + ((cp - 0xD800U) << 10U) + (low - 0xDC00U);
                        } else if (cp >= 0xDC00U && cp <= 0xDFFFU) {
                            return make_error("unexpected low surrogate");
                        }
                        serialization_detail::append_utf8(result, cp);
                        break;
                    }
                    default:
                        return make_error(std::string("unknown escape: \\") + esc);
                }
            } else {
                if (static_cast<unsigned char>(c) < 0x20U)
                    return make_error("unescaped control character in string");
                result.push_back(c);
            }
        }
        return result;
    }

    Result<JsonValue> parse_string_value() {
        auto r = parse_raw_string();
        if (!r) return r.error();
        return JsonValue::make_string(std::move(r.value()));
    }

    Result<JsonValue> parse_array(std::size_t depth) {
        if (advance() != '[') return make_error("expected '['");
        JsonArray arr;
        skip_ws();
        if (!at_end() && peek() == ']') { advance(); return JsonValue::make_array(std::move(arr)); }
        while (true) {
            auto elem = parse_value(depth);
            if (!elem) return elem.error();
            arr.push_back(std::move(elem.value()));
            skip_ws();
            if (!at_end() && peek() == ',') { advance(); continue; }
            if (!try_consume(']')) return make_error("expected ',' or ']' in array");
            break;
        }
        return JsonValue::make_array(std::move(arr));
    }

    Result<JsonValue> parse_object(std::size_t depth) {
        if (advance() != '{') return make_error("expected '{'");
        JsonObject obj;
        skip_ws();
        if (!at_end() && peek() == '}') { advance(); return JsonValue::make_object(std::move(obj)); }
        while (true) {
            skip_ws();
            auto key = parse_raw_string();
            if (!key) return key.error();
            for (const auto& [existing, _] : obj) {
                if (existing == key.value()) {
                    return make_error(
                        "duplicate object field: " + key.value());
                }
            }
            if (!try_consume(':')) return make_error("expected ':' in object");
            auto val = parse_value(depth);
            if (!val) return val.error();
            obj.push_back({std::move(key.value()), std::move(val.value())});
            skip_ws();
            if (!at_end() && peek() == ',') { advance(); continue; }
            if (!try_consume('}')) return make_error("expected ',' or '}' in object");
            break;
        }
        return JsonValue::make_object(std::move(obj));
    }
};

// ---------------------------------------------------------------------------
// Parse helpers
// ---------------------------------------------------------------------------

static Error parse_error(std::string msg) {
    return Error{ErrorCode::invalid_argument, std::move(msg)};
}

// Check that every key in obj is in allowed. Returns error on unknown key.
static std::optional<Error> check_no_unknown_fields(
    const JsonObject& obj, std::initializer_list<std::string_view> allowed) {
    for (const auto& [k, _] : obj) {
        bool found = false;
        for (const auto& a : allowed) {
            if (k == a) { found = true; break; }
        }
        if (!found)
            return parse_error("unknown field: " + k);
    }
    return std::nullopt;
}

static Result<std::string_view> req_string(const JsonObject& obj,
                                            std::string_view key) {
    const JsonValue* v = nullptr;
    for (const auto& [k, val] : obj) { if (k == key) { v = &val; break; } }
    if (!v) return parse_error(std::string("missing required field: ") + std::string(key));
    if (!v->is_string())
        return parse_error(std::string("field '") + std::string(key) + "' must be a string");
    return std::string_view{v->s};
}

static Result<double> req_finite_double(const JsonObject& obj,
                                         std::string_view key) {
    const JsonValue* v = nullptr;
    for (const auto& [k, val] : obj) { if (k == key) { v = &val; break; } }
    if (!v) return parse_error(std::string("missing required field: ") + std::string(key));
    if (!v->is_number())
        return parse_error(std::string("field '") + std::string(key) + "' must be a number");
    if (!std::isfinite(v->n))
        return parse_error(std::string("field '") + std::string(key) + "' must be finite");
    return v->n;
}

static Result<std::int64_t> req_int64(const JsonObject& obj,
                                       std::string_view key) {
    const JsonValue* value = nullptr;
    for (const auto& [field, candidate] : obj) {
        if (field == key) {
            value = &candidate;
            break;
        }
    }
    if (!value) {
        return parse_error(
            std::string("missing required field: ") + std::string(key));
    }
    if (!value->is_number()) {
        return parse_error(
            std::string("field '") + std::string(key) +
            "' must be an integer");
    }
    std::int64_t parsed{};
    const char* const begin = value->number_lexeme.data();
    const char* const end = begin + value->number_lexeme.size();
    const auto conversion = std::from_chars(begin, end, parsed);
    if (conversion.ec != std::errc{} || conversion.ptr != end) {
        return parse_error(
            std::string("field '") + std::string(key) +
            "' must be an in-range integer");
    }
    return parsed;
}

template <typename Integer>
    requires std::is_unsigned_v<Integer>
static Result<Integer> req_unsigned_integer(
    const JsonObject& obj,
    std::string_view key) {
    const JsonValue* value = nullptr;
    for (const auto& [field, candidate] : obj) {
        if (field == key) {
            value = &candidate;
            break;
        }
    }
    if (!value) {
        return parse_error(
            std::string("missing required field: ") + std::string(key));
    }
    if (!value->is_number()) {
        return parse_error(
            std::string("field '") + std::string(key) +
            "' must be an unsigned integer");
    }
    Integer parsed{};
    const char* const begin = value->number_lexeme.data();
    const char* const end = begin + value->number_lexeme.size();
    const auto conversion = std::from_chars(begin, end, parsed);
    if (conversion.ec != std::errc{} || conversion.ptr != end) {
        return parse_error(
            std::string("field '") + std::string(key) +
            "' must be an in-range unsigned integer");
    }
    return parsed;
}

static Result<bool> req_bool(const JsonObject& obj, std::string_view key) {
    const JsonValue* v = nullptr;
    for (const auto& [k, val] : obj) { if (k == key) { v = &val; break; } }
    if (!v) return parse_error(std::string("missing required field: ") + std::string(key));
    if (!v->is_bool())
        return parse_error(std::string("field '") + std::string(key) + "' must be a boolean");
    return v->b;
}

static Result<const JsonObject*> req_object(const JsonObject& obj,
                                              std::string_view key) {
    const JsonValue* v = nullptr;
    for (const auto& [k, val] : obj) { if (k == key) { v = &val; break; } }
    if (!v) return parse_error(std::string("missing required field: ") + std::string(key));
    if (!v->is_object())
        return parse_error(std::string("field '") + std::string(key) + "' must be an object");
    return &v->obj;
}

static Result<const JsonArray*> req_array(const JsonObject& obj,
                                           std::string_view key) {
    const JsonValue* v = nullptr;
    for (const auto& [k, val] : obj) { if (k == key) { v = &val; break; } }
    if (!v) return parse_error(std::string("missing required field: ") + std::string(key));
    if (!v->is_array())
        return parse_error(std::string("field '") + std::string(key) + "' must be an array");
    return &v->arr;
}

// Optional helpers

static Result<std::optional<double>> opt_finite_double(
    const JsonObject& obj,
    std::string_view key) {
    for (const auto& [k, v] : obj) {
        if (k == key) {
            if (!v.is_number() || !std::isfinite(v.n)) {
                return parse_error(
                    std::string("field '") + std::string(key) +
                    "' must be a finite number");
            }
            return std::optional<double>{v.n};
        }
    }
    return std::optional<double>{};
}

static bool field_present(const JsonObject& obj, std::string_view key) {
    for (const auto& [k, _] : obj) { if (k == key) return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Parse domain types
// ---------------------------------------------------------------------------

static Result<Coordinate> parse_coordinate(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(obj, {"latitude_degrees",
                                                "longitude_degrees"})) {
        return *e;
    }
    auto lat = req_finite_double(obj, "latitude_degrees");
    if (!lat) return lat.error();
    auto lon = req_finite_double(obj, "longitude_degrees");
    if (!lon) return lon.error();
    return Coordinate{lat.value(), lon.value()};
}

static Result<EnsembleObjectiveValue> parse_objective_value(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(obj, {"value_class", "finite_value"}))
        return *e;
    auto vc_str = req_string(obj, "value_class");
    if (!vc_str) return vc_str.error();
    EnsembleObjectiveValueClass vc{};
    if (vc_str.value() == "finite") {
        vc = EnsembleObjectiveValueClass::finite;
    } else if (vc_str.value() == "positive_infinity") {
        vc = EnsembleObjectiveValueClass::positive_infinity;
    } else {
        return parse_error("unknown value_class: " + std::string(vc_str.value()));
    }
    EnsembleObjectiveValue result;
    result.value_class = vc;
    if (vc == EnsembleObjectiveValueClass::finite) {
        auto fv = req_finite_double(obj, "finite_value");
        if (!fv) return fv.error();
        result.finite_value = fv.value();
    } else if (field_present(obj, "finite_value")) {
        return parse_error(
            "positive_infinity objective values must not contain finite_value");
    }
    return result;
}

static Result<EnsembleMemberOutcome> parse_member_outcome(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(obj, {"member_identifier",
                                               "outcome_class",
                                               "elapsed_arrival_seconds",
                                               "error"})) {
        return *e;
    }
    auto mid = req_string(obj, "member_identifier");
    if (!mid) return mid.error();
    auto oc_str = req_string(obj, "outcome_class");
    if (!oc_str) return oc_str.error();
    EnsembleMemberOutcomeClass oc{};
    const auto oc_sv = oc_str.value();
    if (oc_sv == "reached")              oc = EnsembleMemberOutcomeClass::reached;
    else if (oc_sv == "forecast_exhausted") oc = EnsembleMemberOutcomeClass::forecast_exhausted;
    else if (oc_sv == "duration_exhausted") oc = EnsembleMemberOutcomeClass::duration_exhausted;
    else if (oc_sv == "infeasible_no_route") oc = EnsembleMemberOutcomeClass::infeasible_no_route;
    else if (oc_sv == "missing_data")    oc = EnsembleMemberOutcomeClass::missing_data;
    else if (oc_sv == "provider_failure") oc = EnsembleMemberOutcomeClass::provider_failure;
    else if (oc_sv == "cancelled")       oc = EnsembleMemberOutcomeClass::cancelled;
    else if (oc_sv == "other_error")     oc = EnsembleMemberOutcomeClass::other_error;
    else return parse_error("unknown outcome_class: " + std::string(oc_sv));

    EnsembleMemberOutcome out;
    out.member_identifier = std::string{mid.value()};
    if (out.member_identifier.empty()) {
        return parse_error("member_identifier must not be empty");
    }
    out.outcome_class = oc;
    auto elapsed = opt_finite_double(obj, "elapsed_arrival_seconds");
    if (!elapsed) return elapsed.error();
    out.elapsed_arrival_seconds = elapsed.value();
    if (oc == EnsembleMemberOutcomeClass::reached) {
        if (!out.elapsed_arrival_seconds ||
            *out.elapsed_arrival_seconds < 0.0) {
            return parse_error(
                "reached member outcomes require a non-negative "
                "elapsed_arrival_seconds");
        }
    } else if (out.elapsed_arrival_seconds) {
        return parse_error(
            "incomplete member outcomes must not contain "
            "elapsed_arrival_seconds");
    }

    // Parse optional error field inline to avoid Result<Error> ambiguity.
    if (field_present(obj, "error")) {
        for (const auto& [k, v] : obj) {
            if (k != "error") continue;
            if (!v.is_object())
                return parse_error("field 'error' must be an object");
            if (auto e = check_no_unknown_fields(v.obj, {"code", "message"}))
                return *e;
            auto code_sv = req_string(v.obj, "code");
            if (!code_sv) return code_sv.error();
            auto msg = req_string(v.obj, "message");
            if (!msg) return msg.error();
            const auto code_str = code_sv.value();
            ErrorCode ec{};
            if (code_str == "invalid_argument")      ec = ErrorCode::invalid_argument;
            else if (code_str == "file_io")          ec = ErrorCode::file_io;
            else if (code_str == "grib_decode")      ec = ErrorCode::grib_decode;
            else if (code_str == "unsupported_grib") ec = ErrorCode::unsupported_grib;
            else if (code_str == "incomplete_forecast") ec = ErrorCode::incomplete_forecast;
            else if (code_str == "invalid_polar")    ec = ErrorCode::invalid_polar;
            else if (code_str == "departure_outside_forecast") ec = ErrorCode::departure_outside_forecast;
            else if (code_str == "coordinate_outside_forecast") ec = ErrorCode::coordinate_outside_forecast;
            else if (code_str == "no_route")         ec = ErrorCode::no_route;
            else if (code_str == "forecast_exhausted") ec = ErrorCode::forecast_exhausted;
            else if (code_str == "output_error")     ec = ErrorCode::output_error;
            else if (code_str == "cancelled")        ec = ErrorCode::cancelled;
            else if (code_str == "invalid_environment") ec = ErrorCode::invalid_environment;
            else if (code_str == "environment_data_unavailable") ec = ErrorCode::environment_data_unavailable;
            else return parse_error("unknown error code: " + std::string(code_str));
            out.error = Error{ec, std::string{msg.value()}};
            break;
        }
    }
    return out;
}

static Result<EnsembleRouteAction> parse_route_action(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(obj, {"kind", "target",
                                               "heading_degrees",
                                               "duration_seconds"})) {
        return *e;
    }
    auto kind_str = req_string(obj, "kind");
    if (!kind_str) return kind_str.error();
    EnsembleRouteActionKind kind{};
    const auto ks = kind_str.value();
    if (ks == "target")                  kind = EnsembleRouteActionKind::target;
    else if (ks == "heading_for_duration") kind = EnsembleRouteActionKind::heading_for_duration;
    else if (ks == "wait_for_duration")  kind = EnsembleRouteActionKind::wait_for_duration;
    else return parse_error("unknown action kind: " + std::string(ks));

    auto tgt_obj = req_object(obj, "target");
    if (!tgt_obj) return tgt_obj.error();
    auto tgt = parse_coordinate(*tgt_obj.value());
    if (!tgt) return tgt.error();

    auto hdg = req_finite_double(obj, "heading_degrees");
    if (!hdg) return hdg.error();
    auto dur = req_int64(obj, "duration_seconds");
    if (!dur) return dur.error();
    if (dur.value() < 0) {
        return parse_error("duration_seconds must be non-negative");
    }

    EnsembleRouteAction a;
    a.kind = kind;
    a.target = tgt.value();
    a.heading_degrees = hdg.value();
    a.duration = std::chrono::seconds{dur.value()};
    return a;
}

static Result<RoutePoint> parse_route_point(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"position", "time", "heading_degrees", "boat_speed_knots",
                  "true_wind_speed_knots", "true_wind_direction_degrees",
                  "cumulative_distance_nautical_miles", "environment"})) {
        return *e;
    }
    auto pos_obj = req_object(obj, "position");
    if (!pos_obj) return pos_obj.error();
    auto pos = parse_coordinate(*pos_obj.value());
    if (!pos) return pos.error();

    auto t_str = req_string(obj, "time");
    if (!t_str) return t_str.error();
    auto tp = parse_utc_time(t_str.value());
    if (!tp) return parse_error("invalid time: " + std::string(t_str.value()));

    auto hdg = req_finite_double(obj, "heading_degrees");
    if (!hdg) return hdg.error();
    auto spd = req_finite_double(obj, "boat_speed_knots");
    if (!spd) return spd.error();
    auto tws = req_finite_double(obj, "true_wind_speed_knots");
    if (!tws) return tws.error();
    auto twd = req_finite_double(obj, "true_wind_direction_degrees");
    if (!twd) return twd.error();
    auto cdnm = req_finite_double(obj, "cumulative_distance_nautical_miles");
    if (!cdnm) return cdnm.error();

    RoutePoint rp;
    rp.position = pos.value();
    rp.time = tp.value();
    rp.heading_degrees = hdg.value();
    rp.boat_speed_knots = spd.value();
    rp.true_wind_speed_knots = tws.value();
    rp.true_wind_direction_degrees = twd.value();
    rp.cumulative_distance_nautical_miles = cdnm.value();

    if (field_present(obj, "environment")) {
        for (const auto& [k, v] : obj) {
            if (k != "environment") continue;
            if (!v.is_object())
                return parse_error("field 'environment' must be an object");
            if (auto e = check_no_unknown_fields(
                    v.obj, {"speed_over_ground_knots",
                             "course_over_ground_degrees",
                             "current_east_knots", "current_north_knots",
                             "flat_water_speed_knots",
                             "significant_wave_height_metres",
                             "wave_period_seconds",
                             "relative_wave_angle_degrees",
                             "current_applied", "wave_applied"})) {
                return *e;
            }
            RoutePointEnvironment env;
            auto sog = req_finite_double(v.obj, "speed_over_ground_knots");
            if (!sog) return sog.error();
            env.speed_over_ground_knots = sog.value();
            auto cog = req_finite_double(v.obj, "course_over_ground_degrees");
            if (!cog) return cog.error();
            env.course_over_ground_degrees = cog.value();
            auto ce = req_finite_double(v.obj, "current_east_knots");
            if (!ce) return ce.error();
            env.current_east_knots = ce.value();
            auto cn = req_finite_double(v.obj, "current_north_knots");
            if (!cn) return cn.error();
            env.current_north_knots = cn.value();
            auto fw = req_finite_double(v.obj, "flat_water_speed_knots");
            if (!fw) return fw.error();
            env.flat_water_speed_knots = fw.value();
            auto swh = req_finite_double(v.obj, "significant_wave_height_metres");
            if (!swh) return swh.error();
            env.significant_wave_height_metres = swh.value();
            auto wp = req_finite_double(v.obj, "wave_period_seconds");
            if (!wp) return wp.error();
            env.wave_period_seconds = wp.value();
            auto rwa = req_finite_double(v.obj, "relative_wave_angle_degrees");
            if (!rwa) return rwa.error();
            env.relative_wave_angle_degrees = rwa.value();
            auto ca = req_bool(v.obj, "current_applied");
            if (!ca) return ca.error();
            env.current_applied = ca.value();
            auto wa = req_bool(v.obj, "wave_applied");
            if (!wa) return wa.error();
            env.wave_applied = wa.value();
            rp.environment = env;
            break;
        }
    }
    return rp;
}

static Result<EnvironmentDiagnostics> parse_env_diagnostics(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"current_samples", "current_rejections", "wave_samples",
                  "wave_rejections", "sea_state_evaluations", "land_checks",
                  "land_distance_queries", "land_rejections",
                  "exclusion_checks", "exclusion_geometry_tests",
                  "exclusion_rejections"})) {
        return *e;
    }
    EnvironmentDiagnostics d;
    auto parse_usize = [&](std::string_view k, std::size_t& out) -> std::optional<Error> {
        auto v = req_unsigned_integer<std::size_t>(obj, k);
        if (!v) return v.error();
        out = v.value();
        return std::nullopt;
    };
    if (auto e = parse_usize("current_samples",          d.current_samples)) return *e;
    if (auto e = parse_usize("current_rejections",       d.current_rejections)) return *e;
    if (auto e = parse_usize("wave_samples",             d.wave_samples)) return *e;
    if (auto e = parse_usize("wave_rejections",          d.wave_rejections)) return *e;
    if (auto e = parse_usize("sea_state_evaluations",    d.sea_state_evaluations)) return *e;
    if (auto e = parse_usize("land_checks",              d.land_checks)) return *e;
    if (auto e = parse_usize("land_distance_queries",    d.land_distance_queries)) return *e;
    if (auto e = parse_usize("land_rejections",          d.land_rejections)) return *e;
    if (auto e = parse_usize("exclusion_checks",         d.exclusion_checks)) return *e;
    if (auto e = parse_usize("exclusion_geometry_tests", d.exclusion_geometry_tests)) return *e;
    if (auto e = parse_usize("exclusion_rejections",     d.exclusion_rejections)) return *e;
    return d;
}

static Result<EnsembleObjectiveMemberDiagnostic>
parse_objective_member_diagnostic(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"normalized_weight", "candidate", "rival", "elapsed_arrival",
                  "probability_score", "weighted_contribution",
                  "selected_quantile_member"})) {
        return *e;
    }
    auto nw = req_finite_double(obj, "normalized_weight");
    if (!nw) return nw.error();
    auto cand_obj = req_object(obj, "candidate");
    if (!cand_obj) return cand_obj.error();
    auto cand = parse_member_outcome(*cand_obj.value());
    if (!cand) return cand.error();
    auto ea_obj = req_object(obj, "elapsed_arrival");
    if (!ea_obj) return ea_obj.error();
    auto ea = parse_objective_value(*ea_obj.value());
    if (!ea) return ea.error();
    auto ps = req_finite_double(obj, "probability_score");
    if (!ps) return ps.error();
    auto wc = req_finite_double(obj, "weighted_contribution");
    if (!wc) return wc.error();
    auto sqm = req_bool(obj, "selected_quantile_member");
    if (!sqm) return sqm.error();

    EnsembleObjectiveMemberDiagnostic d;
    d.normalized_weight = nw.value();
    d.candidate = std::move(cand.value());
    d.elapsed_arrival = ea.value();
    d.probability_score = ps.value();
    d.weighted_contribution = wc.value();
    d.selected_quantile_member = sqm.value();

    if (field_present(obj, "rival")) {
        for (const auto& [k, v] : obj) {
            if (k != "rival") continue;
            if (!v.is_object())
                return parse_error("field 'rival' must be an object");
            auto r = parse_member_outcome(v.obj);
            if (!r) return r.error();
            d.rival = std::move(r.value());
            break;
        }
    }
    return d;
}

static Result<EnsembleObjectiveDiagnostics> parse_objective_diagnostics(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"members", "incomplete_member_weight",
                  "weighted_finite_mean_arrival", "worst_finite_arrival",
                  "generated_states", "settled_states",
                  "canonical_action_sequence_identity"})) {
        return *e;
    }
    auto members_arr = req_array(obj, "members");
    if (!members_arr) return members_arr.error();
    auto imw = req_finite_double(obj, "incomplete_member_weight");
    if (!imw) return imw.error();
    auto wfma_obj = req_object(obj, "weighted_finite_mean_arrival");
    if (!wfma_obj) return wfma_obj.error();
    auto wfma = parse_objective_value(*wfma_obj.value());
    if (!wfma) return wfma.error();
    auto wfa_obj = req_object(obj, "worst_finite_arrival");
    if (!wfa_obj) return wfa_obj.error();
    auto wfa = parse_objective_value(*wfa_obj.value());
    if (!wfa) return wfa.error();
    auto gs = req_unsigned_integer<std::size_t>(obj, "generated_states");
    if (!gs) return gs.error();
    auto ss = req_unsigned_integer<std::size_t>(obj, "settled_states");
    if (!ss) return ss.error();
    auto casi = req_string(obj, "canonical_action_sequence_identity");
    if (!casi) return casi.error();

    EnsembleObjectiveDiagnostics d;
    d.incomplete_member_weight = imw.value();
    d.weighted_finite_mean_arrival = wfma.value();
    d.worst_finite_arrival = wfa.value();
    d.generated_states = gs.value();
    d.settled_states = ss.value();
    d.canonical_action_sequence_identity = std::string{casi.value()};

    for (const auto& elem : *members_arr.value()) {
        if (!elem.is_object())
            return parse_error("members array element must be an object");
        auto md = parse_objective_member_diagnostic(elem.obj);
        if (!md) return md.error();
        d.members.push_back(std::move(md.value()));
    }
    return d;
}

static Result<EnsembleObjectiveEvaluation> parse_objective_evaluation(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(obj, {"value", "diagnostics"}))
        return *e;
    auto val_obj = req_object(obj, "value");
    if (!val_obj) return val_obj.error();
    auto val = parse_objective_value(*val_obj.value());
    if (!val) return val.error();
    auto diag_obj = req_object(obj, "diagnostics");
    if (!diag_obj) return diag_obj.error();
    auto diag = parse_objective_diagnostics(*diag_obj.value());
    if (!diag) return diag.error();
    EnsembleObjectiveEvaluation ev;
    ev.value = val.value();
    ev.diagnostics = std::move(diag.value());
    return ev;
}

static Result<EnsembleLatticeDiagnostics> parse_lattice_diagnostics(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"settled_labels", "queued_labels", "generated_labels",
                  "retained_labels", "dominated_labels", "stale_queue_entries",
                  "peak_labels_per_state", "max_labels_per_state",
                  "max_total_labels", "subdivision_level", "zero_heuristic",
                  "refinement_performed"})) {
        return *e;
    }
    auto parse_sz = [&](std::string_view k) -> Result<std::size_t> {
        return req_unsigned_integer<std::size_t>(obj, k);
    };
    EnsembleLatticeDiagnostics d;
    auto sl = parse_sz("settled_labels"); if (!sl) return sl.error(); d.settled_labels = sl.value();
    auto ql = parse_sz("queued_labels"); if (!ql) return ql.error(); d.queued_labels = ql.value();
    auto gl = parse_sz("generated_labels"); if (!gl) return gl.error(); d.generated_labels = gl.value();
    auto rl = parse_sz("retained_labels"); if (!rl) return rl.error(); d.retained_labels = rl.value();
    auto dl = parse_sz("dominated_labels"); if (!dl) return dl.error(); d.dominated_labels = dl.value();
    auto sq = parse_sz("stale_queue_entries"); if (!sq) return sq.error(); d.stale_queue_entries = sq.value();
    auto pk = parse_sz("peak_labels_per_state"); if (!pk) return pk.error(); d.peak_labels_per_state = pk.value();
    auto ml = parse_sz("max_labels_per_state"); if (!ml) return ml.error(); d.max_labels_per_state = ml.value();
    auto mt = parse_sz("max_total_labels"); if (!mt) return mt.error(); d.max_total_labels = mt.value();
    auto sv = parse_sz("subdivision_level"); if (!sv) return sv.error(); d.subdivision_level = sv.value();
    auto zh = req_bool(obj, "zero_heuristic"); if (!zh) return zh.error(); d.zero_heuristic = zh.value();
    auto rp = req_bool(obj, "refinement_performed"); if (!rp) return rp.error(); d.refinement_performed = rp.value();
    return d;
}

static Result<EnsembleBeamDiagnostics> parse_beam_diagnostics(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"expanded_nodes", "generated_nodes", "accepted_nodes",
                  "retained_nodes", "rejected_common_actions",
                  "pruned_by_bucket", "pruned_by_beam", "peak_frontier",
                  "completed_nodes", "beam_width", "max_nodes_per_bucket",
                  "max_steps", "max_total_nodes"})) {
        return *e;
    }
    auto parse_sz = [&](std::string_view k) -> Result<std::size_t> {
        return req_unsigned_integer<std::size_t>(obj, k);
    };
    EnsembleBeamDiagnostics d;
    auto en = parse_sz("expanded_nodes"); if (!en) return en.error(); d.expanded_nodes = en.value();
    auto gn = parse_sz("generated_nodes"); if (!gn) return gn.error(); d.generated_nodes = gn.value();
    auto an = parse_sz("accepted_nodes"); if (!an) return an.error(); d.accepted_nodes = an.value();
    auto rn = parse_sz("retained_nodes"); if (!rn) return rn.error(); d.retained_nodes = rn.value();
    auto rc = parse_sz("rejected_common_actions"); if (!rc) return rc.error(); d.rejected_common_actions = rc.value();
    auto pb = parse_sz("pruned_by_bucket"); if (!pb) return pb.error(); d.pruned_by_bucket = pb.value();
    auto pbm = parse_sz("pruned_by_beam"); if (!pbm) return pbm.error(); d.pruned_by_beam = pbm.value();
    auto pf = parse_sz("peak_frontier"); if (!pf) return pf.error(); d.peak_frontier = pf.value();
    auto cn = parse_sz("completed_nodes"); if (!cn) return cn.error(); d.completed_nodes = cn.value();
    auto bw = parse_sz("beam_width"); if (!bw) return bw.error(); d.beam_width = bw.value();
    auto mn = parse_sz("max_nodes_per_bucket"); if (!mn) return mn.error(); d.max_nodes_per_bucket = mn.value();
    auto ms = parse_sz("max_steps"); if (!ms) return ms.error(); d.max_steps = ms.value();
    auto mt = parse_sz("max_total_nodes"); if (!mt) return mt.error(); d.max_total_nodes = mt.value();
    return d;
}

static Result<EnsembleMemberRouteResult> parse_member_route_result(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"outcome", "points", "environment_diagnostics"})) {
        return *e;
    }
    auto oc_obj = req_object(obj, "outcome");
    if (!oc_obj) return oc_obj.error();
    auto oc = parse_member_outcome(*oc_obj.value());
    if (!oc) return oc.error();
    auto pts_arr = req_array(obj, "points");
    if (!pts_arr) return pts_arr.error();
    auto ed_obj = req_object(obj, "environment_diagnostics");
    if (!ed_obj) return ed_obj.error();
    auto ed = parse_env_diagnostics(*ed_obj.value());
    if (!ed) return ed.error();

    EnsembleMemberRouteResult r;
    r.outcome = std::move(oc.value());
    r.environment_diagnostics = ed.value();
    for (const auto& elem : *pts_arr.value()) {
        if (!elem.is_object())
            return parse_error("points array element must be an object");
        auto pt = parse_route_point(elem.obj);
        if (!pt) return pt.error();
        r.points.push_back(std::move(pt.value()));
    }
    return r;
}

static Result<EnsemblePolicyAlternative> parse_policy_alternative(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"branch_identity", "selected", "requires_re_evaluation",
                  "common_actions", "objective", "member_outcomes",
                  "supporting_member_weight", "wrong_choice_cost"})) {
        return *e;
    }
    auto bi = req_string(obj, "branch_identity");
    if (!bi) return bi.error();
    auto sel = req_bool(obj, "selected");
    if (!sel) return sel.error();
    auto rre = req_bool(obj, "requires_re_evaluation");
    if (!rre) return rre.error();
    auto ca_arr = req_array(obj, "common_actions");
    if (!ca_arr) return ca_arr.error();
    auto obj_obj = req_object(obj, "objective");
    if (!obj_obj) return obj_obj.error();
    auto objv = parse_objective_evaluation(*obj_obj.value());
    if (!objv) return objv.error();
    auto mo_arr = req_array(obj, "member_outcomes");
    if (!mo_arr) return mo_arr.error();
    auto smw = req_finite_double(obj, "supporting_member_weight");
    if (!smw) return smw.error();
    auto wcc_obj = req_object(obj, "wrong_choice_cost");
    if (!wcc_obj) return wcc_obj.error();
    auto wcc = parse_objective_value(*wcc_obj.value());
    if (!wcc) return wcc.error();

    EnsemblePolicyAlternative a;
    a.branch_identity = std::string{bi.value()};
    a.selected = sel.value();
    a.requires_re_evaluation = rre.value();
    a.objective = std::move(objv.value());
    a.supporting_member_weight = smw.value();
    a.wrong_choice_cost = wcc.value();

    for (const auto& elem : *ca_arr.value()) {
        if (!elem.is_object())
            return parse_error("common_actions element must be an object");
        auto act = parse_route_action(elem.obj);
        if (!act) return act.error();
        a.common_actions.push_back(std::move(act.value()));
    }
    for (const auto& elem : *mo_arr.value()) {
        if (!elem.is_object())
            return parse_error("member_outcomes element must be an object");
        auto mo = parse_member_outcome(elem.obj);
        if (!mo) return mo.error();
        a.member_outcomes.push_back(std::move(mo.value()));
    }
    return a;
}

static Result<EnsemblePolicyNode> parse_policy_node(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"node_identity", "canonical_member_positions",
                  "earliest_member_time", "latest_member_time", "terminal",
                  "outgoing_branch_identities"})) {
        return *e;
    }
    auto ni = req_string(obj, "node_identity");
    if (!ni) return ni.error();
    auto cmp_arr = req_array(obj, "canonical_member_positions");
    if (!cmp_arr) return cmp_arr.error();
    auto emt = req_string(obj, "earliest_member_time");
    if (!emt) return emt.error();
    auto lmt = req_string(obj, "latest_member_time");
    if (!lmt) return lmt.error();
    auto term = req_bool(obj, "terminal");
    if (!term) return term.error();
    auto obi_arr = req_array(obj, "outgoing_branch_identities");
    if (!obi_arr) return obi_arr.error();

    auto emt_tp = parse_utc_time(emt.value());
    if (!emt_tp) return parse_error("invalid earliest_member_time");
    auto lmt_tp = parse_utc_time(lmt.value());
    if (!lmt_tp) return parse_error("invalid latest_member_time");

    EnsemblePolicyNode n;
    n.node_identity = std::string{ni.value()};
    n.earliest_member_time = emt_tp.value();
    n.latest_member_time = lmt_tp.value();
    n.terminal = term.value();

    for (const auto& elem : *cmp_arr.value()) {
        if (!elem.is_object())
            return parse_error("canonical_member_positions element must be an object");
        auto coord = parse_coordinate(elem.obj);
        if (!coord) return coord.error();
        n.canonical_member_positions.push_back(coord.value());
    }
    for (const auto& elem : *obi_arr.value()) {
        if (!elem.is_string())
            return parse_error("outgoing_branch_identities element must be a string");
        n.outgoing_branch_identities.push_back(elem.s);
    }
    return n;
}

static Result<EnsemblePolicyBranch> parse_policy_branch(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"branch_identity", "from_node_identity", "to_node_identity",
                  "action", "selected", "requires_re_evaluation",
                  "supporting_member_weight", "wrong_choice_cost"})) {
        return *e;
    }
    auto bi = req_string(obj, "branch_identity");
    if (!bi) return bi.error();
    auto fn = req_string(obj, "from_node_identity");
    if (!fn) return fn.error();
    auto tn = req_string(obj, "to_node_identity");
    if (!tn) return tn.error();
    auto act_obj = req_object(obj, "action");
    if (!act_obj) return act_obj.error();
    auto act = parse_route_action(*act_obj.value());
    if (!act) return act.error();
    auto sel = req_bool(obj, "selected");
    if (!sel) return sel.error();
    auto rre = req_bool(obj, "requires_re_evaluation");
    if (!rre) return rre.error();
    auto smw = req_finite_double(obj, "supporting_member_weight");
    if (!smw) return smw.error();
    auto wcc_obj = req_object(obj, "wrong_choice_cost");
    if (!wcc_obj) return wcc_obj.error();
    auto wcc = parse_objective_value(*wcc_obj.value());
    if (!wcc) return wcc.error();

    EnsemblePolicyBranch b;
    b.branch_identity = std::string{bi.value()};
    b.from_node_identity = std::string{fn.value()};
    b.to_node_identity = std::string{tn.value()};
    b.action = std::move(act.value());
    b.selected = sel.value();
    b.requires_re_evaluation = rre.value();
    b.supporting_member_weight = smw.value();
    b.wrong_choice_cost = wcc.value();
    return b;
}

static Result<EnsemblePolicyGraph> parse_policy_graph(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"schema_revision", "root_node_identity", "nodes", "branches",
                  "alternatives"})) {
        return *e;
    }
    auto sr = req_unsigned_integer<std::uint32_t>(obj, "schema_revision");
    if (!sr) return sr.error();
    auto rni = req_string(obj, "root_node_identity");
    if (!rni) return rni.error();
    auto nodes_arr = req_array(obj, "nodes");
    if (!nodes_arr) return nodes_arr.error();
    auto branches_arr = req_array(obj, "branches");
    if (!branches_arr) return branches_arr.error();
    auto alts_arr = req_array(obj, "alternatives");
    if (!alts_arr) return alts_arr.error();

    EnsemblePolicyGraph g;
    g.schema_revision = sr.value();
    if (g.schema_revision == 0U) {
        return parse_error("policy schema_revision must be positive");
    }
    g.root_node_identity = std::string{rni.value()};

    for (const auto& elem : *nodes_arr.value()) {
        if (!elem.is_object())
            return parse_error("nodes element must be an object");
        auto n = parse_policy_node(elem.obj);
        if (!n) return n.error();
        g.nodes.push_back(std::move(n.value()));
    }
    for (const auto& elem : *branches_arr.value()) {
        if (!elem.is_object())
            return parse_error("branches element must be an object");
        auto b = parse_policy_branch(elem.obj);
        if (!b) return b.error();
        g.branches.push_back(std::move(b.value()));
    }
    for (const auto& elem : *alts_arr.value()) {
        if (!elem.is_object())
            return parse_error("alternatives element must be an object");
        auto a = parse_policy_alternative(elem.obj);
        if (!a) return a.error();
        g.alternatives.push_back(std::move(a.value()));
    }

    // Validate branch node references against node list.
    std::unordered_set<std::string> node_ids;
    for (const auto& n : g.nodes) node_ids.insert(n.node_identity);
    for (const auto& b : g.branches) {
        if (node_ids.count(b.from_node_identity) == 0)
            return parse_error("branch references unknown from_node: " +
                               b.from_node_identity);
        if (node_ids.count(b.to_node_identity) == 0)
            return parse_error("branch references unknown to_node: " +
                               b.to_node_identity);
    }
    return g;
}

static Result<EnsembleDecisionBranch> parse_decision_branch(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"policy_branch_identity", "action", "selected",
                  "requires_re_evaluation", "supporting_member_weight",
                  "wrong_choice_cost"})) {
        return *e;
    }
    auto pbi = req_string(obj, "policy_branch_identity");
    if (!pbi) return pbi.error();
    auto act_obj = req_object(obj, "action");
    if (!act_obj) return act_obj.error();
    auto act = parse_route_action(*act_obj.value());
    if (!act) return act.error();
    auto sel = req_bool(obj, "selected");
    if (!sel) return sel.error();
    auto rre = req_bool(obj, "requires_re_evaluation");
    if (!rre) return rre.error();
    auto smw = req_finite_double(obj, "supporting_member_weight");
    if (!smw) return smw.error();
    auto wcc_obj = req_object(obj, "wrong_choice_cost");
    if (!wcc_obj) return wcc_obj.error();
    auto wcc = parse_objective_value(*wcc_obj.value());
    if (!wcc) return wcc.error();

    EnsembleDecisionBranch b;
    b.policy_branch_identity = std::string{pbi.value()};
    b.action = std::move(act.value());
    b.selected = sel.value();
    b.requires_re_evaluation = rre.value();
    b.supporting_member_weight = smw.value();
    b.wrong_choice_cost = wcc.value();
    return b;
}

static Result<EnsembleDecisionPoint> parse_decision_point(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"decision_identity", "policy_node_identity",
                  "canonical_member_positions", "earliest_time",
                  "latest_commitment_time", "branches"})) {
        return *e;
    }
    auto di = req_string(obj, "decision_identity");
    if (!di) return di.error();
    auto pni = req_string(obj, "policy_node_identity");
    if (!pni) return pni.error();
    auto cmp_arr = req_array(obj, "canonical_member_positions");
    if (!cmp_arr) return cmp_arr.error();
    auto et = req_string(obj, "earliest_time");
    if (!et) return et.error();
    auto lct = req_string(obj, "latest_commitment_time");
    if (!lct) return lct.error();
    auto br_arr = req_array(obj, "branches");
    if (!br_arr) return br_arr.error();

    auto et_tp = parse_utc_time(et.value());
    if (!et_tp) return parse_error("invalid earliest_time");
    auto lct_tp = parse_utc_time(lct.value());
    if (!lct_tp) return parse_error("invalid latest_commitment_time");

    EnsembleDecisionPoint dp;
    dp.decision_identity = std::string{di.value()};
    dp.policy_node_identity = std::string{pni.value()};
    dp.earliest_time = et_tp.value();
    dp.latest_commitment_time = lct_tp.value();

    for (const auto& elem : *cmp_arr.value()) {
        if (!elem.is_object())
            return parse_error("canonical_member_positions element must be an object");
        auto coord = parse_coordinate(elem.obj);
        if (!coord) return coord.error();
        dp.canonical_member_positions.push_back(coord.value());
    }
    for (const auto& elem : *br_arr.value()) {
        if (!elem.is_object())
            return parse_error("branches element must be an object");
        auto b = parse_decision_branch(elem.obj);
        if (!b) return b.error();
        dp.branches.push_back(std::move(b.value()));
    }
    return dp;
}

static Result<EnsembleObjective> parse_objective_spec(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(obj, {"kind", "target",
                                               "rival_outcomes"})) {
        return *e;
    }
    auto kind_str = req_string(obj, "kind");
    if (!kind_str) return kind_str.error();
    EnsembleObjectiveKind kind{};
    const auto ks = kind_str.value();
    if (ks == "weighted_mean_elapsed_arrival")
        kind = EnsembleObjectiveKind::weighted_mean_elapsed_arrival;
    else if (ks == "weighted_p75_elapsed_arrival")
        kind = EnsembleObjectiveKind::weighted_p75_elapsed_arrival;
    else if (ks == "weighted_p90_elapsed_arrival")
        kind = EnsembleObjectiveKind::weighted_p90_elapsed_arrival;
    else if (ks == "probability_before_target")
        kind = EnsembleObjectiveKind::probability_before_target;
    else if (ks == "probability_beating_rival")
        kind = EnsembleObjectiveKind::probability_beating_rival;
    else
        return parse_error("unknown objective kind: " + std::string(ks));

    EnsembleObjective out;
    out.kind = kind;

    if (field_present(obj, "target")) {
        for (const auto& [k, v] : obj) {
            if (k != "target") continue;
            if (!v.is_object())
                return parse_error("field 'target' must be an object");
            if (auto e = check_no_unknown_fields(v.obj, {"elapsed_seconds"}))
                return *e;
            auto es = req_finite_double(v.obj, "elapsed_seconds");
            if (!es) return es.error();
            if (es.value() < 0.0) {
                return parse_error(
                    "objective target elapsed_seconds must be non-negative");
            }
            out.target = EnsembleArrivalTarget{es.value()};
            break;
        }
    }
    if (field_present(obj, "rival_outcomes")) {
        for (const auto& [k, v] : obj) {
            if (k != "rival_outcomes") continue;
            if (!v.is_array())
                return parse_error("field 'rival_outcomes' must be an array");
            for (const auto& elem : v.arr) {
                if (!elem.is_object())
                    return parse_error("rival_outcomes element must be an object");
                auto mo = parse_member_outcome(elem.obj);
                if (!mo) return mo.error();
                out.rival_outcomes.push_back(std::move(mo.value()));
            }
            break;
        }
    }
    switch (kind) {
        case EnsembleObjectiveKind::weighted_mean_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p75_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p90_elapsed_arrival:
            if (out.target || !out.rival_outcomes.empty()) {
                return parse_error(
                    "arrival objectives do not accept target or rival inputs");
            }
            break;
        case EnsembleObjectiveKind::probability_before_target:
            if (!out.target || !out.rival_outcomes.empty()) {
                return parse_error(
                    "probability_before_target requires only a target");
            }
            break;
        case EnsembleObjectiveKind::probability_beating_rival:
            if (out.target || out.rival_outcomes.empty()) {
                return parse_error(
                    "probability_beating_rival requires only rival_outcomes");
            }
            break;
    }
    return out;
}

static Result<EnsembleReevaluationState> parse_re_evaluation_state(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"schema_revision", "prior_run_identifier",
                  "selected_branch_identity", "canonical_branch_identities",
                  "objective", "spatial_tolerance_nautical_miles",
                  "time_tolerance_seconds"})) {
        return *e;
    }
    auto sr = req_unsigned_integer<std::uint32_t>(obj, "schema_revision");
    if (!sr) return sr.error();
    auto pri = req_string(obj, "prior_run_identifier");
    if (!pri) return pri.error();
    auto sbi = req_string(obj, "selected_branch_identity");
    if (!sbi) return sbi.error();
    auto cbi_arr = req_array(obj, "canonical_branch_identities");
    if (!cbi_arr) return cbi_arr.error();
    auto obj_obj = req_object(obj, "objective");
    if (!obj_obj) return obj_obj.error();
    auto objv = parse_objective_spec(*obj_obj.value());
    if (!objv) return objv.error();
    auto stnm = req_finite_double(obj, "spatial_tolerance_nautical_miles");
    if (!stnm) return stnm.error();
    auto tts = req_int64(obj, "time_tolerance_seconds");
    if (!tts) return tts.error();
    if (tts.value() < 0) {
        return parse_error("time_tolerance_seconds must be non-negative");
    }

    EnsembleReevaluationState s;
    s.schema_revision = sr.value();
    if (s.schema_revision == 0U) {
        return parse_error("re_evaluation schema_revision must be positive");
    }
    s.prior_run_identifier = std::string{pri.value()};
    s.selected_branch_identity = std::string{sbi.value()};
    s.objective = std::move(objv.value());
    s.spatial_tolerance_nautical_miles = stnm.value();
    s.time_tolerance = std::chrono::seconds{tts.value()};

    for (const auto& elem : *cbi_arr.value()) {
        if (!elem.is_string())
            return parse_error("canonical_branch_identities element must be a string");
        s.canonical_branch_identities.push_back(elem.s);
    }
    return s;
}

static Result<GeographicBounds> parse_geographic_bounds(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"south_latitude_degrees", "west_longitude_degrees",
                  "north_latitude_degrees", "east_longitude_degrees"})) {
        return *e;
    }
    auto south = req_finite_double(obj, "south_latitude_degrees");
    if (!south) return south.error();
    auto west = req_finite_double(obj, "west_longitude_degrees");
    if (!west) return west.error();
    auto north = req_finite_double(obj, "north_latitude_degrees");
    if (!north) return north.error();
    auto east = req_finite_double(obj, "east_longitude_degrees");
    if (!east) return east.error();
    GeographicBounds bounds{
        south.value(), west.value(), north.value(), east.value()};
    if (bounds.south_latitude_degrees < -90.0 ||
        bounds.north_latitude_degrees > 90.0 ||
        bounds.south_latitude_degrees > bounds.north_latitude_degrees ||
        bounds.west_longitude_degrees < -180.0 ||
        bounds.west_longitude_degrees > 180.0 ||
        bounds.east_longitude_degrees < -180.0 ||
        bounds.east_longitude_degrees > 180.0) {
        return parse_error("invalid geographic bounds");
    }
    return bounds;
}

static Result<ForecastMetadata> parse_forecast_metadata(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"initialization_time", "first_valid_time", "last_valid_time",
                  "latitude_count",
                  "longitude_count", "global_longitude_coverage", "source"})) {
        return *e;
    }
    auto initialization = req_string(obj, "initialization_time");
    if (!initialization) return initialization.error();
    auto first = req_string(obj, "first_valid_time");
    if (!first) return first.error();
    auto last = req_string(obj, "last_valid_time");
    if (!last) return last.error();
    auto latitude_count =
        req_unsigned_integer<std::size_t>(obj, "latitude_count");
    if (!latitude_count) return latitude_count.error();
    auto longitude_count =
        req_unsigned_integer<std::size_t>(obj, "longitude_count");
    if (!longitude_count) return longitude_count.error();
    auto global = req_bool(obj, "global_longitude_coverage");
    if (!global) return global.error();
    auto source = req_string(obj, "source");
    if (!source) return source.error();
    auto first_time = parse_utc_time(first.value());
    if (!first_time) return parse_error("invalid first_valid_time");
    auto last_time = parse_utc_time(last.value());
    if (!last_time) return parse_error("invalid last_valid_time");
    auto initialization_time = parse_utc_time(initialization.value());
    if (!initialization_time)
        return parse_error("invalid forecast initialization_time");
    if (latitude_count.value() == 0U || longitude_count.value() == 0U ||
        first_time.value() > last_time.value()) {
        return parse_error("invalid forecast metadata range or grid size");
    }
    return ForecastMetadata{
        first_time.value(),
        last_time.value(),
        latitude_count.value(),
        longitude_count.value(),
        global.value(),
        std::string{source.value()},
        initialization_time.value()};
}

static Result<ForecastGridIdentity> parse_forecast_grid_identity(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"latitude_count", "longitude_count",
                  "south_latitude_degrees", "west_longitude_degrees",
                  "latitude_step_degrees", "longitude_step_degrees",
                  "global_longitude_coverage",
                  "duplicate_longitude_endpoint", "interpolation_bounds"})) {
        return *e;
    }
    auto latitude_count =
        req_unsigned_integer<std::size_t>(obj, "latitude_count");
    if (!latitude_count) return latitude_count.error();
    auto longitude_count =
        req_unsigned_integer<std::size_t>(obj, "longitude_count");
    if (!longitude_count) return longitude_count.error();
    auto south = req_finite_double(obj, "south_latitude_degrees");
    if (!south) return south.error();
    auto west = req_finite_double(obj, "west_longitude_degrees");
    if (!west) return west.error();
    auto latitude_step = req_finite_double(obj, "latitude_step_degrees");
    if (!latitude_step) return latitude_step.error();
    auto longitude_step = req_finite_double(obj, "longitude_step_degrees");
    if (!longitude_step) return longitude_step.error();
    auto global = req_bool(obj, "global_longitude_coverage");
    if (!global) return global.error();
    auto duplicate = req_bool(obj, "duplicate_longitude_endpoint");
    if (!duplicate) return duplicate.error();

    ForecastGridIdentity grid;
    grid.latitude_count = latitude_count.value();
    grid.longitude_count = longitude_count.value();
    grid.south_latitude_degrees = south.value();
    grid.west_longitude_degrees = west.value();
    grid.latitude_step_degrees = latitude_step.value();
    grid.longitude_step_degrees = longitude_step.value();
    grid.global_longitude_coverage = global.value();
    grid.duplicate_longitude_endpoint = duplicate.value();
    if (grid.latitude_count == 0U || grid.longitude_count == 0U ||
        grid.south_latitude_degrees < -90.0 ||
        grid.south_latitude_degrees > 90.0 ||
        grid.west_longitude_degrees < 0.0 ||
        grid.west_longitude_degrees >= 360.0 ||
        grid.latitude_step_degrees <= 0.0 ||
        grid.longitude_step_degrees <= 0.0) {
        return parse_error("invalid forecast grid identity");
    }
    if (field_present(obj, "interpolation_bounds")) {
        auto bounds_obj = req_object(obj, "interpolation_bounds");
        if (!bounds_obj) return bounds_obj.error();
        auto bounds = parse_geographic_bounds(*bounds_obj.value());
        if (!bounds) return bounds.error();
        grid.interpolation_bounds = bounds.value();
    }
    return grid;
}

static Result<ProviderMetadata> parse_provider_metadata(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"name", "source", "revision"})) {
        return *e;
    }
    auto name = req_string(obj, "name");
    if (!name) return name.error();
    auto source = req_string(obj, "source");
    if (!source) return source.error();
    auto revision = req_string(obj, "revision");
    if (!revision) return revision.error();
    if (name.value().empty()) {
        return parse_error("provider name must not be empty");
    }
    return ProviderMetadata{
        std::string{name.value()},
        std::string{source.value()},
        std::string{revision.value()}};
}

static Result<MissingDataPolicy> parse_missing_data_policy(
    std::string_view value) {
    if (value == "fail_route") return MissingDataPolicy::fail_route;
    if (value == "reject_transition") {
        return MissingDataPolicy::reject_transition;
    }
    return parse_error("unknown missing-data policy: " + std::string{value});
}

static Result<EnvironmentSampling> parse_environment_sampling(
    std::string_view value) {
    if (value == "segment_start") return EnvironmentSampling::segment_start;
    if (value == "midpoint") return EnvironmentSampling::midpoint;
    return parse_error("unknown environment sampling: " + std::string{value});
}

static Result<ExclusionBoundaryPolicy> parse_exclusion_boundary_policy(
    std::string_view value) {
    if (value == "boundary_excluded") {
        return ExclusionBoundaryPolicy::boundary_excluded;
    }
    if (value == "boundary_allowed") {
        return ExclusionBoundaryPolicy::boundary_allowed;
    }
    return parse_error(
        "unknown exclusion boundary policy: " + std::string{value});
}

static Result<RouteEnvironmentMetadata> parse_route_environment_metadata(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"current_provider", "wave_provider", "sea_state_model",
                  "landmask", "exclusions", "current_policy", "wave_policy",
                  "land_policy", "sampling",
                  "land_resolution_nautical_miles",
                  "land_interpolation_error_nautical_miles",
                  "land_clearance_nautical_miles",
                  "exclusion_boundary_policy", "exclusion_zone_count",
                  "exclusion_revision"})) {
        return *e;
    }
    RouteEnvironmentMetadata environment;
    const auto parse_provider =
        [&](std::string_view key,
            std::optional<ProviderMetadata>& output) -> std::optional<Error> {
            if (!field_present(obj, key)) return std::nullopt;
            auto provider_obj = req_object(obj, key);
            if (!provider_obj) return provider_obj.error();
            auto provider = parse_provider_metadata(*provider_obj.value());
            if (!provider) return provider.error();
            output = std::move(provider.value());
            return std::nullopt;
        };
    if (auto e = parse_provider(
            "current_provider", environment.current_provider)) return *e;
    if (auto e = parse_provider(
            "wave_provider", environment.wave_provider)) return *e;
    if (auto e = parse_provider(
            "sea_state_model", environment.sea_state_model)) return *e;
    if (auto e = parse_provider("landmask", environment.landmask)) return *e;
    if (auto e = parse_provider(
            "exclusions", environment.exclusions)) return *e;

    auto current_policy_text = req_string(obj, "current_policy");
    if (!current_policy_text) return current_policy_text.error();
    auto current_policy =
        parse_missing_data_policy(current_policy_text.value());
    if (!current_policy) return current_policy.error();
    auto wave_policy_text = req_string(obj, "wave_policy");
    if (!wave_policy_text) return wave_policy_text.error();
    auto wave_policy = parse_missing_data_policy(wave_policy_text.value());
    if (!wave_policy) return wave_policy.error();
    auto land_policy_text = req_string(obj, "land_policy");
    if (!land_policy_text) return land_policy_text.error();
    auto land_policy = parse_missing_data_policy(land_policy_text.value());
    if (!land_policy) return land_policy.error();
    auto sampling_text = req_string(obj, "sampling");
    if (!sampling_text) return sampling_text.error();
    auto sampling = parse_environment_sampling(sampling_text.value());
    if (!sampling) return sampling.error();
    auto resolution =
        req_finite_double(obj, "land_resolution_nautical_miles");
    if (!resolution) return resolution.error();
    auto interpolation_error =
        req_finite_double(obj, "land_interpolation_error_nautical_miles");
    if (!interpolation_error) return interpolation_error.error();
    auto clearance =
        req_finite_double(obj, "land_clearance_nautical_miles");
    if (!clearance) return clearance.error();
    auto boundary_text = req_string(obj, "exclusion_boundary_policy");
    if (!boundary_text) return boundary_text.error();
    auto boundary = parse_exclusion_boundary_policy(boundary_text.value());
    if (!boundary) return boundary.error();
    auto zone_count =
        req_unsigned_integer<std::size_t>(obj, "exclusion_zone_count");
    if (!zone_count) return zone_count.error();
    auto revision =
        req_unsigned_integer<std::uint64_t>(obj, "exclusion_revision");
    if (!revision) return revision.error();
    if (resolution.value() < 0.0 || interpolation_error.value() < 0.0 ||
        clearance.value() < 0.0) {
        return parse_error("environment distances must be non-negative");
    }
    environment.current_policy = current_policy.value();
    environment.wave_policy = wave_policy.value();
    environment.land_policy = land_policy.value();
    environment.sampling = sampling.value();
    environment.land_resolution_nautical_miles = resolution.value();
    environment.land_interpolation_error_nautical_miles =
        interpolation_error.value();
    environment.land_clearance_nautical_miles = clearance.value();
    environment.exclusion_boundary_policy = boundary.value();
    environment.exclusion_zone_count = zone_count.value();
    environment.exclusion_revision = revision.value();
    return environment;
}

static Result<EnvironmentCoverage> parse_environment_coverage(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"south_latitude_degrees", "north_latitude_degrees",
                  "west_longitude_degrees", "east_longitude_degrees",
                  "first_valid_time", "last_valid_time",
                  "global_longitude_coverage"})) {
        return *e;
    }
    EnvironmentCoverage coverage;
    auto south = opt_finite_double(obj, "south_latitude_degrees");
    if (!south) return south.error();
    coverage.south_latitude_degrees = south.value();
    auto north = opt_finite_double(obj, "north_latitude_degrees");
    if (!north) return north.error();
    coverage.north_latitude_degrees = north.value();
    auto west = opt_finite_double(obj, "west_longitude_degrees");
    if (!west) return west.error();
    coverage.west_longitude_degrees = west.value();
    auto east = opt_finite_double(obj, "east_longitude_degrees");
    if (!east) return east.error();
    coverage.east_longitude_degrees = east.value();
    auto global = req_bool(obj, "global_longitude_coverage");
    if (!global) return global.error();
    coverage.global_longitude_coverage = global.value();
    if (field_present(obj, "first_valid_time")) {
        auto text = req_string(obj, "first_valid_time");
        if (!text) return text.error();
        auto time = parse_utc_time(text.value());
        if (!time) return parse_error("invalid coverage first_valid_time");
        coverage.first_valid_time = time.value();
    }
    if (field_present(obj, "last_valid_time")) {
        auto text = req_string(obj, "last_valid_time");
        if (!text) return text.error();
        auto time = parse_utc_time(text.value());
        if (!time) return parse_error("invalid coverage last_valid_time");
        coverage.last_valid_time = time.value();
    }
    if (coverage.south_latitude_degrees &&
        (*coverage.south_latitude_degrees < -90.0 ||
         *coverage.south_latitude_degrees > 90.0)) {
        return parse_error("invalid coverage south latitude");
    }
    if (coverage.north_latitude_degrees &&
        (*coverage.north_latitude_degrees < -90.0 ||
         *coverage.north_latitude_degrees > 90.0)) {
        return parse_error("invalid coverage north latitude");
    }
    if (coverage.south_latitude_degrees &&
        coverage.north_latitude_degrees &&
        *coverage.south_latitude_degrees >
            *coverage.north_latitude_degrees) {
        return parse_error("coverage latitude bounds are inverted");
    }
    if (coverage.first_valid_time && coverage.last_valid_time &&
        *coverage.first_valid_time > *coverage.last_valid_time) {
        return parse_error("coverage time bounds are inverted");
    }
    return coverage;
}

static Result<EnsembleMemberMetadata> parse_member_metadata(
    const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"identifier", "original_weight", "normalized_weight",
                  "weather", "wind_valid_times", "wind_grid",
                  "configured_variables", "environment", "current_coverage",
                  "wave_coverage"})) {
        return *e;
    }
    auto identifier = req_string(obj, "identifier");
    if (!identifier) return identifier.error();
    auto original_weight = req_finite_double(obj, "original_weight");
    if (!original_weight) return original_weight.error();
    auto normalized_weight = req_finite_double(obj, "normalized_weight");
    if (!normalized_weight) return normalized_weight.error();
    auto weather_obj = req_object(obj, "weather");
    if (!weather_obj) return weather_obj.error();
    auto weather = parse_forecast_metadata(*weather_obj.value());
    if (!weather) return weather.error();
    auto valid_times = req_array(obj, "wind_valid_times");
    if (!valid_times) return valid_times.error();
    auto grid_obj = req_object(obj, "wind_grid");
    if (!grid_obj) return grid_obj.error();
    auto grid = parse_forecast_grid_identity(*grid_obj.value());
    if (!grid) return grid.error();
    auto variables_obj = req_object(obj, "configured_variables");
    if (!variables_obj) return variables_obj.error();
    if (auto e = check_no_unknown_fields(
            *variables_obj.value(),
            {"wind", "currents", "waves", "land", "exclusions"})) {
        return *e;
    }
    auto wind = req_bool(*variables_obj.value(), "wind");
    if (!wind) return wind.error();
    auto currents = req_bool(*variables_obj.value(), "currents");
    if (!currents) return currents.error();
    auto waves = req_bool(*variables_obj.value(), "waves");
    if (!waves) return waves.error();
    auto land = req_bool(*variables_obj.value(), "land");
    if (!land) return land.error();
    auto exclusions = req_bool(*variables_obj.value(), "exclusions");
    if (!exclusions) return exclusions.error();
    if (identifier.value().empty() || original_weight.value() < 0.0 ||
        normalized_weight.value() < 0.0 ||
        normalized_weight.value() > 1.0 || !wind.value()) {
        return parse_error("invalid ensemble member metadata");
    }

    EnsembleMemberMetadata member;
    member.identifier = std::string{identifier.value()};
    member.original_weight = original_weight.value();
    member.normalized_weight = normalized_weight.value();
    member.weather = std::move(weather.value());
    member.wind_grid = std::move(grid.value());
    member.configured_variables = EnsembleVariableCategories{
        wind.value(), currents.value(), waves.value(), land.value(),
        exclusions.value()};
    for (const JsonValue& value : *valid_times.value()) {
        if (!value.is_string()) {
            return parse_error("wind_valid_times elements must be strings");
        }
        auto time = parse_utc_time(value.s);
        if (!time) return parse_error("invalid wind_valid_times element");
        if (!member.wind_valid_times.empty() &&
            time.value() <= member.wind_valid_times.back()) {
            return parse_error(
                "wind_valid_times must be strictly increasing");
        }
        member.wind_valid_times.push_back(time.value());
    }
    if (member.wind_valid_times.empty() ||
        member.wind_valid_times.front() != member.weather.first_valid_time ||
        member.wind_valid_times.back() != member.weather.last_valid_time ||
        member.weather.latitude_count != member.wind_grid.latitude_count ||
        member.weather.longitude_count != member.wind_grid.longitude_count ||
        member.weather.global_longitude_coverage !=
            member.wind_grid.global_longitude_coverage) {
        return parse_error(
            "member weather, valid-time, and grid metadata are inconsistent");
    }
    if (field_present(obj, "environment")) {
        auto environment_obj = req_object(obj, "environment");
        if (!environment_obj) return environment_obj.error();
        auto environment =
            parse_route_environment_metadata(*environment_obj.value());
        if (!environment) return environment.error();
        member.environment = std::move(environment.value());
    }
    if (field_present(obj, "current_coverage")) {
        auto coverage_obj = req_object(obj, "current_coverage");
        if (!coverage_obj) return coverage_obj.error();
        auto coverage = parse_environment_coverage(*coverage_obj.value());
        if (!coverage) return coverage.error();
        member.current_coverage = std::move(coverage.value());
    }
    if (field_present(obj, "wave_coverage")) {
        auto coverage_obj = req_object(obj, "wave_coverage");
        if (!coverage_obj) return coverage_obj.error();
        auto coverage = parse_environment_coverage(*coverage_obj.value());
        if (!coverage) return coverage.error();
        member.wave_coverage = std::move(coverage.value());
    }
    if (member.configured_variables.currents !=
            member.current_coverage.has_value() ||
        member.configured_variables.waves !=
            member.wave_coverage.has_value() ||
        (member.environment.has_value() !=
         (member.configured_variables.currents ||
          member.configured_variables.waves ||
          member.configured_variables.land ||
          member.configured_variables.exclusions))) {
        return parse_error(
            "configured variables and environment attribution are inconsistent");
    }
    return member;
}

static Result<EnsembleRunMetadata> parse_run_metadata(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"run_identifier", "model_identifier", "initialization_time",
                  "attribution", "schema_revision"})) {
        return *e;
    }
    auto ri = req_string(obj, "run_identifier");
    if (!ri) return ri.error();
    auto mi = req_string(obj, "model_identifier");
    if (!mi) return mi.error();
    auto it = req_string(obj, "initialization_time");
    if (!it) return it.error();
    auto attr = req_string(obj, "attribution");
    if (!attr) return attr.error();
    auto srev = req_unsigned_integer<std::uint32_t>(obj, "schema_revision");
    if (!srev) return srev.error();

    auto it_tp = parse_utc_time(it.value());
    if (!it_tp) return parse_error("invalid initialization_time");

    EnsembleRunMetadata m;
    m.run_identifier = std::string{ri.value()};
    m.model_identifier = std::string{mi.value()};
    m.initialization_time = it_tp.value();
    m.attribution = std::string{attr.value()};
    m.schema_revision = srev.value();
    if (m.run_identifier.empty() || m.model_identifier.empty() ||
        m.schema_revision == 0U) {
        return parse_error(
            "run metadata identifiers and schema_revision must be non-empty "
            "and positive");
    }
    return m;
}

static Result<DepartureSource> parse_departure_source(std::string_view sv) {
    if (sv == "explicit_time")        return DepartureSource::explicit_time;
    if (sv == "current_time")         return DepartureSource::current_time;
    if (sv == "forecast_start_fallback") return DepartureSource::forecast_start_fallback;
    return parse_error("unknown departure_source: " + std::string(sv));
}

static Result<EnsembleSolver> parse_solver(std::string_view sv) {
    if (sv == "time_dependent_lattice")
        return EnsembleSolver::time_dependent_lattice;
    if (sv == "experimental_isochrone_beam")
        return EnsembleSolver::experimental_isochrone_beam;
    return parse_error("unknown solver: " + std::string(sv));
}

static Result<EnsembleRouteResult> parse_result(const JsonObject& obj) {
    if (auto e = check_no_unknown_fields(
            obj, {"departure_time", "departure_source", "solver",
                  "common_actions", "canonical_action_sequence_identity",
                  "members", "objective", "lattice_diagnostics",
                  "beam_diagnostics", "policy", "decision_points",
                  "re_evaluation", "experimental"})) {
        return *e;
    }
    auto dt_str = req_string(obj, "departure_time");
    if (!dt_str) return dt_str.error();
    auto dt_tp = parse_utc_time(dt_str.value());
    if (!dt_tp) return parse_error("invalid departure_time");
    auto ds_str = req_string(obj, "departure_source");
    if (!ds_str) return ds_str.error();
    auto ds = parse_departure_source(ds_str.value());
    if (!ds) return ds.error();
    auto sv_str = req_string(obj, "solver");
    if (!sv_str) return sv_str.error();
    auto sv = parse_solver(sv_str.value());
    if (!sv) return sv.error();
    auto ca_arr = req_array(obj, "common_actions");
    if (!ca_arr) return ca_arr.error();
    auto casi = req_string(obj, "canonical_action_sequence_identity");
    if (!casi) return casi.error();
    auto mem_arr = req_array(obj, "members");
    if (!mem_arr) return mem_arr.error();
    auto obj_obj = req_object(obj, "objective");
    if (!obj_obj) return obj_obj.error();
    auto objv = parse_objective_evaluation(*obj_obj.value());
    if (!objv) return objv.error();
    auto ld_obj = req_object(obj, "lattice_diagnostics");
    if (!ld_obj) return ld_obj.error();
    auto ld = parse_lattice_diagnostics(*ld_obj.value());
    if (!ld) return ld.error();
    auto bd_obj = req_object(obj, "beam_diagnostics");
    if (!bd_obj) return bd_obj.error();
    auto bd = parse_beam_diagnostics(*bd_obj.value());
    if (!bd) return bd.error();
    auto pg_obj = req_object(obj, "policy");
    if (!pg_obj) return pg_obj.error();
    auto pg = parse_policy_graph(*pg_obj.value());
    if (!pg) return pg.error();
    auto dp_arr = req_array(obj, "decision_points");
    if (!dp_arr) return dp_arr.error();
    auto re_obj = req_object(obj, "re_evaluation");
    if (!re_obj) return re_obj.error();
    auto re = parse_re_evaluation_state(*re_obj.value());
    if (!re) return re.error();
    auto exp = req_bool(obj, "experimental");
    if (!exp) return exp.error();

    EnsembleRouteResult r;
    r.departure_time = dt_tp.value();
    r.departure_source = ds.value();
    r.solver = sv.value();
    r.canonical_action_sequence_identity = std::string{casi.value()};
    r.objective = std::move(objv.value());
    r.lattice_diagnostics = ld.value();
    r.beam_diagnostics = bd.value();
    r.policy = std::move(pg.value());
    r.re_evaluation = std::move(re.value());
    r.experimental = exp.value();

    for (const auto& elem : *ca_arr.value()) {
        if (!elem.is_object())
            return parse_error("common_actions element must be an object");
        auto act = parse_route_action(elem.obj);
        if (!act) return act.error();
        r.common_actions.push_back(std::move(act.value()));
    }
    for (const auto& elem : *mem_arr.value()) {
        if (!elem.is_object())
            return parse_error("members element must be an object");
        auto m = parse_member_route_result(elem.obj);
        if (!m) return m.error();
        r.members.push_back(std::move(m.value()));
    }
    for (const auto& elem : *dp_arr.value()) {
        if (!elem.is_object())
            return parse_error("decision_points element must be an object");
        auto dp = parse_decision_point(elem.obj);
        if (!dp) return dp.error();
        r.decision_points.push_back(std::move(dp.value()));
    }

    // Member identity consistency: no duplicate member_identifiers in members.
    std::unordered_set<std::string> seen;
    for (const auto& m : r.members) {
        const auto& id = m.outcome.member_identifier;
        if (!seen.insert(id).second)
            return parse_error("duplicate member identifier: " + id);
    }
    return r;
}

static std::optional<Error> validate_objective_value(
    const EnsembleObjectiveValue& value,
    std::string_view context) {
    if (value.is_finite() && !std::isfinite(value.finite_value)) {
        return parse_error(
            std::string{context} + " contains a non-finite finite_value");
    }
    return std::nullopt;
}

static std::optional<Error> validate_member_outcome(
    const EnsembleMemberOutcome& outcome,
    std::string_view context) {
    switch (outcome.outcome_class) {
        case EnsembleMemberOutcomeClass::reached:
        case EnsembleMemberOutcomeClass::forecast_exhausted:
        case EnsembleMemberOutcomeClass::duration_exhausted:
        case EnsembleMemberOutcomeClass::infeasible_no_route:
        case EnsembleMemberOutcomeClass::missing_data:
        case EnsembleMemberOutcomeClass::provider_failure:
        case EnsembleMemberOutcomeClass::cancelled:
        case EnsembleMemberOutcomeClass::other_error:
            break;
        default:
            return parse_error(
                std::string{context} + " has an unknown outcome class");
    }
    if (outcome.error) {
        switch (outcome.error->code) {
            case ErrorCode::invalid_argument:
            case ErrorCode::file_io:
            case ErrorCode::grib_decode:
            case ErrorCode::unsupported_grib:
            case ErrorCode::incomplete_forecast:
            case ErrorCode::invalid_polar:
            case ErrorCode::departure_outside_forecast:
            case ErrorCode::coordinate_outside_forecast:
            case ErrorCode::no_route:
            case ErrorCode::forecast_exhausted:
            case ErrorCode::output_error:
            case ErrorCode::cancelled:
            case ErrorCode::invalid_environment:
            case ErrorCode::environment_data_unavailable:
                break;
            default:
                return parse_error(
                    std::string{context} + " has an unknown error code");
        }
    }
    if (outcome.member_identifier.empty()) {
        return parse_error(
            std::string{context} + " has an empty member identifier");
    }
    if (outcome.outcome_class == EnsembleMemberOutcomeClass::reached) {
        if (!outcome.elapsed_arrival_seconds ||
            !std::isfinite(*outcome.elapsed_arrival_seconds) ||
            *outcome.elapsed_arrival_seconds < 0.0) {
            return parse_error(
                std::string{context} +
                " reached outcome requires a finite non-negative arrival");
        }
    } else if (outcome.elapsed_arrival_seconds) {
        return parse_error(
            std::string{context} +
            " incomplete outcome must not contain an arrival");
    }
    return std::nullopt;
}

static bool identities_match(
    const std::vector<EnsembleMemberOutcome>& outcomes,
    const std::vector<std::string>& expected) {
    if (outcomes.size() != expected.size()) return false;
    for (std::size_t i = 0U; i < expected.size(); ++i) {
        if (outcomes[i].member_identifier != expected[i]) return false;
    }
    return true;
}

static bool objective_values_match(
    const EnsembleObjectiveValue& left,
    const EnsembleObjectiveValue& right) {
    return left.value_class == right.value_class &&
        (left.is_positive_infinity() ||
         left.finite_value == right.finite_value);
}

static bool coordinates_match(
    const Coordinate& left,
    const Coordinate& right) {
    return left.latitude_degrees == right.latitude_degrees &&
        left.longitude_degrees == right.longitude_degrees;
}

static bool coordinate_vectors_match(
    const std::vector<Coordinate>& left,
    const std::vector<Coordinate>& right) {
    if (left.size() != right.size()) return false;
    for (std::size_t index = 0U; index < left.size(); ++index) {
        if (!coordinates_match(left[index], right[index])) return false;
    }
    return true;
}

static bool actions_match(
    const EnsembleRouteAction& left,
    const EnsembleRouteAction& right) {
    if (left.kind != right.kind) return false;
    switch (left.kind) {
        case EnsembleRouteActionKind::target:
            return coordinates_match(left.target, right.target);
        case EnsembleRouteActionKind::heading_for_duration:
            return left.heading_degrees == right.heading_degrees &&
                left.duration == right.duration;
        case EnsembleRouteActionKind::wait_for_duration:
            return left.duration == right.duration;
    }
    return false;
}

static std::optional<Error> validate_objective_evaluation_members(
    const EnsembleObjectiveEvaluation& evaluation,
    const std::vector<EnsembleMemberMetadata>& expected,
    std::string_view context) {
    if (const auto error = validate_objective_value(
            evaluation.value, context)) {
        return error;
    }
    if (evaluation.diagnostics.members.size() != expected.size()) {
        return parse_error(
            std::string{context} +
            " objective diagnostics do not match member count");
    }
    for (std::size_t i = 0U; i < expected.size(); ++i) {
        const auto& member = evaluation.diagnostics.members[i];
        if (member.candidate.member_identifier != expected[i].identifier ||
            (member.rival &&
             member.rival->member_identifier != expected[i].identifier)) {
            return parse_error(
                std::string{context} +
                " objective diagnostics use inconsistent member identities");
        }
        if (const auto error = validate_member_outcome(
                member.candidate, context)) {
            return error;
        }
        if (member.rival) {
            if (const auto error = validate_member_outcome(
                    *member.rival, context)) {
                return error;
            }
        }
        if (!std::isfinite(member.normalized_weight) ||
            member.normalized_weight < 0.0 ||
            member.normalized_weight > 1.0 ||
            member.normalized_weight != expected[i].normalized_weight ||
            !std::isfinite(member.probability_score) ||
            !std::isfinite(member.weighted_contribution)) {
            return parse_error(
                std::string{context} +
                " objective diagnostics contain invalid numeric values");
        }
        if (const auto error = validate_objective_value(
                member.elapsed_arrival, context)) {
            return error;
        }
    }
    return std::nullopt;
}

static std::optional<Error> validate_objective_spec(
    const EnsembleObjective& objective,
    const std::vector<std::string>& expected,
    std::string_view context) {
    switch (objective.kind) {
        case EnsembleObjectiveKind::weighted_mean_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p75_elapsed_arrival:
        case EnsembleObjectiveKind::weighted_p90_elapsed_arrival:
            if (objective.target || !objective.rival_outcomes.empty()) {
                return parse_error(
                    std::string{context} +
                    " arrival objective has target or rival inputs");
            }
            break;
        case EnsembleObjectiveKind::probability_before_target:
            if (!objective.target ||
                !std::isfinite(objective.target->elapsed_seconds) ||
                objective.target->elapsed_seconds < 0.0 ||
                !objective.rival_outcomes.empty()) {
                return parse_error(
                    std::string{context} +
                    " probability-before-target inputs are invalid");
            }
            break;
        case EnsembleObjectiveKind::probability_beating_rival:
            if (objective.target ||
                !identities_match(objective.rival_outcomes, expected)) {
                return parse_error(
                    std::string{context} +
                    " rival objective member identities are invalid");
            }
            for (const auto& outcome : objective.rival_outcomes) {
                if (const auto error = validate_member_outcome(
                        outcome, context)) {
                    return error;
                }
            }
            break;
    }
    return std::nullopt;
}

static std::optional<Error> validate_document(
    const EnsembleRouteDocument& doc) {
    if (doc.metadata.run_identifier.empty() ||
        doc.metadata.model_identifier.empty() ||
        doc.metadata.schema_revision == 0U) {
        return parse_error("invalid run metadata");
    }
    if (doc.member_metadata.empty()) {
        return parse_error("member_metadata must not be empty");
    }

    std::vector<std::string> member_ids;
    member_ids.reserve(doc.member_metadata.size());
    long double normalized_total = 0.0L;
    bool positive_original_weight = false;
    for (const auto& member : doc.member_metadata) {
        if (member.identifier.empty() ||
            (!member_ids.empty() &&
             member.identifier <= member_ids.back()) ||
            !std::isfinite(member.original_weight) ||
            member.original_weight < 0.0 ||
            !std::isfinite(member.normalized_weight) ||
            member.normalized_weight < 0.0 ||
            member.normalized_weight > 1.0 ||
            ((member.original_weight == 0.0) !=
             (member.normalized_weight == 0.0))) {
            return parse_error(
                "member_metadata must use canonical identifiers and valid "
                "weights");
        }
        member_ids.push_back(member.identifier);
        normalized_total +=
            static_cast<long double>(member.normalized_weight);
        positive_original_weight =
            positive_original_weight || member.original_weight > 0.0;
    }
    if (!positive_original_weight ||
        std::abs(normalized_total - 1.0L) >
            64.0L * std::numeric_limits<double>::epsilon()) {
        return parse_error(
            "normalized member weights must sum to one");
    }

    const EnsembleRouteResult& result = doc.result;
    if (result.members.size() != member_ids.size()) {
        return parse_error(
            "result member count does not match member_metadata");
    }
    for (std::size_t i = 0U; i < member_ids.size(); ++i) {
        if (result.members[i].outcome.member_identifier != member_ids[i]) {
            return parse_error(
                "result members are not in canonical metadata order");
        }
        if (const auto error = validate_member_outcome(
                result.members[i].outcome, "result member")) {
            return error;
        }
    }
    if (const auto error = validate_objective_evaluation_members(
            result.objective, doc.member_metadata, "selected result")) {
        return error;
    }
    if ((result.solver == EnsembleSolver::experimental_isochrone_beam) !=
        result.experimental) {
        return parse_error(
            "experimental flag does not match the selected solver");
    }

    const EnsemblePolicyGraph& policy = result.policy;
    if (policy.schema_revision == 0U || policy.nodes.empty() ||
        policy.root_node_identity.empty()) {
        return parse_error("policy graph has no valid root");
    }
    std::unordered_set<std::string> node_ids;
    for (const auto& node : policy.nodes) {
        if (node.node_identity.empty() ||
            !node_ids.insert(node.node_identity).second ||
            node.canonical_member_positions.size() != member_ids.size()) {
            return parse_error(
                "policy nodes have duplicate identities or inconsistent "
                "member positions");
        }
        for (const Coordinate coordinate :
             node.canonical_member_positions) {
            if (!is_valid(coordinate)) {
                return parse_error(
                    "policy node has an invalid member position");
            }
        }
    }
    if (node_ids.count(policy.root_node_identity) == 0U) {
        return parse_error("policy root_node_identity is unknown");
    }

    std::unordered_set<std::string> branch_ids;
    for (const auto& branch : policy.branches) {
        if (branch.branch_identity.empty() ||
            !branch_ids.insert(branch.branch_identity).second ||
            node_ids.count(branch.from_node_identity) == 0U ||
            node_ids.count(branch.to_node_identity) == 0U) {
            return parse_error(
                "policy branches have duplicate identities or unknown nodes");
        }
        if (const auto error = validate_objective_value(
                branch.wrong_choice_cost, "policy branch")) {
            return error;
        }
    }
    for (const auto& node : policy.nodes) {
        std::unordered_set<std::string> declared;
        for (const std::string& branch_id :
             node.outgoing_branch_identities) {
            if (!declared.insert(branch_id).second ||
                branch_ids.count(branch_id) == 0U) {
                return parse_error(
                    "policy node has duplicate or unknown outgoing branches");
            }
        }
        std::unordered_set<std::string> actual;
        for (const auto& branch : policy.branches) {
            if (branch.from_node_identity == node.node_identity) {
                actual.insert(branch.branch_identity);
            }
        }
        if (declared != actual || node.terminal != actual.empty()) {
            return parse_error(
                "policy node outgoing branches or terminal state are "
                "inconsistent");
        }
    }

    std::unordered_map<std::string, std::size_t> node_index;
    node_index.reserve(policy.nodes.size());
    for (std::size_t index = 0U; index < policy.nodes.size(); ++index) {
        node_index.emplace(policy.nodes[index].node_identity, index);
    }
    std::vector<std::vector<std::size_t>> successors(policy.nodes.size());
    std::vector<std::size_t> indegree(policy.nodes.size(), 0U);
    for (const auto& branch : policy.branches) {
        const std::size_t from = node_index.at(branch.from_node_identity);
        const std::size_t to = node_index.at(branch.to_node_identity);
        successors[from].push_back(to);
        ++indegree[to];
    }

    std::vector<bool> reachable(policy.nodes.size(), false);
    std::vector<std::size_t> pending{
        node_index.at(policy.root_node_identity)};
    std::size_t reachable_nodes = 0U;
    while (!pending.empty()) {
        const std::size_t node = pending.back();
        pending.pop_back();
        if (reachable[node]) continue;
        reachable[node] = true;
        ++reachable_nodes;
        for (const std::size_t successor : successors[node]) {
            if (!reachable[successor]) pending.push_back(successor);
        }
    }
    if (reachable_nodes != policy.nodes.size()) {
        return parse_error(
            "policy graph contains nodes unreachable from the root");
    }

    std::vector<std::size_t> ready;
    ready.reserve(policy.nodes.size());
    for (std::size_t index = 0U; index < indegree.size(); ++index) {
        if (indegree[index] == 0U) ready.push_back(index);
    }
    std::size_t ordered_nodes = 0U;
    while (!ready.empty()) {
        const std::size_t node = ready.back();
        ready.pop_back();
        ++ordered_nodes;
        for (const std::size_t successor : successors[node]) {
            --indegree[successor];
            if (indegree[successor] == 0U) ready.push_back(successor);
        }
    }
    if (ordered_nodes != policy.nodes.size()) {
        return parse_error("policy graph contains a cycle");
    }

    std::unordered_set<std::string> alternative_ids;
    std::size_t selected_alternatives = 0U;
    for (const auto& alternative : policy.alternatives) {
        if (alternative.branch_identity.empty() ||
            !alternative_ids.insert(
                alternative.branch_identity).second ||
            !identities_match(
                alternative.member_outcomes, member_ids)) {
            return parse_error(
                "policy alternatives have duplicate identities or "
                "inconsistent members");
        }
        for (const auto& outcome : alternative.member_outcomes) {
            if (const auto error = validate_member_outcome(
                    outcome, "policy alternative")) {
                return error;
            }
        }
        if (const auto error = validate_objective_evaluation_members(
                alternative.objective, doc.member_metadata,
                "policy alternative")) {
            return error;
        }
        if (const auto error = validate_objective_value(
                alternative.wrong_choice_cost, "policy alternative")) {
            return error;
        }
        if (alternative.selected) ++selected_alternatives;
    }
    if (policy.alternatives.empty() || selected_alternatives != 1U) {
        return parse_error(
            "policy alternatives must contain exactly one selected route");
    }

    std::unordered_set<std::string> decision_ids;
    std::unordered_set<std::string> decision_node_ids;
    for (const auto& decision : result.decision_points) {
        if (decision.decision_identity.empty() ||
            !decision_ids.insert(decision.decision_identity).second ||
            !decision_node_ids.insert(
                decision.policy_node_identity).second ||
            node_ids.count(decision.policy_node_identity) == 0U ||
            decision.canonical_member_positions.size() != member_ids.size()) {
            return parse_error(
                "decision points have duplicate identities or nodes, unknown "
                "nodes, or inconsistent member positions");
        }
        std::unordered_set<std::string> decision_branch_ids;
        for (const auto& decision_branch : decision.branches) {
            if (!decision_branch_ids.insert(
                    decision_branch.policy_branch_identity).second) {
                return parse_error(
                    "decision point contains duplicate policy branches");
            }
            const auto branch = std::find_if(
                policy.branches.begin(),
                policy.branches.end(),
                [&](const EnsemblePolicyBranch& candidate) {
                    return candidate.branch_identity ==
                            decision_branch.policy_branch_identity &&
                        candidate.from_node_identity ==
                            decision.policy_node_identity;
                });
            if (branch == policy.branches.end()) {
                return parse_error(
                    "decision point references an unknown policy branch");
            }
            if (!actions_match(decision_branch.action, branch->action) ||
                decision_branch.selected != branch->selected ||
                decision_branch.requires_re_evaluation !=
                    branch->requires_re_evaluation ||
                decision_branch.supporting_member_weight !=
                    branch->supporting_member_weight ||
                !objective_values_match(
                    decision_branch.wrong_choice_cost,
                    branch->wrong_choice_cost)) {
                return parse_error(
                    "decision branch contradicts its policy branch");
            }
        }
        const auto node = std::find_if(
            policy.nodes.begin(),
            policy.nodes.end(),
            [&](const EnsemblePolicyNode& candidate) {
                return candidate.node_identity ==
                    decision.policy_node_identity;
            });
        std::unordered_set<std::string> expected_branches{
            node->outgoing_branch_identities.begin(),
            node->outgoing_branch_identities.end()};
        if (decision.branches.size() < 2U ||
            decision_branch_ids != expected_branches) {
            return parse_error(
                "decision point branches do not match its policy node");
        }
        if (!coordinate_vectors_match(
                decision.canonical_member_positions,
                node->canonical_member_positions) ||
            decision.earliest_time != node->earliest_member_time) {
            return parse_error(
                "decision point state contradicts its policy node");
        }
    }
    for (const EnsemblePolicyNode& node : policy.nodes) {
        const bool branching = node.outgoing_branch_identities.size() >= 2U;
        const bool has_decision =
            decision_node_ids.count(node.node_identity) != 0U;
        if (branching != has_decision) {
            return parse_error(
                "decision points must exactly cover branching policy nodes");
        }
    }

    const EnsembleReevaluationState& reevaluation = result.re_evaluation;
    if (reevaluation.schema_revision == 0U ||
        reevaluation.prior_run_identifier != doc.metadata.run_identifier ||
        !std::isfinite(
            reevaluation.spatial_tolerance_nautical_miles) ||
        reevaluation.spatial_tolerance_nautical_miles < 0.0 ||
        reevaluation.time_tolerance < std::chrono::seconds::zero()) {
        return parse_error("invalid re_evaluation state");
    }
    if (const auto error = validate_objective_spec(
            reevaluation.objective, member_ids, "re_evaluation")) {
        return error;
    }
    for (const EnsembleDecisionPoint& decision : result.decision_points) {
        const EnsemblePolicyNode& node =
            policy.nodes[node_index.at(decision.policy_node_identity)];
        if (decision.latest_commitment_time < decision.earliest_time ||
            decision.latest_commitment_time < node.latest_member_time ||
            decision.latest_commitment_time -
                    node.latest_member_time !=
                reevaluation.time_tolerance) {
            return parse_error(
                "decision commitment deadline contradicts its policy node "
                "and re_evaluation tolerance");
        }
    }
    std::vector<std::string> expected_alternative_ids{
        alternative_ids.begin(), alternative_ids.end()};
    std::sort(
        expected_alternative_ids.begin(), expected_alternative_ids.end());
    if (reevaluation.canonical_branch_identities !=
            expected_alternative_ids ||
        alternative_ids.count(
            reevaluation.selected_branch_identity) == 0U) {
        return parse_error(
            "re_evaluation branch identities do not match policy "
            "alternatives");
    }
    const auto selected = std::find_if(
        policy.alternatives.begin(),
        policy.alternatives.end(),
        [&](const EnsemblePolicyAlternative& alternative) {
            return alternative.branch_identity ==
                reevaluation.selected_branch_identity;
        });
    if (selected == policy.alternatives.end() || !selected->selected) {
        return parse_error(
            "re_evaluation selected branch is not the selected policy "
            "alternative");
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------
// Public: ensemble_route_from_json
// ---------------------------------------------------------------------------

Result<EnsembleRouteDocument> ensemble_route_from_json(std::string_view json) {
    if (json.size() > ensemble_json_max_input_bytes) {
        return parse_error("ensemble JSON input size limit exceeded");
    }
    JsonParser parser{json};
    auto top = parser.parse_value();
    if (!top) return top.error();
    parser.skip_ws();
    if (!parser.at_end())
        return parse_error("trailing content after JSON value");
    if (!top.value().is_object())
        return parse_error("top-level JSON value must be an object");

    const auto& root = top.value().obj;
    if (auto e = check_no_unknown_fields(
            root, {"schema_version", "run_metadata", "member_metadata",
                   "result"})) {
        return *e;
    }
    auto sv = req_string(root, "schema_version");
    if (!sv) return sv.error();
    if (sv.value() != "ensemble_route_result_v1")
        return parse_error("unsupported schema_version: " + std::string(sv.value()));

    auto meta_obj = req_object(root, "run_metadata");
    if (!meta_obj) return meta_obj.error();
    auto meta = parse_run_metadata(*meta_obj.value());
    if (!meta) return meta.error();

    auto members_array = req_array(root, "member_metadata");
    if (!members_array) return members_array.error();
    std::vector<EnsembleMemberMetadata> member_metadata;
    member_metadata.reserve(members_array.value()->size());
    for (const JsonValue& value : *members_array.value()) {
        if (!value.is_object()) {
            return parse_error(
                "member_metadata elements must be objects");
        }
        auto member = parse_member_metadata(value.obj);
        if (!member) return member.error();
        member_metadata.push_back(std::move(member.value()));
    }

    auto res_obj = req_object(root, "result");
    if (!res_obj) return res_obj.error();
    auto res = parse_result(*res_obj.value());
    if (!res) return res.error();

    EnsembleRouteDocument document{
        std::move(meta.value()),
        std::move(member_metadata),
        std::move(res.value())};
    if (const auto validation = validate_document(document)) {
        return *validation;
    }
    return document;
}

// ---------------------------------------------------------------------------
// Rival outcomes serialization
// ---------------------------------------------------------------------------

Result<std::string> ensemble_rival_outcomes_to_json(
    const EnsembleRivalOutcomesDocument& doc) {
    std::unordered_set<std::string> member_identifiers;
    for (const auto& outcome : doc.member_outcomes) {
        if (const auto validation =
                validate_member_outcome(outcome, "rival outcome")) {
            return *validation;
        }
        if (!member_identifiers.insert(
                outcome.member_identifier).second) {
            return parse_error(
                "duplicate rival member identifier: " +
                outcome.member_identifier);
        }
    }
    std::string out;
    out.reserve(4096);
    out.push_back('{');
    afield(out, "schema_version", "ensemble_rival_outcomes_v1");
    out.push_back(',');
    out.append("\"member_outcomes\":[");
    for (std::size_t i = 0; i < doc.member_outcomes.size(); ++i) {
        if (i > 0) out.push_back(',');
        if (auto e = append_member_outcome(out, doc.member_outcomes[i]))
            return *e;
    }
    out.push_back(']');
    out.push_back('}');
    return out;
}

// ---------------------------------------------------------------------------
// Public: ensemble_rival_outcomes_from_json
// ---------------------------------------------------------------------------

Result<EnsembleRivalOutcomesDocument> ensemble_rival_outcomes_from_json(
    std::string_view json) {
    if (json.size() > ensemble_json_max_input_bytes) {
        return parse_error("ensemble JSON input size limit exceeded");
    }
    JsonParser parser{json};
    auto top = parser.parse_value();
    if (!top) return top.error();
    parser.skip_ws();
    if (!parser.at_end())
        return parse_error("trailing content after JSON value");
    if (!top.value().is_object())
        return parse_error("top-level JSON value must be an object");

    const auto& root = top.value().obj;
    if (auto e = check_no_unknown_fields(root, {"schema_version",
                                                "member_outcomes"})) {
        return *e;
    }
    auto sv = req_string(root, "schema_version");
    if (!sv) return sv.error();
    if (sv.value() != "ensemble_rival_outcomes_v1")
        return parse_error("unsupported schema_version: " +
                           std::string(sv.value()));

    auto mo_arr = req_array(root, "member_outcomes");
    if (!mo_arr) return mo_arr.error();

    EnsembleRivalOutcomesDocument doc;
    for (const auto& elem : *mo_arr.value()) {
        if (!elem.is_object())
            return parse_error("member_outcomes element must be an object");
        auto mo = parse_member_outcome(elem.obj);
        if (!mo) return mo.error();
        doc.member_outcomes.push_back(std::move(mo.value()));
    }

    // No duplicate member identifiers.
    std::unordered_set<std::string> seen;
    for (const auto& mo : doc.member_outcomes) {
        if (!seen.insert(mo.member_identifier).second)
            return parse_error("duplicate member identifier: " +
                               mo.member_identifier);
    }
    return doc;
}

}  // namespace sailroute
