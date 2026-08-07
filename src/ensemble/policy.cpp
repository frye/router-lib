#include "ensemble/policy.hpp"

#include <algorithm>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>

namespace sailroute::detail {
namespace {

struct Candidate {
    std::size_t terminal{};
    std::vector<std::size_t> path;
    std::vector<EnsembleRouteAction> actions;
    std::vector<EnsembleMemberOutcome> outcomes;
    EnsembleObjectiveEvaluation objective;
    std::string identity;
    double support{};
    EnsembleObjectiveValue cost;
    bool selected{};
};

[[nodiscard]] Error invalid_policy(std::string message) {
    return Error{ErrorCode::invalid_argument, std::move(message)};
}

void append_unsigned(std::string& output, std::uint64_t value) {
    char buffer[32];
    const auto [end, error] = std::to_chars(
        std::begin(buffer), std::end(buffer), value, 16);
    if (error == std::errc{}) {
        output.append(buffer, end);
    }
    output.push_back('|');
}

void append_double(std::string& output, double value) {
    append_unsigned(output, std::bit_cast<std::uint64_t>(value));
}

[[nodiscard]] std::string stable_identity(
    std::string_view prefix,
    std::string_view content) {
    std::uint64_t hash = 14695981039346656037ULL;
    for (const char raw_character : content) {
        const auto character = static_cast<unsigned char>(raw_character);
        hash ^= character;
        hash *= 1099511628211ULL;
    }
    std::string result{prefix};
    char buffer[16];
    for (int shift = 60; shift >= 0; shift -= 4) {
        const unsigned digit = static_cast<unsigned>((hash >> shift) & 0xFULL);
        buffer[(60 - shift) / 4] =
            static_cast<char>(digit < 10U ? '0' + digit : 'a' + digit - 10U);
    }
    result.append(buffer, sizeof(buffer));
    return result;
}

[[nodiscard]] EnsembleRouteAction public_action(
    const EnsembleCommonAction& action) {
    EnsembleRouteAction result;
    result.target = action.target;
    result.heading_degrees = action.heading_degrees;
    result.duration = action.duration;
    switch (action.kind) {
        case EnsembleCommonActionKind::target:
            result.kind = EnsembleRouteActionKind::target;
            break;
        case EnsembleCommonActionKind::heading_for_duration:
            result.kind = EnsembleRouteActionKind::heading_for_duration;
            break;
        case EnsembleCommonActionKind::wait_for_duration:
            result.kind = EnsembleRouteActionKind::wait_for_duration;
            break;
    }
    return result;
}

[[nodiscard]] Result<std::vector<std::size_t>> ancestry(
    std::span<const EnsembleSearchLabel> labels,
    std::size_t terminal) {
    if (terminal >= labels.size()) {
        return invalid_policy("policy terminal label is outside label storage");
    }
    std::vector<std::size_t> reversed;
    std::set<std::size_t> visited;
    std::size_t current = terminal;
    while (current != no_ensemble_label) {
        if (current >= labels.size() || !visited.insert(current).second) {
            return invalid_policy("policy label ancestry is malformed");
        }
        reversed.push_back(current);
        current = labels[current].parent_label;
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

[[nodiscard]] std::string node_content(
    const EnsembleSearchLabel& label) {
    std::string content;
    for (const EnsembleMemberSearchState& member : label.members) {
        content.append(member.member_identifier);
        content.push_back('|');
        append_double(content, member.point.position.latitude_degrees);
        append_double(content, member.point.position.longitude_degrees);
        append_double(content, member.point.heading_degrees);
        append_double(content, member.point.boat_speed_knots);
        append_double(content, member.point.true_wind_speed_knots);
        append_double(content, member.point.true_wind_direction_degrees);
        append_double(
            content, member.point.cumulative_distance_nautical_miles);
        append_unsigned(content, member.point.environment ? 1U : 0U);
        if (member.point.environment) {
            const RoutePointEnvironment& environment =
                *member.point.environment;
            append_double(content, environment.speed_over_ground_knots);
            append_double(content, environment.course_over_ground_degrees);
            append_double(content, environment.current_east_knots);
            append_double(content, environment.current_north_knots);
            append_double(content, environment.flat_water_speed_knots);
            append_double(
                content, environment.significant_wave_height_metres);
            append_double(content, environment.wave_period_seconds);
            append_double(
                content, environment.relative_wave_angle_degrees);
            append_unsigned(content, environment.current_applied ? 1U : 0U);
            append_unsigned(content, environment.wave_applied ? 1U : 0U);
        }
        append_unsigned(
            content,
            static_cast<std::uint64_t>(
                member.point.time.time_since_epoch().count()));
        append_unsigned(content, static_cast<std::uint64_t>(member.status));
        append_unsigned(
            content, static_cast<std::uint64_t>(member.configuration.board));
        append_unsigned(content, member.configuration.sail);
        append_unsigned(content, member.configuration.reef);
        append_unsigned(
            content,
            member.outcome_class
                ? static_cast<std::uint64_t>(*member.outcome_class) + 1U
                : 0U);
        append_unsigned(
            content,
            member.error
                ? static_cast<std::uint64_t>(member.error->code) + 1U
                : 0U);
    }
    return content;
}

[[nodiscard]] int compare_member_outcomes(
    const EnsembleMemberOutcome& left,
    const EnsembleMemberOutcome& right) noexcept {
    const bool left_reached =
        left.outcome_class == EnsembleMemberOutcomeClass::reached;
    const bool right_reached =
        right.outcome_class == EnsembleMemberOutcomeClass::reached;
    if (left_reached != right_reached) {
        return left_reached ? -1 : 1;
    }
    if (left_reached) {
        if (*left.elapsed_arrival_seconds < *right.elapsed_arrival_seconds) {
            return -1;
        }
        if (*left.elapsed_arrival_seconds > *right.elapsed_arrival_seconds) {
            return 1;
        }
        return 0;
    }
    if (left.outcome_class < right.outcome_class) {
        return -1;
    }
    if (left.outcome_class > right.outcome_class) {
        return 1;
    }
    return 0;
}

[[nodiscard]] int compare_member_candidates(
    const Candidate& left,
    const Candidate& right,
    EnsembleObjectiveKind kind,
    std::size_t member) noexcept {
    if (objective_direction(kind) == EnsembleObjectiveDirection::maximize) {
        const double left_score =
            left.objective.diagnostics.members[member].probability_score;
        const double right_score =
            right.objective.diagnostics.members[member].probability_score;
        if (left_score > right_score) {
            return -1;
        }
        if (left_score < right_score) {
            return 1;
        }
        return 0;
    }
    return compare_member_outcomes(
        left.outcomes[member], right.outcomes[member]);
}

[[nodiscard]] EnsembleObjectiveValue wrong_choice_cost(
    EnsembleObjectiveKind kind,
    const EnsembleObjectiveValue& selected,
    const EnsembleObjectiveValue& alternative) noexcept {
    if (selected.is_positive_infinity() ||
        alternative.is_positive_infinity()) {
        if (selected.value_class == alternative.value_class) {
            return EnsembleObjectiveValue{};
        }
        const bool minimize =
            objective_direction(kind) == EnsembleObjectiveDirection::minimize;
        const bool infinite_cost =
            (minimize && alternative.is_positive_infinity()) ||
            (!minimize && selected.is_positive_infinity());
        return infinite_cost
            ? EnsembleObjectiveValue{
                  EnsembleObjectiveValueClass::positive_infinity, 0.0}
            : EnsembleObjectiveValue{};
    }
    const double raw =
        objective_direction(kind) == EnsembleObjectiveDirection::minimize
        ? alternative.finite_value - selected.finite_value
        : selected.finite_value - alternative.finite_value;
    return EnsembleObjectiveValue{
        EnsembleObjectiveValueClass::finite, std::max(0.0, raw)};
}

[[nodiscard]] bool cost_less(
    const EnsembleObjectiveValue& left,
    const EnsembleObjectiveValue& right) noexcept {
    if (left.value_class != right.value_class) {
        return left.is_finite();
    }
    return left.is_finite() && left.finite_value < right.finite_value;
}

}  // namespace

bool policy_terminal_dominates(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& left,
    const EnsembleSearchLabel& right) noexcept {
    if (left.members.size() != right.members.size() ||
        left.members.size() != dataset.member_count() ||
        left.members.empty()) {
        return false;
    }
    bool strict = false;
    for (std::size_t index = 0U; index < left.members.size(); ++index) {
        if (dataset.members()[index].normalized_weight == 0.0) {
            continue;
        }
        const EnsembleMemberSearchState& left_member = left.members[index];
        const EnsembleMemberSearchState& right_member = right.members[index];
        if (left_member.member_identifier != right_member.member_identifier ||
            left_member.status == EnsembleMemberSearchStatus::active ||
            right_member.status == EnsembleMemberSearchStatus::active) {
            return false;
        }
        if (left_member.status != right_member.status) {
            if (left_member.status == EnsembleMemberSearchStatus::completed) {
                strict = true;
                continue;
            }
            return false;
        }
        if (left_member.status == EnsembleMemberSearchStatus::completed) {
            if (left_member.point.time > right_member.point.time) {
                return false;
            }
            strict = strict ||
                left_member.point.time < right_member.point.time;
            continue;
        }
        if (!left_member.outcome_class || !right_member.outcome_class ||
            *left_member.outcome_class > *right_member.outcome_class) {
            return false;
        }
        strict = strict ||
            *left_member.outcome_class < *right_member.outcome_class;
    }
    return strict;
}

Result<EnsemblePolicyBuildResult> build_ensemble_policy(
    const EnsembleDataset& dataset,
    const EnsembleObjective& objective,
    const EnsemblePolicyOptions& options,
    std::span<const EnsembleSearchLabel> labels,
    std::span<const std::size_t> terminal_labels,
    std::size_t selected_terminal,
    TimePoint departure) {
    if (terminal_labels.empty() || selected_terminal >= labels.size() ||
        !labels[selected_terminal].aggregate_objective.has_value()) {
        return invalid_policy(
            "policy construction requires a selected terminal candidate");
    }
    if (!std::isfinite(
            options.commitment_spatial_tolerance_nautical_miles) ||
        options.commitment_spatial_tolerance_nautical_miles < 0.0 ||
        options.commitment_time_tolerance < std::chrono::minutes::zero()) {
        return invalid_policy("policy commitment tolerances must be non-negative");
    }

    std::vector<std::size_t> ordered{
        terminal_labels.begin(), terminal_labels.end()};
    for (const std::size_t terminal : ordered) {
        if (terminal >= labels.size() ||
            !labels[terminal].aggregate_objective.has_value()) {
            return invalid_policy(
                "policy terminal is missing an objective evaluation");
        }
    }
    if (std::find(ordered.begin(), ordered.end(), selected_terminal) ==
        ordered.end()) {
        ordered.push_back(selected_terminal);
    }
    std::sort(
        ordered.begin(),
        ordered.end(),
        [&](std::size_t left, std::size_t right) {
            if (left == right) {
                return false;
            }
            const auto& left_objective = labels[left].aggregate_objective;
            const auto& right_objective = labels[right].aggregate_objective;
            if (!left_objective || !right_objective) {
                return static_cast<bool>(left_objective);
            }
            const int comparison = compare_ensemble_objective_evaluations(
                objective.kind, *left_objective, *right_objective);
            return comparison != 0
                ? comparison < 0
                : ensemble_label_less(labels[left], labels[right]);
        });
    ordered.erase(std::unique(ordered.begin(), ordered.end()), ordered.end());
    const std::size_t maximum_candidates =
        options.max_alternatives == std::numeric_limits<std::size_t>::max()
        ? options.max_alternatives
        : options.max_alternatives + 1U;
    if (ordered.size() > maximum_candidates) {
        ordered.resize(maximum_candidates);
    }
    if (std::find(ordered.begin(), ordered.end(), selected_terminal) ==
        ordered.end()) {
        if (ordered.empty()) {
            ordered.push_back(selected_terminal);
        } else {
            ordered.back() = selected_terminal;
        }
    }

    const EnsembleObjectiveEvaluation& selected_objective =
        *labels[selected_terminal].aggregate_objective;
    std::vector<Candidate> candidates;
    candidates.reserve(ordered.size());
    for (const std::size_t terminal : ordered) {
        if (terminal >= labels.size() ||
            !labels[terminal].aggregate_objective.has_value()) {
            return invalid_policy(
                "policy terminal is missing an objective evaluation");
        }
        auto path = ancestry(labels, terminal);
        auto outcomes = ensemble_label_outcomes(
            dataset, labels[terminal], departure);
        auto actions = reconstruct_common_actions(labels, terminal);
        if (!path) {
            return path.error();
        }
        if (!outcomes) {
            return outcomes.error();
        }
        if (!actions) {
            return actions.error();
        }
        if (path.value().empty()) {
            return invalid_policy("policy candidate has an empty ancestry");
        }
        for (std::size_t depth = 0U; depth < path.value().size(); ++depth) {
            const EnsembleSearchLabel& path_label =
                labels[path.value()[depth]];
            if (path_label.members.size() != dataset.member_count()) {
                return invalid_policy(
                    "policy label member vector does not match the dataset");
            }
            for (std::size_t member = 0U;
                 member < path_label.members.size();
                 ++member) {
                if (path_label.members[member].member_identifier !=
                    dataset.members()[member].identifier) {
                    return invalid_policy(
                        "policy label member order is not canonical");
                }
            }
            if ((depth == 0U) !=
                (path_label.parent_label == no_ensemble_label) ||
                (depth > 0U && !path_label.incoming_action.has_value())) {
                return invalid_policy(
                    "policy label ancestry is missing its incoming action");
            }
        }
        Candidate candidate;
        candidate.terminal = terminal;
        candidate.path = std::move(path.value());
        candidate.outcomes = std::move(outcomes.value());
        candidate.objective = *labels[terminal].aggregate_objective;
        candidate.identity = stable_identity(
            "alternative-",
            labels[terminal].canonical_action_sequence_identity);
        candidate.cost = wrong_choice_cost(
            objective.kind,
            selected_objective.value,
            candidate.objective.value);
        candidate.selected = terminal == selected_terminal;
        candidate.actions.reserve(actions.value().size());
        for (const EnsembleCommonAction& action : actions.value()) {
            candidate.actions.push_back(public_action(action));
        }
        candidates.push_back(std::move(candidate));
    }

    for (std::size_t member = 0U; member < dataset.member_count(); ++member) {
        std::vector<std::size_t> winners;
        for (std::size_t index = 0U; index < candidates.size(); ++index) {
            if (winners.empty()) {
                winners.push_back(index);
                continue;
            }
            const int comparison = compare_member_candidates(
                candidates[index],
                candidates[winners.front()],
                objective.kind,
                member);
            if (comparison < 0) {
                winners.assign(1U, index);
            } else if (comparison == 0) {
                winners.push_back(index);
            }
        }
        const double share =
            dataset.members()[member].normalized_weight /
            static_cast<double>(winners.size());
        for (const std::size_t winner : winners) {
            candidates[winner].support += share;
        }
    }

    EnsemblePolicyBuildResult result;
    result.graph.schema_revision = 1U;
    std::map<std::string, std::size_t> node_by_content;
    std::map<std::string, std::string> node_content_by_identity;
    std::map<std::string, std::size_t> branch_by_content;
    std::map<std::string, std::string> branch_content_by_identity;
    std::map<std::string, std::string> alternative_content_by_identity;
    std::map<std::string, std::vector<std::size_t>> branch_candidates;

    const auto ensure_node = [&](const EnsembleSearchLabel& label)
        -> Result<std::size_t> {
        if (label.members.empty()) {
            return invalid_policy("policy node has no member state");
        }
        const std::string content = node_content(label);
        if (const auto found = node_by_content.find(content);
            found != node_by_content.end()) {
            return found->second;
        }
        const std::string identity = stable_identity("policy-node-", content);
        if (const auto collision = node_content_by_identity.find(identity);
            collision != node_content_by_identity.end() &&
            collision->second != content) {
            return invalid_policy("policy node identity collision");
        }
        EnsemblePolicyNode node;
        node.node_identity = identity;
        node.earliest_member_time = label.members.front().point.time;
        node.latest_member_time = label.members.front().point.time;
        node.canonical_member_positions.reserve(label.members.size());
        for (const EnsembleMemberSearchState& member : label.members) {
            node.canonical_member_positions.push_back(member.point.position);
            node.earliest_member_time =
                std::min(node.earliest_member_time, member.point.time);
            node.latest_member_time =
                std::max(node.latest_member_time, member.point.time);
        }
        const std::size_t index = result.graph.nodes.size();
        result.graph.nodes.push_back(std::move(node));
        node_by_content.emplace(content, index);
        node_content_by_identity.emplace(identity, content);
        return index;
    };

    for (std::size_t candidate_index = 0U;
         candidate_index < candidates.size();
         ++candidate_index) {
        const Candidate& candidate = candidates[candidate_index];
        for (std::size_t depth = 0U; depth < candidate.path.size(); ++depth) {
            auto from = ensure_node(labels[candidate.path[depth]]);
            if (!from) {
                return from.error();
            }
            if (depth == 0U && result.graph.root_node_identity.empty()) {
                result.graph.root_node_identity =
                    result.graph.nodes[from.value()].node_identity;
            }
            if (depth + 1U == candidate.path.size()) {
                result.graph.nodes[from.value()].terminal = true;
                continue;
            }
            auto to = ensure_node(labels[candidate.path[depth + 1U]]);
            if (!to) {
                return to.error();
            }
            const EnsembleCommonAction& incoming =
                *labels[candidate.path[depth + 1U]].incoming_action;
            std::string content =
                result.graph.nodes[from.value()].node_identity;
            content.push_back('|');
            content.append(common_action_identity(incoming));
            content.push_back('|');
            content.append(result.graph.nodes[to.value()].node_identity);
            std::size_t branch_index{};
            if (const auto found = branch_by_content.find(content);
                found != branch_by_content.end()) {
                branch_index = found->second;
            } else {
                EnsemblePolicyBranch branch;
                branch.branch_identity =
                    stable_identity("policy-branch-", content);
                if (const auto collision =
                        branch_content_by_identity.find(
                            branch.branch_identity);
                    collision != branch_content_by_identity.end() &&
                    collision->second != content) {
                    return invalid_policy(
                        "policy branch identity collision");
                }
                branch.from_node_identity =
                    result.graph.nodes[from.value()].node_identity;
                branch.to_node_identity =
                    result.graph.nodes[to.value()].node_identity;
                branch.action = public_action(incoming);
                branch.wrong_choice_cost = candidate.cost;
                branch_index = result.graph.branches.size();
                result.graph.branches.push_back(std::move(branch));
                branch_by_content.emplace(content, branch_index);
                branch_content_by_identity.emplace(
                    result.graph.branches.back().branch_identity, content);
                result.graph.nodes[from.value()].outgoing_branch_identities.
                    push_back(result.graph.branches.back().branch_identity);
            }
            branch_candidates[
                result.graph.branches[branch_index].branch_identity].
                push_back(candidate_index);
        }
    }

    for (EnsemblePolicyBranch& branch : result.graph.branches) {
        const std::vector<std::size_t>& users =
            branch_candidates[branch.branch_identity];
        branch.wrong_choice_cost = candidates[users.front()].cost;
        for (const std::size_t index : users) {
            branch.selected = branch.selected || candidates[index].selected;
            if (cost_less(
                    candidates[index].cost, branch.wrong_choice_cost)) {
                branch.wrong_choice_cost = candidates[index].cost;
            }
        }
        branch.requires_re_evaluation = !branch.selected;
    }
    for (const EnsemblePolicyNode& node : result.graph.nodes) {
        if (node.outgoing_branch_identities.empty()) {
            continue;
        }
        std::vector<std::size_t> outgoing;
        outgoing.reserve(node.outgoing_branch_identities.size());
        for (const std::string& identity :
             node.outgoing_branch_identities) {
            const auto branch = std::find_if(
                result.graph.branches.begin(),
                result.graph.branches.end(),
                [&](const EnsemblePolicyBranch& value) {
                    return value.branch_identity == identity;
                });
            if (branch == result.graph.branches.end()) {
                return invalid_policy(
                    "policy node references a missing branch");
            }
            outgoing.push_back(static_cast<std::size_t>(
                branch - result.graph.branches.begin()));
        }
        for (std::size_t member = 0U;
             member < dataset.member_count();
             ++member) {
            std::vector<std::size_t> winners;
            std::vector<std::size_t> representative_candidates;
            representative_candidates.reserve(outgoing.size());
            for (const std::size_t branch_index : outgoing) {
                const std::vector<std::size_t>& users =
                    branch_candidates[
                        result.graph.branches[branch_index].branch_identity];
                std::size_t representative = users.front();
                for (const std::size_t user : users) {
                    if (compare_member_candidates(
                            candidates[user],
                            candidates[representative],
                            objective.kind,
                            member) < 0) {
                        representative = user;
                    }
                }
                representative_candidates.push_back(representative);
                if (winners.empty()) {
                    winners.push_back(branch_index);
                    continue;
                }
                const auto winner_position = std::find(
                    outgoing.begin(), outgoing.end(), winners.front());
                const std::size_t winner_representative =
                    representative_candidates[static_cast<std::size_t>(
                        winner_position - outgoing.begin())];
                const int comparison = compare_member_candidates(
                    candidates[representative],
                    candidates[winner_representative],
                    objective.kind,
                    member);
                if (comparison < 0) {
                    winners.assign(1U, branch_index);
                } else if (comparison == 0) {
                    winners.push_back(branch_index);
                }
            }
            const double share =
                dataset.members()[member].normalized_weight /
                static_cast<double>(winners.size());
            for (const std::size_t winner : winners) {
                result.graph.branches[winner].
                    supporting_member_weight += share;
            }
        }
    }

    for (const Candidate& candidate : candidates) {
        const std::string& alternative_content =
            labels[candidate.terminal].canonical_action_sequence_identity;
        if (const auto collision =
                alternative_content_by_identity.find(candidate.identity);
            collision != alternative_content_by_identity.end() &&
            collision->second != alternative_content) {
            return invalid_policy(
                "policy alternative identity collision");
        }
        alternative_content_by_identity.emplace(
            candidate.identity, alternative_content);
        result.graph.alternatives.push_back(EnsemblePolicyAlternative{
            candidate.identity,
            candidate.selected,
            !candidate.selected,
            candidate.actions,
            candidate.objective,
            candidate.outcomes,
            candidate.support,
            candidate.cost});
    }
    std::sort(
        result.graph.branches.begin(),
        result.graph.branches.end(),
        [](const auto& left, const auto& right) {
            return left.branch_identity < right.branch_identity;
        });
    for (EnsemblePolicyNode& node : result.graph.nodes) {
        std::sort(
            node.outgoing_branch_identities.begin(),
            node.outgoing_branch_identities.end());
    }
    std::sort(
        result.graph.nodes.begin(),
        result.graph.nodes.end(),
        [](const auto& left, const auto& right) {
            return left.node_identity < right.node_identity;
        });
    std::sort(
        result.graph.alternatives.begin(),
        result.graph.alternatives.end(),
        [](const auto& left, const auto& right) {
            if (left.selected != right.selected) {
                return left.selected;
            }
            return left.branch_identity < right.branch_identity;
        });

    for (const EnsemblePolicyNode& node : result.graph.nodes) {
        if (node.outgoing_branch_identities.size() < 2U) {
            continue;
        }
        EnsembleDecisionPoint decision;
        decision.decision_identity = stable_identity(
            "decision-", node.node_identity);
        decision.policy_node_identity = node.node_identity;
        decision.canonical_member_positions =
            node.canonical_member_positions;
        decision.earliest_time = node.earliest_member_time;
        decision.latest_commitment_time =
            node.latest_member_time +
            std::chrono::duration_cast<std::chrono::seconds>(
                options.commitment_time_tolerance);
        for (const std::string& branch_identity :
             node.outgoing_branch_identities) {
            const auto branch = std::find_if(
                result.graph.branches.begin(),
                result.graph.branches.end(),
                [&](const EnsemblePolicyBranch& value) {
                    return value.branch_identity == branch_identity;
                });
            decision.branches.push_back(EnsembleDecisionBranch{
                branch->branch_identity,
                branch->action,
                branch->selected,
                branch->requires_re_evaluation,
                branch->supporting_member_weight,
                branch->wrong_choice_cost});
        }
        result.decision_points.push_back(std::move(decision));
    }

    result.re_evaluation.schema_revision = 1U;
    result.re_evaluation.prior_run_identifier =
        dataset.metadata().run_identifier;
    result.re_evaluation.objective = objective;
    result.re_evaluation.spatial_tolerance_nautical_miles =
        options.commitment_spatial_tolerance_nautical_miles;
    result.re_evaluation.time_tolerance =
        std::chrono::duration_cast<std::chrono::seconds>(
            options.commitment_time_tolerance);
    for (const EnsemblePolicyAlternative& alternative :
         result.graph.alternatives) {
        result.re_evaluation.canonical_branch_identities.push_back(
            alternative.branch_identity);
        if (alternative.selected) {
            result.re_evaluation.selected_branch_identity =
                alternative.branch_identity;
        }
    }
    std::sort(
        result.re_evaluation.canonical_branch_identities.begin(),
        result.re_evaluation.canonical_branch_identities.end());
    return result;
}

}  // namespace sailroute::detail
