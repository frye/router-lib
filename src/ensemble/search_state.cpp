#include "search_state.hpp"

#include "routing/geodesy.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <tuple>
#include <utility>

namespace sailroute::detail {
namespace {

[[nodiscard]] Error invalid_state(std::string message) {
    return Error{ErrorCode::invalid_argument, std::move(message)};
}

[[nodiscard]] double canonical_zero(double value) noexcept {
    return value == 0.0 ? 0.0 : value;
}

[[nodiscard]] Coordinate canonical_coordinate(Coordinate coordinate) noexcept {
    coordinate.latitude_degrees = canonical_zero(coordinate.latitude_degrees);
    coordinate.longitude_degrees = canonical_zero(coordinate.longitude_degrees);
    if (coordinate.longitude_degrees == 180.0) {
        coordinate.longitude_degrees = -180.0;
    }
    return coordinate;
}

[[nodiscard]] std::uint64_t bits(double value) noexcept {
    return std::bit_cast<std::uint64_t>(canonical_zero(value));
}

void append_hex(std::ostringstream& stream, std::uint64_t value) {
    stream << std::hex << std::setw(16) << std::setfill('0') << value
           << std::dec;
}

[[nodiscard]] bool same_position(
    Coordinate left,
    Coordinate right) noexcept {
    left = canonical_coordinate(left);
    right = canonical_coordinate(right);
    return bits(left.latitude_degrees) == bits(right.latitude_degrees) &&
        bits(left.longitude_degrees) == bits(right.longitude_degrees);
}

[[nodiscard]] bool same_point(const RoutePoint& left, const RoutePoint& right) {
    return same_position(left.position, right.position) &&
        left.time == right.time;
}

[[nodiscard]] bool same_environment(
    const std::optional<RoutePointEnvironment>& left,
    const std::optional<RoutePointEnvironment>& right) noexcept {
    if (left.has_value() != right.has_value()) {
        return false;
    }
    if (!left) {
        return true;
    }
    return bits(left->speed_over_ground_knots) ==
            bits(right->speed_over_ground_knots) &&
        bits(left->course_over_ground_degrees) ==
            bits(right->course_over_ground_degrees) &&
        bits(left->current_east_knots) == bits(right->current_east_knots) &&
        bits(left->current_north_knots) == bits(right->current_north_knots) &&
        bits(left->flat_water_speed_knots) ==
            bits(right->flat_water_speed_knots) &&
        bits(left->significant_wave_height_metres) ==
            bits(right->significant_wave_height_metres) &&
        bits(left->wave_period_seconds) == bits(right->wave_period_seconds) &&
        bits(left->relative_wave_angle_degrees) ==
            bits(right->relative_wave_angle_degrees) &&
        left->current_applied == right->current_applied &&
        left->wave_applied == right->wave_applied;
}

[[nodiscard]] bool same_continuation_point(
    const RoutePoint& left,
    const RoutePoint& right) noexcept {
    return same_position(left.position, right.position) &&
        bits(left.heading_degrees) == bits(right.heading_degrees) &&
        bits(left.boat_speed_knots) == bits(right.boat_speed_knots) &&
        bits(left.true_wind_speed_knots) == bits(right.true_wind_speed_knots) &&
        bits(left.true_wind_direction_degrees) ==
            bits(right.true_wind_direction_degrees) &&
        bits(left.cumulative_distance_nautical_miles) ==
            bits(right.cumulative_distance_nautical_miles) &&
        same_environment(left.environment, right.environment);
}

[[nodiscard]] bool compatible(
    const EnsembleMemberSearchState& left,
    const EnsembleMemberSearchState& right) noexcept {
    if (left.member_identifier != right.member_identifier ||
        !same_continuation_point(left.point, right.point) ||
        !(left.configuration == right.configuration) ||
        left.status != right.status ||
        left.outcome_class != right.outcome_class) {
        return false;
    }
    if (left.status != EnsembleMemberSearchStatus::failed) {
        return true;
    }
    if (left.error.has_value() != right.error.has_value()) {
        return false;
    }
    return !left.error || left.error->code == right.error->code;
}

[[nodiscard]] std::string make_label_identity(
    const EnsembleSearchLabel& label) {
    std::ostringstream stream;
    stream << label.canonical_action_sequence_identity;
    for (const EnsembleMemberSearchState& member : label.members) {
        const Coordinate position = canonical_coordinate(member.point.position);
        stream << "|m" << member.member_identifier.size() << ':'
               << member.member_identifier << '@';
        append_hex(stream, bits(position.latitude_degrees));
        stream << ',';
        append_hex(stream, bits(position.longitude_degrees));
        stream << ',' << member.point.time.time_since_epoch().count()
               << ',' << static_cast<int>(member.configuration.board)
               << ',' << member.configuration.sail
               << ',' << member.configuration.reef
               << ',' << static_cast<unsigned>(member.status);
        if (member.outcome_class) {
            stream << ',' << static_cast<unsigned>(*member.outcome_class);
        }
        if (member.error) {
            stream << ',' << static_cast<unsigned>(member.error->code);
        }
    }
    return stream.str();
}

[[nodiscard]] Result<std::vector<std::size_t>> ancestry(
    std::span<const EnsembleSearchLabel> labels,
    std::size_t terminal_label) {
    if (terminal_label >= labels.size()) {
        return invalid_state("terminal ensemble label is outside the label arena");
    }
    std::vector<std::size_t> reverse;
    std::size_t current = terminal_label;
    while (current != no_ensemble_label) {
        if (current >= labels.size()) {
            return invalid_state("ensemble label parent is outside the label arena");
        }
        if (reverse.size() >= labels.size()) {
            return invalid_state("ensemble label parent chain contains a cycle");
        }
        reverse.push_back(current);
        current = labels[current].parent_label;
    }
    std::reverse(reverse.begin(), reverse.end());
    return reverse;
}

}  // namespace

bool operator<(
    const EnsembleCommonAction& left,
    const EnsembleCommonAction& right) noexcept {
    const Coordinate lhs = canonical_coordinate(left.target);
    const Coordinate rhs = canonical_coordinate(right.target);
    return std::tie(
               left.kind,
               lhs.latitude_degrees,
               lhs.longitude_degrees,
               left.heading_degrees,
               left.duration) <
        std::tie(
               right.kind,
               rhs.latitude_degrees,
               rhs.longitude_degrees,
               right.heading_degrees,
               right.duration);
}

bool operator==(
    const EnsembleCommonAction& left,
    const EnsembleCommonAction& right) noexcept {
    return left.kind == right.kind &&
        same_position(left.target, right.target) &&
        bits(left.heading_degrees) == bits(right.heading_degrees) &&
        left.duration == right.duration;
}

Result<EnsembleCommonAction> make_common_target_action(Coordinate target) {
    if (!is_valid(target)) {
        return invalid_state("ensemble common action target is invalid");
    }
    return EnsembleCommonAction{canonical_coordinate(target)};
}

Result<EnsembleCommonAction> make_common_heading_action(
    double heading_degrees,
    std::chrono::seconds duration) {
    if (!std::isfinite(heading_degrees) ||
        duration <= std::chrono::seconds::zero()) {
        return invalid_state(
            "ensemble heading action requires a finite heading and positive duration");
    }
    EnsembleCommonAction action;
    action.kind = EnsembleCommonActionKind::heading_for_duration;
    action.heading_degrees = normalize_degrees(heading_degrees);
    action.duration = duration;
    return action;
}

Result<EnsembleCommonAction> make_common_wait_action(
    std::chrono::seconds duration) {
    if (duration <= std::chrono::seconds::zero()) {
        return invalid_state(
            "ensemble wait action requires a positive duration");
    }
    EnsembleCommonAction action;
    action.kind = EnsembleCommonActionKind::wait_for_duration;
    action.duration = duration;
    return action;
}

std::string common_action_identity(const EnsembleCommonAction& action) {
    std::ostringstream stream;
    switch (action.kind) {
        case EnsembleCommonActionKind::target: {
            const Coordinate target = canonical_coordinate(action.target);
            stream << "target:";
            append_hex(stream, bits(target.latitude_degrees));
            stream << ':';
            append_hex(stream, bits(target.longitude_degrees));
            break;
        }
        case EnsembleCommonActionKind::heading_for_duration:
            stream << "heading:";
            append_hex(stream, bits(normalize_degrees(action.heading_degrees)));
            stream << ':' << action.duration.count();
            break;
        case EnsembleCommonActionKind::wait_for_duration:
            stream << "wait:" << action.duration.count();
            break;
    }
    return stream.str();
}

Result<EnsembleSearchLabel> make_initial_ensemble_label(
    const EnsembleDataset& dataset,
    std::span<const RoutePoint> member_points,
    std::span<const OperationalConfiguration> member_configurations) {
    if (!dataset.alignment().shared_search_compatible()) {
        return invalid_state(
            "initial ensemble label requires a shared-search-compatible dataset");
    }
    if (member_points.size() != dataset.member_count() ||
        member_configurations.size() != dataset.member_count()) {
        return invalid_state(
            "initial ensemble label requires one point and configuration per member");
    }

    EnsembleSearchLabel label;
    label.members.reserve(dataset.member_count());
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        if (!is_valid(member_points[index].position)) {
            return invalid_state(
                "initial ensemble member '" +
                dataset.members()[index].identifier + "' has an invalid position");
        }
        RoutePoint point = member_points[index];
        point.position = canonical_coordinate(point.position);
        label.members.push_back(EnsembleMemberSearchState{
            dataset.members()[index].identifier,
            std::move(point),
            member_configurations[index],
            EnsembleMemberSearchStatus::active,
            std::nullopt,
            std::nullopt});
    }
    canonicalize_ensemble_label(label);
    return label;
}

void canonicalize_ensemble_label(EnsembleSearchLabel& label) {
    for (EnsembleMemberSearchState& member : label.members) {
        member.point.position = canonical_coordinate(member.point.position);
    }
    for (EnsembleCommonAction& action : label.common_action_history) {
        if (action.kind == EnsembleCommonActionKind::target) {
            action.target = canonical_coordinate(action.target);
        } else if (
            action.kind == EnsembleCommonActionKind::heading_for_duration) {
            action.heading_degrees = normalize_degrees(action.heading_degrees);
        }
    }
    if (label.incoming_action) {
        if (label.incoming_action->kind == EnsembleCommonActionKind::target) {
            label.incoming_action->target =
                canonical_coordinate(label.incoming_action->target);
        } else if (
            label.incoming_action->kind ==
            EnsembleCommonActionKind::heading_for_duration) {
            label.incoming_action->heading_degrees =
                normalize_degrees(label.incoming_action->heading_degrees);
        }
    }
    label.canonical_action_sequence_identity = "root";
    for (const EnsembleCommonAction& action : label.common_action_history) {
        label.canonical_action_sequence_identity +=
            "/" + common_action_identity(action);
    }
    label.canonical_label_identity = make_label_identity(label);
}

Result<std::vector<EnsembleMemberOutcome>> ensemble_label_outcomes(
    const EnsembleDataset& dataset,
    const EnsembleSearchLabel& label,
    TimePoint departure) {
    if (label.members.size() != dataset.member_count()) {
        return invalid_state(
            "ensemble label member count does not match the dataset");
    }
    std::vector<EnsembleMemberOutcome> outcomes;
    outcomes.reserve(label.members.size());
    for (std::size_t index = 0U; index < label.members.size(); ++index) {
        const EnsembleMemberSearchState& state = label.members[index];
        const std::string& expected = dataset.members()[index].identifier;
        if (state.member_identifier != expected) {
            return invalid_state(
                "ensemble label members are not in dataset canonical order");
        }
        if (state.status == EnsembleMemberSearchStatus::active) {
            return invalid_state(
                "cannot evaluate an ensemble label with active members");
        }
        if (!state.outcome_class) {
            return invalid_state(
                "resolved ensemble member '" + expected +
                "' has no outcome class");
        }
        const bool reached =
            state.status == EnsembleMemberSearchStatus::completed &&
            *state.outcome_class == EnsembleMemberOutcomeClass::reached;
        if (state.status == EnsembleMemberSearchStatus::completed && !reached) {
            return invalid_state(
                "completed ensemble member '" + expected +
                "' must have reached outcome");
        }
        if (state.status == EnsembleMemberSearchStatus::failed &&
            *state.outcome_class == EnsembleMemberOutcomeClass::reached) {
            return invalid_state(
                "failed ensemble member '" + expected +
                "' cannot have reached outcome");
        }
        std::optional<double> elapsed;
        if (reached) {
            const auto duration =
                std::chrono::duration<double>(state.point.time - departure).count();
            if (!std::isfinite(duration) || duration < 0.0) {
                return invalid_state(
                    "ensemble member '" + expected +
                    "' has an invalid elapsed arrival");
            }
            elapsed = duration;
        }
        outcomes.push_back(EnsembleMemberOutcome{
            expected,
            *state.outcome_class,
            elapsed,
            state.error});
    }
    return outcomes;
}

Result<EnsembleObjectiveEvaluation> evaluate_ensemble_label_objective(
    const EnsembleDataset& dataset,
    const EnsembleObjective& objective,
    EnsembleSearchLabel& label,
    TimePoint departure,
    EnsembleObjectiveTieBreakInputs tie_break) {
    auto outcomes = ensemble_label_outcomes(dataset, label, departure);
    if (!outcomes) {
        return outcomes.error();
    }
    tie_break.canonical_action_sequence_identity =
        label.canonical_action_sequence_identity;
    auto evaluation = evaluate_ensemble_objective(
        dataset, objective, outcomes.value(), std::move(tie_break));
    if (evaluation) {
        label.aggregate_objective = evaluation.value();
        canonicalize_ensemble_label(label);
    }
    return evaluation;
}

bool dominates(
    const EnsembleSearchLabel& left,
    const EnsembleSearchLabel& right) noexcept {
    if (left.members.size() != right.members.size() ||
        left.common_action_history.empty() != right.common_action_history.empty()) {
        return false;
    }
    if (!left.common_action_history.empty() &&
        !(left.common_action_history.back() == right.common_action_history.back())) {
        return false;
    }

    bool strict = false;
    for (std::size_t index = 0U; index < left.members.size(); ++index) {
        const auto& lhs = left.members[index];
        const auto& rhs = right.members[index];
        if (!compatible(lhs, rhs) || lhs.point.time > rhs.point.time) {
            return false;
        }
        if (lhs.status == EnsembleMemberSearchStatus::completed) {
            strict = strict || lhs.point.time < rhs.point.time;
        } else if (lhs.point.time != rhs.point.time) {
            return false;
        }
    }
    return strict;
}

bool ensemble_label_less(
    const EnsembleSearchLabel& left,
    const EnsembleSearchLabel& right) noexcept {
    return left.canonical_label_identity < right.canonical_label_identity;
}

Result<std::vector<EnsembleCommonAction>> reconstruct_common_actions(
    std::span<const EnsembleSearchLabel> labels,
    std::size_t terminal_label) {
    auto chain = ancestry(labels, terminal_label);
    if (!chain) {
        return chain.error();
    }
    std::vector<EnsembleCommonAction> actions;
    for (std::size_t offset = 1U; offset < chain.value().size(); ++offset) {
        const EnsembleSearchLabel& child = labels[chain.value()[offset]];
        if (!child.incoming_action) {
            return invalid_state("non-root ensemble label has no action backpointer");
        }
        actions.push_back(*child.incoming_action);
    }
    const EnsembleSearchLabel& terminal = labels[terminal_label];
    if (actions != terminal.common_action_history) {
        return invalid_state(
            "ensemble action backpointers disagree with canonical history");
    }
    return actions;
}

Result<std::vector<RoutePoint>> reconstruct_member_route(
    std::span<const EnsembleSearchLabel> labels,
    std::size_t terminal_label,
    std::size_t member_index) {
    auto chain = ancestry(labels, terminal_label);
    if (!chain) {
        return chain.error();
    }
    std::vector<RoutePoint> route;
    for (const std::size_t label_index : chain.value()) {
        const EnsembleSearchLabel& label = labels[label_index];
        if (member_index >= label.members.size()) {
            return invalid_state(
                "ensemble member index is outside a label's canonical members");
        }
        const RoutePoint& point = label.members[member_index].point;
        if (route.empty() || !same_point(route.back(), point)) {
            route.push_back(point);
        }
    }
    return route;
}

}  // namespace sailroute::detail
