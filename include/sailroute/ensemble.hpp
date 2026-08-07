#pragma once

#include "sailroute/environment.hpp"
#include "sailroute/error.hpp"
#include "sailroute/polar.hpp"
#include "sailroute/router.hpp"
#include "sailroute/weather.hpp"

#include <chrono>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace sailroute {

/// Stable identity and attribution for one ensemble forecast cycle.
struct EnsembleRunMetadata {
    std::string run_identifier;
    std::string model_identifier;
    TimePoint initialization_time{};
    std::string attribution;
    std::uint32_t schema_revision{1U};
};

/// Named weighted input loaded through `WeatherDataset::load`.
struct EnsembleMemberInput {
    std::string identifier;
    double weight{1.0};
    std::filesystem::path grib_path;
    std::optional<GeographicBounds> bounds;
    RoutingEnvironment environment;
};

/// Configured variable categories. Wind is always present for a loaded member.
struct EnsembleVariableCategories {
    bool wind{true};
    bool currents{false};
    bool waves{false};
    bool land{false};
    bool exclusions{false};

    bool operator==(const EnsembleVariableCategories&) const = default;
};

/// Member metadata retained in canonical identifier order.
struct EnsembleMemberMetadata {
    std::string identifier;
    double original_weight{};
    double normalized_weight{};
    ForecastMetadata weather;
    std::vector<TimePoint> wind_valid_times;
    ForecastGridIdentity wind_grid;
    EnsembleVariableCategories configured_variables;
    std::optional<RouteEnvironmentMetadata> environment;
    std::optional<EnvironmentCoverage> current_coverage;
    std::optional<EnvironmentCoverage> wave_coverage;
};

/// Whether loading requires one shared wind search domain.
enum class EnsembleAlignmentMode {
    strict_shared_search,
    permissive_member_local,
};

/// Returns the stable snake_case alignment-mode name.
std::string_view to_string(EnsembleAlignmentMode mode) noexcept;

struct EnsembleLoadOptions {
    EnsembleAlignmentMode alignment{EnsembleAlignmentMode::strict_shared_search};
};

/// Exact alignment facts computed after every member is loaded.
///
/// In permissive mode, false fields are explicit and member-local samplers
/// remain usable. Strict mode rejects wind-axis, grid/coverage, and configured
/// variable-category mismatches. Stage 3 provider coverage is advisory; runtime
/// provider sample status remains authoritative.
struct EnsembleAlignmentStatus {
    bool exact_initialization_time{true};
    bool exact_wind_valid_times{true};
    bool exact_wind_grid_geometry{true};
    bool exact_wind_coverage{true};
    bool equal_wind_horizon{true};
    bool partial_wind_coverage{false};
    bool aligned_environment_categories{true};

    [[nodiscard]] bool shared_search_compatible() const noexcept {
        return exact_initialization_time && exact_wind_valid_times &&
            exact_wind_grid_geometry &&
            exact_wind_coverage &&
            aligned_environment_categories;
    }
};

/// Stable risk objective identifier.
enum class EnsembleObjectiveKind {
    weighted_mean_elapsed_arrival,
    weighted_p75_elapsed_arrival,
    weighted_p90_elapsed_arrival,
    probability_before_target,
    probability_beating_rival,
};

/// Whether a smaller or larger primary objective value is preferred.
enum class EnsembleObjectiveDirection {
    minimize,
    maximize,
};

/// Returns the stable snake_case objective identifier.
std::string_view to_string(EnsembleObjectiveKind kind) noexcept;
/// Returns the stable snake_case optimization direction.
std::string_view to_string(EnsembleObjectiveDirection direction) noexcept;
/// Returns `maximize` only for probability objectives.
EnsembleObjectiveDirection objective_direction(EnsembleObjectiveKind kind) noexcept;

/// Auditable completion or failure class for one member.
///
/// `reached` is the only complete class. In incomplete-vs-incomplete rival
/// comparisons, different classes use the stable best-to-worst order declared
/// here: forecast exhaustion, duration exhaustion, infeasible/no route, missing
/// data, provider failure, cancellation, then other error.
enum class EnsembleMemberOutcomeClass : std::uint8_t {
    reached = 0U,
    forecast_exhausted = 1U,
    duration_exhausted = 2U,
    infeasible_no_route = 3U,
    missing_data = 4U,
    provider_failure = 5U,
    cancelled = 6U,
    other_error = 7U,
};

/// Returns the stable snake_case member outcome class.
std::string_view to_string(EnsembleMemberOutcomeClass outcome_class) noexcept;

/// Raw outcome for one named member.
///
/// Reached outcomes require a finite, non-negative elapsed arrival. Incomplete
/// outcomes must not provide one. `error` retains the original member-local
/// diagnostic and is never replaced by the aggregate objective.
struct EnsembleMemberOutcome {
    std::string member_identifier;
    EnsembleMemberOutcomeClass outcome_class{
        EnsembleMemberOutcomeClass::other_error};
    std::optional<double> elapsed_arrival_seconds;
    std::optional<Error> error;
};

/// Strict elapsed-time threshold used by `probability_before_target`.
struct EnsembleArrivalTarget {
    double elapsed_seconds{};
};

/// Objective selection plus the input required by that objective.
///
/// `target` must be present only for `probability_before_target`.
/// `rival_outcomes` must be present only for `probability_beating_rival` and
/// contain exactly one outcome for every dataset member. Evaluation validates
/// these combinations and canonicalizes outcomes by member identifier.
struct EnsembleObjective {
    EnsembleObjectiveKind kind{
        EnsembleObjectiveKind::weighted_mean_elapsed_arrival};
    std::optional<EnsembleArrivalTarget> target;
    std::vector<EnsembleMemberOutcome> rival_outcomes;
};

/// Explicit finite/infinite representation for objective and diagnostic values.
///
/// `finite_value` is meaningful only for `finite` and must itself be finite.
/// This representation lets later JSON encode infinity as a tagged value rather
/// than an invalid non-finite JSON number.
enum class EnsembleObjectiveValueClass : std::uint8_t {
    finite,
    positive_infinity,
};

/// Returns the stable snake_case objective-value class.
std::string_view to_string(EnsembleObjectiveValueClass value_class) noexcept;

struct EnsembleObjectiveValue {
    EnsembleObjectiveValueClass value_class{
        EnsembleObjectiveValueClass::finite};
    double finite_value{};

    [[nodiscard]] bool is_finite() const noexcept {
        return value_class == EnsembleObjectiveValueClass::finite;
    }

    [[nodiscard]] bool is_positive_infinity() const noexcept {
        return value_class == EnsembleObjectiveValueClass::positive_infinity;
    }
};

/// Per-member audit used to reconstruct an aggregate objective.
struct EnsembleObjectiveMemberDiagnostic {
    double normalized_weight{};
    EnsembleMemberOutcome candidate;
    std::optional<EnsembleMemberOutcome> rival;
    EnsembleObjectiveValue elapsed_arrival;
    /// Unweighted probability score: 0, 0.5, or 1; zero for arrival objectives.
    double probability_score{};
    /// `weight * elapsed` for mean, or `weight * score` for probabilities.
    /// Quantile objectives are non-additive and leave this at zero.
    double weighted_contribution{};
    bool selected_quantile_member{};
};

/// Aggregate audit fields and stable secondary ordering inputs.
struct EnsembleObjectiveDiagnostics {
    std::vector<EnsembleObjectiveMemberDiagnostic> members;
    double incomplete_member_weight{};
    /// Mean among reached positive-weight members, normalized by their weight.
    EnsembleObjectiveValue weighted_finite_mean_arrival;
    /// Worst reached arrival among positive-weight members.
    EnsembleObjectiveValue worst_finite_arrival;
    std::size_t generated_states{};
    std::size_t settled_states{};
    std::string canonical_action_sequence_identity;
};

/// Primary value and complete audit produced by objective evaluation.
struct EnsembleObjectiveEvaluation {
    EnsembleObjectiveValue value;
    EnsembleObjectiveDiagnostics diagnostics;
};

/// One canonical member's wind value or member-local sampling error.
struct EnsembleMemberWindSample {
    std::string member_identifier;
    Result<Wind> wind;
};

class EnsembleDataset;

/// Time-resolved weather samplers for every member in canonical order.
class EnsembleSampler {
public:
    EnsembleSampler() noexcept;
    ~EnsembleSampler();
    EnsembleSampler(const EnsembleSampler&);
    EnsembleSampler(EnsembleSampler&&) noexcept;
    EnsembleSampler& operator=(const EnsembleSampler&);
    EnsembleSampler& operator=(EnsembleSampler&&) noexcept;

    [[nodiscard]] std::size_t member_count() const noexcept;
    /// True when every member resolved its requested forecast time.
    [[nodiscard]] bool valid() const noexcept;
    /// Samples one coordinate for every member without aggregating values.
    [[nodiscard]] std::vector<EnsembleMemberWindSample> sample(
        Coordinate coordinate) const;
    /// Samples one member-local coordinate per canonical member.
    [[nodiscard]] Result<std::vector<EnsembleMemberWindSample>> sample(
        std::span<const Coordinate> coordinates) const;

private:
    friend class EnsembleDataset;

    struct Impl;
    std::shared_ptr<const Impl> impl_;
};

/// Immutable owning collection of canonically ordered forecast members.
class EnsembleDataset {
public:
    EnsembleDataset();
    ~EnsembleDataset();
    EnsembleDataset(const EnsembleDataset&);
    EnsembleDataset(EnsembleDataset&&) noexcept;
    EnsembleDataset& operator=(const EnsembleDataset&);
    EnsembleDataset& operator=(EnsembleDataset&&) noexcept;

    [[nodiscard]] static Result<EnsembleDataset> load(
        EnsembleRunMetadata metadata,
        std::vector<EnsembleMemberInput> members,
        EnsembleLoadOptions options = {});

    [[nodiscard]] const EnsembleRunMetadata& metadata() const noexcept;
    [[nodiscard]] const std::vector<EnsembleMemberMetadata>& members() const noexcept;
    [[nodiscard]] const EnsembleAlignmentStatus& alignment() const noexcept;
    [[nodiscard]] std::size_t member_count() const noexcept;
    /// Returns null when `index` is outside canonical member order.
    [[nodiscard]] const WeatherDataset* member_weather(std::size_t index) const noexcept;
    /// Returns null when `index` is outside canonical member order.
    [[nodiscard]] const RoutingEnvironment* member_environment(
        std::size_t index) const noexcept;

    /// Resolves a common time for all canonical members.
    [[nodiscard]] Result<EnsembleSampler> sampler_at(TimePoint time) const;
    /// Resolves one member-local time per canonical member.
    [[nodiscard]] Result<EnsembleSampler> sampler_at(
        std::span<const TimePoint> member_times) const;

private:
    struct Impl;
    explicit EnsembleDataset(std::shared_ptr<const Impl> impl);
    std::shared_ptr<const Impl> impl_;
};

/// Search engine selected for one ensemble optimization.
enum class EnsembleSolver {
    time_dependent_lattice,
    experimental_isochrone_beam,
};

/// Returns the stable snake_case ensemble solver name.
std::string_view to_string(EnsembleSolver solver) noexcept;

/// Resolution and explicit resource limits for the ensemble lattice search.
///
/// Limits are hard correctness boundaries: exceeding either returns an error
/// rather than silently dropping labels or switching algorithms.
struct EnsembleLatticeRoutingOptions {
    std::size_t subdivision_level{4U};
    std::chrono::minutes time_bucket{30};
    LatticeSearchAlgorithm search_algorithm{LatticeSearchAlgorithm::a_star};
    std::size_t max_labels_per_state{512U};
    std::size_t max_total_labels{500'000U};
};

/// Resolution, pruning, and hard resource limits for the experimental beam.
///
/// The beam advances every active member with one fixed-duration water heading.
/// It is deliberately bounded and approximate; exceeding a configured limit is
/// an error and never falls back to the deterministic lattice solver.
struct EnsembleBeamRoutingOptions {
    std::chrono::minutes time_step{30};
    double heading_step_degrees{10.0};
    double centroid_bucket_nautical_miles{10.0};
    std::size_t max_nodes_per_bucket{8U};
    std::size_t beam_width{512U};
    std::size_t max_steps{4'096U};
    std::size_t max_total_nodes{500'000U};
};

/// Bounds policy alternatives and defines cross-cycle commitment tolerances.
struct EnsemblePolicyOptions {
    /// Additional alternatives retained beside the selected route.
    std::size_t max_alternatives{3U};
    double commitment_spatial_tolerance_nautical_miles{2.0};
    std::chrono::minutes commitment_time_tolerance{15};
};

/// Additive request surface for a shared-action ensemble optimization.
///
/// `options` supplies the existing transition physics and Stage 3 eligibility
/// controls. Its deterministic `solver` and lattice fields are not consulted.
/// Beam selection is rejected unless `enable_experimental_beam` is explicitly
/// true, and that opt-in is itself rejected for every other solver.
struct EnsembleRouteRequest {
    Coordinate start;
    Coordinate destination;
    std::optional<TimePoint> departure_time;
    RoutingOptions options;
    EnsembleObjective objective;
    EnsembleSolver solver{EnsembleSolver::time_dependent_lattice};
    EnsembleLatticeRoutingOptions lattice;
    EnsembleBeamRoutingOptions beam;
    EnsemblePolicyOptions policy;
    /// Must be true exactly when selecting `experimental_isochrone_beam`.
    bool enable_experimental_beam{false};
};

enum class EnsembleRouteActionKind : std::uint8_t {
    target,
    heading_for_duration,
    wait_for_duration,
};

/// One selected action applied to every member that was still active.
struct EnsembleRouteAction {
    EnsembleRouteActionKind kind{EnsembleRouteActionKind::target};
    Coordinate target;
    double heading_degrees{};
    std::chrono::seconds duration{};
};

/// Search-work audit for a bounded label-correcting ensemble search.
struct EnsembleLatticeDiagnostics {
    std::size_t settled_labels{};
    std::size_t queued_labels{};
    std::size_t generated_labels{};
    std::size_t retained_labels{};
    std::size_t dominated_labels{};
    std::size_t stale_queue_entries{};
    std::size_t peak_labels_per_state{};
    std::size_t max_labels_per_state{};
    std::size_t max_total_labels{};
    std::size_t subdivision_level{};
    bool zero_heuristic{};
    /// Phase 4 performs one fixed-resolution search and never claims refinement.
    bool refinement_performed{};
};

/// Search-work audit for the bounded experimental isochrone beam.
struct EnsembleBeamDiagnostics {
    std::size_t expanded_nodes{};
    std::size_t generated_nodes{};
    std::size_t accepted_nodes{};
    std::size_t retained_nodes{};
    std::size_t rejected_common_actions{};
    std::size_t pruned_by_bucket{};
    std::size_t pruned_by_beam{};
    std::size_t peak_frontier{};
    std::size_t completed_nodes{};
    std::size_t beam_width{};
    std::size_t max_nodes_per_bucket{};
    std::size_t max_steps{};
    std::size_t max_total_nodes{};
};

/// Canonically ordered route and outcome for one member.
struct EnsembleMemberRouteResult {
    EnsembleMemberOutcome outcome;
    std::vector<RoutePoint> points;
    EnvironmentDiagnostics environment_diagnostics;
};

/// One complete common-action alternative retained from the solver.
struct EnsemblePolicyAlternative {
    std::string branch_identity;
    bool selected{};
    bool requires_re_evaluation{true};
    std::vector<EnsembleRouteAction> common_actions;
    EnsembleObjectiveEvaluation objective;
    std::vector<EnsembleMemberOutcome> member_outcomes;
    double supporting_member_weight{};
    EnsembleObjectiveValue wrong_choice_cost;
};

/// One canonical policy state. Multiple incoming branches represent
/// reconvergence; multiple outgoing branches represent a decision.
struct EnsemblePolicyNode {
    std::string node_identity;
    std::vector<Coordinate> canonical_member_positions;
    TimePoint earliest_member_time{};
    TimePoint latest_member_time{};
    bool terminal{};
    std::vector<std::string> outgoing_branch_identities;
};

/// One canonical action edge in the non-clairvoyant policy DAG.
struct EnsemblePolicyBranch {
    std::string branch_identity;
    std::string from_node_identity;
    std::string to_node_identity;
    EnsembleRouteAction action;
    bool selected{};
    bool requires_re_evaluation{true};
    double supporting_member_weight{};
    EnsembleObjectiveValue wrong_choice_cost;
};

struct EnsemblePolicyGraph {
    std::uint32_t schema_revision{1U};
    std::string root_node_identity;
    std::vector<EnsemblePolicyNode> nodes;
    std::vector<EnsemblePolicyBranch> branches;
    std::vector<EnsemblePolicyAlternative> alternatives;
};

struct EnsembleDecisionBranch {
    std::string policy_branch_identity;
    EnsembleRouteAction action;
    bool selected{};
    bool requires_re_evaluation{true};
    double supporting_member_weight{};
    EnsembleObjectiveValue wrong_choice_cost;
};

/// Operational choice extracted from a policy node with multiple legal actions.
struct EnsembleDecisionPoint {
    std::string decision_identity;
    std::string policy_node_identity;
    std::vector<Coordinate> canonical_member_positions;
    TimePoint earliest_time{};
    TimePoint latest_commitment_time{};
    std::vector<EnsembleDecisionBranch> branches;
};

/// Content needed to compare this policy with a later forecast cycle.
struct EnsembleReevaluationState {
    std::uint32_t schema_revision{1U};
    std::string prior_run_identifier;
    std::string selected_branch_identity;
    std::vector<std::string> canonical_branch_identities;
    EnsembleObjective objective;
    double spatial_tolerance_nautical_miles{};
    std::chrono::seconds time_tolerance{};
};

/// Complete selected shared-action route and its member-local audit.
struct EnsembleRouteResult {
    TimePoint departure_time;
    DepartureSource departure_source{DepartureSource::explicit_time};
    EnsembleSolver solver{EnsembleSolver::time_dependent_lattice};
    std::vector<EnsembleRouteAction> common_actions;
    std::string canonical_action_sequence_identity;
    std::vector<EnsembleMemberRouteResult> members;
    EnsembleObjectiveEvaluation objective;
    EnsembleLatticeDiagnostics lattice_diagnostics;
    EnsembleBeamDiagnostics beam_diagnostics;
    EnsemblePolicyGraph policy;
    std::vector<EnsembleDecisionPoint> decision_points;
    EnsembleReevaluationState re_evaluation;
    /// True only for results produced by an explicitly enabled experimental solver.
    bool experimental{};
};

/// Phase of the ensemble solver reported via progress callbacks.
enum class EnsembleSolverPhase {
    initializing,
    searching,
    finalizing,
};

/// Returns the stable snake_case phase name ("initializing", "searching", "finalizing").
std::string_view to_string(EnsembleSolverPhase phase) noexcept;

/// Owning progress snapshot delivered to ensemble solver progress callbacks.
struct EnsembleProgress {
    EnsembleSolverPhase phase{EnsembleSolverPhase::initializing};
    /// Number of members that are still actively reaching new positions.
    std::size_t active_member_count{};
    /// Active (queued) labels in the current search frontier.
    std::size_t active_label_count{};
    /// Cumulative labels/nodes accepted and retained since search start.
    std::size_t retained_label_count{};
    /// Cumulative labels/nodes generated (expanded-from) since search start.
    std::size_t generated_states{};
    /// Cumulative labels/nodes settled (finalized) since search start.
    std::size_t settled_states{};
    /// Best known objective bound; positive_infinity until any terminal found.
    EnsembleObjectiveValue current_objective_bound;
    /// Number of retained terminal (policy alternative) labels.
    std::size_t policy_alternative_count{};
};

/// Non-owning progress snapshot with callback-scoped lifetime.
///
/// All fields are value types so the snapshot may safely be copied inside the
/// callback; there are no dangling pointers after the callback returns.
struct EnsembleProgressView {
    EnsembleSolverPhase phase{EnsembleSolverPhase::initializing};
    std::size_t active_member_count{};
    std::size_t active_label_count{};
    std::size_t retained_label_count{};
    std::size_t generated_states{};
    std::size_t settled_states{};
    EnsembleObjectiveValue current_objective_bound;
    std::size_t policy_alternative_count{};
};

/// Owning, notification-only ensemble progress callback.
using EnsembleProgressCallback =
    std::function<void(const EnsembleProgress&)>;

/// Owning ensemble progress callback with cancellation control.
using EnsembleControlCallback =
    std::function<RoutingProgressDecision(const EnsembleProgress&)>;

/// Non-owning, notification-only ensemble progress callback.
using EnsembleProgressViewCallback =
    std::function<void(const EnsembleProgressView&)>;

/// Non-owning ensemble progress callback with cancellation control.
using EnsembleViewControlCallback =
    std::function<RoutingProgressDecision(const EnsembleProgressView&)>;

/// Optimizes one common action sequence against every canonical member.
class EnsembleRouter {
public:
    EnsembleRouter(
        EnsembleDataset dataset,
        VesselPolar polar = VesselPolar::default_racer_cruiser_45ft());

    [[nodiscard]] const EnsembleDataset& dataset() const noexcept;

    /// Optimizes without intermediate progress delivery.
    [[nodiscard]] Result<EnsembleRouteResult> optimize(
        const EnsembleRouteRequest& request) const;
    /// Optimizes with owning, notification-only progress snapshots.
    [[nodiscard]] Result<EnsembleRouteResult> optimize(
        const EnsembleRouteRequest& request,
        const EnsembleProgressCallback& on_progress) const;

    template <typename Callback>
        requires std::same_as<
            std::invoke_result_t<Callback&, const EnsembleProgress&>,
            RoutingProgressDecision> &&
            std::constructible_from<EnsembleControlCallback, Callback&&>
    [[nodiscard]] Result<EnsembleRouteResult> optimize(
        const EnsembleRouteRequest& request,
        Callback&& on_progress) const {
        return optimize_controlled(
            request,
            EnsembleControlCallback{std::forward<Callback>(on_progress)});
    }

    /// Optimizes with allocation-efficient callback-scoped progress views.
    ///
    /// Progress views remain valid only for the synchronous callback invocation.
    [[nodiscard]] Result<EnsembleRouteResult> optimize_view(
        const EnsembleRouteRequest& request,
        const EnsembleProgressViewCallback& on_progress) const;

    template <typename Callback>
        requires std::same_as<
            std::invoke_result_t<Callback&, const EnsembleProgressView&>,
            RoutingProgressDecision> &&
            std::constructible_from<EnsembleViewControlCallback, Callback&&>
    [[nodiscard]] Result<EnsembleRouteResult> optimize_view(
        const EnsembleRouteRequest& request,
        Callback&& on_progress) const {
        return optimize_view_controlled(
            request,
            EnsembleViewControlCallback{std::forward<Callback>(on_progress)});
    }

private:
    [[nodiscard]] Result<EnsembleRouteResult> optimize_controlled(
        const EnsembleRouteRequest& request,
        const EnsembleControlCallback& on_progress) const;
    [[nodiscard]] Result<EnsembleRouteResult> optimize_view_controlled(
        const EnsembleRouteRequest& request,
        const EnsembleViewControlCallback& on_progress) const;

    EnsembleDataset dataset_;
    VesselPolar polar_;
};

}  // namespace sailroute
