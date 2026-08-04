#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace sailroute {

/// UTC system-clock time at whole-second precision.
using TimePoint = std::chrono::sys_seconds;

/// Canonical latitude/longitude coordinates in degrees.
struct Coordinate {
    double latitude_degrees{};
    double longitude_degrees{};
};

/// Eastward and northward wind-vector components in metres per second.
struct Wind {
    double east_mps{};
    double north_mps{};

    [[nodiscard]] double speed_knots() const noexcept;
    [[nodiscard]] double direction_from_degrees() const noexcept;
};

/// One departure-relative tier in a variable routing step schedule.
struct RoutingInterval {
    std::chrono::minutes interval{};
    std::optional<std::chrono::minutes> until_elapsed;
};

/// Controls alpha-shape construction for display contours.
struct DisplayContourOptions {
    // When omitted, the builder derives a deterministic scale from the
    // triangulation's median circumradius.
    std::optional<double> alpha_nautical_miles;
};

/// Controls destination-facing isochrone front construction for display.
enum class DestinationFrontSegmentPolicy {
    provisional_component,
    all_meaningful_components,
};

/// Selects the candidate population and topology rules used for progress fronts.
enum class DestinationFrontMode {
    /// v0.3 behavior: build one principal component from retained frontier nodes.
    retained_frontier,
    /// Build from all eligible pre-prune candidates and preserve the route anchor.
    eligible_pre_prune,
};

struct DestinationFrontOptions {
    // Angular extent on each side of the centroid-to-destination bearing.
    double half_angle_degrees{90.0};
    DestinationFrontSegmentPolicy segment_policy{
        DestinationFrontSegmentPolicy::provisional_component};
    std::size_t minimum_secondary_segment_points{3U};
    DestinationFrontMode mode{DestinationFrontMode::retained_frontier};
};

/// References one contiguous component in a flattened display contour.
struct DisplayContourSegment {
    // Identifies a contiguous range in DisplayContours::points or
    // DisplayContourView::points.
    std::size_t point_offset{};
    std::size_t point_count{};
    // Closed segments implicitly connect their final point to their first.
    bool closed{};
};

/// Owning flattened contour coordinates and their component ranges.
struct DisplayContours {
    std::vector<Coordinate> points;
    std::vector<DisplayContourSegment> segments;
};

/// Non-owning contour data valid for the enclosing callback invocation.
struct DisplayContourView {
    std::span<const Coordinate> points;
    std::span<const DisplayContourSegment> segments;
};

// A contiguous segment of the destination-facing isochrone front. Points are
// ordered port-to-starboard (left to right relative to the destination bearing).
// Segments are open (not closed); the first and last point are not implicitly
// connected.
struct IsochroneFrontSegment {
    std::size_t point_offset{};
    std::size_t point_count{};
};

// Owning result produced by build_destination_front.
struct IsochroneFront {
    std::vector<Coordinate> points;
    std::vector<IsochroneFrontSegment> segments;
};

// Span-based, callback-lifetime view of a destination front delivered via
// RoutingProgressView. Spans are valid only for the synchronous callback
// invocation; copy before retaining or crossing threads.
struct IsochroneFrontView {
    std::span<const Coordinate> points;
    std::span<const IsochroneFrontSegment> segments;
};

/// Bit flags selecting data populated in RoutingProgressView.
enum class RoutingProgressPayload : std::uint8_t {
    none = 0U,
    retained_points = 1U << 0U,
    provisional_route = 1U << 1U,
    display_contours = 1U << 2U,
    destination_front = 1U << 3U,
    search_points = 1U << 4U,
};

constexpr RoutingProgressPayload operator|(
    RoutingProgressPayload left,
    RoutingProgressPayload right) noexcept {
    return static_cast<RoutingProgressPayload>(
        static_cast<std::uint8_t>(left) |
        static_cast<std::uint8_t>(right));
}

constexpr RoutingProgressPayload operator&(
    RoutingProgressPayload left,
    RoutingProgressPayload right) noexcept {
    return static_cast<RoutingProgressPayload>(
        static_cast<std::uint8_t>(left) &
        static_cast<std::uint8_t>(right));
}

[[nodiscard]] constexpr bool has_payload(
    RoutingProgressPayload payloads,
    RoutingProgressPayload payload) noexcept {
    return (payloads & payload) != RoutingProgressPayload::none;
}

/// Controls view callback cadence, payloads, and contour construction.
struct RoutingProgressOptions {
    // Deliver one callback for every Nth retained frontier.
    std::size_t every_n_steps{1U};
    RoutingProgressPayload payload{
        RoutingProgressPayload::retained_points |
        RoutingProgressPayload::provisional_route};
    DisplayContourOptions display_contours;
    DestinationFrontOptions destination_front;
};

/// Routing engine selected for one optimization request.
enum class RoutingSolver {
    isochrone_beam,
    time_dependent_lattice,
};

/// Label ordering used by the time-dependent lattice solver.
enum class LatticeSearchAlgorithm {
    a_star,
    dijkstra,
};

/// Resolution, temporal state, refinement, and progress controls for Stage 2.
struct LatticeRoutingOptions {
    std::size_t subdivision_level{4U};
    std::chrono::minutes time_bucket{30};
    std::size_t refinement_levels{1U};
    double corridor_width_nautical_miles{450.0};
    std::size_t corridor_widening_retries{2U};
    std::size_t progress_every_n_expansions{250U};
    LatticeSearchAlgorithm search_algorithm{LatticeSearchAlgorithm::a_star};
};

/// Explains how a route's effective departure time was selected.
enum class DepartureSource {
    explicit_time,
    current_time,
    forecast_start_fallback,
};

/// One timestamped point in a selected or provisional route.
struct RoutePoint {
    Coordinate position;
    TimePoint time;
    double heading_degrees{};
    double boat_speed_knots{};
    double true_wind_speed_knots{};
    double true_wind_direction_degrees{};
    double cumulative_distance_nautical_miles{};
};

/// Callback-scoped parent and candidate pair for eligibility decisions.
struct RouteSegmentView {
    const RoutePoint& parent;
    const RoutePoint& candidate;
};

using RouteSegmentEligibilityCallback =
    std::function<bool(const RouteSegmentView&)>;

/// Indicates whether a successful result reached the requested destination.
enum class RouteCompletion {
    destination_reached,
    forecast_exhausted,
};

/// Time deducted from a step when a candidate heading changes tack or gybe.
///
/// A sailboat loses time and distance through every tack and gybe, but an
/// isochrone search has no memory of which board a node is on, so it will
/// happily zigzag for free and report an arrival no real boat can match. When
/// any penalty is non-zero the search tracks the board each node is sailing and
/// charges these costs, and pruning keeps the best candidate on each board so a
/// marginally-closer wrong-tack candidate no longer displaces a faster one.
///
/// All penalties default to zero, which reproduces the unpenalised search.
struct ManeuverPenalties {
    std::chrono::seconds tack_penalty{0};
    std::chrono::seconds gybe_penalty{0};
    /// A true wind angle at or above this counts as downwind, so a board change
    /// is a gybe. At smaller angles a board change is charged as a tack.
    double downwind_true_wind_angle_degrees{150.0};

    [[nodiscard]] bool active() const noexcept {
        return tack_penalty > std::chrono::seconds::zero() ||
            gybe_penalty > std::chrono::seconds::zero();
    }
};

/// Extra headings evaluated per node beyond the fixed heading grid.
enum class HeadingAugmentation {
    /// Only multiples of `heading_step_degrees`, measured from 0 degrees true.
    none,
    /// Also the great-circle bearing to the destination.
    destination_bearing,
    /// Also the headings that maximise upwind and downwind velocity made good.
    velocity_made_good,
    /// Both of the above.
    destination_bearing_and_velocity_made_good,
};

/// Where along a step the wind used to advance a candidate is sampled.
enum class WindSampling {
    /// One sample at the segment start, held for the whole step.
    segment_start,
    /// A second sample at the midpoint of a provisional segment, which raises
    /// the integration from first to second order.
    midpoint,
};

/// How boat speed is interpolated between the polar's true wind angle rows.
enum class PolarAngleInterpolation {
    /// Straight lines between rows.
    linear,
    /// Monotone cubic (PCHIP) through the rows, which resolves the polar's
    /// peaks without the overshoot an unconstrained spline would introduce.
    monotone_cubic,
};

/// What to do when the true wind speed exceeds the polar's last column.
enum class AbovePolarRangePolicy {
    /// Hold the polar's highest tabulated wind speed.
    clamp,
    /// Treat the vessel as unable to sail, which removes the candidate.
    no_speed,
};

/// Which candidates a pruning pass keeps as the frontier advances.
enum class PruningStrategy {
    /// Grid buckets in destination-relative east/north nautical miles.
    destination_distance_grid,
    /// Sectors of bearing from the destination, which stay proportional to
    /// range and so preserve wide-angle diversity further out.
    bearing_sectors,
};

/// Search resolution, pruning, concurrency, progress, and eligibility controls.
struct RoutingOptions {
    std::chrono::minutes time_step{30};
    double heading_step_degrees{10.0};
    double arrival_radius_nautical_miles{2.0};
    double spatial_bucket_nautical_miles{10.0};
    std::size_t max_nodes_per_bucket{10};
    std::size_t worker_count{0};
    std::chrono::hours maximum_route_duration{240};
    double minimum_boat_speed_knots{0.05};
    bool capture_isochrones{false};
    bool use_routing_intervals{true};
    std::vector<RoutingInterval> routing_intervals{
        {std::chrono::minutes{30}, std::chrono::minutes{240}},
        {std::chrono::minutes{60}, std::chrono::minutes{1'440}},
        {std::chrono::minutes{180}, std::nullopt},
    };
    RoutingProgressOptions progress;

    // Accuracy controls. Every default below reproduces the search exactly as it
    // behaved before these options existed.
    ManeuverPenalties maneuver;
    HeadingAugmentation heading_augmentation{HeadingAugmentation::none};
    WindSampling wind_sampling{WindSampling::segment_start};
    /// Skips midpoint sampling for steps shorter than this, where the extra
    /// interpolation buys little. Zero applies it to every step.
    std::chrono::minutes midpoint_wind_sampling_threshold{0};
    PolarAngleInterpolation polar_angle_interpolation{
        PolarAngleInterpolation::linear};
    /// Wind speed above which the vessel is treated as unable to sail, modelling
    /// a storm limit the polar itself does not express. Unset imposes no limit.
    std::optional<double> maximum_true_wind_speed_knots;
    AbovePolarRangePolicy above_polar_range{AbovePolarRangePolicy::clamp};
    PruningStrategy pruning_strategy{PruningStrategy::destination_distance_grid};
    /// Angular width of a bearing sector, used only by `bearing_sectors`.
    double pruning_sector_degrees{2.0};

    // Empty accepts every segment without invocation. Otherwise called
    // synchronously before retention; true accepts and false rejects.
    // The view is valid only for the callback.
    RouteSegmentEligibilityCallback segment_eligibility;

    // Stage 2 controls are trailing so positional initializers written against
    // the pre-Stage-2 aggregate keep their original field mapping.
    RoutingSolver solver{RoutingSolver::isochrone_beam};
    LatticeRoutingOptions lattice;
};

/// Start, destination, optional departure, and routing configuration.
struct RouteRequest {
    Coordinate start;
    Coordinate destination;
    std::optional<TimePoint> departure_time;
    RoutingOptions options;
};

/// Cumulative search-work counters.
struct RouteDiagnostics {
    std::size_t expanded_nodes{};
    std::size_t generated_candidates{};
    std::size_t retained_candidates{};
    std::size_t time_steps{};
};

/// Lattice-only search and refinement work. Absent for the isochrone solver.
struct LatticeRouteDiagnostics {
    std::size_t settled_labels{};
    std::size_t queued_labels{};
    std::size_t relaxed_labels{};
    std::size_t wait_transitions{};
    std::size_t refinement_runs{};
    std::size_t accepted_refinements{};
    std::size_t subdivision_level{};
    bool refinement_fallback{};
};

/// Lattice-only callback counters for the currently active search pass.
struct LatticeSearchProgress {
    std::size_t settled_labels{};
    std::size_t queued_labels{};
    std::size_t relaxed_labels{};
    std::size_t refinement_index{};
    std::size_t subdivision_level{};
};

/// One retained search frontier at a completed routing time step.
struct Isochrone {
    TimePoint time;
    std::vector<Coordinate> points;
};

/// Owning progress snapshot used by Router::optimize callbacks.
struct RoutingProgress {
    Isochrone isochrone;
    std::vector<RoutePoint> provisional_route;
    IsochroneFront destination_front;
    RouteDiagnostics diagnostics;
    RoutingSolver solver{RoutingSolver::isochrone_beam};
    std::vector<Coordinate> search_points;
    LatticeSearchProgress search;
};

/// Callback-scoped progress data used by Router::optimize_view.
struct RoutingProgressView {
    TimePoint time;
    std::span<const Coordinate> retained_points;
    std::span<const RoutePoint> provisional_route;
    DisplayContourView display_contours;
    IsochroneFrontView destination_front;
    RouteDiagnostics diagnostics;
    RoutingSolver solver{RoutingSolver::isochrone_beam};
    std::span<const Coordinate> search_points;
    LatticeSearchProgress search;
};

/// Successful complete or forecast-exhausted routing output.
struct RouteResult {
    TimePoint departure_time;
    // For partial results, this is the time of the final forecast-supported point.
    TimePoint arrival_time;
    DepartureSource departure_source;
    std::string forecast_source;
    std::string polar_source;
    std::vector<RoutePoint> points;
    std::vector<Isochrone> isochrones;
    RouteDiagnostics diagnostics;
    RouteCompletion completion{RouteCompletion::destination_reached};
    std::optional<LatticeRouteDiagnostics> lattice_diagnostics;
};

/// Checks coordinate finiteness and canonical latitude/longitude bounds.
[[nodiscard]] bool is_valid(Coordinate coordinate) noexcept;
/// Returns the stable snake_case name of a departure source.
std::string_view to_string(DepartureSource source) noexcept;
/// Returns the stable snake_case name of a completion state.
std::string_view to_string(RouteCompletion completion) noexcept;
/// Returns the stable snake_case name of a routing solver.
std::string_view to_string(RoutingSolver solver) noexcept;

}  // namespace sailroute
