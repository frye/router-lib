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
struct DestinationFrontOptions {
    // Angular extent on each side of the centroid-to-destination bearing.
    double half_angle_degrees{90.0};
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
    // Empty accepts every segment without invocation. Otherwise called
    // synchronously before retention; true accepts and false rejects.
    // The view is valid only for the callback.
    RouteSegmentEligibilityCallback segment_eligibility;
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
};

/// Callback-scoped progress data used by Router::optimize_view.
struct RoutingProgressView {
    TimePoint time;
    std::span<const Coordinate> retained_points;
    std::span<const RoutePoint> provisional_route;
    DisplayContourView display_contours;
    IsochroneFrontView destination_front;
    RouteDiagnostics diagnostics;
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
};

/// Checks coordinate finiteness and canonical latitude/longitude bounds.
[[nodiscard]] bool is_valid(Coordinate coordinate) noexcept;
/// Returns the stable snake_case name of a departure source.
std::string_view to_string(DepartureSource source) noexcept;
/// Returns the stable snake_case name of a completion state.
std::string_view to_string(RouteCompletion completion) noexcept;

}  // namespace sailroute
