#pragma once

#include "sailroute/environment.hpp"
#include "sailroute/types.hpp"

#include <optional>

namespace sailroute::detail {

/// How an environment evaluation ended.
enum class EnvironmentOutcome {
    /// Usable data; routing continues.
    accepted,
    /// The configured policy drops this transition.
    rejected,
    /// The configured policy fails the whole optimization.
    failed,
};

/// Environmental state applied to one transition.
struct EnvironmentSamples {
    bool has_current{false};
    CurrentVector current{};
    bool has_wave{false};
    WaveState wave{};
};

/// Result of sampling every configured field at one position and time.
struct EnvironmentSampleResult {
    EnvironmentOutcome outcome{EnvironmentOutcome::accepted};
    EnvironmentSamples samples;
    std::optional<Error> error;
};

/// Ground-frame motion produced by translating water velocity by the current.
struct GroundVelocity {
    double course_degrees{};
    double speed_knots{};
};

/// Samples currents and waves, honouring each provider's missing-data policy.
///
/// Counters are accumulated into `diagnostics`, which the caller owns; the
/// isochrone solver keeps one per worker and merges the sums afterwards, so
/// the totals never depend on worker count.
[[nodiscard]] EnvironmentSampleResult sample_environment(
    const RoutingEnvironment& environment,
    Coordinate coordinate,
    TimePoint time,
    EnvironmentDiagnostics& diagnostics);

/// Returns the angle between a water-frame heading and the direction the waves
/// travel toward: zero is a following sea and 180 degrees a head sea.
[[nodiscard]] double relative_wave_angle_degrees(
    double heading_degrees,
    double wave_direction_from_degrees) noexcept;

/// Applies the configured sea-state model to a flat-water polar speed.
///
/// Returns an error when the model breaks its contract by producing a
/// non-finite, negative, or accelerated speed, because that is a configuration
/// fault rather than a sea state to be silently clamped.
[[nodiscard]] Result<double> apply_sea_state(
    const RoutingEnvironment& environment,
    double flat_water_speed_knots,
    double true_wind_speed_knots,
    double true_wind_angle_degrees,
    double heading_degrees,
    const WaveState& wave,
    EnvironmentDiagnostics& diagnostics);

/// Translates a water-frame velocity into the ground frame.
[[nodiscard]] GroundVelocity ground_velocity(
    double water_heading_degrees,
    double water_speed_knots,
    CurrentVector current) noexcept;

/// Solves for the water heading whose ground track follows `ground_course`.
///
/// Returns the water-heading offset in degrees, or `std::nullopt` when the
/// current is too strong for the vessel to hold the track at all.
[[nodiscard]] std::optional<double> water_heading_offset_degrees(
    double ground_course_degrees,
    double water_speed_knots,
    CurrentVector current) noexcept;

/// Result of checking one timed segment against land and exclusion zones.
struct SegmentCheckResult {
    EnvironmentOutcome outcome{EnvironmentOutcome::accepted};
    std::optional<Error> error;
};

/// Runs the landmask and exclusion checks in that fixed order.
[[nodiscard]] SegmentCheckResult check_segment_geometry(
    const RoutingEnvironment& environment,
    Coordinate from,
    TimePoint from_time,
    Coordinate to,
    TimePoint to_time,
    EnvironmentDiagnostics& diagnostics);

/// Adds one worker's counters into a shared total.
void merge(EnvironmentDiagnostics& into, const EnvironmentDiagnostics& from) noexcept;

}  // namespace sailroute::detail
