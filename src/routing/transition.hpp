#pragma once

#include "sailroute/environment.hpp"
#include "sailroute/polar.hpp"
#include "sailroute/types.hpp"
#include "sailroute/weather.hpp"

#include "routing/state.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace sailroute::detail {

struct VariableTransition {
    RoutePoint point;
    OperationalConfiguration configuration;
};

struct EvaluatedWind {
    double speed_knots{};
    double direction_from_degrees{};
};

/// Converts an east/north wind vector in metres per second to finite routing
/// values in knots/degrees. A successful empty result means a configured wind
/// limit makes the state infeasible.
[[nodiscard]] Result<std::optional<EvaluatedWind>> evaluate_wind(
    Wind wind,
    const RoutingOptions& options);

/// Returns the board for headings and meteorological wind-from directions in
/// degrees true. Zero is reserved for head-to-wind and dead-downwind states.
[[nodiscard]] std::int8_t board_for_heading(
    double heading_degrees,
    double wind_from_degrees) noexcept;

/// Returns the configured tack/gybe delay from true-wind angles in degrees.
[[nodiscard]] std::chrono::seconds maneuver_delay(
    const ManeuverPenalties& penalties,
    OperationalConfiguration parent,
    double parent_true_wind_angle_degrees,
    OperationalConfiguration candidate,
    double candidate_true_wind_angle_degrees) noexcept;

/// Resolves boat speed in knots for a true-wind angle in degrees. Empty means
/// the polar range or minimum-speed policy makes the action infeasible.
[[nodiscard]] std::optional<double> boat_speed_for_angle(
    const PolarSlice& slice,
    const RoutingOptions& options,
    double true_wind_angle_degrees) noexcept;

/// Evaluates one time-dependent edge. A successful empty result means the edge
/// is infeasible under the configured wind, polar, environment, duration, or
/// eligibility policy; an error means weather interpolation, an environmental
/// provider, or a sea-state model failed under a `fail_route` policy.
///
/// The ground target is fixed, so when a current is configured this solves for
/// the water heading whose ground track reaches the target rather than adding
/// the current to boat speed. An inactive `environment` reproduces the
/// pre-Stage 3 arithmetic exactly.
[[nodiscard]] Result<std::optional<VariableTransition>>
evaluate_variable_transition(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutingEnvironment& environment,
    EnvironmentDiagnostics& diagnostics,
    const RoutePoint& parent,
    OperationalConfiguration parent_configuration,
    Coordinate destination,
    TimePoint route_end);

}  // namespace sailroute::detail
