#pragma once

#include "sailroute/environment.hpp"
#include "sailroute/polar.hpp"
#include "sailroute/types.hpp"
#include "sailroute/weather.hpp"

#include <cstdint>
#include <optional>

namespace sailroute::detail {

struct VariableTransition {
    RoutePoint point;
    std::int8_t board{};
};

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
    std::int8_t parent_board,
    Coordinate destination,
    TimePoint route_end);

}  // namespace sailroute::detail
