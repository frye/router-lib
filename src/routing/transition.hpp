#pragma once

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
/// is infeasible under the configured wind, polar, duration, or eligibility
/// policy; an error means weather interpolation itself failed.
[[nodiscard]] Result<std::optional<VariableTransition>>
evaluate_variable_transition(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingOptions& options,
    const RoutePoint& parent,
    std::int8_t parent_board,
    Coordinate destination,
    TimePoint route_end);

}  // namespace sailroute::detail
