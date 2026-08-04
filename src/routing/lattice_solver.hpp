#pragma once

#include "sailroute/router.hpp"

namespace sailroute::detail {

[[nodiscard]] Result<RouteResult> optimize_lattice_route(
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RoutingEnvironment& environment,
    const RouteRequest& request,
    TimePoint departure,
    DepartureSource departure_source,
    const RoutingViewControlCallback& on_progress);

}  // namespace sailroute::detail
