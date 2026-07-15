#pragma once

#include "sailroute/error.hpp"
#include "sailroute/polar.hpp"
#include "sailroute/types.hpp"
#include "sailroute/weather.hpp"

#include <functional>

namespace sailroute {

using RoutingProgressCallback = std::function<void(const RoutingProgress&)>;

class Router {
public:
    Router(WeatherDataset weather, VesselPolar polar = VesselPolar::default_racer_cruiser_45ft());

    [[nodiscard]] Result<RouteResult> optimize(const RouteRequest& request) const;
    [[nodiscard]] Result<RouteResult> optimize(
        const RouteRequest& request,
        const RoutingProgressCallback& on_progress) const;

private:
    WeatherDataset weather_;
    VesselPolar polar_;
};

}  // namespace sailroute
