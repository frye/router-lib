#pragma once

#include "sailroute/error.hpp"
#include "sailroute/polar.hpp"
#include "sailroute/types.hpp"
#include "sailroute/weather.hpp"

namespace sailroute {

class Router {
public:
    Router(WeatherDataset weather, VesselPolar polar = VesselPolar::default_racer_cruiser_45ft());

    [[nodiscard]] Result<RouteResult> optimize(const RouteRequest& request) const;

private:
    WeatherDataset weather_;
    VesselPolar polar_;
};

}  // namespace sailroute
