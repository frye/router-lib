#pragma once

#include "sailroute/error.hpp"
#include "sailroute/polar.hpp"
#include "sailroute/types.hpp"
#include "sailroute/weather.hpp"

#include <concepts>
#include <functional>
#include <type_traits>
#include <utility>

namespace sailroute {

using RoutingProgressCallback = std::function<void(const RoutingProgress&)>;

enum class RoutingProgressDecision {
    continue_routing,
    cancel,
};

using RoutingControlCallback =
    std::function<RoutingProgressDecision(const RoutingProgress&)>;

class Router {
public:
    Router(WeatherDataset weather, VesselPolar polar = VesselPolar::default_racer_cruiser_45ft());

    [[nodiscard]] Result<RouteResult> optimize(const RouteRequest& request) const;
    [[nodiscard]] Result<RouteResult> optimize(
        const RouteRequest& request,
        const RoutingProgressCallback& on_progress) const;

    template <typename Callback>
        requires std::same_as<
            std::invoke_result_t<Callback&, const RoutingProgress&>,
            RoutingProgressDecision>
    [[nodiscard]] Result<RouteResult> optimize(
        const RouteRequest& request,
        Callback&& on_progress) const {
        return optimize_controlled(
            request,
            RoutingControlCallback{std::forward<Callback>(on_progress)});
    }

private:
    [[nodiscard]] Result<RouteResult> optimize_controlled(
        const RouteRequest& request,
        const RoutingControlCallback& on_progress) const;

    WeatherDataset weather_;
    VesselPolar polar_;
};

}  // namespace sailroute
