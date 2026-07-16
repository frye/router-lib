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

using RoutingProgressViewCallback =
    std::function<void(const RoutingProgressView&)>;

using RoutingViewControlCallback =
    std::function<RoutingProgressDecision(const RoutingProgressView&)>;

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
            RoutingProgressDecision> &&
            std::constructible_from<RoutingControlCallback, Callback&&>
    [[nodiscard]] Result<RouteResult> optimize(
        const RouteRequest& request,
        Callback&& on_progress) const {
        return optimize_controlled(
            request,
            RoutingControlCallback{std::forward<Callback>(on_progress)});
    }

    // All spans in RoutingProgressView remain valid only for the synchronous
    // callback invocation. Copy them before retaining data or crossing threads.
    [[nodiscard]] Result<RouteResult> optimize_view(
        const RouteRequest& request,
        const RoutingProgressViewCallback& on_progress) const;

    template <typename Callback>
        requires std::same_as<
            std::invoke_result_t<Callback&, const RoutingProgressView&>,
            RoutingProgressDecision> &&
            std::constructible_from<RoutingViewControlCallback, Callback&&>
    [[nodiscard]] Result<RouteResult> optimize_view(
        const RouteRequest& request,
        Callback&& on_progress) const {
        return optimize_view_controlled(
            request,
            RoutingViewControlCallback{std::forward<Callback>(on_progress)});
    }

private:
    [[nodiscard]] Result<RouteResult> optimize_controlled(
        const RouteRequest& request,
        const RoutingControlCallback& on_progress) const;
    [[nodiscard]] Result<RouteResult> optimize_view_controlled(
        const RouteRequest& request,
        const RoutingViewControlCallback& on_progress) const;

    WeatherDataset weather_;
    VesselPolar polar_;
};

}  // namespace sailroute
