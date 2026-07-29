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

/// Owning, notification-only progress callback.
using RoutingProgressCallback = std::function<void(const RoutingProgress&)>;

/// Return value for callbacks that can stop an optimization.
enum class RoutingProgressDecision {
    continue_routing,
    cancel,
};

/// Owning progress callback with cancellation control.
using RoutingControlCallback =
    std::function<RoutingProgressDecision(const RoutingProgress&)>;

/// Non-owning, notification-only progress callback.
using RoutingProgressViewCallback =
    std::function<void(const RoutingProgressView&)>;

/// Non-owning progress callback with cancellation control.
using RoutingViewControlCallback =
    std::function<RoutingProgressDecision(const RoutingProgressView&)>;

class Router {
public:
    /// Takes immutable weather and polar data reusable across optimizations.
    Router(WeatherDataset weather, VesselPolar polar = VesselPolar::default_racer_cruiser_45ft());

    /// Optimizes a route without intermediate progress delivery.
    [[nodiscard]] Result<RouteResult> optimize(const RouteRequest& request) const;
    /// Optimizes a route with owning, notification-only progress snapshots.
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

    /// Optimizes with allocation-efficient callback-scoped progress views.
    ///
    /// All spans remain valid only for the synchronous callback invocation.
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
