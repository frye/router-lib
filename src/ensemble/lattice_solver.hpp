#pragma once

#include "sailroute/ensemble.hpp"

#include <functional>

namespace sailroute::detail {

using EnsembleCancellationCallback = std::function<bool()>;
using EnsembleStepCallback =
    std::function<RoutingProgressDecision(const EnsembleProgressView&)>;

[[nodiscard]] Result<EnsembleRouteResult> optimize_ensemble_lattice_route(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const EnsembleRouteRequest& request,
    const EnsembleCancellationCallback& cancelled = {},
    const EnsembleStepCallback& on_step = {});

}  // namespace sailroute::detail
