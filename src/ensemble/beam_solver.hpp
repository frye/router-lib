#pragma once

#include "ensemble/lattice_solver.hpp"
#include "sailroute/ensemble.hpp"

namespace sailroute::detail {

[[nodiscard]] Result<EnsembleRouteResult> optimize_ensemble_beam_route(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const EnsembleRouteRequest& request,
    const EnsembleCancellationCallback& cancelled = {},
    const EnsembleStepCallback& on_step = {});

}  // namespace sailroute::detail
