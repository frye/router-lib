#pragma once

#include "sailroute/polar.hpp"
#include "sailroute/router.hpp"

#include <filesystem>

namespace sailroute::benchmarks {

void report_ensemble_scaling(
    const std::filesystem::path& grib_path,
    const VesselPolar& polar,
    const RouteRequest& deterministic_request);

}  // namespace sailroute::benchmarks
