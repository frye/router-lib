#pragma once

#include "sailroute/router.hpp"

#include <cstddef>
#include <iosfwd>

namespace sailroute::benchmarks {

struct StrategicScenario {
    WeatherDataset weather;
    VesselPolar polar;
    RoutingEnvironment environment;
    RouteRequest request;
};

/// Controlled regular_ll GRIB and discrete synthetic polar, shared with tests.
/// Local generated input files are removed before returning the loaded data.
[[nodiscard]] StrategicScenario make_strategic_scenario(
    bool mirrored = false, bool with_island = false);

/// Explicitly selected, small comparison; never runs the exhaustive reference
/// or an ensemble matrix. Emits CSV, including timed-action replay of each
/// completed candidate. One iteration per policy is the default. Optional
/// waypoint replay is a distinct geometric steering problem, not the same policy.
void report_strategic_benchmark(
    std::ostream& output, bool mirrored = false, bool with_island = false,
    std::size_t iterations = 1U, bool replay_waypoints = false);

}  // namespace sailroute::benchmarks
