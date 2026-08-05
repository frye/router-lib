# router-lib

`router-lib` is a C++20 library for fastest-arrival sailing route optimization
using downloaded GRIB weather forecasts and vessel polars. It includes the
`sailroute` command-line tool, uses ECMWF ecCodes for GRIB1/GRIB2 decoding, and
provides deterministic isochrone-beam and time-dependent geodesic-lattice
routing.

> [!WARNING]
> Currents, sea state, land avoidance, and exclusion zones are opt-in and must
> be configured with providers the application supplies; see
> [Environmental physics](#environmental-physics). With none configured, the
> library does not model land, shorelines, currents, waves, traffic, restricted
> areas, or safety limits, and routes may cross land. It never models traffic or
> bathymetry. The built-in polar and the built-in sea-state derating model are
> approximate demonstration models, not navigation-certified data.

## Supported MVP data

- GRIB1 or GRIB2 regular latitude/longitude grids
- Paired 10-metre U/V wind fields at one or more valid forecast times
- CSV matrix and common Expedition-style `.pol` vessel polars
- UTC departure timestamps formatted as `YYYY-MM-DDTHH:MM:SSZ`
- In-memory current, sea-state, landmask, and exclusion-zone providers supplied
  by the application; no environmental file formats are read

An explicit departure outside forecast coverage is rejected. If departure is
omitted, the library uses current UTC time when it can be interpolated from the
forecast, otherwise it uses the forecast's first valid time.

If forecast coverage ends before the destination is reached, routing succeeds
with the best forecast-supported partial route and
`RouteCompletion::forecast_exhausted`. Other incomplete searches, including
maximum-duration exhaustion, remain routing errors.

## Requirements and build

Building requires CMake 3.20 or newer, a C++20 compiler, and ECMWF ecCodes.
The build first looks for an ecCodes CMake package and then falls back to
`pkg-config`.

On macOS, install the dependencies and build with:

```sh
brew install cmake eccodes
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

| CMake option | Default | Purpose |
| --- | --- | --- |
| `SAILROUTE_BUILD_TESTS` | `ON` | Build the test executable and CLI tests |
| `SAILROUTE_BUILD_BENCHMARKS` | `OFF` | Build the microbenchmark executable |
| `SAILROUTE_ENABLE_LTO` | `OFF` | Enable link-time optimization when supported |

### Install and consume with CMake

Install the library, headers, CLI, and CMake package to a chosen prefix:

```sh
cmake --install build --prefix "$PWD/install"
```

Point a consuming project at that prefix, then link the exported target:

```cmake
find_package(sailroute 0.4 CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE sailroute::sailroute)
```

```sh
cmake -S consumer -B consumer/build \
  -DCMAKE_PREFIX_PATH="$PWD/install"
```

The package also resolves its Threads and ecCodes dependencies. Include
`<sailroute/sailroute.hpp>` for the complete public API or include individual
headers such as `<sailroute/weather.hpp>` and `<sailroute/router.hpp>`.

## CLI

```sh
./build/sailroute \
  --grib forecast.grib2 \
  --start 37.7749,-122.4194 \
  --destination 21.3069,-157.8583 \
  --polar boat.pol \
  --departure 2026-07-14T16:19:01Z \
  --routing-intervals 15m@6h,30m@24h,2h \
  --json route.json \
  --gpx route.gpx \
  --isochrones-json isochrones.json \
  --isochrones-gpx isochrones.gpx
```

Omit `--polar` to use the approximate built-in 45-foot racer-cruiser polar.
Omit `--json` to write JSON to stdout. Run `sailroute --help` for routing
resolution controls. Isochrone output is optional and contains the retained
post-pruning search frontier at each completed routing time step. The JSON
output is a GeoJSON `FeatureCollection` of `LineString` or `MultiLineString`
contours; the GPX output contains one track per frontier and one track segment
per contour component. When the forecast is exhausted, the CLI writes the
partial route and emits a warning on standard error. Route JSON and GPX include
the completion status so downstream consumers can distinguish partial output.

The router retains up to 10 nodes per spatial bucket by default. Increase
`--max-nodes-per-bucket` to preserve a larger set of alternate paths, or reduce
it when runtime and memory are more important than search breadth.

The accuracy options described under
[Accuracy modeling](#accuracy-modeling) have matching flags, all defaulting to
the behavior described above:

| Flag | Values |
| --- | --- |
| `--tack-penalty-seconds N` | non-negative integer, default `0` |
| `--gybe-penalty-seconds N` | non-negative integer, default `0` |
| `--downwind-twa-degrees N` | `[0,180]`, default `150` |
| `--heading-augmentation MODE` | `none`, `destination-bearing`, `vmg`, `both` |
| `--wind-sampling MODE` | `segment-start`, `midpoint` |
| `--midpoint-wind-threshold-minutes N` | non-negative integer, default `0` |
| `--polar-angle-interpolation MODE` | `linear`, `monotone-cubic` |
| `--maximum-wind-speed-knots N` | positive, unset by default |
| `--above-polar-range MODE` | `clamp`, `no-speed` |
| `--pruning-strategy MODE` | `distance-grid`, `bearing-sectors` |
| `--pruning-sector-degrees N` | `(0,180]`, default `2` |

The lattice solver is opt-in through `--solver lattice`. Its controls are
rejected unless that solver is selected:

| Flag | Values |
| --- | --- |
| `--lattice-level N` | subdivision level, default `4` |
| `--lattice-time-bucket-minutes N` | positive integer, default `30` |
| `--lattice-refinement-levels N` | non-negative integer, default `1` |
| `--lattice-corridor-nm N` | positive number, default `450` |
| `--lattice-corridor-retries N` | non-negative integer, default `2` |
| `--lattice-progress-expansions N` | positive integer, default `250` |
| `--lattice-search MODE` | `a-star`, `dijkstra`; default `a-star` |

For example, this explicitly requests two refinement levels with the Dijkstra
oracle:

```sh
./build/sailroute \
  --grib forecast.grib2 \
  --start 37.7749,-122.4194 \
  --destination 21.3069,-157.8583 \
  --solver lattice \
  --lattice-refinement-levels 2 \
  --lattice-corridor-retries 3 \
  --lattice-search dijkstra
```

For example, to charge for maneuvers, sail the polar's VMG optima, and resolve
the polar peak with a shape-preserving fit:

```sh
./build/sailroute \
  --grib forecast.grib2 \
  --start 48.294300,-123.531697 \
  --destination 48.141100,-123.402687 \
  --tack-penalty-seconds 60 \
  --gybe-penalty-seconds 30 \
  --heading-augmentation both \
  --polar-angle-interpolation monotone-cubic \
  --wind-sampling midpoint
```

The CLI returns `0` on success (including a forecast-exhausted partial route),
`2` for command-line usage errors, `3` for weather or polar input errors, `4`
for routing errors, and `5` for serialization or file-output errors.

Routing intervals are measured from departure. By default, the router creates
points every 30 minutes for the first 4 hours, every hour through the first 24
hours, and every 3 hours thereafter. Override the schedule with
`--routing-intervals`; each bounded tier uses `INTERVAL@CUTOFF`, followed by one
open-ended interval. Durations are positive integers suffixed with `m` or `h`,
for example `5m@2h,30m@12h,1h`. Cutoffs must increase, and configured intervals
must be at least 5 minutes. A step is shortened when necessary to land exactly
on a cutoff or the routing horizon. Use `--time-step-minutes N` for a constant
interval; it is mutually exclusive with `--routing-intervals`. C++ callers can
set `RoutingOptions::time_step` and set
`RoutingOptions::use_routing_intervals` to `false` for the same
constant-interval compatibility behavior.

The `samples/` directory contains an approximate First 44-class polar and
offshore coordinates for a Race Rocks to Port Angeles demonstration:

```sh
./build/sailroute \
  --grib forecast.grib2 \
  --start 48.294300,-123.531697 \
  --destination 48.141100,-123.402687 \
  --polar samples/sample.pol \
  --json route.json \
  --gpx route.gpx \
  --isochrones-json isochrones.json \
  --isochrones-gpx isochrones.gpx
```

## C++ API

```cpp
#include <sailroute/sailroute.hpp>

#include <iostream>
#include <utility>

int main() {
    auto weather = sailroute::WeatherDataset::load("forecast.grib2");
    if (!weather) {
        std::cerr << weather.error().message << '\n';
        return 1;
    }

    sailroute::Router router{std::move(weather.value())};
    sailroute::RouteRequest request{
        .start = {37.7749, -122.4194},
        .destination = {21.3069, -157.8583},
    };

    auto route = router.optimize(request);
    if (!route) {
        std::cerr << route.error().message << '\n';
        return 1;
    }

    auto json = sailroute::route_to_json(route.value());
    if (!json) {
        std::cerr << json.error().message << '\n';
        return 1;
    }
    std::cout << json.value();
}
```

Loaded weather and polar objects are immutable and reusable across route
requests, avoiding repeated GRIB decoding and polar preprocessing. Isochrone
capture is disabled by default so route-only callers do not retain the full
search frontier history. A successful `RouteResult` has completion
`destination_reached` or `forecast_exhausted`. For a forecast-exhausted result,
`points` owns the best route through the final supported frontier,
`arrival_time` is the final point's time, and diagnostics and requested
isochrones cover all completed routing steps.

### Error handling

All fallible library operations return `Result<T>`. Test it with `operator bool`
or `has_value()` before calling `value()`; failures provide an `ErrorCode` and a
human-readable `message`. `to_string(ErrorCode)` returns a stable symbolic name.
The library does not use exceptions for expected input, routing, or output
failures. Exceptions thrown by application callbacks are not caught.

Error categories distinguish invalid arguments, file I/O, GRIB decoding and
support, incomplete forecasts, invalid polars, coordinates or departures
outside forecast coverage, unavailable routes, exhausted forecasts, output
failures, callback cancellation, contradictory environment configuration, and
unavailable environmental data.

### Weather and polar data

`WeatherDataset::load(path)` decodes all supported wind slices. To reduce
memory, `load(path, GeographicBounds{south, west, north, east})` retains only
the interpolation subgrid needed for the requested canonical bounds. Bounds
may cross the antimeridian by setting west greater than east, but must remain
inside the source grid. `metadata()` reports the valid-time range, retained
grid dimensions, global-longitude coverage, and source path.

`WeatherDataset::interpolate(coordinate, time)` performs spatial and temporal
interpolation and returns eastward and northward wind components in metres per
second. `Wind::speed_knots()` converts the vector magnitude to knots, and
`direction_from_degrees()` returns the meteorological direction the wind comes
from in `[0, 360)`.

`VesselPolar::load(path)` loads a matrix polar; use
`default_racer_cruiser_45ft()` only for demonstrations. `boat_speed_knots()`
bilinearly interpolates within the matrix, folds angles to `[0, 180]`, and
clamps values outside either axis to the nearest edge. Non-finite inputs and
non-positive true-wind speeds return zero. `source()` identifies the loaded or
built-in data.

Both types expose a prepared-lookup form for the common case of many samples
sharing one coordinate of the lookup. `WeatherDataset::sampler_at(time)` resolves
the forecast time bracket once and returns a `WeatherSampler` whose `sample()`
takes only a coordinate. `VesselPolar::slice_at(wind_speed)` resolves the wind
column once and returns a `PolarSlice` whose `speed_knots()` takes only a true
wind angle. Both borrow nothing that can dangle: the sampler shares ownership of
the forecast, and a slice is valid for the lifetime of its polar.

```cpp
const auto sampler = weather.sampler_at(when);  // Result<WeatherSampler>
const auto wind = sampler.value().sample({37.7749, -122.4194});
const sailroute::PolarSlice slice = polar.slice_at(20.0);
const double speed = slice.speed_knots(52.0);
```

`sampler_at` reports errors exactly where `interpolate` does. A time outside
forecast coverage is carried on the sampler and surfaced by `sample()` only
after the coordinate is validated, so a request invalid in both respects reports
the coordinate problem, matching `interpolate`. `slice_at` also reports whether
the wind speed was past the polar's last column via
`PolarSlice::above_tabulated_wind_speed()`, and
`VesselPolar::maximum_tabulated_wind_speed_knots()` returns that column.

### Routing configuration

`RouteRequest` contains the start, destination, optional departure time, and
the following `RoutingOptions`:

| Option | Default | Meaning |
| --- | --- | --- |
| `solver` | `isochrone_beam` | Per-request selection of the legacy beam or Stage 2 lattice solver |
| `time_step` | 30 minutes | Constant step used when `use_routing_intervals` is `false` |
| `routing_intervals` | `30m@4h, 1h@24h, 3h` | Departure-relative variable step schedule |
| `use_routing_intervals` | `true` | Select variable intervals instead of `time_step` |
| `heading_step_degrees` | `10` | Candidate heading spacing in `(0, 180]` |
| `arrival_radius_nautical_miles` | `2` | Distance at which the destination is reached |
| `spatial_bucket_nautical_miles` | `10` | Spatial pruning bucket size |
| `max_nodes_per_bucket` | `10` | Alternate nodes retained in each bucket |
| `worker_count` | `0` | Expansion workers; zero selects automatically |
| `maximum_route_duration` | 240 hours | Search horizon from departure |
| `minimum_boat_speed_knots` | `0.05` | Slower candidates are discarded |
| `capture_isochrones` | `false` | Retain every completed frontier in `RouteResult` |
| `progress` | every step, points + route | View callback cadence and payload selection |
| `segment_eligibility` | empty | Optional synchronous candidate predicate |

Currents, sea state, land, and exclusion zones are configured on the `Router`
rather than per request, because they are immutable data reusable across
optimizations in the same way weather and polars are. See
[Environmental physics](#environmental-physics).

#### Time-dependent lattice solver

Set `RoutingOptions::solver` to
`RoutingSolver::time_dependent_lattice`, or pass `--solver lattice`, to use
deterministic A* over a hierarchical icosahedral lattice. The default remains
`isochrone_beam`; unchanged callers retain their existing route, progress,
diagnostic, and serialization behavior.

Lattice states include the geodesic cell, a seam-safe spherical position bucket,
exact arrival time, time bucket, and board. In addition to fixed-target neighbor
edges, the solver can sail a bucket-length successor on either of the polar's
upwind VMG headings. These off-vertex moves are generated when no fixed-target
edge is feasible or the destination is closer than one local edge, so a coarse
lattice can tack without turning every settled label into a continuous heading
fan. Their exact endpoints are associated with the nearest cell for subsequent
neighbor expansion and are never snapped to its centre.

Neighbor and VMG transitions share the same weather, polar, maneuver,
wind-envelope, midpoint-sampling, current, wave, land, exclusion, and
segment-eligibility rules. The VMG move ends at the next lattice time boundary
after deducting any tack penalty; its internal position bucket is derived from
the time bucket and polar speed bound. Explicit wait transitions remain
available for genuine later-departure opportunities. A* uses remaining
great-circle distance divided by the polar's global maximum boat speed; select
`LatticeSearchAlgorithm::dijkstra` as a zero-heuristic oracle.

| Lattice option | Default | Meaning |
| --- | --- | --- |
| `subdivision_level` | `4` | Coarse icosphere level; cell count is `10 * 4^level + 2` |
| `time_bucket` | 30 minutes | Temporal dominance bucket |
| `refinement_levels` | `1` | Successive coarse-to-fine passes |
| `corridor_width_nautical_miles` | `450` | Initial corridor around the incumbent |
| `corridor_widening_retries` | `2` | Bounded retries when a refined corridor disconnects |
| `progress_every_n_expansions` | `250` | Lattice callback cadence |
| `search_algorithm` | `a_star` | `a_star` or the Dijkstra oracle |

For source compatibility, Stage 2 fields are appended to the public aggregates
and default to the legacy beam. Existing source that does not select a solver
continues to produce the v0.3.2 route, progress, diagnostics, errors, and JSON
bytes covered by the compatibility corpus below. Rebuild C++ consumers when
upgrading because the public aggregate layouts have grown; do not mix headers
and binaries from different router-lib versions.

Refinement builds a mixed-resolution graph from the complete coarse lattice and
only subdivides faces intersecting the geodesic corridor around complete
incumbent route segments. Coarse cells remain outside the corridor, split
boundary edges reconnect deterministically, and a globally fine lattice is
never allocated. A refined route is accepted only when it is complete and no
later than the incumbent. If every bounded widening attempt disconnects or
regresses, the previous route is retained and the lattice diagnostics report
the fallback reason, failed attempts, accepted corridor width, and active
cell/face counts. Start and destination remain exact virtual anchors; neither
anchors nor VMG endpoints are snapped to cell centres. The solver is
intentionally serial, so `worker_count` has no effect on its deterministic
output.

Lattice callbacks identify
`RoutingSolver::time_dependent_lattice`, populate the distinct
`search_points`/`LatticeSearchProgress` fields, and may still populate the
existing provisional route payload. They do not repurpose retained isochrones,
display contours, or destination fronts. Lattice requests reject
`capture_isochrones` and those isochrone-specific progress payloads. Route JSON
adds `latticeDiagnostics` only for lattice results, preserving legacy JSON bytes
for the default solver.

The Stage 2 topology spike compared three equal-area/geodesic families before
search integration:

| Candidate | Regularity | Stable hierarchy | Neighbor graph | Local refinement | Outcome |
| --- | --- | --- | --- | --- | --- |
| Subdivided icosahedron | 5/6 neighbors, bounded edge spread | inherited vertex IDs | explicit reciprocal edges | natural face subdivision | selected |
| HEALPix | equal-area cells | strong | requires specialized polar/seam rules | hierarchical | rejected to avoid a bespoke indexing dependency |
| Fibonacci sphere | uniform representatives | no parent/child identity | k-nearest graph must be derived | global rebuild | rejected for unstable refinement boundaries |

The benchmark executable reports beam and lattice arrival quality, generated
work, settled labels, selected level, and wall time on the same forecast leg:

```sh
cmake -S . -B build-bench -DSAILROUTE_BUILD_BENCHMARKS=ON \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-bench --parallel
./build-bench/sailroute_benchmarks forecast.grib2
```

For external peak-memory measurements, run the same command under
`/usr/bin/time -l` on macOS or `/usr/bin/time -v` on Linux.

### v0.3.2 compatibility corpus

The offline compatibility corpus compiles against both the current tree and the
immutable pre-Stage-2 baseline at
`cd99342cdaeb6725639f6ae53a384db50b0e0ad0`. It creates deterministic regional
and global GRIB fixtures locally and covers antimeridian and high-latitude
routes, constant and scheduled intervals, accuracy controls, forecast
exhaustion, cancellation, segment eligibility, progress ordering, and worker
counts 1, 4, and automatic. CTest compares the complete route JSON, diagnostics,
errors, progress sequence, and callback counts byte-for-byte with the committed
v0.3.2 output:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build -R sailroute_v032_compatibility --output-on-failure
```

The golden output is generated only by compiling
`tests/compatibility_corpus.cpp` unchanged against the v0.3.2 tag. A mismatch
writes the candidate output to `build/compatibility-actual.txt`; do not replace
the golden file to approve an intentional behavior change.

The benchmark header records the candidate revision, compatibility baseline,
build type, compiler, operating system/architecture, and hardware concurrency.
Record the CPU model alongside published results with
`sysctl -n machdep.cpu.brand_string` on macOS or `lscpu` on Linux. Measure the
corpus peak resident memory independently with:

```sh
/usr/bin/time -l ./build/sailroute_compatibility_corpus  # macOS
/usr/bin/time -v ./build/sailroute_compatibility_corpus  # Linux
```

The following accuracy controls are opt-in. Every default reproduces the search
exactly as it behaved before these options existed, so enabling none of them
leaves routes bit-identical.

| Option | Default | Meaning |
| --- | --- | --- |
| `maneuver.tack_penalty` | `0s` | Time lost to a tack, deducted from the step |
| `maneuver.gybe_penalty` | `0s` | Time lost to a gybe, deducted from the step |
| `maneuver.downwind_true_wind_angle_degrees` | `150` | At or above this a board change is a gybe |
| `heading_augmentation` | `none` | Extra headings beyond the fixed grid |
| `wind_sampling` | `segment_start` | Where along a step wind is sampled |
| `midpoint_wind_sampling_threshold` | `0m` | Minimum step length for midpoint sampling |
| `polar_angle_interpolation` | `linear` | Interpolation between polar TWA rows |
| `maximum_true_wind_speed_knots` | unset | Wind speed above which the vessel will not sail |
| `above_polar_range` | `clamp` | Behavior past the polar's last wind column |
| `pruning_strategy` | `destination_distance_grid` | How the frontier is bucketed for pruning |
| `pruning_sector_degrees` | `2` | Sector width used by `bearing_sectors` |

Intervals and `time_step` must be at least five minutes. The final interval
tier must be open-ended, preceding cutoffs must be positive and strictly
increasing, and `progress.every_n_steps` must be positive. Other numeric
distances and durations must satisfy the ranges shown above.

`RouteResult::points` owns the selected route, including per-point heading,
boat speed, true wind, and cumulative distance. `diagnostics` reports expanded
nodes, generated and retained candidates, and completed time steps.
`departure_source` records whether departure was explicit, current time, or the
forecast-start fallback. Use `to_string()` for departure and completion enums.

### Accuracy modeling

The default search advances a beam of candidates one time step at a time. It
samples wind once per segment, evaluates headings on a fixed grid measured from
0 degrees true, interpolates the polar bilinearly, and keeps the candidate
closest to the destination in each spatial bucket. That is fast and
deterministic, but it makes several simplifications that the options below let
you trade against runtime.

**What is not modeled at all.** Currents, sea state, land, and exclusion zones
are outside the router. Land avoidance is delegated to
`segment_eligibility`. Boat speed comes from the polar alone, so it is a
flat-water, fully-crewed number.

#### Maneuver penalties

Without a cost for changing boards the search can zigzag for free, which
overstates upwind and downwind velocity made good. Set `maneuver.tack_penalty`
and `maneuver.gybe_penalty` to charge for the maneuver:

```cpp
request.options.maneuver.tack_penalty = std::chrono::seconds{60};
request.options.maneuver.gybe_penalty = std::chrono::seconds{30};
```

A candidate's board is the sign of its heading relative to the wind direction;
dead upwind and dead downwind count as neither board and are never penalized.
The maneuver is classified by the mean of the parent and candidate true wind
angles: at or above `downwind_true_wind_angle_degrees` it is a gybe, otherwise
it is a tack. The penalty is deducted from the usable step time before any
distance is made, and a penalty at least as long as the step removes the
candidate.

Activating penalties also splits each pruning bucket by board, so a candidate
on the favorable tack is no longer displaced by a marginally closer one on the
wrong tack. This reduces, but does not eliminate, the underlying limitation
described under [Known limitations](#known-limitations).

Because that split retains more of the frontier, charging for maneuvers can
occasionally report a marginally *earlier* arrival than not charging: the wider
beam found a better route, and the saving exceeded the maneuvers the winning
route pays for. The penalty itself is always a cost. Every leg's distance is its
boat speed times the time actually spent sailing, and on a leg that changes
boards the leg's duration exceeds that by exactly the penalty.

#### Heading augmentation

Headings are multiples of `heading_step_degrees` measured from 0 degrees true,
so the bearing to the destination and the polar's velocity-made-good optima are
essentially never sailed exactly. `heading_augmentation` appends extra headings
per node:

| Value | Extra headings per node |
| --- | --- |
| `none` | none |
| `destination_bearing` | the great-circle bearing to the destination |
| `velocity_made_good` | both boards of the upwind and downwind VMG optima |
| `destination_bearing_and_velocity_made_good` | all of the above |

The VMG optima are precomputed per wind-speed column when the polar is loaded
and blended for the sampled wind speed. They only *propose* headings; the search
still evaluates each one exactly through the polar, so the proposal being
approximate cannot make the reported route speed wrong.

Augmentation raises candidate count by roughly `5 / heading_count`. Because the
beam prunes, adding headings occasionally produces a marginally *later* arrival:
a new candidate can win its bucket on distance while being a worse platform for
the remaining passage. This is inherent to beam search rather than a defect in
the extra headings, and it applies to any option that changes which candidates
survive pruning. Measure on your own forecasts rather than assuming a gain.

#### Midpoint wind sampling

By default the wind at the segment start is held for the whole step, which is a
first-order integration; on the default three-hour tier that is roughly 30
nautical miles on one sample. `WindSampling::midpoint` re-evaluates boat speed
using the wind halfway along the provisional segment in both space and time,
raising the step to second order:

```cpp
request.options.wind_sampling = sailroute::WindSampling::midpoint;
request.options.midpoint_wind_sampling_threshold = std::chrono::minutes{60};
```

This costs a second wind interpolation and polar lookup per candidate. The
threshold skips it for shorter steps, where it buys little. If the midpoint
falls outside the forecast the first-order speed stands, so enabling this never
makes a route unreachable.

#### Monotone cubic polar interpolation

Linear interpolation between polar rows 15 to 20 degrees apart flattens the peak
and misplaces the VMG optimum. `PolarAngleInterpolation::monotone_cubic` fits a
shape-preserving (PCHIP) curve through the true-wind-angle rows, which resolves
the peak without the overshoot an unconstrained spline would introduce. Wind
speed is still interpolated linearly between columns. Enable it before
benchmarking `velocity_made_good` augmentation, which depends on peak
resolution.

#### Wind speed envelope

By default a wind speed past the polar's last column is clamped to that column,
so a 60-knot storm reads as the polar's top-end speed. Two independent controls
change that:

```cpp
request.options.maximum_true_wind_speed_knots = 35.0;
request.options.above_polar_range = sailroute::AbovePolarRangePolicy::no_speed;
```

`maximum_true_wind_speed_knots` expresses a limit the polar itself does not
carry and removes any node where the wind exceeds it. `above_polar_range` set to
`no_speed` refuses to extrapolate past the tabulated data. Either can make a
route unreachable, which is reported as a routing error rather than a silently
slow route.

#### Bearing sector pruning

`PruningStrategy::bearing_sectors` buckets the frontier by bearing from the
destination and range to it, instead of a fixed grid in destination-relative
east/north nautical miles. Sector width stays proportional to range, so far from
the destination it preserves wide-angle diversity a fixed grid would merge,
while converging and over-merging as the fan closes in. It retains noticeably
more candidates and is correspondingly slower. It is offered for measurement,
not recommended by default.

#### Measuring the trade-offs

`sailroute_benchmarks` reports arrival time, leg count, candidate count, and
wall time for each option and for all of them combined, using the longest leg
the supplied forecast covers:

```sh
cmake -S . -B build -DSAILROUTE_BUILD_BENCHMARKS=ON
cmake --build build
./build/sailroute_benchmarks samples/sample.grib
```

### Environmental physics

Currents, sea state, land, and time-varying exclusion zones are optional. A
`Router` built without a `RoutingEnvironment`, or with one that configures no
provider, evaluates exactly the arithmetic it evaluated before these providers
existed, down to the last bit of every route point, diagnostic counter, and
serialized value.

```cpp
sailroute::RoutingEnvironment environment;
environment.currents.provider =
    sailroute::make_uniform_current_provider(
        {.east_knots = 0.8, .north_knots = -0.2},
        {"gulf_stream_climatology", "internal dataset", "2026.07"})
        .value();
environment.waves.provider = my_wave_field;
environment.waves.model = sailroute::make_wave_height_derating_model().value();
environment.land.landmask = my_landmask;
environment.land.clearance_nautical_miles = 2.0;
environment.exclusions.zones = my_zones;

sailroute::Router router{
    std::move(weather.value()),
    sailroute::VesselPolar::default_racer_cruiser_45ft(),
    std::move(environment)};
```

Providers are supplied programmatically. The library ships deterministic
in-memory fields — uniform values and bilinearly interpolated regular grids —
so applications and tests can configure physics without any external dataset.
File-format adapters for current and wave GRIB, landmask rasters, and exclusion
records are deliberately not part of this stage; see
[Deferred work](#deferred-work).

#### Units and reference frames

| Quantity | Units | Convention |
| --- | --- | --- |
| Current | knots, east/north components | Vector points where the water flows *to* |
| Significant wave height | metres | — |
| Wave period | seconds | Representative or peak period |
| Wave direction | degrees true | Direction the waves come *from*, as for wind |
| Signed land distance | nautical miles | Positive over water, negative over land |
| Relative wave angle | degrees | 0 is a following sea, 180 a head sea |

`RoutePoint::heading_degrees` and `RoutePoint::boat_speed_knots` stay
water-relative whatever is configured, because that is the frame the polar and
the apparent wind are defined in. Ground motion lives in the optional
`RoutePoint::environment` audit record.

#### Transition evaluation order

Every transition, in both solvers, is evaluated in this fixed order:

1. sample the configured current and wave fields at the position and time;
2. evaluate true wind, polar speed, board, and maneuver delay in the water
   frame;
3. apply the sea-state model to the flat-water polar speed;
4. translate the water-frame velocity into ground velocity using the current;
5. derive ground position, arrival-radius crossing, and cumulative ground
   distance;
6. certify the segment against the landmask, then against active exclusion
   zones;
7. invoke the caller's `segment_eligibility` callback.

Land and exclusion rejection therefore happens strictly before retention,
progress output, and the caller callback. A rejected transition is never
visible to an application.

The two solvers differ only in what is held fixed. The isochrone beam fixes the
water heading and lets the current decide where the candidate lands. The
time-dependent lattice fixes the ground target, so it solves for the water
heading whose ground track reaches that target: the cross-track current is
cancelled by a heading offset and the along-track component is added to ground
speed. Because the through-water speed itself depends on the heading, that
solve is a bounded, deterministic fixed-point iteration rather than a single
step, and a cross-track current stronger than the vessel makes the transition
infeasible instead of merely slow.

Set `RoutingEnvironment::sampling` to `EnvironmentSampling::midpoint` to sample
the environment a second time at the provisional segment midpoint, matching
what `WindSampling::midpoint` does for wind.

#### Missing data

Current, wave, and land providers carry a `MissingDataPolicy`. `fail_route`
fails the whole optimization with `ErrorCode::environment_data_unavailable`;
`reject_transition` drops the affected transition as if it were infeasible.
There is no third option, and in particular missing data is never reinterpreted
as zero current, a calm sea, or open water. Exclusion sets are immutable,
globally defined geometry and therefore have no missing-data state. A provider
that reports `available` while returning a non-finite or out-of-range value is
treated as invalid data, not as a value.

`fail_route` is the default because silently narrowing a search is worse than
stopping. It does mean a bounded field fails as soon as the search fans outside
it, so a provider whose coverage is smaller than the search area normally wants
`reject_transition`.

#### Sea-state derating

`SeaStatePerformanceModel` is an interface, not a formula. It receives the
flat-water polar speed together with the water-frame wind and wave state and
returns a derated through-water speed. The router validates that result: it
must be finite, non-negative, and no greater than the flat-water speed. A model
that accelerates the vessel in waves fails the optimization with
`ErrorCode::invalid_environment` rather than being silently clamped, because
that is a configuration fault.

`make_wave_height_derating_model()` supplies a documented, bounded default: the
loss grows with significant wave height, scales between a following and a head
sea through the cosine of the relative wave angle, optionally sharpens for short
steep seas, and saturates at a configured maximum. Its coefficients are
approximate demonstration values, not a validated seakeeping prediction. Zero
wave height returns the flat-water speed exactly, so a calm-sea field reproduces
the flat-water route bit for bit.

#### Signed-distance landmask

`SignedDistanceLandmask` holds signed distance to the nearest coastline on a
regular grid, positive over water and negative over land, and interpolates it
bilinearly. It declares its own node spacing and an upper bound on its
interpolation error; that error is added to the configured clearance before any
decision, so a coarse mask can never round toward accepting land.

Segment certification uses the defining property of a distance field: it is
1-Lipschitz along a path, so no point on a segment of length `L` whose endpoints
sit at signed distances `a` and `b` can be nearer land than `(a + b - L) / 2`.
When that bound already clears the requirement the whole segment is certified
from two samples. Otherwise the segment is halved and each half is certified
recursively up to `maximum_subdivision_depth`. Running out of depth rejects the
segment, so two water endpoints can never hide a crossing between them. Leaving
the mask is reported as `outside_coverage` and handled by the configured policy
rather than assumed to be open water.

#### Time-varying exclusion zones

`ExclusionZoneSet` takes versioned records with a stable identifier, source
attribution, a revision, an optional half-open `[active_from, active_until)`
activation window, and one or more polygons with holes. Rings are closed
implicitly, are made of great-circle arcs, and need no special encoding for the
antimeridian or the poles, because every test runs in earth-centred Cartesian
coordinates. A ring's interior is the side of it that does not contain the
region outside its bounding cap, which is what makes a ring drawn along a
parallel — an Antarctic Exclusion Zone, say — enclose the polar cap it is meant
to.

Construction validates topology and rejects rings with fewer than three distinct
vertices, coordinates outside canonical bounds, holes that escape their outer
ring, duplicate identifier and revision pairs, inverted activation windows, and
rings so large that their interior is ambiguous. Zones are then sorted by
identifier and revision, so input order cannot change a route, a diagnostic, or
the identifier reported for a rejection.

A segment is tested only over the part of its traversal during which a zone is
actually active: the activation window is clipped against the traversal interval
and converted to a fraction of the segment. A zone that opens midway through a
leg constrains only the part still to be sailed. `ExclusionBoundaryPolicy`
decides whether touching a boundary counts as entering: `boundary_excluded`, the
default, treats contact as a violation, while `boundary_allowed` requires some
point of the segment to be strictly inside, which lets a route run along a
shared boundary between adjacent zones.

Waiting is checked too. The lattice solver's wait transitions are tested as
degenerate segments, so a vessel cannot legally sit still inside a zone that
opens around it.

The spatial index is a precomputed bounding cap per zone, compared against the
segment's own cap with a dot product. Cost is therefore linear in zone count but
with a very small constant; the benchmark reports the scaling directly.

#### Audit output

When an environment is configured, `RouteResult::environment` records the
sources, model names, policies, landmask resolution and clearance, and exclusion
revision that were actually applied, and `RouteResult::environment_diagnostics`
counts samples, evaluations, checks, and rejections per provider. The counters
are sums, so they are identical for any worker count. Each affected
`RoutePoint` carries an optional `RoutePointEnvironment` with speed and course
over ground, the applied current, the flat-water polar speed before derating,
and/or the applied sea state, with presence flags distinguishing configured
components from zero-valued samples.

All of this is additive. With no environment configured every existing JSON key,
GPX element, and diagnostic value is unchanged, and the new ones are absent
rather than empty.

#### Cost

Providers are sampled from the isochrone solver's worker threads, so every
provider must be immutable after construction and safe for concurrent sampling.
Provider input order and worker count cannot affect route selection or
diagnostics.

`sailroute_benchmarks` reports the no-provider baseline, each provider
individually, exclusion scaling, and all providers combined, with arrival deltas
and per-provider counters. Carrying the optional per-point audit record costs
roughly five percent on the no-provider isochrone path, because it widens every
candidate in the search array; that is the only measurable cost of Stage 3 when
nothing is configured, and route results remain bit-identical.

### Known limitations

The default search is a forward beam. Pruning freezes each surviving
candidate's parent chain, and there is no backtracking, so an earlier leg is
never revisited once its step completes. If a discarded candidate would have
led to a materially faster passage later, that outcome is unrecoverable:
pruning is the only place the optimum is lost, and it is lost permanently.

Board-aware pruning narrows this failure, because the common case is a
wrong-tack candidate displacing a faster one inside a single bucket. It does not
remove it. The opt-in lattice solver re-relaxes labels through a
time-dependent shortest-path search, but its finite spatial subdivision and
time buckets still approximate a continuous sailing problem. Refinement only
improves the corridor around its incumbent and deliberately retains that
incumbent if a finer attempt disconnects or regresses.

### Route segment eligibility contract

`RoutingOptions::segment_eligibility` can reject a parent-to-candidate segment
before it enters retained routing state:

```cpp
request.options.segment_eligibility =
    [](const sailroute::RouteSegmentView& segment) {
        return !crosses_land(
            segment.parent.position,
            segment.candidate.position);
    };
```

The callback is optional and defaults to accepting every segment. It receives
the complete candidate `RoutePoint`, including an adjusted endpoint, timestamp,
and cumulative distance when the segment is shortened upon entering the arrival
radius. Returning `false` prevents that candidate from being selected as an
arrival, retained in a frontier, reported through progress, or emitted in the
final route.

Candidate expansion can remain parallel, but eligibility callbacks are invoked
synchronously on the thread that called `Router::optimize` or
`Router::optimize_view`. Within one optimization, each generated candidate is
presented exactly once in deterministic parent-and-heading expansion order
before arrival selection and frontier pruning. The callback is therefore never
concurrent within one optimization, though separate concurrent optimizations
can invoke shared callback state concurrently. Callers sharing mutable state
between requests must synchronize it and must return the same decision for the
same segment and configuration to preserve deterministic results.

This contract is unchanged by the accuracy options. Augmented headings are
appended in a fixed order after the heading grid, maneuver penalties and
midpoint sampling are pure functions of the candidate and the forecast, and both
pruning strategies sort their retained set by expansion ordinal. Every
configuration therefore produces identical results for any `worker_count`.

Built-in landmask and exclusion checks compose in front of this callback rather
than replacing it. They run first, so the callback is only ever shown segments
that already cleared them, and it retains full veto power over the rest. Its
thread, ordering, lifetime, cancellation, and exception behavior are unchanged
by any provider configuration.

The `RouteSegmentView` and its references are valid only until the callback
returns. Eligibility runs serially for every generated candidate, so predicates
should return promptly. Exceptions propagate out of the optimization unchanged,
matching progress callback behavior. If a step generates candidates but rejects
all of them, optimization returns `ErrorCode::no_route`; rejecting a shortened
arrival alone does not stop the search when eligible frontier candidates remain.

### Progress callback contract

The optional callback receives one `RoutingProgress` snapshot after each
completed search step that produces a retained frontier. Each snapshot contains:

- `isochrone`: the retained frontier for that step;
- `provisional_route`: the route from the departure to the first retained node
  with the shortest great-circle distance to the destination; and
- `diagnostics`: cumulative work through that step.

Callbacks are synchronous, ordered by increasing isochrone time, and invoked on
the thread that called `Router::optimize`, never on the candidate-expansion
worker threads. The snapshot reference is valid only during the callback.
Applications that update another thread, including a UI thread, must copy the
snapshot into their own event queue and return promptly.

A callback can return `RoutingProgressDecision::continue_routing` or
`RoutingProgressDecision::cancel`. Cancellation takes effect after the reported
frontier is complete and before the next search step begins, so it does not
interrupt candidate expansion already in progress. `optimize` then returns an
error with `ErrorCode::cancelled`; the last provisional route and cumulative
diagnostics remain available in the callback snapshot. Existing `void` callbacks
and explicitly typed `RoutingProgressCallback` values remain supported as
notification-only callbacks and always continue routing. Exceptions thrown by
either callback form propagate out of `optimize`.

Progress delivery does not require `capture_isochrones`; that option controls
only whether isochrones are retained in the final `RouteResult`. Validation
failures and requests already within the arrival radius produce no progress
updates. The callback reports intermediate frontiers only: the consuming
application must still inspect the `Result<RouteResult>` returned by `optimize`
for the final complete or forecast-exhausted route, or for a routing error.
Partial-route ownership does not depend on installing a callback, callback
cadence, or callback payload selection.

For allocation-sensitive consumers, `Router::optimize_view` exposes the same
notification and cancellation forms with callback-scoped spans:

```cpp
request.options.progress.every_n_steps = 2;
request.options.progress.payload =
    sailroute::RoutingProgressPayload::retained_points |
    sailroute::RoutingProgressPayload::display_contours;

auto result = router.optimize_view(
    request,
    [](const sailroute::RoutingProgressView& progress) {
        render_frontier(
            progress.display_contours.points,
            progress.display_contours.segments);
        return sailroute::RoutingProgressDecision::continue_routing;
    });
```

The library reuses backing buffers between view callbacks. Every span in a
`RoutingProgressView` is valid only until that synchronous callback returns;
copy required data before retaining it or sending it to another thread.
Callbacks remain ordered, run on the thread that called `optimize_view`, and
default to every retained step. `every_n_steps` throttles callback delivery
only: `capture_isochrones` still records every frontier. Payload flags select
raw retained points, the provisional route, and display contours; unrequested
payloads have empty spans and are not constructed.

`build_display_contours` is also available independently. It projects points
around a circular-mean meridian, constructs a deterministic Delaunay
alpha-shape boundary, preserves disconnected components and open degenerate
chains, and splits antimeridian crossings instead of drawing wraparound
chords. Callers can supply `alpha_nautical_miles`; otherwise a deterministic
scale is derived from the frontier. A `DisplayContourSegment` references a
range in the flattened point array and marks whether that range closes back to
its first point.

### Serialization and time utilities

Serialization functions return an in-memory string and report invalid numeric
data as `ErrorCode::output_error`; callers are responsible for writing files.

| Function | Output |
| --- | --- |
| `route_to_json` | Route metadata, diagnostics, and points as JSON |
| `route_to_gpx` | GPX 1.1 track with route values in `sailroute` extensions |
| `isochrones_to_json` | GeoJSON `FeatureCollection` of display contours |
| `isochrones_to_gpx` | GPX 1.1 track per isochrone and segment per component |

`route_to_json` and `route_to_gpx` add an environment block, environment
diagnostics, and per-point audit values only when a Stage 3 environment was
configured; with none, their output is byte-for-byte what it was before those
providers existed.

Isochrone serializers operate on `RouteResult::isochrones`; set
`capture_isochrones` before routing or the output collection is empty.
`parse_utc_time()` accepts exactly `YYYY-MM-DDTHH:MM:SSZ`, and
`format_utc_time()` emits the same UTC form at whole-second precision.
Coordinates use latitude/longitude degrees in canonical ranges `[-90, 90]` and
`[-180, 180]`; `is_valid()` checks both bounds and finiteness.

## Polar formats

CSV matrix files place true-wind speeds in knots across the first row and
true-wind angles in degrees down the first column:

```text
TWA/TWS,6,10,14,20
0,0,0,0,0
45,4.8,6.7,7.8,8.5
90,5.5,7.4,8.8,9.8
135,5.2,7.1,8.5,9.5
180,4.5,6.4,7.8,8.8
```

Expedition-style files may use comma, semicolon, tab, or whitespace delimiters.
Full-line comments may begin with `#`, `!`, `%`, `;`, or `//`; inline comments
may begin with `#`, `!`, or `//`. Axes must be strictly increasing, contain at
least two values, and all boat speeds must be finite and non-negative. TWA must
be in `[0, 180]`; TWS and boat speeds are in knots.

## Roadmap

The options above are what can be improved inside a forward isochrone beam. The
stages below are not implemented; they are recorded so the design direction is
explicit.

### Time-dependent shortest-path solver

Replace the beam with a label-correcting time-dependent A\*/Dijkstra search over
a space-time lattice. Minimum-time navigation through a flow field is Zermelo's
problem; travel times are non-negative and the wind evolves forward, so
label-setting is valid and optimal on the lattice, and earlier legs are
re-relaxed automatically. This is the only stage that genuinely delivers
rerouting, and it removes the limitation described under
[Known limitations](#known-limitations).

- A geodesic lattice (icosahedral, HEALPix, or Fibonacci) rather than lat/lon,
  which degenerates at high latitude, exactly where ocean races are decided.
- Node state carrying position, time, board, and sail and reef configuration.
- An admissible A\* heuristic: remaining great-circle distance divided by the
  maximum speed the polar can achieve.
- Coarse-to-fine refinement around the incumbent corridor.

### Physics

Implemented and documented under
[Environmental physics](#environmental-physics): currents in the correct water
and ground frames, replaceable sea-state derating, a signed-distance landmask
with conservative segment certification, and time-varying exclusion polygons.
What remains is listed under [Deferred work](#deferred-work).

### Ensemble and risk-adjusted routing

Route across ensemble members rather than a single deterministic GRIB, and
optimize the mean, a quantile, or the probability of beating a rival. The true
optimum is a policy rather than a track, because the route is recomputed every
forecast cycle. The primary deliverable is decision points: where members
diverge, when commitment is required, and what being wrong costs.

### Continuous optimal-control polish

Remove lattice discretization error with direct collocation or multiple
shooting, or apply the Zermelo steering law from Pontryagin's Minimum Principle.
Run on a rolling horizon, warm-started from the previous solution each forecast
cycle.

## Deferred work

The physics stage deliberately stops at programmatic providers. The following
are explicit follow-ups rather than omissions:

- **Format adapters.** Current and wave GRIB decoding, landmask raster loading,
  and exclusion-record parsing. The sampling interfaces are separate from
  loading precisely so adapters can be added without changing the physics.
- **Remote data acquisition** for any environmental field.
- **CLI configuration.** The CLI takes no environment options, because there are
  no file formats to point them at yet. CLI routes remain the no-provider
  compatibility path, while serialization already emits the audit output for
  programmatically configured routes.
- **Time-varying currents and waves.** The provider interfaces already take a
  time, and the in-memory fields ignore it; time-dependent fields are an
  implementation detail of a future adapter.
- **Downstream bridge adoption.** Native and managed integration of the Stage 3
  audit fields.

## Portability

The implementation uses standard C++ and target-based CMake without Apple-only
APIs. macOS is the validated MVP platform; Linux and Windows require a C++20
toolchain plus an ecCodes installation discoverable by CMake or `pkg-config`.
