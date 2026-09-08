# router-lib

A C++20 library for earliest-arrival sailboat routing from a boat polar and
downloaded regional GRIB forecasts. Includes the `sailroute` CLI, JSON/GPX
output, optional local coastline loading, and optional advanced ensemble
workflows.

The result is the **best route found under the supplied data, constraints and
search resolution**, not a globally optimal or navigation-certified passage.
No traffic, bathymetry, chart rendering, or automatic downloads are provided.
Land avoidance is disabled unless a land provider or local dataset is supplied;
results and the CLI report this explicitly. A requested dataset is never
silently replaced by open water.

## Build

Requires CMake 3.20+, a C++20 compiler, Threads, and ECMWF ecCodes. On macOS:

```sh
brew install cmake eccodes
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

| Option | Default | Purpose |
| --- | --- | --- |
| `SAILROUTE_BUILD_TESTS` | ON | Existing offline test runner and CLI tests |
| `SAILROUTE_BUILD_BENCHMARKS` | OFF | Sampling, routing and optional advanced benchmarks |
| `SAILROUTE_ENABLE_ENSEMBLE` | ON | Optional shared-plan ensemble/racing implementation |
| `SAILROUTE_ENABLE_LAND_DATA` | ON | Optional local native GSHHG adapter |
| `SAILROUTE_ENABLE_LTO` | OFF | Link-time optimization when available |
| `SAILROUTE_BUILD_LEGACY_CORPUS` | OFF | Archived compatibility output tool |

For a smaller wind/polar core, disable ensemble and land-data loading. The
programmatic environmental interfaces remain available:

```sh
cmake -S . -B build-core -DCMAKE_BUILD_TYPE=Release \
  -DSAILROUTE_ENABLE_ENSEMBLE=OFF -DSAILROUTE_ENABLE_LAND_DATA=OFF
```

Install with `cmake --install build --prefix /your/prefix`, then consume using
`find_package(sailroute 0.6 CONFIG REQUIRED)` and
`target_link_libraries(my_app PRIVATE sailroute::sailroute)`.

## Basic CLI

The bundled example is historical data, so its departure is explicit:

```sh
./build/sailroute \
  --grib samples/sample.grib --polar samples/sample.pol \
  --start 48.294300,-123.531697 --destination 48.141100,-123.402687 \
  --departure 2026-07-14T20:19:01Z \
  --quality balanced --boat-speed-factor 0.9 \
  --json passage.json --gpx passage.gpx
```

Use `--demo-polar` instead of `--polar` only for demonstrations. The CLI never
silently chooses that approximate boat model. An omitted departure means current
UTC and must be covered by the forecast; it no longer rewinds to forecast start.

| Ordinary control | Meaning |
| --- | --- |
| `--quality fast\|balanced\|high` | Search breadth preset; explicit numerical overrides apply after the preset regardless of argument order |
| `--boat-speed-factor N` | Positive multiplier of polar boat speed, not wind speed |
| `--integration-minutes N` | Maximum physical substep, default 15 minutes |
| `--arrival-radius-nm N` | Arrival region, default 0.1 nm; not a coastal safety margin |
| `--maximum-wind-speed-knots N` | Ground-relative forecast wind limit; unset means no limit |
| `--maximum-forecast-gap-minutes N` | Reject wider valid-time gaps; omit for a product with unrestricted changing cadence |
| `--bounds S,W,N,E` | Forecast crop; west greater than east crosses the antimeridian |
| `--maximum-candidates N`, `--maximum-nodes N` | Explicit search-work limits |

`fast` uses fewer headings and retained candidates, `balanced` uses 10-degree
headings and ten candidates per bucket, and `high` uses five-degree headings,
smaller buckets, twenty retained candidates and a fifteen-minute search step.
All preserve the same validity/physics rules. More search breadth is not a
mathematical guarantee of a better route: evaluate final arrivals, not just
candidate counts.

Run `sailroute --help` for advanced heading, maneuver, polar, pruning, lattice
and output controls. The CLI returns 0 for arrived or explicitly partial output,
2 for usage errors, 3 for input errors, 4 for routing errors, and 5 for output
errors. Always inspect `completion`; exit 0 alone does not establish arrival.

## How future positioning affects the result

The default beam keeps **many possible positions and their parent histories** at
each search time. It expands every retained alternative using weather at the
future position and time. It does not commit to the nearest-to-destination path
shown in a progress update.

A route can initially sail farther from the destination to reach a better wind
band and finish earlier. Retention combines a bounded future-weather ranking
signal with spatial, heading and board diversity. The probe is only a heuristic
at candidate positions, not a prediction of an optimized continuation and not
an admissible lower bound. Missing probes are counted separately and never
substitute for actual transition weather.

The final winner is the earliest eligible arrival among the surviving paths.
For the synchronous beam, all arrivals in the first arriving time layer are
compared using their actual timestamps. Its parent pointers then reconstruct
the complete route. Reconstruction cannot recover a branch already pruned.
`prunedCandidates` and `futureProbeMisses` help expose that approximation.

The optional `--solver lattice` uses a geodesic neighbor graph plus VMG moves.
It retains exact whole-second arrival identities, supports explicit holding,
and provides coarse-to-fine refinement that keeps a complete incumbent when a
refinement regresses. It uses Dijkstra ordering when current providers are
configured rather than an invalid polar-only SOG bound. Its spatial buckets and
finite action set still approximate the problem; it is not automatically more
accurate than the beam.

Both solvers use the same arrival-region convention. The requested destination,
actual endpoint and remaining distance are separate in output; no unsupported
connector is drawn to make a partial route appear complete.

## Weather, boat and movement contracts

Supported weather is GRIB1/GRIB2 paired instantaneous earth-relative 10 m U/V
wind on regular latitude/longitude grids. The loader handles supported scan
orders, units, adjacent tiles, cropping and the antimeridian. U/V components are
interpolated in space and time. Mixed initialization times, incompatible grids,
participating missing stencil values and unsupported field semantics are
errors. Forecast metadata includes coverage and minimum/maximum time spacing.

Polars support both TWA-row/TWS-column matrices and native Expedition rows:

```text
TWA/TWS,6,10,16
0,0,0,0
45,4.0,5.5,6.5
90,4.5,6.0,7.0
180,3.5,5.0,6.0
```

```text
6 45 4.0 90 4.5 180 3.5
10 45 5.5 90 6.0 180 5.0
16 45 6.5 90 7.0 180 6.0
```

TWS and boat speed are in knots; TWA is in degrees. Native rows may contain
different angle samples. Use `--polar-format matrix` or `expedition` if numeric
input is ambiguous. `--minimum-sailing-angle` can further restrict input support.

Sailing feasibility is distinct from raw table interpolation. A table starting
at 45 degrees does not authorize sailing at zero degrees by clamping its first
speed. Zero-speed points and unsupported angle intervals are not interpolated
into valid sailing sectors. `PolarSlice::supports_sailing_angle()` exposes this
contract; `speed_knots()` remains a raw lookup. Optional monotone-cubic angle
interpolation and cached VMG proposals are retained.

Search intervals and integration intervals are separate. Long beam actions and
lattice edges are split into bounded physical segments, respecting forecast time
boundaries. Their intermediate vertices are retained in route output. Missing
or forbidden midpoint/endpoint conditions reject the transition rather than
restoring a previous speed.

Current is a knots-valued east/north vector pointing **toward** its set. The
polar uses air velocity relative to water: ground-relative wind minus current.
Ground movement is through-water boat velocity plus current. `headingDegrees`
and `boatSpeedKnots` are water-relative; the environmental audit reports SOG,
COG, current and effective polar wind separately from forecast wind.

Tack/gybe costs are explicit maneuver phases, including current drift at zero
through-water speed, followed by sailing. They are charged once rather than
again at every integration substep. Arbitrary stationary waiting is disabled.
Applications can explicitly authorize holding through
`RoutingOptions::holding_eligibility`; authorized waits still obey geometry,
forecast/environment availability, wind limits and segment eligibility.

## Local land data and optional environment

With `SAILROUTE_ENABLE_LAND_DATA`, add a complete local native GSHHG shoreline
file. The adapter uses the effective forecast region; crop a global or oversized
forecast first:

```sh
./build/sailroute \
  --grib forecast.grib2 --polar boat.pol \
  --start 48.294300,-123.531697 --destination 48.141100,-123.402687 \
  --bounds 47.8,-124,48.6,-123 \
  --land /local/data/gshhs_h.b --land-resolution-nm 0.25 \
  --land-clearance-nm 0.5 --json passage.json
```

The adapter supports bounded regional conversions, hierarchy levels 1-4, and
dateline crossings. Unsupported polar/global geometry, incomplete hierarchies,
malformed data and exhausted budgets produce errors. The header
`<sailroute/land_data.hpp>` documents supported extents and budgets.

Node distances are computed from shoreline geometry, not only node land/water
labels. Conservative interpolation-error allowance and whole-segment
certification prevent treating a positive interpolated value as sufficient proof
of clearance. A complete source dataset or complete hierarchy-preserving regional
extraction is required: native GSHHG files cannot prove that a whole polygon was
not omitted from the source.

**GSHHG is not a current nautical chart.** Its source inaccuracies, missing
hazards, depths and licensing obligations remain separate from numerical mask
error. No adapter can certify hazards absent from its input.

Programmatic `RoutingEnvironment` providers remain available independently of
the file adapter: currents, wave fields with a replaceable derating model,
signed-distance landmasks and timed spherical exclusion polygons. Providers must
be immutable and safe for concurrent sampling. Missing data either fails the
route or rejects the transition according to an explicit policy; it never
becomes zero current, calm sea or open water.

The built-in wave-height derating model is illustrative, not validated
seakeeping data. Current/wave GRIB loaders, bathymetry and downloads are not
included.

## C++ API and outputs

```cpp
auto weather = sailroute::WeatherDataset::load(
    "forecast.grib2",
    sailroute::WeatherLoadOptions{
        .maximum_interpolation_gap = std::chrono::hours{6}});
auto polar = sailroute::VesselPolar::load("boat.pol");
if (!weather || !polar) {
    // Report the failing Result's error rather than routing with substitute data.
    return 1;
}
sailroute::Router router{weather.value(), polar.value()};
sailroute::RouteRequest request{
    .start = {48.294300, -123.531697},
    .destination = {48.141100, -123.402687},
};
request.options = sailroute::routing_options_for_quality(
    sailroute::RoutingQuality::balanced);
auto route = router.optimize(request);
```

The convenience one-argument `Router` still uses a demonstration polar; production
callers should supply their actual boat model. Loaded forecasts and polars are
immutable and reusable. Expected failures use `Result<T>` with an `ErrorCode`
and message. Callback exceptions propagate.

`Router::evaluate_route(request, waypoints)` replays a fixed geometric waypoint
path with the same physics and no alternative-course search. Waypoints exclude
departure. It can expose unrealistic ETA differences between resolutions; it
does not silently re-optimize an infeasible leg.

`Router::evaluate_actions(request, actions)` instead preserves a sequence of
`SailingAction{heading_degrees, duration}` controls, terminating early on arrival.
Use this to compare timed heading policies: replaying their coordinates as
fixed targets can legitimately require different steering and is not equivalent.

`optimize` and `optimize_view` expose owning and callback-scoped progress plus
cancellation. Intermediate paths are provisional. Eligibility is invoked in
deterministic order on the caller thread for beam physical segments, including
intermediate/maneuver vertices. It is not one invocation per coarse search node.
Separate simultaneous calls must synchronize any shared application state.

Raw isochrones are optional. Display contours and destination fronts are opt-in
presentation helpers, not additional routing algorithms.

Deterministic JSON uses `route_result_v2`, with exact `completion`, a partial
reason when applicable, requested/actual arrival data, effective routing settings,
forecast validity, warning strings and work counters. GPX carries corresponding
extensions. Resource exhaustion and cancellation are explicit errors, not claims
of a completed optimal search.

## Advanced ensemble and racing workflows

Include `<sailroute/ensemble.hpp>` and `<sailroute/ensemble_serialization.hpp>`
explicitly. The core umbrella and ordinary serialization headers no longer
include these large optional APIs.

Shared-action ensemble search retains weighted mean/P75/P90 arrival, deadline
and rival objectives with member-local audits. These are optional racing/risk
capabilities, not prerequisites for ordinary cruising. Alternative generation
is disabled by default; compact output does not require a policy DAG.

```sh
./build/sailroute \
  --ensemble-member control:0.6:control.grib2 \
  --ensemble-member alternate:0.4:alternate.grib2 \
  --polar boat.pol --start 37,-123 --destination 35,-125 \
  --ensemble-objective weighted_p90_elapsed_arrival --json ensemble.json
```

Retained alternatives and decision diagnostics are **not executable adaptive
policies or calculated last-safe-turn boundaries**. To use a new forecast during
a passage, the application must replan from the observed position/time. It must
not pretend an unexecuted old branch is the vessel's actual history.

The experimental ensemble beam stays explicitly gated for existing users.
Further solver proliferation, continuous optimal control, automatic polar
learning and a full feedback-policy system are deferred.

## Benchmarks and migration

Enable the existing benchmark target with `SAILROUTE_BUILD_BENCHMARKS=ON`.
Ordinary runs no longer implicitly execute the historical topology experiment or
the full ensemble matrix:

```sh
./build/sailroute_benchmarks samples/sample.grib
./build/sailroute_benchmarks --topology
./build/sailroute_benchmarks --strategic
./build/sailroute_benchmarks samples/sample.grib --ensemble
```

Compare final arrival under matched physics/arrival constraints, including cases
where the winning route loses progress over several early steps. Candidate count,
a shorter reported ETA under coarser integration, or A*/Dijkstra agreement alone
does not establish better continuous routing.

See [migration notes](docs/migration-v0.6.md) for deliberate default, callback and
schema changes. The v0.3.2 golden corpus is preserved as historical evidence,
not overwritten to approve v0.6 behavior.

The controlled strategic corpus includes mirrored wind-band and island-side
choices whose favorable weather appears only after five hours. Its exhaustive
discrete reference is independent of beam pruning. Passing these cases supports
the retained beam baseline for this model; it does not establish global optimality
or empirical parity with proprietary routing products.
