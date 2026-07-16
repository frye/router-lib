# router-lib

`router-lib` is a C++20 library for fastest-arrival sailing route optimization
using downloaded GRIB weather forecasts and vessel polars. It includes the
`sailroute` command-line tool, uses ECMWF ecCodes for GRIB1/GRIB2 decoding, and
performs an isochrone search for time-dependent routing.

> [!WARNING]
> The MVP does not model land, shorelines, currents, waves, traffic, restricted
> areas, or safety limits. Routes may cross land. The built-in polar is an
> approximate demonstration model, not navigation-certified data.

## Supported MVP data

- GRIB1 or GRIB2 regular latitude/longitude grids
- Paired 10-metre U/V wind fields at one or more valid forecast times
- CSV matrix and common Expedition-style `.pol` vessel polars
- UTC departure timestamps formatted as `YYYY-MM-DDTHH:MM:SSZ`

An explicit departure outside forecast coverage is rejected. If departure is
omitted, the library uses current UTC time when it can be interpolated from the
forecast, otherwise it uses the forecast's first valid time.

## macOS build

Install CMake and ecCodes, then configure and build:

```sh
brew install cmake eccodes
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Enable the microbenchmarks with
`-DSAILROUTE_BUILD_BENCHMARKS=ON`. Enable link-time optimization with
`-DSAILROUTE_ENABLE_LTO=ON`.

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
per contour component.

The router retains up to 10 nodes per spatial bucket by default. Increase
`--max-nodes-per-bucket` to preserve a larger set of alternate paths, or reduce
it when runtime and memory are more important than search breadth.

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
auto weather = sailroute::WeatherDataset::load("forecast.grib2");
auto polar = sailroute::VesselPolar::load("boat.pol");
sailroute::Router router{weather.value(), polar.value()};

sailroute::RouteRequest request{
    .start = {37.7749, -122.4194},
    .destination = {21.3069, -157.8583},
};
request.options.routing_intervals = {
    {std::chrono::minutes{15}, std::chrono::hours{6}},
    {std::chrono::minutes{30}, std::chrono::hours{24}},
    {std::chrono::hours{2}, std::nullopt},
};
request.options.capture_isochrones = true;
std::vector<sailroute::RoutingProgress> updates;
auto result = router.optimize(
    request,
    [&updates](const sailroute::RoutingProgress& progress) {
        updates.push_back(progress);
        return sailroute::RoutingProgressDecision::continue_routing;
    });
auto isochrones_json = sailroute::isochrones_to_json(result.value());
auto isochrones_gpx = sailroute::isochrones_to_gpx(result.value());
```

Loaded weather and polar objects are immutable and reusable across route
requests, avoiding repeated GRIB decoding and polar preprocessing. Isochrone
capture is disabled by default so route-only callers do not retain the full
search frontier history.

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
for the final route or routing error.

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

Expedition-style files may use comma, semicolon, tab, or whitespace delimiters
and comment lines beginning with `#`, `;`, or `//`. Axes must be strictly
increasing and all boat speeds must be finite and non-negative.

## Portability

The implementation uses standard C++ and target-based CMake without Apple-only
APIs. macOS is the validated MVP platform; Linux and Windows require a C++20
toolchain plus an ecCodes installation discoverable by CMake or `pkg-config`.
