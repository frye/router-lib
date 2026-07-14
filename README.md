# sailroute

`sailroute` is a C++20 library and command-line tool for fastest-arrival sailing
route optimization using downloaded GRIB weather forecasts and vessel polars.
It uses ECMWF ecCodes for GRIB1/GRIB2 decoding and an isochrone search for
time-dependent routing.

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
  --json route.json \
  --gpx route.gpx
```

Omit `--polar` to use the approximate built-in 45-foot racer-cruiser polar.
Omit `--json` to write JSON to stdout. Run `sailroute --help` for routing
resolution controls.

The `samples/` directory contains an approximate First 44-class polar and
offshore coordinates for a Race Rocks to Port Angeles demonstration:

```sh
./build/sailroute \
  --grib forecast.grib2 \
  --start 48.294300,-123.531697 \
  --destination 48.141100,-123.402687 \
  --polar samples/sample.pol \
  --json route.json \
  --gpx route.gpx
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
auto result = router.optimize(request);
```

Loaded weather and polar objects are immutable and reusable across route
requests, avoiding repeated GRIB decoding and polar preprocessing.

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
