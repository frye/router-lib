#include "sailroute/environment.hpp"
#include "sailroute/router.hpp"
#include "sailroute/time.hpp"

#include "lattice_comparison.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>
#if defined(_WIN32)
#include <windows.h>

#include <psapi.h>
#else
#include <sys/resource.h>
#endif
#ifndef SAILROUTE_BASELINE_REVISION
#define SAILROUTE_BASELINE_REVISION "unknown"
#endif
#ifndef SAILROUTE_BUILD_TYPE
#define SAILROUTE_BUILD_TYPE "unknown"
#endif
#ifndef SAILROUTE_COMPILER
#define SAILROUTE_COMPILER "unknown"
#endif
#ifndef SAILROUTE_SOURCE_REVISION
#define SAILROUTE_SOURCE_REVISION "unknown"
#endif
#ifndef SAILROUTE_SYSTEM
#define SAILROUTE_SYSTEM "unknown"
#endif

namespace {

enum class ProgressMode {
    none,
    owning,
    view,
    view_with_contours,
};

void benchmark_routing(
    const sailroute::Router& router,
    sailroute::RouteRequest request,
    std::size_t worker_count,
    ProgressMode progress_mode,
    std::string_view label) {
    constexpr std::size_t iterations = 10U;
    request.options.worker_count = worker_count;
    if (progress_mode == ProgressMode::view_with_contours) {
        request.options.progress.payload =
            sailroute::RoutingProgressPayload::display_contours;
    }
    volatile std::size_t checksum = 0U;
    std::size_t delivered_updates = 0U;
    std::vector<double> route_milliseconds;
    route_milliseconds.reserve(iterations);

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t index = 0U; index < iterations; ++index) {
        const auto route_start = std::chrono::steady_clock::now();
        auto route = [&]() -> sailroute::Result<sailroute::RouteResult> {
            switch (progress_mode) {
                case ProgressMode::none:
                    return router.optimize(request);
                case ProgressMode::owning:
                    return router.optimize(
                        request,
                        [&checksum, &delivered_updates](
                            const sailroute::RoutingProgress& progress) {
                            ++delivered_updates;
                            checksum =
                                checksum +
                                progress.isochrone.points.size() +
                                progress.provisional_route.size();
                        });
                case ProgressMode::view:
                case ProgressMode::view_with_contours:
                    return router.optimize_view(
                        request,
                        [&checksum, &delivered_updates](
                            const sailroute::RoutingProgressView& progress) {
                            ++delivered_updates;
                            checksum =
                                checksum +
                                progress.retained_points.size() +
                                progress.provisional_route.size() +
                                progress.display_contours.points.size();
                        });
            }
            throw std::runtime_error("unknown progress benchmark mode");
        }();
        if (!route.has_value()) {
            throw std::runtime_error(route.error().message);
        }
        checksum = checksum + route.value().diagnostics.generated_candidates +
                   route.value().points.size();
        route_milliseconds.push_back(std::chrono::duration<double, std::milli>(
                                         std::chrono::steady_clock::now() -
                                         route_start)
                                         .count());
    }
    const double seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - start)
                               .count();
    std::ranges::sort(route_milliseconds);
    const double minimum = route_milliseconds.front();
    const double median = route_milliseconds[route_milliseconds.size() / 2U];
    const double percentile_95 =
        route_milliseconds[(route_milliseconds.size() * 95U + 99U) / 100U - 1U];
    const double maximum = route_milliseconds.back();
    std::cout << label << ": " << iterations / seconds << " routes/s ("
              << seconds * 1000.0 / static_cast<double>(iterations)
              << " ms/route; min/p50/p95/max " << minimum << '/' << median << '/'
              << percentile_95 << '/' << maximum << " ms), updates: "
              << delivered_updates
              << ", checksum: " << checksum << '\n';
}

// The forecast footprint is not published in the metadata, so bisect on
// interpolate to find it. This lets the benchmark route the longest leg any
// supplied GRIB actually supports instead of hard-coding a domain.
double coverage_edge(
    const sailroute::WeatherDataset& weather,
    sailroute::TimePoint when,
    sailroute::Coordinate inside,
    sailroute::Coordinate outside) {
    const bool vary_latitude =
        inside.latitude_degrees != outside.latitude_degrees;
    if (!vary_latitude &&
        weather.interpolate(outside, when).has_value()) {
        // The opposite meridian is still covered, so there is no edge to
        // bracket in this direction. Use a quarter-turn from the seed on each
        // side; together they form the longest unique great-circle longitude
        // separation instead of a 360-degree span that normalizes to a short leg.
        return (inside.longitude_degrees + outside.longitude_degrees) * 0.5;
    }
    for (int iteration = 0; iteration < 40; ++iteration) {
        sailroute::Coordinate midpoint{
            (inside.latitude_degrees + outside.latitude_degrees) * 0.5,
            (inside.longitude_degrees + outside.longitude_degrees) * 0.5};
        if (weather.interpolate(midpoint, when).has_value()) {
            inside = midpoint;
        } else {
            outside = midpoint;
        }
    }
    return vary_latitude ? inside.latitude_degrees : inside.longitude_degrees;
}

double normalize_longitude(double longitude_degrees) noexcept {
    double normalized = std::fmod(longitude_degrees + 180.0, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized - 180.0;
}

// Routes once and reports arrival and search effort, so accuracy options can be
// judged on route quality rather than asserted.
void report_route_quality(
    const sailroute::Router& router,
    sailroute::RouteRequest request,
    std::string_view label) {
    const auto start = std::chrono::steady_clock::now();
    auto route = router.optimize(request);
    const double milliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    std::cout << "  " << std::left << std::setw(30) << label << std::right;
    if (!route.has_value()) {
        std::cout << "  no route (" << route.error().message << ")\n";
        return;
    }
    const double hours = std::chrono::duration<double, std::ratio<3600>>(
                             route.value().arrival_time -
                             route.value().departure_time)
                             .count();
    std::cout << "  " << std::fixed << std::setprecision(4) << std::setw(9)
              << hours << " h"
              << "  legs " << std::setw(4) << route.value().points.size()
              << "  candidates " << std::setw(9)
              << route.value().diagnostics.generated_candidates
              << "  " << std::setprecision(1) << std::setw(8) << milliseconds
              << " ms";
    if (route.value().lattice_diagnostics.has_value()) {
        std::cout << "  settled "
                 << route.value().lattice_diagnostics->settled_labels
                 << "  level "
                 << route.value().lattice_diagnostics->subdivision_level
                 << "  active cells/faces "
                 << route.value().lattice_diagnostics->active_cells << '/'
                 << route.value().lattice_diagnostics->active_faces
                 << "  refinement accepted/runs "
                 << route.value().lattice_diagnostics->accepted_refinements
                 << '/'
                 << route.value().lattice_diagnostics->refinement_runs
                 << "  corridor "
                 << route.value()
                        .lattice_diagnostics
                        ->accepted_corridor_width_nautical_miles
                 << " nm";
    }
    std::cout << '\n' << std::defaultfloat;
}

// Stage 3 environment overhead. Providers are built to cover the whole leg and
// to leave the route legal, so the reported cost is the cost of the physics
// rather than the cost of failing every candidate.
struct EnvironmentFootprint {
    double south_latitude_degrees{};
    double north_latitude_degrees{};
    double west_longitude_degrees{};
    double east_longitude_degrees{};
};

sailroute::EnvironmentGridSpec footprint_grid(
    const EnvironmentFootprint& footprint,
    std::size_t samples) {
    sailroute::EnvironmentGridSpec spec;
    spec.south_latitude_degrees = std::max(-89.0, footprint.south_latitude_degrees - 2.0);
    spec.west_longitude_degrees = footprint.west_longitude_degrees - 2.0;
    const double north = std::min(89.0, footprint.north_latitude_degrees + 2.0);
    const double east = footprint.east_longitude_degrees + 2.0;
    spec.latitude_count = samples;
    spec.longitude_count = samples;
    spec.latitude_step_degrees =
        std::max(1.0e-3, (north - spec.south_latitude_degrees) /
                             static_cast<double>(samples - 1U));
    spec.longitude_step_degrees =
        std::max(1.0e-3, (east - spec.west_longitude_degrees) /
                             static_cast<double>(samples - 1U));
    return spec;
}

sailroute::SignedDistanceLandmask open_water_landmask(
    const EnvironmentFootprint& footprint,
    std::size_t samples) {
    const sailroute::EnvironmentGridSpec spec = footprint_grid(footprint, samples);
    sailroute::LandmaskMetadata metadata;
    metadata.provider = sailroute::ProviderMetadata{
        "benchmark_open_water", "synthetic benchmark mask", "1"};
    metadata.resolution_nautical_miles =
        spec.latitude_step_degrees * 60.0;
    metadata.interpolation_error_nautical_miles = 0.0;
    auto mask = sailroute::SignedDistanceLandmask::create(
        spec,
        std::vector<double>(spec.latitude_count * spec.longitude_count, 5'000.0),
        metadata);
    if (!mask.has_value()) {
        throw std::runtime_error(mask.error().message);
    }
    return std::move(mask.value());
}

sailroute::ExclusionZoneSet benchmark_zones(
    const EnvironmentFootprint& footprint,
    std::size_t count) {
    std::vector<sailroute::ExclusionZone> zones;
    zones.reserve(count);
    const double latitude = footprint.south_latitude_degrees - 5.0;
    for (std::size_t index = 0U; index < count; ++index) {
        const double west = footprint.west_longitude_degrees +
            static_cast<double>(index) * 0.5;
        sailroute::ExclusionZone zone;
        zone.identifier = "benchmark-" + std::to_string(index);
        zone.source = "synthetic benchmark zone";
        zone.revision = 1U;
        zone.polygons.push_back(sailroute::ExclusionPolygon{
            sailroute::ExclusionRing{{
                sailroute::Coordinate{latitude - 0.5, west},
                sailroute::Coordinate{latitude - 0.5, west + 0.4},
                sailroute::Coordinate{latitude, west + 0.4},
                sailroute::Coordinate{latitude, west},
            }},
            {}});
        zones.push_back(std::move(zone));
    }
    auto set = sailroute::ExclusionZoneSet::create(
        std::move(zones),
        sailroute::ProviderMetadata{
            "benchmark_zones", "synthetic benchmark zones", "1"});
    if (!set.has_value()) {
        throw std::runtime_error(set.error().message);
    }
    return std::move(set.value());
}

// Process high-water mark. Providers are configured immediately before the case
// that uses them, so the growth since the previous reading covers both provider
// construction and the route that consumed it.
double peak_resident_mebibytes() {
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS counters{};
    if (GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters)) ==
        0) {
        return 0.0;
    }
    const double bytes = static_cast<double>(counters.PeakWorkingSetSize);
#else
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0.0;
    }
#if defined(__APPLE__)
    const double bytes = static_cast<double>(usage.ru_maxrss);
#else
    const double bytes = static_cast<double>(usage.ru_maxrss) * 1024.0;
#endif
#endif
    return bytes / (1024.0 * 1024.0);
}

void report_peak_resident(double& watermark_mebibytes) {
    const double peak = peak_resident_mebibytes();
    std::cout << "  peak " << std::fixed << std::setprecision(1) << std::setw(6)
              << peak << " MiB " << std::showpos << std::setprecision(2)
              << std::setw(6) << peak - watermark_mebibytes << std::noshowpos
              << std::defaultfloat;
    watermark_mebibytes = peak;
}

void report_environment(
    const sailroute::WeatherDataset& weather,
    const sailroute::VesselPolar& polar,
    const sailroute::RoutingEnvironment& environment,
    const sailroute::RouteRequest& request,
    std::string_view label,
    double baseline_milliseconds,
    double baseline_hours,
    double& peak_watermark_mebibytes) {
    const sailroute::Router router{weather, polar, environment};
    const auto start = std::chrono::steady_clock::now();
    auto route = router.optimize(request);
    const double milliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    std::cout << "  " << std::left << std::setw(28) << label << std::right;
    if (!route.has_value()) {
        std::cout << "  no route (" << route.error().message << ")\n";
        return;
    }
    const double hours = std::chrono::duration<double, std::ratio<3600>>(
                             route.value().arrival_time -
                             route.value().departure_time)
                             .count();
    std::cout << "  " << std::fixed << std::setprecision(4) << std::setw(9)
              << hours << " h";
    if (baseline_hours > 0.0) {
        std::cout << "  arrival delta " << std::showpos << std::setprecision(4)
                  << std::setw(9) << hours - baseline_hours << " h"
                  << std::noshowpos;
    } else {
        std::cout << "                          ";
    }
    std::cout << "  " << std::setprecision(1) << std::setw(8) << milliseconds
              << " ms";
    if (baseline_milliseconds > 0.0) {
        std::cout << "  overhead " << std::setprecision(1) << std::setw(7)
                  << (milliseconds / baseline_milliseconds - 1.0) * 100.0 << "%";
    } else {
        std::cout << "                  ";
    }
    if (route.value().environment_diagnostics.has_value()) {
        const sailroute::EnvironmentDiagnostics& counters =
            *route.value().environment_diagnostics;
        std::cout << "  current " << counters.current_samples << '/'
                  << counters.current_rejections << "  wave "
                  << counters.wave_samples << '/' << counters.sea_state_evaluations
                  << "  land " << counters.land_checks << '/'
                  << counters.land_distance_queries << '/'
                  << counters.land_rejections << "  zones "
                  << counters.exclusion_checks << '/'
                  << counters.exclusion_geometry_tests << '/'
                  << counters.exclusion_rejections;
    }
    report_peak_resident(peak_watermark_mebibytes);
    std::cout << '\n' << std::defaultfloat;
}

double measure_baseline(
    const sailroute::Router& router,
    const sailroute::RouteRequest& request,
    double& hours) {
    const auto start = std::chrono::steady_clock::now();
    auto route = router.optimize(request);
    const double milliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start)
            .count();
    hours = route.has_value()
        ? std::chrono::duration<double, std::ratio<3600>>(
              route.value().arrival_time - route.value().departure_time)
              .count()
        : 0.0;
    return milliseconds;
}

// Sea-state response across wave height and direction. Height 0.0 is the
// flat-water equivalence check: it must reproduce the no-provider arrival
// exactly, not merely closely.
void report_sea_state_matrix(
    const sailroute::WeatherDataset& weather,
    const sailroute::VesselPolar& polar,
    const sailroute::RouteRequest& request,
    double baseline_hours) {
    constexpr double heights[] = {0.0, 0.5, 1.0, 2.0, 4.0};
    constexpr double directions[] = {0.0, 90.0, 180.0, 270.0};

    std::cout << "\nsea-state parameter matrix (arrival hours, period 9.0 s)\n"
              << "  flat-water baseline " << std::fixed << std::setprecision(4)
              << baseline_hours << " h\n  " << std::left << std::setw(10)
              << "Hs (m)" << std::right;
    for (const double direction : directions) {
        std::cout << std::setw(11) << std::setprecision(0) << direction
                  << " deg";
    }
    std::cout << '\n';

    for (const double height : heights) {
        std::cout << "  " << std::left << std::setw(10) << std::setprecision(1)
                  << height << std::right;
        for (const double direction : directions) {
            auto waves = sailroute::make_uniform_wave_provider(
                sailroute::WaveState{height, 9.0, direction},
                sailroute::ProviderMetadata{
                    "benchmark_wave", "synthetic benchmark sea state", "1"});
            auto model = sailroute::make_wave_height_derating_model();
            if (!waves.has_value() || !model.has_value()) {
                std::cout << std::setw(15) << "error";
                continue;
            }
            sailroute::RoutingEnvironment environment;
            environment.waves.provider = waves.value();
            environment.waves.model = model.value();
            const sailroute::Router router{weather, polar, environment};
            auto route = router.optimize(request);
            if (!route.has_value()) {
                std::cout << std::setw(15) << "no route";
                continue;
            }
            const double hours =
                std::chrono::duration<double, std::ratio<3600>>(
                    route.value().arrival_time - route.value().departure_time)
                    .count();
            std::cout << std::setw(11) << std::setprecision(4) << hours;
            std::cout << (height == 0.0 && hours == baseline_hours ? "  ==" : "    ");
        }
        std::cout << '\n';
    }
    std::cout << std::defaultfloat;
}

}  // namespace

int main(int argc, char** argv) {
    std::cout << "benchmark metadata\n"
              << "  source revision: " << SAILROUTE_SOURCE_REVISION << '\n'
              << "  compatibility baseline: " << SAILROUTE_BASELINE_REVISION
              << '\n'
              << "  build: " << SAILROUTE_BUILD_TYPE << '\n'
              << "  compiler: " << SAILROUTE_COMPILER << '\n'
              << "  system: " << SAILROUTE_SYSTEM << '\n'
              << "  hardware concurrency: "
              << std::thread::hardware_concurrency() << "\n\n";

    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    constexpr std::size_t iterations = 5'000'000;
    volatile double checksum = 0.0;

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < iterations; ++index) {
        const double tws = 2.0 + static_cast<double>(index % 300) * 0.1;
        const double twa = static_cast<double>(index % 1800) * 0.1;
        checksum = checksum + polar.boat_speed_knots(tws, twa);
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    const double seconds = std::chrono::duration<double>(elapsed).count();

    std::cout << "polar lookups: " << iterations / seconds << "/s\n";
    std::cout << "checksum: " << checksum << '\n';

    sailroute::benchmarks::report_lattice_comparison();

    if (argc < 2) {
        std::cout << "routing benchmarks skipped; pass a GRIB forecast path\n";
        return 0;
    }

    auto weather = sailroute::WeatherDataset::load(std::filesystem::path{argv[1]});
    if (!weather.has_value()) {
        std::cerr << weather.error().message << '\n';
        return 1;
    }
    const auto departure = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    if (!departure.has_value()) {
        std::cerr << departure.error().message << '\n';
        return 1;
    }

    const sailroute::ForecastMetadata& metadata = weather.value().metadata();
    const sailroute::TimePoint probe_time = metadata.first_valid_time;

    // Find any covered point, then grow a leg across the whole footprint.
    // Sweeps coarse to fine so a global forecast is seeded almost immediately
    // while a small regional domain is still found.
    sailroute::Coordinate seed{0.0, 0.0};
    bool seeded = false;
    for (const double resolution : {1.0, 0.25, 0.05}) {
        const int latitude_steps = static_cast<int>(180.0 / resolution);
        const int longitude_steps = static_cast<int>(360.0 / resolution);
        for (int latitude = 0; latitude <= latitude_steps && !seeded; ++latitude) {
            for (int longitude = 0; longitude < longitude_steps && !seeded;
                 ++longitude) {
                const sailroute::Coordinate probe{
                    -90.0 + static_cast<double>(latitude) * resolution,
                    -180.0 + static_cast<double>(longitude) * resolution};
                if (weather.value().interpolate(probe, probe_time).has_value()) {
                    seed = probe;
                    seeded = true;
                }
            }
        }
        if (seeded) {
            break;
        }
    }
    if (!seeded) {
        std::cerr << "forecast covers no probed coordinate\n";
        return 1;
    }

    const double south = coverage_edge(
        weather.value(), probe_time, seed, {-90.0, seed.longitude_degrees});
    const double north = coverage_edge(
        weather.value(), probe_time, seed, {90.0, seed.longitude_degrees});
    const double west = coverage_edge(
        weather.value(),
        probe_time,
        seed,
        {seed.latitude_degrees, seed.longitude_degrees - 180.0});
    const double east = coverage_edge(
        weather.value(),
        probe_time,
        seed,
        {seed.latitude_degrees, seed.longitude_degrees + 180.0});

    // Inset slightly so rounding never places an endpoint outside the forecast.
    const double latitude_inset = (north - south) * 0.02;
    const double longitude_inset = (east - west) * 0.02;

    sailroute::RouteRequest request;
    request.start = {
        south + latitude_inset,
        normalize_longitude(west + longitude_inset)};
    request.destination = {
        north - latitude_inset,
        normalize_longitude(east - longitude_inset)};
    request.departure_time = departure.has_value() &&
            departure.value() >= metadata.first_valid_time &&
            departure.value() <= metadata.last_valid_time
        ? departure.value()
        : metadata.first_valid_time;
    request.options.time_step = std::chrono::minutes{30};
    request.options.use_routing_intervals = false;
    request.options.heading_step_degrees = 5.0;
    request.options.arrival_radius_nautical_miles = 0.5;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 4U;
    request.options.maximum_route_duration = std::chrono::hours{240};
    const sailroute::Router router{weather.value(), polar};

    std::cout << "forecast " << metadata.source << "\n  leg "
              << std::fixed << std::setprecision(4)
              << request.start.latitude_degrees << ','
              << request.start.longitude_degrees << " -> "
              << request.destination.latitude_degrees << ','
              << request.destination.longitude_degrees
              << " departing " << sailroute::format_utc_time(request.departure_time.value())
              << "\n" << std::defaultfloat;

    benchmark_routing(
        router,
        request,
        1U,
        ProgressMode::none,
        "routing single worker");
    benchmark_routing(
        router,
        request,
        4U,
        ProgressMode::none,
        "routing four workers");
    benchmark_routing(
        router,
        request,
        0U,
        ProgressMode::none,
        "routing automatic workers");
    benchmark_routing(
        router,
        request,
        1U,
        ProgressMode::owning,
        "routing owning progress");
    benchmark_routing(
        router,
        request,
        1U,
        ProgressMode::view,
        "routing progress view");
    benchmark_routing(
        router,
        request,
        1U,
        ProgressMode::view_with_contours,
        "routing progress view with contours");

    std::cout << "\nroute quality by accuracy option\n";
    report_route_quality(router, request, "baseline");
    {
        sailroute::RouteRequest variant = request;
        variant.options.solver =
            sailroute::RoutingSolver::time_dependent_lattice;
        variant.options.lattice.subdivision_level = 3U;
        variant.options.lattice.refinement_levels = 0U;
        report_route_quality(router, variant, "lattice coarse A*");
        variant.options.lattice.search_algorithm =
            sailroute::LatticeSearchAlgorithm::dijkstra;
        report_route_quality(router, variant, "lattice coarse Dijkstra");
        variant.options.lattice.search_algorithm =
            sailroute::LatticeSearchAlgorithm::a_star;
        variant.options.lattice.refinement_levels = 1U;
        report_route_quality(router, variant, "lattice refined A*");
    }

    {
        sailroute::RouteRequest variant = request;
        variant.options.heading_augmentation =
            sailroute::HeadingAugmentation::destination_bearing;
        report_route_quality(router, variant, "destination bearing heading");
    }
    {
        sailroute::RouteRequest variant = request;
        variant.options.heading_augmentation =
            sailroute::HeadingAugmentation::velocity_made_good;
        report_route_quality(router, variant, "velocity made good headings");
    }
    {
        sailroute::RouteRequest variant = request;
        variant.options.heading_augmentation = sailroute::HeadingAugmentation::
            destination_bearing_and_velocity_made_good;
        report_route_quality(router, variant, "both augmentations");
    }
    {
        sailroute::RouteRequest variant = request;
        variant.options.polar_angle_interpolation =
            sailroute::PolarAngleInterpolation::monotone_cubic;
        report_route_quality(router, variant, "monotone cubic polar");
    }
    {
        sailroute::RouteRequest variant = request;
        variant.options.wind_sampling = sailroute::WindSampling::midpoint;
        report_route_quality(router, variant, "midpoint wind sampling");
    }
    {
        sailroute::RouteRequest variant = request;
        variant.options.maneuver.tack_penalty = std::chrono::seconds{60};
        variant.options.maneuver.gybe_penalty = std::chrono::seconds{30};
        report_route_quality(router, variant, "maneuver penalties");
    }
    {
        sailroute::RouteRequest variant = request;
        variant.options.pruning_strategy =
            sailroute::PruningStrategy::bearing_sectors;
        report_route_quality(router, variant, "bearing sector pruning");
    }
    {
        sailroute::RouteRequest variant = request;
        variant.options.heading_augmentation = sailroute::HeadingAugmentation::
            destination_bearing_and_velocity_made_good;
        variant.options.polar_angle_interpolation =
            sailroute::PolarAngleInterpolation::monotone_cubic;
        variant.options.wind_sampling = sailroute::WindSampling::midpoint;
        variant.options.maneuver.tack_penalty = std::chrono::seconds{60};
        variant.options.maneuver.gybe_penalty = std::chrono::seconds{30};
        report_route_quality(router, variant, "all accuracy options");
    }

    std::cout << "\nStage 3 environment overhead\n";
    const EnvironmentFootprint footprint{south, north, west, east};
    double baseline_hours = 0.0;
    double peak_watermark = peak_resident_mebibytes();
    const double baseline_milliseconds =
        measure_baseline(router, request, baseline_hours);
    std::cout << "  " << std::left << std::setw(28) << "no providers"
              << std::right << "  " << std::fixed << std::setprecision(4)
              << std::setw(9) << baseline_hours << " h"
              << "                          " << std::setprecision(1)
              << std::setw(8) << baseline_milliseconds << " ms"
              << std::defaultfloat;
    std::cout << std::string(63, ' ');
    report_peak_resident(peak_watermark);
    std::cout << '\n' << std::defaultfloat;

    auto current = sailroute::make_uniform_current_provider(
        sailroute::CurrentVector{0.6, -0.3},
        sailroute::ProviderMetadata{
            "benchmark_current", "synthetic benchmark current", "1"});
    auto waves = sailroute::make_uniform_wave_provider(
        sailroute::WaveState{2.0, 9.0, 210.0},
        sailroute::ProviderMetadata{
            "benchmark_wave", "synthetic benchmark sea state", "1"});
    auto model = sailroute::make_wave_height_derating_model();
    if (!current.has_value() || !waves.has_value() || !model.has_value()) {
        std::cerr << "unable to build benchmark providers\n";
        return 1;
    }

    sailroute::RoutingEnvironment currents_only;
    currents_only.currents.provider = current.value();
    report_environment(
        weather.value(),
        polar,
        currents_only,
        request,
        "currents",
        baseline_milliseconds,
        baseline_hours,
        peak_watermark);

    sailroute::RoutingEnvironment waves_only;
    waves_only.waves.provider = waves.value();
    waves_only.waves.model = model.value();
    report_environment(
        weather.value(),
        polar,
        waves_only,
        request,
        "sea-state derating",
        baseline_milliseconds,
        baseline_hours,
        peak_watermark);

    sailroute::RoutingEnvironment land_only;
    land_only.land.landmask = open_water_landmask(footprint, 256U);
    land_only.land.missing_data_policy =
        sailroute::MissingDataPolicy::reject_transition;
    report_environment(
        weather.value(),
        polar,
        land_only,
        request,
        "landmask",
        baseline_milliseconds,
        baseline_hours,
        peak_watermark);

    for (const std::size_t zone_count : {4U, 32U, 128U}) {
        sailroute::RoutingEnvironment zones_only;
        zones_only.exclusions.zones = benchmark_zones(footprint, zone_count);
        report_environment(
            weather.value(),
            polar,
            zones_only,
            request,
            "exclusions x" + std::to_string(zone_count),
            baseline_milliseconds,
            baseline_hours,
            peak_watermark);
    }

    sailroute::RoutingEnvironment combined;
    combined.currents.provider = current.value();
    combined.waves.provider = waves.value();
    combined.waves.model = model.value();
    combined.land.landmask = land_only.land.landmask;
    combined.land.missing_data_policy =
        sailroute::MissingDataPolicy::reject_transition;
    combined.exclusions.zones = benchmark_zones(footprint, 32U);
    report_environment(
        weather.value(),
        polar,
        combined,
        request,
        "all providers",
        baseline_milliseconds,
        baseline_hours,
        peak_watermark);

    sailroute::RoutingEnvironment combined_midpoint = combined;
    combined_midpoint.sampling = sailroute::EnvironmentSampling::midpoint;
    sailroute::RouteRequest midpoint_request = request;
    midpoint_request.options.wind_sampling = sailroute::WindSampling::midpoint;
    report_environment(
        weather.value(),
        polar,
        combined_midpoint,
        midpoint_request,
        "all providers, midpoint",
        baseline_milliseconds,
        baseline_hours,
        peak_watermark);

    report_sea_state_matrix(weather.value(), polar, request, baseline_hours);
    return 0;
}
