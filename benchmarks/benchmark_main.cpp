#include "sailroute/router.hpp"
#include "sailroute/time.hpp"

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace {

void benchmark_routing(
    const sailroute::Router& router,
    sailroute::RouteRequest request,
    std::size_t worker_count,
    std::string_view label) {
    constexpr std::size_t iterations = 10U;
    request.options.worker_count = worker_count;
    volatile std::size_t checksum = 0U;

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t index = 0U; index < iterations; ++index) {
        auto route = router.optimize(request);
        if (!route.has_value()) {
            throw std::runtime_error(route.error().message);
        }
        checksum = checksum + route.value().diagnostics.generated_candidates +
                   route.value().points.size();
    }
    const double seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - start)
                               .count();
    std::cout << label << ": " << iterations / seconds << " routes/s ("
              << seconds * 1000.0 / static_cast<double>(iterations)
              << " ms/route), checksum: " << checksum << '\n';
}

}  // namespace

int main(int argc, char** argv) {
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

    sailroute::RouteRequest request;
    request.start = {20.0, 5.0};
    request.destination = {20.0, 5.8};
    request.departure_time = departure.value();
    request.options.time_step = std::chrono::minutes{30};
    request.options.use_routing_intervals = false;
    request.options.heading_step_degrees = 5.0;
    request.options.arrival_radius_nautical_miles = 0.5;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 4U;
    request.options.maximum_route_duration = std::chrono::hours{12};
    const sailroute::Router router{weather.value(), polar};

    benchmark_routing(router, request, 1U, "routing single worker");
    benchmark_routing(router, request, 4U, "routing four workers");
    benchmark_routing(router, request, 0U, "routing automatic workers");
    return 0;
}
