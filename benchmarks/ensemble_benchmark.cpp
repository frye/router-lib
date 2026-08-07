#include "ensemble_benchmark.hpp"

#include "sailroute/ensemble.hpp"
#include "sailroute/serialization.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>
#if defined(_WIN32)
#include <windows.h>

#include <psapi.h>
#else
#include <sys/resource.h>
#endif

namespace sailroute::benchmarks {
namespace {

constexpr std::array<std::size_t, 6U> member_counts{1U, 2U, 5U, 10U, 20U, 50U};
constexpr std::array<EnsembleObjectiveKind, 5U> objective_kinds{
    EnsembleObjectiveKind::weighted_mean_elapsed_arrival,
    EnsembleObjectiveKind::weighted_p75_elapsed_arrival,
    EnsembleObjectiveKind::weighted_p90_elapsed_arrival,
    EnsembleObjectiveKind::probability_before_target,
    EnsembleObjectiveKind::probability_beating_rival};

double peak_resident_mebibytes() noexcept {
#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS counters{};
    counters.cb = sizeof(counters);
    if (GetProcessMemoryInfo(
            GetCurrentProcess(), &counters, sizeof(counters)) == 0) {
        return 0.0;
    }
    return static_cast<double>(counters.PeakWorkingSetSize) /
        (1024.0 * 1024.0);
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
    return bytes / (1024.0 * 1024.0);
#endif
}

std::string member_identifier(std::size_t index) {
    std::string identifier = "member-";
    if (index < 10U) {
        identifier += "00";
    } else if (index < 100U) {
        identifier += '0';
    }
    identifier += std::to_string(index);
    return identifier;
}

EnsembleObjective make_objective(
    EnsembleObjectiveKind kind,
    const EnsembleDataset& dataset,
    std::chrono::seconds route_horizon) {
    EnsembleObjective objective;
    objective.kind = kind;
    if (kind == EnsembleObjectiveKind::probability_before_target) {
        objective.target =
            EnsembleArrivalTarget{static_cast<double>(route_horizon.count()) * 0.75};
    } else if (kind == EnsembleObjectiveKind::probability_beating_rival) {
        const double rival_arrival =
            static_cast<double>(route_horizon.count()) * 0.8;
        for (const EnsembleMemberMetadata& member : dataset.members()) {
            objective.rival_outcomes.push_back(EnsembleMemberOutcome{
                member.identifier,
                EnsembleMemberOutcomeClass::reached,
                rival_arrival,
                std::nullopt});
        }
    }
    return objective;
}

double normalized_longitude(double longitude) noexcept {
    if (longitude >= 180.0) {
        longitude -= 360.0;
    } else if (longitude < -180.0) {
        longitude += 360.0;
    }
    return longitude;
}

Coordinate benchmark_destination(
    const EnsembleDataset& dataset,
    const RouteRequest& request) {
    const WeatherDataset* weather = dataset.member_weather(0U);
    if (weather == nullptr) {
        return request.destination;
    }
    const TimePoint when = request.departure_time.value_or(
        dataset.members().front().weather.first_valid_time);
    const std::array<Coordinate, 4U> local_candidates{
        Coordinate{
            request.start.latitude_degrees,
            normalized_longitude(request.start.longitude_degrees + 0.1)},
        Coordinate{
            request.start.latitude_degrees,
            normalized_longitude(request.start.longitude_degrees - 0.1)},
        Coordinate{
            std::min(90.0, request.start.latitude_degrees + 0.1),
            request.start.longitude_degrees},
        Coordinate{
            std::max(-90.0, request.start.latitude_degrees - 0.1),
            request.start.longitude_degrees}};
    for (const Coordinate candidate : local_candidates) {
        if (weather->interpolate(candidate, when).has_value()) {
            return candidate;
        }
    }
    return request.destination;
}

void report_sampling(
    const EnsembleDataset& dataset,
    TimePoint when,
    Coordinate coordinate) {
    const std::size_t iterations =
        std::max<std::size_t>(128U, 4'096U / dataset.member_count());
    auto batched_sampler = dataset.sampler_at(when);
    if (!batched_sampler.has_value()) {
        std::cout << "  sampling error: " << batched_sampler.error().message << '\n';
        return;
    }

    volatile double checksum = 0.0;
    const auto batched_start = std::chrono::steady_clock::now();
    for (std::size_t iteration = 0U; iteration < iterations; ++iteration) {
        for (const EnsembleMemberWindSample& sample :
             batched_sampler.value().sample(coordinate)) {
            if (sample.wind.has_value()) {
                checksum = checksum + sample.wind.value().speed_knots();
            }
        }
    }
    const double batched_seconds = std::chrono::duration<double>(
                                       std::chrono::steady_clock::now() -
                                       batched_start)
                                       .count();

    std::vector<WeatherSampler> independent_samplers;
    independent_samplers.reserve(dataset.member_count());
    for (std::size_t index = 0U; index < dataset.member_count(); ++index) {
        const WeatherDataset* weather = dataset.member_weather(index);
        if (weather == nullptr) {
            std::cout << "  sampling error: missing member weather\n";
            return;
        }
        auto sampler = weather->sampler_at(when);
        if (!sampler.has_value()) {
            std::cout << "  sampling error: " << sampler.error().message << '\n';
            return;
        }
        independent_samplers.push_back(std::move(sampler.value()));
    }
    const auto independent_start = std::chrono::steady_clock::now();
    for (std::size_t iteration = 0U; iteration < iterations; ++iteration) {
        for (const WeatherSampler& sampler : independent_samplers) {
            auto wind = sampler.sample(coordinate);
            if (wind.has_value()) {
                checksum = checksum + wind.value().speed_knots();
            }
        }
    }
    const double independent_seconds = std::chrono::duration<double>(
                                           std::chrono::steady_clock::now() -
                                           independent_start)
                                           .count();
    const double samples =
        static_cast<double>(iterations * dataset.member_count());
    std::cout << "  samples/s batched/independent "
              << std::fixed << std::setprecision(0)
              << samples / batched_seconds << '/'
              << samples / independent_seconds
              << ", checksum " << std::setprecision(2) << checksum << '\n'
              << std::defaultfloat;
}

void report_route(
    const EnsembleDataset& dataset,
    const VesselPolar& polar,
    const RouteRequest& deterministic_request,
    EnsembleObjectiveKind objective_kind,
    EnsembleSolver solver) {
    EnsembleRouteRequest request;
    request.start = deterministic_request.start;
    request.destination = benchmark_destination(dataset, deterministic_request);
    request.departure_time = deterministic_request.departure_time;
    request.options = deterministic_request.options;
    request.options.maximum_route_duration = std::min(
        request.options.maximum_route_duration, std::chrono::hours{2});
    request.options.arrival_radius_nautical_miles =
        std::min(request.options.arrival_radius_nautical_miles, 0.1);
    request.objective = make_objective(
        objective_kind,
        dataset,
        std::chrono::duration_cast<std::chrono::seconds>(
            request.options.maximum_route_duration));
    request.solver = solver;
    request.policy.max_alternatives = 2U;
    request.lattice.subdivision_level = 0U;
    request.lattice.time_bucket = std::chrono::minutes{30};
    request.lattice.max_labels_per_state = 512U;
    request.lattice.max_total_labels = 5'000U;
    request.beam.time_step = std::chrono::minutes{30};
    request.beam.heading_step_degrees = 20.0;
    request.beam.max_nodes_per_bucket = 4U;
    request.beam.beam_width = 128U;
    request.beam.max_steps = 8U;
    request.beam.max_total_nodes = 10'000U;
    request.enable_experimental_beam =
        solver == EnsembleSolver::experimental_isochrone_beam;

    std::size_t callback_count = 0U;
    EnsembleRouter router{dataset, polar};
    const auto start = std::chrono::steady_clock::now();
    auto result = router.optimize_view(
        request,
        [&callback_count](const EnsembleProgressView&) {
            ++callback_count;
        });
    const double route_milliseconds = std::chrono::duration<double, std::milli>(
                                          std::chrono::steady_clock::now() -
                                          start)
                                          .count();

    std::cout << "  " << std::left << std::setw(31)
              << (std::string{to_string(solver)} + '/' +
                  std::string{to_string(objective_kind)})
              << std::right;
    if (!result.has_value()) {
        std::cout << " error=\"" << result.error().message << "\""
                  << " runtime_ms=" << std::fixed << std::setprecision(1)
                  << route_milliseconds << " callbacks=" << callback_count << '\n'
                  << std::defaultfloat;
        return;
    }

    const EnsembleRouteResult& route = result.value();
    std::size_t reached = 0U;
    for (const EnsembleMemberRouteResult& member : route.members) {
        reached += member.outcome.outcome_class ==
            EnsembleMemberOutcomeClass::reached;
    }
    auto json = ensemble_route_to_json(
        EnsembleRouteDocument{dataset.metadata(), dataset.members(), route});
    const std::size_t serialized_bytes =
        json.has_value() ? json.value().size() : 0U;
    const EnsembleObjectiveValue& value = route.objective.value;
    const std::string value_text = value.is_finite()
        ? std::to_string(value.finite_value)
        : std::string{"positive_infinity"};
    const std::size_t retained =
        solver == EnsembleSolver::time_dependent_lattice
        ? route.lattice_diagnostics.retained_labels
        : route.beam_diagnostics.retained_nodes;
    const std::size_t generated =
        solver == EnsembleSolver::time_dependent_lattice
        ? route.lattice_diagnostics.generated_labels
        : route.beam_diagnostics.generated_nodes;
    const std::size_t settled =
        solver == EnsembleSolver::time_dependent_lattice
        ? route.lattice_diagnostics.settled_labels
        : route.beam_diagnostics.expanded_nodes;
    std::cout << " value=" << value_text
              << " reached=" << reached << '/' << route.members.size()
              << " generated=" << generated
              << " settled=" << settled
              << " retained=" << retained
              << " policy=" << route.policy.nodes.size() << '/'
              << route.policy.branches.size() << '/'
              << route.decision_points.size()
              << " runtime_ms=" << std::fixed << std::setprecision(1)
              << route_milliseconds
              << " peak_rss_mib=" << peak_resident_mebibytes()
              << " json_bytes=" << serialized_bytes
              << " callbacks=" << callback_count << '\n'
              << std::defaultfloat;
}

}  // namespace

void report_ensemble_scaling(
    const std::filesystem::path& grib_path,
    const VesselPolar& polar,
    const RouteRequest& deterministic_request) {
    std::cout << "\nStage 4 ensemble scaling\n"
              << "  solver/objective value reached generated settled retained "
                 "policy(nodes/branches/decisions) runtime peak-rss JSON callbacks\n";

    for (const std::size_t member_count : member_counts) {
        std::vector<EnsembleMemberInput> inputs;
        inputs.reserve(member_count);
        for (std::size_t index = member_count; index > 0U; --index) {
            inputs.push_back(EnsembleMemberInput{
                member_identifier(index - 1U), 1.0, grib_path, std::nullopt, {}});
        }
        EnsembleRunMetadata metadata{
            "benchmark-" + std::to_string(member_count),
            "repeated-input-scaling",
            deterministic_request.departure_time.value_or(TimePoint{}),
            "synthetic repeated-member benchmark",
            1U};
        const auto load_start = std::chrono::steady_clock::now();
        auto dataset = EnsembleDataset::load(
            std::move(metadata), std::move(inputs));
        const double load_milliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - load_start)
                .count();
        std::cout << "\nmembers=" << member_count
                  << " load_ms=" << std::fixed << std::setprecision(1)
                  << load_milliseconds
                  << " peak_rss_mib=" << peak_resident_mebibytes() << '\n'
                  << std::defaultfloat;
        if (!dataset.has_value()) {
            std::cout << "  load error: " << dataset.error().message << '\n';
            continue;
        }
        report_sampling(
            dataset.value(),
            deterministic_request.departure_time.value_or(
                dataset.value().members().front().weather.first_valid_time),
            deterministic_request.start);
        for (const EnsembleObjectiveKind objective_kind : objective_kinds) {
            report_route(
                dataset.value(),
                polar,
                deterministic_request,
                objective_kind,
                EnsembleSolver::time_dependent_lattice);
            if (member_count <= 10U) {
                report_route(
                    dataset.value(),
                    polar,
                    deterministic_request,
                    objective_kind,
                    EnsembleSolver::experimental_isochrone_beam);
            }
        }
    }
}

}  // namespace sailroute::benchmarks
