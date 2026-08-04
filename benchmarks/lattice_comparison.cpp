#include "lattice_comparison.hpp"

#include "routing/lattice.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <numeric>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sailroute::benchmarks {
namespace {

constexpr double earth_radius_nautical_miles = 3440.065;

struct Vector3 {
    double x;
    double y;
    double z;

    bool operator==(const Vector3&) const = default;
};

struct ComparisonModel {
    std::string name;
    std::string resolution;
    std::vector<Vector3> representatives;
    std::vector<std::vector<std::uint32_t>> neighbors;
};

struct Measurements {
    std::size_t cells{};
    std::size_t minimum_degree{};
    double mean_degree{};
    std::size_t maximum_degree{};
    double degree_standard_deviation{};
    double minimum_edge_nautical_miles{};
    double median_edge_nautical_miles{};
    double percentile_95_edge_nautical_miles{};
    double maximum_edge_nautical_miles{};
    double edge_spread{};
    bool stable_identity_and_order{};
    bool antimeridian_same_cell{};
    bool antimeridian_adjacent{};
    double polar_edge_ratio{};
    double lookup_millions_per_second{};
    double enumeration_millions_per_second{};
    double packed_kibibytes{};
    double packed_bytes_per_cell{};
    std::uint64_t checksum{};
};

Vector3 vector_from(double latitude_degrees, double longitude_degrees) noexcept {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    if (longitude_degrees == 180.0) {
        longitude_degrees = -180.0;
    }
    const double latitude = latitude_degrees * degrees_to_radians;
    const double longitude = longitude_degrees * degrees_to_radians;
    const double cosine_latitude = std::cos(latitude);
    return Vector3{
        cosine_latitude * std::cos(longitude),
        cosine_latitude * std::sin(longitude),
        std::sin(latitude)};
}

double dot(Vector3 left, Vector3 right) noexcept {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

std::uint32_t nearest_cell(
    std::span<const Vector3> representatives,
    Vector3 query) noexcept {
    std::uint32_t closest = 0U;
    double closest_dot_product = dot(query, representatives.front());
    for (std::uint32_t cell = 1U;
         static_cast<std::size_t>(cell) < representatives.size();
         ++cell) {
        const double dot_product = dot(query, representatives[cell]);
        if (dot_product > closest_dot_product) {
            closest = cell;
            closest_dot_product = dot_product;
        }
    }
    return closest;
}

double edge_length(Vector3 first, Vector3 second) noexcept {
    return earth_radius_nautical_miles *
           std::acos(std::clamp(dot(first, second), -1.0, 1.0));
}

std::vector<std::vector<std::uint32_t>> proximity_neighbors(
    std::span<const Vector3> representatives,
    std::size_t requested_degree) {
    using Candidate = std::pair<double, std::uint32_t>;
    const auto closer = [](const Candidate& left, const Candidate& right) {
        if (left.first != right.first) {
            return left.first > right.first;
        }
        return left.second < right.second;
    };

    std::vector<std::vector<std::uint32_t>> neighbors(representatives.size());
    std::vector<Candidate> candidates;
    candidates.reserve(representatives.size() - 1U);
    for (std::uint32_t cell = 0U;
         static_cast<std::size_t>(cell) < representatives.size();
         ++cell) {
        candidates.clear();
        for (std::uint32_t other = 0U;
             static_cast<std::size_t>(other) < representatives.size();
             ++other) {
            if (cell != other) {
                candidates.emplace_back(
                    dot(representatives[cell], representatives[other]), other);
            }
        }
        const std::size_t count = std::min(requested_degree, candidates.size());
        std::partial_sort(
            candidates.begin(),
            candidates.begin() + static_cast<std::ptrdiff_t>(count),
            candidates.end(),
            closer);
        for (std::size_t index = 0U; index < count; ++index) {
            const std::uint32_t other = candidates[index].second;
            neighbors[cell].push_back(other);
            neighbors[other].push_back(cell);
        }
    }
    for (std::vector<std::uint32_t>& adjacent : neighbors) {
        std::sort(adjacent.begin(), adjacent.end());
        adjacent.erase(std::unique(adjacent.begin(), adjacent.end()), adjacent.end());
    }
    return neighbors;
}

ComparisonModel make_icosphere(std::size_t level) {
    auto lattice = detail::GeodesicLattice::create(level);
    if (!lattice.has_value()) {
        throw std::runtime_error(lattice.error().message);
    }

    ComparisonModel model{
        "icosphere", "level " + std::to_string(level), {}, {}};
    model.representatives.reserve(lattice.value().vertex_count());
    model.neighbors.resize(lattice.value().vertex_count());
    for (std::size_t cell = 0U; cell < lattice.value().vertex_count(); ++cell) {
        const Coordinate coordinate = lattice.value().coordinate(cell);
        model.representatives.push_back(vector_from(
            coordinate.latitude_degrees, coordinate.longitude_degrees));
        const auto adjacent = lattice.value().neighbors(cell);
        model.neighbors[cell].assign(adjacent.begin(), adjacent.end());
    }
    return model;
}

ComparisonModel make_healpix(std::size_t nside) {
    const std::size_t cell_count = 12U * nside * nside;
    const std::size_t north_cap_count = 2U * nside * (nside - 1U);
    ComparisonModel model{
        "HEALPix", "nside " + std::to_string(nside), {}, {}};
    model.representatives.reserve(cell_count);

    for (std::size_t cell = 0U; cell < cell_count; ++cell) {
        double z = 0.0;
        double longitude_radians = 0.0;
        if (cell < north_cap_count) {
            const std::size_t ring = static_cast<std::size_t>(
                0.5 * (1.0 + std::sqrt(1.0 + 2.0 * static_cast<double>(cell))));
            const std::size_t position =
                cell + 1U - 2U * ring * (ring - 1U);
            z = 1.0 - static_cast<double>(ring * ring) /
                              (3.0 * static_cast<double>(nside * nside));
            longitude_radians =
                (static_cast<double>(position) - 0.5) * std::numbers::pi /
                (2.0 * static_cast<double>(ring));
        } else if (cell < cell_count - north_cap_count) {
            const std::size_t offset = cell - north_cap_count;
            const std::size_t ring = offset / (4U * nside) + nside;
            const std::size_t position = offset % (4U * nside) + 1U;
            const double half_offset =
                0.5 * (1.0 + static_cast<double>((ring + nside) & 1U));
            z = (2.0 * static_cast<double>(nside) -
                 static_cast<double>(ring)) *
                2.0 /
                (3.0 * static_cast<double>(nside));
            longitude_radians =
                (static_cast<double>(position) - half_offset) * std::numbers::pi /
                (2.0 * static_cast<double>(nside));
        } else {
            const std::size_t reverse_position = cell_count - cell;
            const std::size_t ring = static_cast<std::size_t>(
                0.5 *
                (1.0 + std::sqrt(
                           2.0 * static_cast<double>(reverse_position) - 1.0)));
            const std::size_t position =
                4U * ring + 1U -
                (reverse_position - 2U * ring * (ring - 1U));
            z = -1.0 + static_cast<double>(ring * ring) /
                               (3.0 * static_cast<double>(nside * nside));
            longitude_radians =
                (static_cast<double>(position) - 0.5) * std::numbers::pi /
                (2.0 * static_cast<double>(ring));
        }
        const double radial = std::sqrt(std::max(0.0, 1.0 - z * z));
        model.representatives.push_back(Vector3{
            radial * std::cos(longitude_radians),
            radial * std::sin(longitude_radians),
            z});
    }
    model.neighbors = proximity_neighbors(model.representatives, 8U);
    return model;
}

ComparisonModel make_fibonacci(std::size_t cell_count) {
    const double golden_angle =
        std::numbers::pi * (3.0 - std::sqrt(5.0));
    ComparisonModel model{
        "Fibonacci", "N " + std::to_string(cell_count), {}, {}};
    model.representatives.reserve(cell_count);
    for (std::size_t cell = 0U; cell < cell_count; ++cell) {
        const double z =
            1.0 - 2.0 * (static_cast<double>(cell) + 0.5) /
                      static_cast<double>(cell_count);
        const double radial = std::sqrt(std::max(0.0, 1.0 - z * z));
        const double longitude =
            golden_angle * static_cast<double>(cell);
        model.representatives.push_back(Vector3{
            radial * std::cos(longitude), radial * std::sin(longitude), z});
    }
    model.neighbors = proximity_neighbors(model.representatives, 6U);
    return model;
}

bool are_adjacent_or_equal(
    const ComparisonModel& model,
    std::uint32_t first,
    std::uint32_t second) {
    return first == second ||
           std::binary_search(
               model.neighbors[first].begin(), model.neighbors[first].end(), second);
}

std::vector<Vector3> deterministic_queries() {
    constexpr std::size_t query_count = 8192U;
    std::vector<Vector3> queries;
    queries.reserve(query_count);
    queries.push_back(vector_from(0.0, 180.0));
    queries.push_back(vector_from(0.0, -180.0));
    queries.push_back(vector_from(90.0, 0.0));
    queries.push_back(vector_from(90.0, 137.0));
    for (std::size_t index = queries.size(); index < query_count; ++index) {
        const double z =
            1.0 - 2.0 * (static_cast<double>(index) + 0.5) /
                      static_cast<double>(query_count);
        const double latitude =
            std::asin(std::clamp(z, -1.0, 1.0)) * 180.0 / std::numbers::pi;
        const std::uint32_t mixed =
            static_cast<std::uint32_t>(index) * 2'654'435'761U + 1'013'904'223U;
        const double longitude =
            static_cast<double>(mixed) /
                static_cast<double>(std::numeric_limits<std::uint32_t>::max()) *
                360.0 -
            180.0;
        queries.push_back(vector_from(latitude, longitude));
    }
    return queries;
}

double percentile(const std::vector<double>& sorted, double fraction) {
    const std::size_t index = static_cast<std::size_t>(
        fraction * static_cast<double>(sorted.size() - 1U));
    return sorted[index];
}

Measurements measure(
    const ComparisonModel& model,
    const ComparisonModel& repeated) {
    Measurements result;
    result.cells = model.representatives.size();

    std::vector<double> edge_lengths;
    std::size_t degree_sum = 0U;
    result.minimum_degree = model.neighbors.front().size();
    for (std::uint32_t cell = 0U;
         static_cast<std::size_t>(cell) < model.neighbors.size();
         ++cell) {
        const std::size_t degree = model.neighbors[cell].size();
        degree_sum += degree;
        result.minimum_degree = std::min(result.minimum_degree, degree);
        result.maximum_degree = std::max(result.maximum_degree, degree);
        for (const std::uint32_t neighbor : model.neighbors[cell]) {
            if (cell < neighbor) {
                edge_lengths.push_back(edge_length(
                    model.representatives[cell],
                    model.representatives[neighbor]));
            }
        }
    }
    result.mean_degree =
        static_cast<double>(degree_sum) / static_cast<double>(result.cells);
    double squared_degree_difference_sum = 0.0;
    for (const auto& adjacent : model.neighbors) {
        const double difference =
            static_cast<double>(adjacent.size()) - result.mean_degree;
        squared_degree_difference_sum += difference * difference;
    }
    result.degree_standard_deviation = std::sqrt(
        squared_degree_difference_sum / static_cast<double>(result.cells));

    std::sort(edge_lengths.begin(), edge_lengths.end());
    result.minimum_edge_nautical_miles = edge_lengths.front();
    result.median_edge_nautical_miles = percentile(edge_lengths, 0.5);
    result.percentile_95_edge_nautical_miles = percentile(edge_lengths, 0.95);
    result.maximum_edge_nautical_miles = edge_lengths.back();
    result.edge_spread =
        result.percentile_95_edge_nautical_miles /
        std::max(result.minimum_edge_nautical_miles, 1.0e-12);

    const std::vector<Vector3> queries = deterministic_queries();
    std::vector<std::uint32_t> first_results;
    first_results.reserve(queries.size());
    for (const Vector3 query : queries) {
        first_results.push_back(nearest_cell(model.representatives, query));
    }
    std::vector<std::size_t> order(queries.size());
    std::iota(order.begin(), order.end(), 0U);
    std::uint32_t shuffle_state = 0x5a17c9e3U;
    for (std::size_t remaining = order.size(); remaining > 1U; --remaining) {
        shuffle_state = shuffle_state * 1'664'525U + 1'013'904'223U;
        const std::size_t other = shuffle_state % remaining;
        std::swap(order[remaining - 1U], order[other]);
    }
    bool shuffled_queries_stable = true;
    for (const std::size_t index : order) {
        shuffled_queries_stable =
            shuffled_queries_stable &&
            nearest_cell(repeated.representatives, queries[index]) ==
                first_results[index];
    }
    result.stable_identity_and_order =
        model.representatives == repeated.representatives &&
        model.neighbors == repeated.neighbors && shuffled_queries_stable;

    const std::uint32_t east =
        nearest_cell(model.representatives, vector_from(0.0, 180.0));
    const std::uint32_t west =
        nearest_cell(model.representatives, vector_from(0.0, -180.0));
    result.antimeridian_same_cell = east == west;
    const std::uint32_t near_east =
        nearest_cell(model.representatives, vector_from(0.0, 179.9));
    const std::uint32_t near_west =
        nearest_cell(model.representatives, vector_from(0.0, -179.9));
    result.antimeridian_adjacent =
        are_adjacent_or_equal(model, near_east, near_west);

    const std::uint32_t pole =
        nearest_cell(model.representatives, vector_from(90.0, 0.0));
    double maximum_polar_edge = 0.0;
    for (const std::uint32_t neighbor : model.neighbors[pole]) {
        maximum_polar_edge = std::max(
            maximum_polar_edge,
            edge_length(
                model.representatives[pole],
                model.representatives[neighbor]));
    }
    result.polar_edge_ratio =
        maximum_polar_edge / result.median_edge_nautical_miles;

    volatile std::uint64_t lookup_checksum = 0U;
    const auto lookup_start = std::chrono::steady_clock::now();
    for (const Vector3 query : queries) {
        lookup_checksum =
            lookup_checksum + nearest_cell(model.representatives, query);
    }
    const double lookup_seconds = std::chrono::duration<double>(
                                      std::chrono::steady_clock::now() -
                                      lookup_start)
                                      .count();
    result.lookup_millions_per_second =
        static_cast<double>(queries.size()) / lookup_seconds / 1'000'000.0;

    constexpr std::size_t enumeration_count = 2'000'000U;
    volatile std::uint64_t enumeration_checksum = 0U;
    const auto enumeration_start = std::chrono::steady_clock::now();
    for (std::size_t index = 0U; index < enumeration_count; ++index) {
        const std::size_t cell =
            (index * std::size_t{2'654'435'761U}) % result.cells;
        for (const std::uint32_t neighbor : model.neighbors[cell]) {
            enumeration_checksum = enumeration_checksum + neighbor + 1U;
        }
    }
    const double enumeration_seconds = std::chrono::duration<double>(
                                           std::chrono::steady_clock::now() -
                                           enumeration_start)
                                           .count();
    result.enumeration_millions_per_second =
        static_cast<double>(enumeration_count) / enumeration_seconds /
        1'000'000.0;

    const std::size_t packed_bytes =
        model.representatives.size() * sizeof(Vector3) +
        (model.neighbors.size() + 1U) * sizeof(std::uint32_t) +
        degree_sum * sizeof(std::uint32_t);
    result.packed_kibibytes = static_cast<double>(packed_bytes) / 1024.0;
    result.packed_bytes_per_cell =
        static_cast<double>(packed_bytes) / static_cast<double>(result.cells);
    result.checksum = lookup_checksum + enumeration_checksum;
    return result;
}

void print_count_row(
    std::string_view candidate,
    std::string_view resolution,
    std::size_t cells,
    double expected_degree) {
    const double packed_bytes_per_cell =
        static_cast<double>(sizeof(Vector3) + sizeof(std::uint32_t)) +
        expected_degree * static_cast<double>(sizeof(std::uint32_t));
    std::cout << "  " << std::left << std::setw(11) << candidate << std::setw(10)
              << resolution << std::right << std::setw(8) << cells
              << std::setw(12) << std::fixed << std::setprecision(1)
              << packed_bytes_per_cell * static_cast<double>(cells) / 1024.0
              << '\n';
}

void print_measurements(const ComparisonModel& model, const Measurements& value) {
    std::cout << "  " << std::left << std::setw(11) << model.name << std::setw(10)
              << model.resolution << std::right << std::setw(7) << value.cells
              << "  " << value.minimum_degree << '/' << std::fixed
              << std::setprecision(2) << value.mean_degree << '/'
              << value.maximum_degree << " sd=" << std::setprecision(2)
              << value.degree_standard_deviation << '\n'
              << "    edge nm min/p50/p95/max: " << std::setprecision(2)
              << value.minimum_edge_nautical_miles << '/'
              << value.median_edge_nautical_miles << '/'
              << value.percentile_95_edge_nautical_miles << '/'
              << value.maximum_edge_nautical_miles << "  p95/min="
              << value.edge_spread << '\n'
              << "    stable=" << (value.stable_identity_and_order ? "yes" : "NO")
              << "  seam exact/near="
              << (value.antimeridian_same_cell ? "same" : "split") << '/'
              << (value.antimeridian_adjacent ? "adjacent" : "DISCONTINUOUS")
              << "  polar-max/p50=" << value.polar_edge_ratio << '\n'
              << "    lookup=" << std::setprecision(3)
              << value.lookup_millions_per_second << " Mquery/s"
              << "  enumerate=" << value.enumeration_millions_per_second
              << " Mcell/s  packed=" << std::setprecision(1)
              << value.packed_kibibytes << " KiB (" << value.packed_bytes_per_cell
              << " B/cell)  checksum=" << value.checksum << '\n';
}

}  // namespace

void report_lattice_comparison() {
    std::cout
        << "\ngeodesic lattice comparison\n"
        << "benchmark-only deterministic models; HEALPix/Fibonacci adjacency is "
           "a reciprocal nearest-center proxy\n"
        << "packed estimate = xyz doubles + uint32 CSR offsets/neighbors\n\n"
        << "cell count and approximate packed memory by resolution\n"
        << "  candidate  resolution   cells  packed KiB\n";
    constexpr std::array<std::size_t, 5U> icosphere_counts{
        12U, 42U, 162U, 642U, 2562U};
    constexpr std::array<std::size_t, 5U> fibonacci_counts{
        12U, 42U, 162U, 642U, 2562U};
    for (std::size_t level = 0U; level < icosphere_counts.size(); ++level) {
        const std::string resolution = "L" + std::to_string(level);
        const double mean_degree =
            6.0 - 12.0 / static_cast<double>(icosphere_counts[level]);
        print_count_row(
            "icosphere", resolution, icosphere_counts[level], mean_degree);
    }
    for (std::size_t level = 0U; level < icosphere_counts.size(); ++level) {
        const std::size_t nside = std::size_t{1U} << level;
        const std::string resolution = "N" + std::to_string(nside);
        print_count_row("HEALPix", resolution, 12U * nside * nside, 8.0);
    }
    for (std::size_t level = 0U; level < fibonacci_counts.size(); ++level) {
        const std::string resolution = "N" + std::to_string(fibonacci_counts[level]);
        print_count_row("Fibonacci", resolution, fibonacci_counts[level], 6.0);
    }

    std::cout
        << "\ndetailed comparison near 2.5k-3k cells\n"
        << "  candidate  resolution cells  degree min/mean/max\n";
    const ComparisonModel icosphere = make_icosphere(4U);
    const ComparisonModel icosphere_repeated = make_icosphere(4U);
    const ComparisonModel healpix = make_healpix(16U);
    const ComparisonModel healpix_repeated = make_healpix(16U);
    const ComparisonModel fibonacci = make_fibonacci(2562U);
    const ComparisonModel fibonacci_repeated = make_fibonacci(2562U);
    print_measurements(icosphere, measure(icosphere, icosphere_repeated));
    print_measurements(healpix, measure(healpix, healpix_repeated));
    print_measurements(fibonacci, measure(fibonacci, fibonacci_repeated));

    std::cout
        << "\nrefinement suitability\n"
        << "  icosphere: yes; face-local 4-way refinement and inherited vertex IDs\n"
        << "  HEALPix:   yes globally with NESTED IDs; local mixed-level neighbor "
           "stitching is additional work\n"
        << "  Fibonacci: no natural parent/child relation; changing N globally "
           "repositions cells\n"
        << "selection evidence: icosphere retains stable coarse IDs, reciprocal "
           "low-degree topology, and direct corridor-local refinement without a "
           "production dependency\n\n"
        << std::defaultfloat;
}

}  // namespace sailroute::benchmarks
