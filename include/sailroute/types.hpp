#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sailroute {

using TimePoint = std::chrono::sys_seconds;

struct Coordinate {
    double latitude_degrees{};
    double longitude_degrees{};
};

struct Wind {
    double east_mps{};
    double north_mps{};

    [[nodiscard]] double speed_knots() const noexcept;
    [[nodiscard]] double direction_from_degrees() const noexcept;
};

struct RoutingInterval {
    std::chrono::minutes interval{};
    std::optional<std::chrono::minutes> until_elapsed;
};

enum class DepartureSource {
    explicit_time,
    current_time,
    forecast_start_fallback,
};

struct RoutingOptions {
    std::optional<std::chrono::minutes> time_step;
    double heading_step_degrees{10.0};
    double arrival_radius_nautical_miles{2.0};
    double spatial_bucket_nautical_miles{10.0};
    std::size_t max_nodes_per_bucket{10};
    std::size_t worker_count{0};
    std::chrono::hours maximum_route_duration{240};
    double minimum_boat_speed_knots{0.05};
    bool capture_isochrones{false};
    std::vector<RoutingInterval> routing_intervals{
        {std::chrono::minutes{30}, std::chrono::minutes{240}},
        {std::chrono::minutes{60}, std::chrono::minutes{1'440}},
        {std::chrono::minutes{180}, std::nullopt},
    };
};

struct RouteRequest {
    Coordinate start;
    Coordinate destination;
    std::optional<TimePoint> departure_time;
    RoutingOptions options;
};

struct RoutePoint {
    Coordinate position;
    TimePoint time;
    double heading_degrees{};
    double boat_speed_knots{};
    double true_wind_speed_knots{};
    double true_wind_direction_degrees{};
    double cumulative_distance_nautical_miles{};
};

struct RouteDiagnostics {
    std::size_t expanded_nodes{};
    std::size_t generated_candidates{};
    std::size_t retained_candidates{};
    std::size_t time_steps{};
};

struct Isochrone {
    TimePoint time;
    std::vector<Coordinate> points;
};

struct RouteResult {
    TimePoint departure_time;
    TimePoint arrival_time;
    DepartureSource departure_source;
    std::string forecast_source;
    std::string polar_source;
    std::vector<RoutePoint> points;
    std::vector<Isochrone> isochrones;
    RouteDiagnostics diagnostics;
};

[[nodiscard]] bool is_valid(Coordinate coordinate) noexcept;
std::string_view to_string(DepartureSource source) noexcept;

}  // namespace sailroute
