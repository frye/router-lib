#include "sailroute/serialization.hpp"

#include "test_support.hpp"

#include <chrono>
#include <limits>
#include <string>

namespace {

sailroute::RouteResult sample_route() {
    const auto departure = sailroute::TimePoint{std::chrono::seconds{1'700'000'000}};
    return sailroute::RouteResult{
        .departure_time = departure,
        .arrival_time = departure + std::chrono::hours{1},
        .departure_source = sailroute::DepartureSource::explicit_time,
        .forecast_source = "forecast \"A\" & B",
        .polar_source = "polar <demo>",
        .points = {
            sailroute::RoutePoint{
                .position = {10.0, -20.0},
                .time = departure,
                .heading_degrees = 270.0,
                .boat_speed_knots = 7.5,
                .true_wind_speed_knots = 12.0,
                .true_wind_direction_degrees = 90.0,
                .cumulative_distance_nautical_miles = 0.0,
            },
            sailroute::RoutePoint{
                .position = {10.1, -20.2},
                .time = departure + std::chrono::hours{1},
                .heading_degrees = 270.0,
                .boat_speed_knots = 7.5,
                .true_wind_speed_knots = 12.0,
                .true_wind_direction_degrees = 90.0,
                .cumulative_distance_nautical_miles = 7.5,
            },
        },
        .diagnostics = {
            .expanded_nodes = 10,
            .generated_candidates = 100,
            .retained_candidates = 20,
            .time_steps = 2,
        },
    };
}

}  // namespace

TEST_CASE("JSON serialization escapes metadata and includes route points") {
    const auto serialized = sailroute::route_to_json(sample_route());
    REQUIRE(serialized.has_value());
    REQUIRE(serialized.value().find("forecast \\\"A\\\" & B") != std::string::npos);
    REQUIRE(serialized.value().find("\"points\":[") != std::string::npos);
    REQUIRE(serialized.value().find("\"expandedNodes\":10") != std::string::npos);
}

TEST_CASE("GPX serialization escapes XML and emits timestamped track points") {
    auto route = sample_route();
    route.points.front().position = {0.0000001, -0.0000001};
    const auto serialized = sailroute::route_to_gpx(route);
    REQUIRE(serialized.has_value());
    REQUIRE(serialized.value().find("polar &lt;demo&gt;") != std::string::npos);
    REQUIRE(
        serialized.value().find(
            "<trkpt lat=\"0.0000001\" lon=\"-0.0000001\">") !=
        std::string::npos);
    REQUIRE(serialized.value().find("e-") == std::string::npos);
    REQUIRE(serialized.value().find("<time>") != std::string::npos);
}

TEST_CASE("serialization rejects non-finite route values") {
    auto route = sample_route();
    route.points.front().boat_speed_knots = std::numeric_limits<double>::infinity();
    const auto json = sailroute::route_to_json(route);
    REQUIRE(!json.has_value());
    REQUIRE(json.error().code == sailroute::ErrorCode::output_error);

    route = sample_route();
    route.points.front().position.latitude_degrees = 91.0;
    const auto gpx = sailroute::route_to_gpx(route);
    REQUIRE(!gpx.has_value());
    REQUIRE(gpx.error().code == sailroute::ErrorCode::output_error);
}
