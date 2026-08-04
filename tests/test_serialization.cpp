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
        .isochrones = {
            sailroute::Isochrone{
                .time = departure + std::chrono::minutes{30},
                .points = {
                    {11.0, -20.0},
                    {10.0, -21.0},
                    {11.0, -21.0},
                    {10.0, -20.0},
                },
            },
            sailroute::Isochrone{
                .time = departure + std::chrono::hours{1},
                .points = {{12.0, -22.0}},
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
    REQUIRE(
        serialized.value().find("\"completion\":\"destination_reached\"") !=
        std::string::npos);
    REQUIRE(serialized.value().find("forecast \\\"A\\\" & B") != std::string::npos);
    REQUIRE(serialized.value().find("\"points\":[") != std::string::npos);
    REQUIRE(serialized.value().find("\"expandedNodes\":10") != std::string::npos);
    REQUIRE(
        serialized.value().find("\"latticeDiagnostics\"") ==
        std::string::npos);
}

TEST_CASE("JSON conditionally includes lattice diagnostics") {
    auto route = sample_route();
    route.lattice_diagnostics = sailroute::LatticeRouteDiagnostics{
        .settled_labels = 11U,
        .queued_labels = 20U,
        .relaxed_labels = 19U,
        .wait_transitions = 3U,
        .refinement_runs = 2U,
        .accepted_refinements = 1U,
        .subdivision_level = 5U,
        .refinement_fallback = true,
    };
    const auto serialized = sailroute::route_to_json(route);
    REQUIRE(serialized.has_value());
    REQUIRE(
        serialized.value().find("\"latticeDiagnostics\"") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"settledLabels\":11") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"refinementFallback\":true") !=
        std::string::npos);
}

TEST_CASE("JSON omits details for unconfigured environment providers") {
    auto route = sample_route();
    route.environment = sailroute::RouteEnvironmentMetadata{};
    route.environment->current_provider =
        sailroute::ProviderMetadata{"current", "unit test", "1"};

    const auto serialized = sailroute::route_to_json(route);
    REQUIRE(serialized.has_value());
    REQUIRE(
        serialized.value().find("\"currentProvider\":{\"name\":\"current\"") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"landmask\":null") != std::string::npos);
    REQUIRE(
        serialized.value().find("\"exclusions\":null") != std::string::npos);
    REQUIRE(
        serialized.value().find("\"landResolutionNauticalMiles\"") ==
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"landInterpolationErrorNauticalMiles\"") ==
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"landClearanceNauticalMiles\"") ==
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"exclusionBoundaryPolicy\"") ==
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"exclusionZoneCount\"") ==
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"exclusionRevision\"") ==
        std::string::npos);
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
    REQUIRE(
        serialized.value().find(
            "<sailroute:completion>destination_reached"
            "</sailroute:completion>") != std::string::npos);
}

TEST_CASE("route serialization identifies forecast-exhausted partial results") {
    auto route = sample_route();
    route.completion = sailroute::RouteCompletion::forecast_exhausted;

    const auto json = sailroute::route_to_json(route);
    REQUIRE(json.has_value());
    REQUIRE(
        json.value().find("\"completion\":\"forecast_exhausted\"") !=
        std::string::npos);

    const auto gpx = sailroute::route_to_gpx(route);
    REQUIRE(gpx.has_value());
    REQUIRE(
        gpx.value().find(
            "<sailroute:completion>forecast_exhausted"
            "</sailroute:completion>") != std::string::npos);
}

TEST_CASE("isochrones serialize as timestamped GeoJSON lines") {
    const auto serialized = sailroute::isochrones_to_json(sample_route());
    REQUIRE(serialized.has_value());
    REQUIRE(
        serialized.value().find("\"type\":\"FeatureCollection\"") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"geometry\":{\"type\":\"LineString\"") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find("\"retainedPointCount\":4") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find(
            "[-21,10],[-20,10],[-20,11],[-21,11],[-21,10]") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find(
            "[-22,12],[-22,12]") !=
        std::string::npos);
}

TEST_CASE("isochrones serialize as separate timestamped GPX tracks") {
    const auto serialized = sailroute::isochrones_to_gpx(sample_route());
    REQUIRE(serialized.has_value());
    REQUIRE(serialized.value().find("forecast &quot;A&quot; &amp; B") !=
            std::string::npos);
    REQUIRE(
        serialized.value().find(
            "<name>Sailroute isochrone 2023-11-14T22:43:20Z</name>") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find(
            "<trkpt lat=\"12\" lon=\"-22\">") !=
        std::string::npos);
    REQUIRE(
        serialized.value().find(
            "<trkpt lat=\"12\" lon=\"-22\">",
            serialized.value().find(
                "<trkpt lat=\"12\" lon=\"-22\">") +
                1U) != std::string::npos);
}

TEST_CASE("isochrones serialize disconnected contours as multiple lines") {
    auto route = sample_route();
    route.isochrones = {
        sailroute::Isochrone{
            .time = route.departure_time + std::chrono::minutes{30},
            .points = {
                {0.0, 0.0},
                {0.0, 1.0},
                {1.0, 1.0},
                {1.0, 0.0},
                {0.0, 10.0},
                {0.0, 11.0},
                {1.0, 11.0},
                {1.0, 10.0},
            },
        },
    };

    const auto json = sailroute::isochrones_to_json(route);
    REQUIRE(json.has_value());
    REQUIRE(
        json.value().find("\"type\":\"MultiLineString\"") !=
        std::string::npos);

    const auto gpx = sailroute::isochrones_to_gpx(route);
    REQUIRE(gpx.has_value());
    const std::size_t first_segment = gpx.value().find("<trkseg>");
    REQUIRE(first_segment != std::string::npos);
    REQUIRE(
        gpx.value().find("<trkseg>", first_segment + 1U) !=
        std::string::npos);
}

TEST_CASE("isochrone serialization splits antimeridian contours") {
    auto route = sample_route();
    route.isochrones = {
        sailroute::Isochrone{
            .time = route.departure_time + std::chrono::minutes{30},
            .points = {
                {-1.0, 179.0},
                {-1.0, -179.0},
                {1.0, -179.0},
                {1.0, 179.0},
            },
        },
    };

    const auto json = sailroute::isochrones_to_json(route);
    REQUIRE(json.has_value());
    REQUIRE(
        json.value().find("\"type\":\"MultiLineString\"") !=
        std::string::npos);
    REQUIRE(json.value().find("[180,") != std::string::npos);
    REQUIRE(json.value().find("[-180,") != std::string::npos);
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

    route = sample_route();
    route.isochrones.front().points.front().longitude_degrees =
        std::numeric_limits<double>::quiet_NaN();
    const auto isochrones_json = sailroute::isochrones_to_json(route);
    REQUIRE(!isochrones_json.has_value());
    REQUIRE(
        isochrones_json.error().code ==
        sailroute::ErrorCode::output_error);

    route = sample_route();
    route.points.front().environment = sailroute::RoutePointEnvironment{};
    route.points.front().environment->current_east_knots =
        std::numeric_limits<double>::infinity();
    const auto current_json = sailroute::route_to_json(route);
    REQUIRE(!current_json.has_value());
    REQUIRE(
        current_json.error().message.find("current_east_knots") !=
        std::string::npos);
    const auto current_gpx = sailroute::route_to_gpx(route);
    REQUIRE(!current_gpx.has_value());
    REQUIRE(
        current_gpx.error().message.find("current_east_knots") !=
        std::string::npos);

    route = sample_route();
    route.environment = sailroute::RouteEnvironmentMetadata{};
    route.environment->landmask =
        sailroute::ProviderMetadata{"land", "unit test", "1"};
    route.environment->land_resolution_nautical_miles =
        std::numeric_limits<double>::infinity();
    const auto environment_json = sailroute::route_to_json(route);
    REQUIRE(!environment_json.has_value());
    REQUIRE(
        environment_json.error().message.find(
            "land_resolution_nautical_miles") != std::string::npos);
    const auto environment_gpx = sailroute::route_to_gpx(route);
    REQUIRE(!environment_gpx.has_value());
    REQUIRE(
        environment_gpx.error().message.find(
            "land_resolution_nautical_miles") != std::string::npos);
}
