#include "sailroute/router.hpp"
#include "sailroute/serialization.hpp"
#include "routing/environment_context.hpp"
#include "routing/geodesy.hpp"
#include "routing/transition.hpp"
#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <array>

using namespace sailroute;
using namespace std::chrono_literals;

namespace {
RoutePoint start_point(const WeatherDataset& weather, Coordinate position = {1.0, 0.5}) {
    const auto time = weather.metadata().first_valid_time;
    const auto wind = weather.interpolate(position, time);
    REQUIRE(wind.has_value());
    return {position, time, 90.0, 0.0, wind.value().speed_knots(),
            wind.value().direction_from_degrees(), 0.0, std::nullopt};
}

RouteRequest request_for(const WeatherDataset& weather) {
    RouteRequest request;
    request.start = {1.0, 0.5};
    request.destination = {1.0, 1.0};
    request.departure_time = weather.metadata().first_valid_time;
    request.options.worker_count = 1;
    request.options.lattice.refinement_levels = 0;
    return request;
}
}

TEST_CASE("cruising contracts integrate long legs into covered bounded segments") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    RoutingOptions options;
    options.maximum_integration_step = 5min;
    EnvironmentDiagnostics diagnostics;
    const auto parent = start_point(weather.value());
    const auto result = detail::evaluate_variable_transition(
        weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, {},
        diagnostics, parent, {}, {1.0, 1.0}, parent.time + 12h);
    REQUIRE(result && result.value());
    REQUIRE(!result.value()->intermediate_points.empty());
    TimePoint previous = parent.time;
    for (const auto& point : result.value()->intermediate_points) {
        REQUIRE(point.time > previous);
        REQUIRE(point.time - previous <= 5min);
        REQUIRE(weather.value().interpolate(point.position, point.time).has_value());
        previous = point.time;
    }
    REQUIRE(result.value()->point.time - previous <= 5min);
}

TEST_CASE("cruising contracts reject endpoints outside forecast coverage") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto parent = start_point(weather.value(), {1.0, 1.99});
    EnvironmentDiagnostics diagnostics;
    detail::VariableTransitionRejection reason{};
    auto route = detail::evaluate_heading_transition(
        weather.value(), VesselPolar::default_racer_cruiser_45ft(), {}, {},
        diagnostics, parent, {}, 90.0, parent.time + 1h, std::nullopt, 0.0, &reason);
    REQUIRE(route && !route.value());
    REQUIRE(reason == detail::VariableTransitionRejection::missing_data);
}

TEST_CASE("cruising contracts never restore speed after a forbidden midpoint") {
    test::ConstantWindGribFixture::Options fixture_options;
    fixture_options.north_metres_per_second = -5.0;
    fixture_options.final_north_metres_per_second = -25.0;
    fixture_options.final_forecast_hour = 1;
    const test::ConstantWindGribFixture fixture{fixture_options};
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto parent = start_point(weather.value());
    RoutingOptions options;
    options.maximum_true_wind_speed_knots = 15.0;
    EnvironmentDiagnostics diagnostics;
    auto route = detail::evaluate_heading_transition_step(
        weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, {},
        diagnostics, parent, {}, 90.0, parent.time + 30min);
    REQUIRE(route && !route.value());
}

TEST_CASE("cruising contracts require permission and eligibility for holding") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto parent = start_point(weather.value());
    RoutingOptions options;
    EnvironmentDiagnostics diagnostics;
    auto wait = detail::evaluate_wait_transition(
        weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, {},
        diagnostics, parent, {}, parent.time + 30min);
    REQUIRE(wait && !wait.value());
    options.holding_eligibility = [](const RouteSegmentView&) { return true; };
    options.segment_eligibility = [](const RouteSegmentView&) { return false; };
    wait = detail::evaluate_wait_transition(
        weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, {},
        diagnostics, parent, {}, parent.time + 30min);
    REQUIRE(wait && !wait.value());
    options.segment_eligibility = {};
    wait = detail::evaluate_wait_transition(
        weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, {},
        diagnostics, parent, {}, parent.time + 30min);
    REQUIRE(wait && wait.value());
    REQUIRE(wait.value()->point.time == parent.time + 30min);
}

TEST_CASE("cruising contracts preserve a long maneuver as explicit current drift") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    auto parent = start_point(weather.value());
    parent.heading_degrees = 315.0;
    RoutingOptions options;
    options.maximum_integration_step = 5min;
    options.maneuver.tack_penalty = 20min;
    auto current = make_uniform_current_provider({0.5, 0.0}, {"test", "synthetic", "1"});
    REQUIRE(current.has_value());
    RoutingEnvironment environment;
    environment.currents.provider = current.value();
    EnvironmentDiagnostics diagnostics;
    const auto route = detail::evaluate_heading_transition(
        weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, environment,
        diagnostics, parent, {-1}, 50.0, parent.time + 1h);
    REQUIRE(route && route.value());
    bool found_end = false;
    for (const auto& point : route.value()->intermediate_points) {
        if (point.time == parent.time + 20min) {
            REQUIRE(point.boat_speed_knots == 0.0);
            REQUIRE_NEAR(point.cumulative_distance_nautical_miles, 0.5 / 3.0, 1.0e-8);
            found_end = true;
        }
    }
    REQUIRE(found_end);
    REQUIRE(route.value()->point.time == parent.time + 1h);
}

TEST_CASE("cruising contracts reject an immediate arrival on configured land") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    auto mask = SignedDistanceLandmask::create(
        {0.0, 0.0, 1.0, 1.0, 3, 3, false}, std::vector<double>(9, -1.0),
        {{"test", "synthetic", "1"}, 60.0, 0.0});
    REQUIRE(mask.has_value());
    RoutingEnvironment environment;
    environment.land.landmask = mask.value();
    Router router{weather.value(), VesselPolar::default_racer_cruiser_45ft(), environment};
    auto request = request_for(weather.value());
    request.destination = request.start;
    const auto result = router.optimize(request);
    REQUIRE(!result);
    REQUIRE(result.error().code == ErrorCode::no_route);
}

TEST_CASE("cruising contracts replay performance scaling without route search") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    Router router{weather.value()};
    auto request = request_for(weather.value());
    const std::array waypoints{request.destination};
    const auto reference = router.evaluate_route(request, waypoints);
    REQUIRE(reference.has_value());
    request.options.boat_speed_factor = 0.8;
    const auto reduced = router.evaluate_route(request, waypoints);
    REQUIRE(reduced.has_value());
    const double ratio = std::chrono::duration<double>(
        reduced.value().arrival_time - reduced.value().departure_time).count() /
        std::chrono::duration<double>(
            reference.value().arrival_time - reference.value().departure_time).count();
    REQUIRE_NEAR(ratio, 1.25, 0.005);
    REQUIRE(reduced.value().run.has_value());
    REQUIRE(reduced.value().run->boat_speed_factor == 0.8);
}

TEST_CASE("cruising contracts never silently choose a historical departure") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    auto request = request_for(weather.value());
    request.departure_time.reset();
    const auto route = Router{weather.value()}.optimize(request);
    REQUIRE(!route);
    REQUIRE(route.error().code == ErrorCode::departure_outside_forecast);
}

TEST_CASE("cruising contracts bound search work explicitly") {
    const test::ConstantWindGribFixture fixture;
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    auto request = request_for(weather.value());
    request.options.maximum_generated_candidates = 1;
    const auto route = Router{weather.value()}.optimize(request);
    REQUIRE(!route);
    REQUIRE(route.error().code == ErrorCode::resource_limit);
}

TEST_CASE("cruising contracts report the actual midpoint wind used by the polar") {
    test::ConstantWindGribFixture::Options fixture_options;
    fixture_options.north_metres_per_second = -5.0;
    fixture_options.final_north_metres_per_second = -15.0;
    const test::ConstantWindGribFixture fixture{fixture_options};
    const auto weather = WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto polar = VesselPolar::default_racer_cruiser_45ft();
    auto request = request_for(weather.value());
    request.options.boat_speed_factor = 0.9;
    const auto route = Router{weather.value(), polar}.optimize(request);
    REQUIRE(route.has_value());
    for (std::size_t index = 1; index < route.value().points.size(); ++index) {
        const auto& point = route.value().points[index];
        REQUIRE_NEAR(point.boat_speed_knots,
            polar.boat_speed_knots(point.true_wind_speed_knots,
                detail::angular_difference_degrees(
                    point.heading_degrees, point.true_wind_direction_degrees)) *
                request.options.boat_speed_factor, 1.0e-10);
    }

}

TEST_CASE("cruising contracts do not sample beyond an imminent regional arrival") {
        const auto weather = WeatherDataset::load(
            std::filesystem::path{SAILROUTE_SOURCE_DIR} / "samples/sample.grib",
            GeographicBounds{48.0, -123.75, 48.5, -123.25});
        const auto polar = VesselPolar::load(
            std::filesystem::path{SAILROUTE_SOURCE_DIR} / "samples/sample.pol");
        REQUIRE(weather.has_value());
        REQUIRE(polar.has_value());
        RouteRequest request;
        request.start = {48.25, -123.253};
        request.destination = {48.25, -123.2501};
        request.departure_time = weather.value().metadata().first_valid_time + 19min + 1s;
        request.options.maximum_route_duration = 1h;
        request.options.worker_count = 1;
        const auto normal = Router{weather.value(), polar.value()}.optimize(request);
        REQUIRE(normal.has_value());
        REQUIRE(normal.value().completion == RouteCompletion::destination_reached);
        REQUIRE(normal.value().arrival_time - *request.departure_time < 2min);
        request.options.maximum_integration_step = 1min;
        const auto fine = Router{weather.value(), polar.value()}.optimize(request);
        REQUIRE(fine.has_value());
        REQUIRE(std::abs((normal.value().arrival_time - fine.value().arrival_time).count()) <= 2);
    }

TEST_CASE("cruising contracts defer destination exclusion legality until actual arrival") {
        const test::ConstantWindGribFixture fixture;
        const auto weather = WeatherDataset::load(fixture.path());
        REQUIRE(weather.has_value());
        auto request = request_for(weather.value());
        ExclusionZone zone;
        zone.identifier = "temporary-destination";
        zone.source = "synthetic";
        zone.active_until = *request.departure_time + 1h;
        zone.polygons.push_back({{{{0.95, 0.95}, {0.95, 1.05}, {1.05, 1.05}, {1.05, 0.95}}}, {}});
        auto zones = ExclusionZoneSet::create({zone}, {"test", "synthetic", "1"});
        REQUIRE(zones.has_value());
        RoutingEnvironment environment;
        environment.exclusions.zones = zones.value();
        Router router{weather.value(), VesselPolar::default_racer_cruiser_45ft(), environment};
        const std::array waypoints{request.destination};
        const auto route = router.evaluate_route(request, waypoints);
        REQUIRE(route.has_value());
        REQUIRE(route.value().arrival_time > *zone.active_until);
        request.start = request.destination;
        const auto immediate = router.optimize(request);
        REQUIRE(!immediate.has_value());
    }

TEST_CASE("cruising contracts reject only a maneuver that drifts outside coverage") {
        const test::ConstantWindGribFixture fixture;
        const auto weather = WeatherDataset::load(fixture.path());
        REQUIRE(weather.has_value());
        auto parent = start_point(weather.value(), {1.0, 1.999});
        parent.heading_degrees = 315.0;
        auto current = make_uniform_current_provider({1.0, 0.0}, {"test", "synthetic", "1"});
        REQUIRE(current.has_value());
        RoutingEnvironment environment;
        environment.currents.provider = current.value();
        RoutingOptions options;
        options.maneuver.tack_penalty = 20min;
        options.maximum_integration_step = 5min;
        EnvironmentDiagnostics diagnostics;
        detail::VariableTransitionRejection rejection{};
        const auto tack = detail::evaluate_heading_transition(
            weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, environment,
            diagnostics, parent, {-1}, 55.0, parent.time + 1h, std::nullopt, 0.0, &rejection);
        REQUIRE(tack.has_value());
        REQUIRE(!tack.value());
        REQUIRE(rejection == detail::VariableTransitionRejection::missing_data);
        const auto away = detail::evaluate_heading_transition(
            weather.value(), VesselPolar::default_racer_cruiser_45ft(), options, environment,
            diagnostics, parent, {-1}, 300.0, parent.time + 1h);
        REQUIRE(away && away.value());
    }

TEST_CASE("cruising contracts enforce replay budgets before appending action vertices") {
        const test::ConstantWindGribFixture fixture;
        const auto weather = WeatherDataset::load(fixture.path());
        REQUIRE(weather.has_value());
        Router router{weather.value()};
        auto request = request_for(weather.value());
        request.options.maximum_retained_nodes = 2;
        const std::array actions{SailingAction{90.0, 8h}};
        const auto bounded = router.evaluate_actions(request, actions);
        REQUIRE(!bounded.has_value());
        REQUIRE(bounded.error().code == ErrorCode::resource_limit);
        request.options.maximum_retained_nodes = 1000;
        request.options.maximum_generated_candidates = 1;
        const std::array waypoints{Coordinate{1.0, 0.75}, request.destination};
        const auto legs = router.evaluate_route(request, waypoints);
        REQUIRE(!legs.has_value());
        REQUIRE(legs.error().code == ErrorCode::resource_limit);
}
