#include "sailroute/router.hpp"
#include "sailroute/time.hpp"

#include "../benchmarks/strategic_benchmark.hpp"
#include "../src/routing/geodesy.hpp"
#include "../src/routing/transition.hpp"
#include "test_support.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using sailroute::Coordinate;
using sailroute::RoutePoint;
using sailroute::RouteRequest;
using sailroute::RouteResult;
using sailroute::TimePoint;
using sailroute::detail::great_circle_distance_nautical_miles;

RouteResult optimize(
    const sailroute::Router& router, const RouteRequest& request) {
    auto result = router.optimize(request);
    if (!result) {
        throw std::runtime_error(
            std::string{sailroute::to_string(result.error().code)} + ": " +
            result.error().message);
    }
    return std::move(result.value());
}

struct ReferenceRoute {
    std::vector<RoutePoint> points;
    std::size_t evaluated_actions{};
    std::vector<sailroute::SailingAction> actions;
};

// Find the first arrival-circle entry independently of the production wrapper's
// shortening/reintegration loop. Every crossing occurs in the constant 32 kt
// corridor, so the full and shortened midpoint solutions have identical speed.
bool append_until_arrival(
    const sailroute::WeatherDataset& weather, const RouteRequest& request,
    const sailroute::detail::VariableTransition& transition,
    std::vector<RoutePoint>& points) {
    auto vertices = transition.intermediate_points;
    vertices.push_back(transition.point);
    for (const auto& vertex : vertices) {
        const auto& parent = points.back();
        const double length = vertex.cumulative_distance_nautical_miles -
            parent.cumulative_distance_nautical_miles;
        const auto fraction = sailroute::detail::arrival_fraction(
            sailroute::detail::prepare_origin(parent.position),
            great_circle_distance_nautical_miles(parent.position, request.destination),
            sailroute::detail::initial_bearing_degrees(parent.position, request.destination),
            vertex.heading_degrees, length, request.destination,
            request.options.arrival_radius_nautical_miles);
        if (fraction) {
            const auto start_wind = weather.interpolate(parent.position, parent.time);
            const auto end_wind = weather.interpolate(vertex.position, vertex.time);
            REQUIRE(start_wind.has_value());
            REQUIRE(end_wind.has_value());
            REQUIRE_NEAR(start_wind.value().speed_knots(), 32.0, 1e-4);
            REQUIRE_NEAR(end_wind.value().speed_knots(), 32.0, 1e-4);
            RoutePoint arrival = vertex;
            arrival.position = sailroute::detail::destination_point(
                parent.position, vertex.heading_degrees, length * *fraction);
            arrival.time = parent.time + std::chrono::seconds{
                static_cast<std::chrono::seconds::rep>(std::llround(
                    static_cast<double>((vertex.time - parent.time).count()) * *fraction))};
            arrival.cumulative_distance_nautical_miles =
                parent.cumulative_distance_nautical_miles + length * *fraction;
            points.push_back(arrival);
            return true;
        }
        points.push_back(vertex);
    }
    return false;
}

// Exhaustive enumeration over the same fixed-duration action grid and arrival
// circle, with NO position merging, heading diversity or beam pruning. This is
// a search reference, not an independent continuous-physics optimum.
ReferenceRoute exhaustive_reference(
    const sailroute::WeatherDataset& weather,
    const sailroute::VesselPolar& polar,
    const RouteRequest& request,
    const sailroute::RoutingEnvironment& environment) {
    struct State {
        std::vector<RoutePoint> points;
        sailroute::detail::OperationalConfiguration configuration;
        std::vector<sailroute::SailingAction> actions;
    };
    RoutePoint departure;
    departure.position = request.start;
    departure.time = *request.departure_time;
    std::vector<State> frontier{{{departure}, {}, {}}};
    ReferenceRoute result;
    sailroute::EnvironmentDiagnostics diagnostics;
    const TimePoint end =
        departure.time + request.options.maximum_route_duration;
    for (std::size_t layer = 0U; layer < 18U; ++layer) {
        std::vector<State> next;
        std::optional<State> winner;
        for (const State& parent : frontier) {
            for (const double heading : {45.0, 135.0}) {
                ++result.evaluated_actions;
                const auto transition = sailroute::detail::evaluate_heading_transition(
                    weather, polar, request.options, environment, diagnostics,
                    parent.points.back(), parent.configuration, heading,
                    std::min(end, parent.points.back().time + request.options.time_step));
                if (!transition) {
                    throw std::runtime_error(transition.error().message);
                }
                if (!transition.value()) {
                    continue;
                }
                const auto& leg = *transition.value();
                State child{parent.points, leg.configuration, parent.actions};
                child.actions.push_back({heading, request.options.time_step});
                if (append_until_arrival(weather, request, leg, child.points)) {
                    if (!winner || child.points.back().time < winner->points.back().time) {
                        winner = std::move(child);
                    }
                } else {
                    next.push_back(std::move(child));
                }
            }
        }
        if (winner) {
            result.points = std::move(winner->points);
            result.actions = std::move(winner->actions);
            return result;
        }
        frontier = std::move(next);
        REQUIRE(result.evaluated_actions < 200'000U);
    }
    throw std::runtime_error("strategic exhaustive reference did not arrive");
}

const RoutePoint& point_at(const std::vector<RoutePoint>& route, TimePoint time) {
    const auto found = std::find_if(route.begin(), route.end(),
        [time](const RoutePoint& point) { return point.time == time; });
    if (found == route.end()) {
        throw std::runtime_error("missing strategic route layer");
    }
    return *found;
}

void require_multi_step_sacrifice(
    const std::vector<RoutePoint>& route, const RouteResult& greedy,
    const RouteRequest& request, double reflection, bool with_island) {
    unsigned consecutive_losing_layers = 0U;
    unsigned longest_losing_run = 0U;
    for (unsigned hour = 1U; hour <= 5U; ++hour) {
        const auto time = *request.departure_time + std::chrono::hours{hour};
        const auto& actual = point_at(route, time);
        const auto& local = point_at(greedy.points, time);
        const double remaining =
            great_circle_distance_nautical_miles(actual.position, request.destination);
        if (remaining >
            great_circle_distance_nautical_miles(local.position, request.destination) + 1.0) {
            longest_losing_run = std::max(longest_losing_run, ++consecutive_losing_layers);
        } else {
            consecutive_losing_layers = 0U;
        }
        REQUIRE_NEAR(actual.boat_speed_knots, 6.0, 1e-4);
        REQUIRE_NEAR(local.boat_speed_knots, 6.0, 1e-4);
    }
    const bool initially_losing_tack = reflection * point_at(
        route, *request.departure_time + std::chrono::hours{1})
            .position.latitude_degrees < -0.04;
    REQUIRE(longest_losing_run >= 3U);
    if (with_island) {
        // In open water, equal-speed early tacks can commute. The island
        // separates those histories and requires the losing first tack.
        REQUIRE(initially_losing_tack);
    }
}

void check_delayed_corridor(double reflection, bool with_island = false) {
    const auto scenario = sailroute::benchmarks::make_strategic_scenario(
        reflection < 0.0, with_island);
    const auto& weather = scenario.weather;
    const auto& polar = scenario.polar;
    const auto& environment = scenario.environment;
    const sailroute::Router router(weather, polar, environment);
    const auto& request = scenario.request;
    const auto strategic = optimize(router, request);
    auto greedy_request = request;
    greedy_request.options.strategic_retention = false;
    greedy_request.options.max_nodes_per_bucket = 1U;
    const auto greedy = optimize(router, greedy_request);
    const auto reference = exhaustive_reference(weather, polar, request, environment);
    std::cout << "strategic corridor reflection=" << reflection
              << " island=" << with_island
              << " strategic_s=" << (strategic.arrival_time - *request.departure_time).count()
              << " greedy_s=" << (greedy.arrival_time - *request.departure_time).count()
              << " reference_s=" << (reference.points.back().time - *request.departure_time).count()
              << " reference_actions=" << reference.evaluated_actions << '\n';
    REQUIRE(reference.evaluated_actions <= 1024U);
    REQUIRE(reference.points.back().time + std::chrono::minutes{30} < greedy.arrival_time);
    require_multi_step_sacrifice(reference.points, greedy, request, reflection, with_island);
    REQUIRE(strategic.completion == sailroute::RouteCompletion::destination_reached);
    REQUIRE(greedy.completion == sailroute::RouteCompletion::destination_reached);
    REQUIRE(strategic.arrival_time + std::chrono::minutes{30} < greedy.arrival_time);
    REQUIRE(std::abs((strategic.arrival_time - reference.points.back().time).count()) <= 2);
    require_multi_step_sacrifice(strategic.points, greedy, request, reflection, with_island);
    if (with_island) {
        REQUIRE(strategic.environment_diagnostics.has_value());
        REQUIRE(strategic.environment_diagnostics->exclusion_rejections > 0U);
        for (std::size_t index = 1U; index < strategic.points.size(); ++index) {
            REQUIRE(!environment.exclusions.zones->intersects_segment(
                strategic.points[index - 1U].position, strategic.points[index - 1U].time,
                strategic.points[index].position, strategic.points[index].time,
                sailroute::ExclusionBoundaryPolicy::boundary_excluded).violated);
        }
    }
}

}  // namespace

TEST_CASE("strategic delayed southern wind corridor accepts several locally losing steps") {
    check_delayed_corridor(1.0);
}

TEST_CASE("strategic delayed northern wind corridor accepts several locally losing steps") {
    check_delayed_corridor(-1.0);
}

TEST_CASE("strategic island detour preserves the losing tack for delayed southern weather") {
    check_delayed_corridor(1.0, true);
}

TEST_CASE("strategic island detour preserves the losing tack for delayed northern weather") {
    check_delayed_corridor(-1.0, true);
}

TEST_CASE("strategic fixture has no immediate wind advantage for five hours") {
    const auto scenario = sailroute::benchmarks::make_strategic_scenario();
    const auto& weather = scenario.weather;
    const auto& request = scenario.request;
    const auto departure = *request.departure_time;
    for (const unsigned hour : {0U, 3U, 5U}) {
        const auto sampler = weather.sampler_at(departure + std::chrono::hours{hour});
        REQUIRE(sampler.has_value());
        for (const Coordinate position : {
                 Coordinate{0.25, 0.25}, Coordinate{-0.25, 0.25},
                 Coordinate{-0.50, 0.50}, request.destination}) {
            const auto wind = sampler.value().sample(position);
            REQUIRE(wind.has_value());
            REQUIRE_NEAR(wind.value().speed_knots(), 8.0, 1e-5);
            REQUIRE_NEAR(wind.value().direction_from_degrees(), 90.0, 1e-5);
        }
    }

}

TEST_CASE("strategic competing histories share a bucket heading and board before payoff") {
    const auto scenario = sailroute::benchmarks::make_strategic_scenario();
    const auto& weather = scenario.weather;
    const auto& polar = scenario.polar;
    const auto& request = scenario.request;
    RoutePoint start;
    start.position = request.start;
    start.time = *request.departure_time;
    sailroute::EnvironmentDiagnostics diagnostics;
    std::vector<sailroute::detail::VariableTransition> competitors;
    for (const double first_heading : {45.0, 135.0}) {
        const auto first = sailroute::detail::evaluate_heading_transition(
            weather, polar, request.options, {}, diagnostics, start, {}, first_heading,
            start.time + request.options.time_step);
        REQUIRE(first.has_value());
        REQUIRE(first.value().has_value());
        const auto second = sailroute::detail::evaluate_heading_transition(
            weather, polar, request.options, {}, diagnostics, first.value()->point,
            first.value()->configuration, 45.0,
            start.time + 2 * request.options.time_step);
        REQUIRE(second.has_value());
        REQUIRE(second.value().has_value());
        competitors.push_back(*second.value());
    }
    const auto& locally_leading = competitors[0];
    const auto& locally_losing = competitors[1];
    REQUIRE(locally_leading.point.heading_degrees == locally_losing.point.heading_degrees);
    REQUIRE(locally_leading.configuration == locally_losing.configuration);
    REQUIRE(great_circle_distance_nautical_miles(
        locally_leading.point.position, locally_losing.point.position) > 8.0);
    const auto bucket = [&](Coordinate position) {
        const double mean_latitude = (position.latitude_degrees +
            request.destination.latitude_degrees) * std::numbers::pi / 360.0;
        return std::array{
            std::floor((position.longitude_degrees -
                request.destination.longitude_degrees) * 60.0 * std::cos(mean_latitude) /
                request.options.spatial_bucket_nautical_miles),
            std::floor((position.latitude_degrees -
                request.destination.latitude_degrees) * 60.0 /
                request.options.spatial_bucket_nautical_miles)};
    };
    REQUIRE(bucket(locally_leading.point.position) == bucket(locally_losing.point.position));
    REQUIRE(great_circle_distance_nautical_miles(
        locally_leading.point.position, request.destination) <
        great_circle_distance_nautical_miles(locally_losing.point.position, request.destination));
    for (const auto& competitor : competitors) {
        const auto probe = weather.interpolate(
            competitor.point.position, competitor.point.time + std::chrono::hours{3});
        REQUIRE(probe.has_value());
        REQUIRE_NEAR(probe.value().speed_knots(), 8.0, 1e-5);
    }
}

TEST_CASE("strategic shared arrival does not discard an earlier constant-wind circle entry") {
    const auto scenario = sailroute::benchmarks::make_strategic_scenario();
    const auto& weather = scenario.weather;
    const auto& polar = scenario.polar;
    const auto& request = scenario.request;
    RoutePoint parent;
    parent.position = request.start;
    parent.time = *request.departure_time;
    sailroute::detail::OperationalConfiguration configuration;
    sailroute::EnvironmentDiagnostics diagnostics;
    for (const double heading : {135.0, 135.0, 135.0, 45.0, 45.0, 135.0, 45.0}) {
        const auto leg = sailroute::detail::evaluate_heading_transition(
            weather, polar, request.options, {}, diagnostics, parent, configuration,
            heading, parent.time + request.options.time_step);
        REQUIRE(leg.has_value());
        REQUIRE(leg.value().has_value());
        parent = leg.value()->point;
        configuration = leg.value()->configuration;
    }
    const auto full = sailroute::detail::evaluate_heading_transition(
        weather, polar, request.options, {}, diagnostics, parent, configuration,
        45.0, parent.time + request.options.time_step);
    REQUIRE(full.has_value());
    REQUIRE(full.value().has_value());
    std::vector<RoutePoint> reference{parent};
    REQUIRE(append_until_arrival(weather, request, *full.value(), reference));
    REQUIRE(reference.back().time > parent.time);
    REQUIRE(reference.back().time < parent.time + request.options.time_step);
    const auto shortened = sailroute::detail::evaluate_heading_transition(
        weather, polar, request.options, {}, diagnostics, parent, configuration,
        45.0, parent.time + request.options.time_step,
        request.destination, request.options.arrival_radius_nautical_miles);
    REQUIRE(shortened.has_value());
    REQUIRE(shortened.value().has_value());
    REQUIRE(std::abs((shortened.value()->point.time - reference.back().time).count()) <= 1);
    REQUIRE(great_circle_distance_nautical_miles(
        shortened.value()->point.position, request.destination) <=
        request.options.arrival_radius_nautical_miles + 1e-5);
}

TEST_CASE("strategic timed action replay evaluates the exhaustive winning detour") {
    const auto scenario = sailroute::benchmarks::make_strategic_scenario(false, true);
    const auto reference = exhaustive_reference(
        scenario.weather, scenario.polar, scenario.request, scenario.environment);
    REQUIRE(!reference.actions.empty());
    std::chrono::seconds full_duration{};
    for (const auto& action : reference.actions) {
        REQUIRE(action.duration == std::chrono::hours{1});
        full_duration += action.duration;
    }
    REQUIRE(*scenario.request.departure_time + full_duration > reference.points.back().time);
    const sailroute::Router router(scenario.weather, scenario.polar, scenario.environment);
    const auto replay = router.evaluate_actions(scenario.request, reference.actions);
    if (!replay) {
        throw std::runtime_error("exhaustive strategic action replay: " + replay.error().message);
    }
    REQUIRE(replay.value().completion == sailroute::RouteCompletion::destination_reached);
    REQUIRE(std::abs((replay.value().arrival_time - reference.points.back().time).count()) <= 1);
    REQUIRE(replay.value().diagnostics.expanded_nodes == 0U);
    REQUIRE(replay.value().diagnostics.time_steps == 0U);
}

TEST_CASE("strategic timed action replay rejects invalid actions") {
    const auto scenario = sailroute::benchmarks::make_strategic_scenario();
    const sailroute::Router router(scenario.weather, scenario.polar);
    const auto empty = router.evaluate_actions(scenario.request, {});
    REQUIRE(!empty.has_value());
    REQUIRE(empty.error().code == sailroute::ErrorCode::invalid_argument);
    const std::array invalid{
        sailroute::SailingAction{45.0, std::chrono::seconds{0}},
        sailroute::SailingAction{45.0, std::chrono::seconds{-1}},
        sailroute::SailingAction{45.0, std::chrono::hours{19}},
        sailroute::SailingAction{std::numeric_limits<double>::quiet_NaN(), std::chrono::hours{1}},
        sailroute::SailingAction{std::numeric_limits<double>::infinity(), std::chrono::hours{1}}};
    for (const auto& action : invalid) {
        const auto result = router.evaluate_actions(
            scenario.request, std::span<const sailroute::SailingAction>{&action, 1U});
        REQUIRE(!result.has_value());
        REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
    }
}

TEST_CASE("strategic constant-wind integration matches the analytic spherical solution") {
    const auto scenario = sailroute::benchmarks::make_strategic_scenario();
    const auto& weather = scenario.weather;
    const auto& polar = scenario.polar;
    const auto& request = scenario.request;
    const auto departure = *request.departure_time;
    const auto future = weather.sampler_at(departure + std::chrono::hours{6});
    REQUIRE(future.has_value());
    REQUIRE_NEAR(future.value().sample({-0.5, 0.5}).value().speed_knots(), 32.0, 1e-5);
    REQUIRE_NEAR(future.value().sample({0.5, 0.5}).value().speed_knots(), 8.0, 1e-5);
    REQUIRE_NEAR(polar.boat_speed_knots(8.0, 45.0), 6.0, 1e-10);
    REQUIRE_NEAR(polar.boat_speed_knots(8.0, 135.0), 6.0, 1e-10);

    RoutePoint start;
    start.position = request.start;
    start.time = departure;
    sailroute::EnvironmentDiagnostics diagnostics;
    constexpr double radians = std::numbers::pi / 180.0;
    // Independent great-circle solution for 1.5 nm from the equator at 45°.
    const double angular_distance = 1.5 / 3440.065;
    const double expected_latitude =
        std::asin(std::sin(angular_distance) / std::sqrt(2.0)) / radians;
    const double expected_longitude = std::atan2(
        std::sin(angular_distance) / std::sqrt(2.0),
        std::cos(angular_distance)) / radians;
    for (const double heading : {45.0, 135.0}) {
        const auto leg = sailroute::detail::evaluate_heading_transition(
            weather, polar, request.options, {}, diagnostics, start, {}, heading,
            departure + std::chrono::minutes{15});
        REQUIRE(leg.has_value());
        REQUIRE(leg.value().has_value());
        const auto& point = leg.value()->point;
        REQUIRE_NEAR(point.position.latitude_degrees,
                     heading == 45.0 ? expected_latitude : -expected_latitude, 1e-7);
        REQUIRE_NEAR(point.position.longitude_degrees, expected_longitude, 1e-7);
        REQUIRE_NEAR(point.cumulative_distance_nautical_miles, 1.5, 1e-5);
    }
}
