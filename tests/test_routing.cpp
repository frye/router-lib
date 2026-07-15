#include "sailroute/router.hpp"
#include "sailroute/time.hpp"

#include "../src/routing/geodesy.hpp"
#include "../src/routing/intervals.hpp"
#include "test_support.hpp"

#include <eccodes.h>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

void require_codes(int status, const char* operation) {
    if (status != CODES_SUCCESS) {
        throw std::runtime_error(
            std::string{operation} + ": " + codes_get_error_message(status));
    }
}

class RoutingGribFixture {
public:
    explicit RoutingGribFixture(long data_date = 20260714, long data_time = 1200)
        : path_(
              std::filesystem::current_path() /
              ("sailroute-routing-" +
               std::to_string(
                   std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".grib")) {
        write_message("10u", 0, 0.0, data_date, data_time, "w");
        write_message("10v", 0, -10.0, data_date, data_time, "a");
        write_message("10u", 12, 0.0, data_date, data_time, "a");
        write_message("10v", 12, -10.0, data_date, data_time, "a");
    }

    ~RoutingGribFixture() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    void write_message(
        const char* short_name,
        long forecast_hour,
        double value,
        long data_date,
        long data_time,
        const char* mode) {
        codes_handle* handle =
            codes_grib_handle_new_from_samples(nullptr, "regular_ll_sfc_grib2");
        if (handle == nullptr) {
            throw std::runtime_error("unable to create routing GRIB fixture");
        }

        try {
            require_codes(codes_set_long(handle, "Ni", 3), "set Ni");
            require_codes(codes_set_long(handle, "Nj", 3), "set Nj");
            require_codes(
                codes_set_double(
                    handle,
                    "latitudeOfFirstGridPointInDegrees",
                    2.0),
                "set first latitude");
            require_codes(
                codes_set_double(
                    handle,
                    "longitudeOfFirstGridPointInDegrees",
                    0.0),
                "set first longitude");
            require_codes(
                codes_set_double(
                    handle,
                    "latitudeOfLastGridPointInDegrees",
                    0.0),
                "set last latitude");
            require_codes(
                codes_set_double(
                    handle,
                    "longitudeOfLastGridPointInDegrees",
                    2.0),
                "set last longitude");
            require_codes(
                codes_set_double(handle, "iDirectionIncrementInDegrees", 1.0),
                "set longitude increment");
            require_codes(
                codes_set_double(handle, "jDirectionIncrementInDegrees", 1.0),
                "set latitude increment");
            require_codes(codes_set_long(handle, "dataDate", data_date), "set date");
            require_codes(codes_set_long(handle, "dataTime", data_time), "set time");
            require_codes(
                codes_set_long(handle, "forecastTime", forecast_hour),
                "set forecast time");
            std::size_t short_name_size =
                std::char_traits<char>::length(short_name);
            require_codes(
                codes_set_string(
                    handle,
                    "shortName",
                    short_name,
                    &short_name_size),
                "set wind component");
            require_codes(codes_set_long(handle, "level", 10), "set wind level");
            const std::array<double, 9> values{
                value, value, value,
                value, value, value,
                value, value, value};
            require_codes(
                codes_set_double_array(
                    handle,
                    "values",
                    values.data(),
                    values.size()),
                "set wind values");
            require_codes(
                codes_write_message(handle, path_.string().c_str(), mode),
                "write routing GRIB fixture");
        } catch (...) {
            codes_handle_delete(handle);
            throw;
        }
        codes_handle_delete(handle);
    }

    std::filesystem::path path_;
};

sailroute::RouteResult route_with_workers(
    const sailroute::Router& router,
    std::size_t worker_count,
    bool capture_isochrones = true,
    sailroute::RoutingProgressCallback on_progress = {}) {
    const auto departure = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    REQUIRE(departure.has_value());

    sailroute::RouteRequest request;
    request.start = {1.0, 0.5};
    request.destination = {1.0, 1.0};
    request.departure_time = departure.value();
    request.options.time_step = std::chrono::minutes{30};
    request.options.use_routing_intervals = false;
    request.options.heading_step_degrees = 10.0;
    request.options.arrival_radius_nautical_miles = 0.5;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 3;
    request.options.worker_count = worker_count;
    request.options.maximum_route_duration = std::chrono::hours{12};
    request.options.capture_isochrones = capture_isochrones;

    auto result = router.optimize(request, on_progress);
    if (!result.has_value()) {
        throw std::runtime_error(result.error().message);
    }
    return std::move(result.value());
}

void require_same_route(
    const sailroute::RouteResult& left,
    const sailroute::RouteResult& right) {
    REQUIRE(left.departure_time == right.departure_time);
    REQUIRE(left.arrival_time == right.arrival_time);
    REQUIRE(left.departure_source == right.departure_source);
    REQUIRE(left.forecast_source == right.forecast_source);
    REQUIRE(left.polar_source == right.polar_source);
    REQUIRE(
        left.diagnostics.expanded_nodes ==
        right.diagnostics.expanded_nodes);
    REQUIRE(
        left.diagnostics.generated_candidates ==
        right.diagnostics.generated_candidates);
    REQUIRE(
        left.diagnostics.retained_candidates ==
        right.diagnostics.retained_candidates);
    REQUIRE(left.diagnostics.time_steps == right.diagnostics.time_steps);
    REQUIRE(left.points.size() == right.points.size());
    for (std::size_t index = 0U; index < left.points.size(); ++index) {
        const sailroute::RoutePoint& left_point = left.points[index];
        const sailroute::RoutePoint& right_point = right.points[index];
        REQUIRE(left_point.position.latitude_degrees ==
                right_point.position.latitude_degrees);
        REQUIRE(left_point.position.longitude_degrees ==
                right_point.position.longitude_degrees);
        REQUIRE(left_point.time == right_point.time);
        REQUIRE(left_point.heading_degrees == right_point.heading_degrees);
        REQUIRE(left_point.boat_speed_knots == right_point.boat_speed_knots);
        REQUIRE(
            left_point.true_wind_speed_knots ==
            right_point.true_wind_speed_knots);
        REQUIRE(
            left_point.true_wind_direction_degrees ==
            right_point.true_wind_direction_degrees);
        REQUIRE(
            left_point.cumulative_distance_nautical_miles ==
            right_point.cumulative_distance_nautical_miles);
    }
    REQUIRE(left.isochrones.size() == right.isochrones.size());
    for (std::size_t index = 0U; index < left.isochrones.size(); ++index) {
        const sailroute::Isochrone& left_isochrone = left.isochrones[index];
        const sailroute::Isochrone& right_isochrone = right.isochrones[index];
        REQUIRE(left_isochrone.time == right_isochrone.time);
        REQUIRE(left_isochrone.points.size() == right_isochrone.points.size());
        for (std::size_t point_index = 0U;
             point_index < left_isochrone.points.size();
             ++point_index) {
            REQUIRE(
                left_isochrone.points[point_index].latitude_degrees ==
                right_isochrone.points[point_index].latitude_degrees);
            REQUIRE(
                left_isochrone.points[point_index].longitude_degrees ==
                right_isochrone.points[point_index].longitude_degrees);
        }
    }
}

void require_same_progress(
    const std::vector<sailroute::RoutingProgress>& left,
    const std::vector<sailroute::RoutingProgress>& right) {
    REQUIRE(left.size() == right.size());
    for (std::size_t index = 0U; index < left.size(); ++index) {
        const sailroute::RoutingProgress& left_progress = left[index];
        const sailroute::RoutingProgress& right_progress = right[index];
        REQUIRE(left_progress.isochrone.time == right_progress.isochrone.time);
        REQUIRE(
            left_progress.isochrone.points.size() ==
            right_progress.isochrone.points.size());
        for (std::size_t point_index = 0U;
             point_index < left_progress.isochrone.points.size();
             ++point_index) {
            REQUIRE(
                left_progress.isochrone.points[point_index].latitude_degrees ==
                right_progress.isochrone.points[point_index].latitude_degrees);
            REQUIRE(
                left_progress.isochrone.points[point_index].longitude_degrees ==
                right_progress.isochrone.points[point_index].longitude_degrees);
        }

        REQUIRE(
            left_progress.provisional_route.size() ==
            right_progress.provisional_route.size());
        for (std::size_t point_index = 0U;
             point_index < left_progress.provisional_route.size();
             ++point_index) {
            const sailroute::RoutePoint& left_point =
                left_progress.provisional_route[point_index];
            const sailroute::RoutePoint& right_point =
                right_progress.provisional_route[point_index];
            REQUIRE(
                left_point.position.latitude_degrees ==
                right_point.position.latitude_degrees);
            REQUIRE(
                left_point.position.longitude_degrees ==
                right_point.position.longitude_degrees);
            REQUIRE(left_point.time == right_point.time);
            REQUIRE(left_point.heading_degrees == right_point.heading_degrees);
            REQUIRE(left_point.boat_speed_knots == right_point.boat_speed_knots);
            REQUIRE(
                left_point.true_wind_speed_knots ==
                right_point.true_wind_speed_knots);
            REQUIRE(
                left_point.true_wind_direction_degrees ==
                right_point.true_wind_direction_degrees);
            REQUIRE(
                left_point.cumulative_distance_nautical_miles ==
                right_point.cumulative_distance_nautical_miles);
        }

        REQUIRE(
            left_progress.diagnostics.expanded_nodes ==
            right_progress.diagnostics.expanded_nodes);
        REQUIRE(
            left_progress.diagnostics.generated_candidates ==
            right_progress.diagnostics.generated_candidates);
        REQUIRE(
            left_progress.diagnostics.retained_candidates ==
            right_progress.diagnostics.retained_candidates);
        REQUIRE(
            left_progress.diagnostics.time_steps ==
            right_progress.diagnostics.time_steps);
    }
}

}  // namespace

TEST_CASE("coordinates require finite canonical latitude and longitude") {
    REQUIRE(sailroute::is_valid({0.0, 0.0}));
    REQUIRE(sailroute::is_valid({-90.0, 180.0}));
    REQUIRE(!sailroute::is_valid({90.1, 0.0}));
    REQUIRE(!sailroute::is_valid({0.0, -180.1}));
    REQUIRE(!sailroute::is_valid(
        {std::numeric_limits<double>::quiet_NaN(), 0.0}));
}

TEST_CASE("wind exposes speed and meteorological direction") {
    const sailroute::Wind westward{-10.0, 0.0};
    REQUIRE_NEAR(westward.speed_knots(), 19.4384449244, 1.0e-9);
    REQUIRE_NEAR(westward.direction_from_degrees(), 90.0, 1.0e-12);

    const sailroute::Wind southward{0.0, -4.0};
    REQUIRE_NEAR(southward.direction_from_degrees(), 0.0, 1.0e-12);
    REQUIRE_NEAR(sailroute::Wind{}.direction_from_degrees(), 0.0, 1.0e-12);
}

TEST_CASE("departure sources have stable names") {
    REQUIRE(
        sailroute::to_string(sailroute::DepartureSource::explicit_time) ==
        std::string_view{"explicit_time"});
    REQUIRE(
        sailroute::to_string(sailroute::DepartureSource::current_time) ==
        std::string_view{"current_time"});
    REQUIRE(
        sailroute::to_string(sailroute::DepartureSource::forecast_start_fallback) ==
        std::string_view{"forecast_start_fallback"});
}

TEST_CASE("routing defaults retain a wider configurable frontier") {
    const sailroute::RoutingOptions options;
    REQUIRE(options.max_nodes_per_bucket == 10U);
    REQUIRE(!options.capture_isochrones);
    REQUIRE(options.time_step == std::chrono::minutes{30});
    REQUIRE(options.use_routing_intervals);
    REQUIRE(options.routing_intervals.size() == 3U);
    REQUIRE(
        options.routing_intervals[0U].interval ==
        std::chrono::minutes{30});
    REQUIRE(
        options.routing_intervals[0U].until_elapsed ==
        std::chrono::minutes{240});
    REQUIRE(
        options.routing_intervals[1U].interval ==
        std::chrono::minutes{60});
    REQUIRE(
        options.routing_intervals[1U].until_elapsed ==
        std::chrono::minutes{1'440});
    REQUIRE(
        options.routing_intervals[2U].interval ==
        std::chrono::minutes{180});
    REQUIRE(!options.routing_intervals[2U].until_elapsed.has_value());
}

TEST_CASE("routing interval schedules select and clamp elapsed-time tiers") {
    const sailroute::RoutingOptions options;
    REQUIRE(!sailroute::detail::validate_routing_intervals(options).has_value());
    const auto remaining = std::chrono::hours{100};

    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::seconds::zero(),
            remaining) ==
        std::chrono::minutes{30});
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::hours{3} + std::chrono::minutes{45},
            remaining) ==
        std::chrono::minutes{15});
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::hours{4},
            remaining) ==
        std::chrono::hours{1});
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::hours{23} + std::chrono::minutes{30},
            remaining) ==
        std::chrono::minutes{30});
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::hours{24},
            remaining) ==
        std::chrono::hours{3});
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::hours{24},
            std::chrono::minutes{45}) ==
        std::chrono::minutes{45});
}

TEST_CASE("routing intervals enforce valid tiers and a five-minute minimum") {
    sailroute::RoutingOptions options;
    options.routing_intervals = {
        {std::chrono::minutes{5}, std::chrono::minutes{10}},
        {std::chrono::minutes{15}, std::nullopt},
    };
    REQUIRE(!sailroute::detail::validate_routing_intervals(options).has_value());
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::minutes{5},
            std::chrono::hours{1}) ==
        std::chrono::minutes{5});
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::minutes{10},
            std::chrono::hours{1}) ==
        std::chrono::minutes{15});

    options.routing_intervals.front().interval = std::chrono::minutes{4};
    REQUIRE(sailroute::detail::validate_routing_intervals(options).has_value());
    options.routing_intervals.front().interval = std::chrono::minutes{5};
    options.routing_intervals.front().until_elapsed = std::chrono::minutes{20};
    options.routing_intervals.back().until_elapsed = std::chrono::minutes{10};
    REQUIRE(sailroute::detail::validate_routing_intervals(options).has_value());

    options.time_step = std::chrono::minutes{4};
    options.use_routing_intervals = false;
    REQUIRE(sailroute::detail::validate_routing_intervals(options).has_value());
    options.time_step = std::chrono::minutes{5};
    REQUIRE(!sailroute::detail::validate_routing_intervals(options).has_value());
    REQUIRE(
        sailroute::detail::routing_step(
            options,
            std::chrono::hours{24},
            std::chrono::hours{1}) ==
        std::chrono::minutes{5});
}

TEST_CASE("spherical navigation handles cardinal courses and the antimeridian") {
    const sailroute::Coordinate origin{0.0, 0.0};
    const sailroute::Coordinate east{0.0, 1.0};
    REQUIRE_NEAR(
        sailroute::detail::great_circle_distance_nautical_miles(origin, east),
        60.0405,
        0.001);
    REQUIRE_NEAR(sailroute::detail::initial_bearing_degrees(origin, east), 90.0, 1.0e-12);

    const sailroute::Coordinate projected =
        sailroute::detail::destination_point(origin, 90.0, 60.0405);
    REQUIRE_NEAR(projected.latitude_degrees, 0.0, 1.0e-6);
    REQUIRE_NEAR(projected.longitude_degrees, 1.0, 1.0e-5);

    const double crossing = sailroute::detail::great_circle_distance_nautical_miles(
        {0.0, 179.5},
        {0.0, -179.5});
    REQUIRE_NEAR(crossing, 60.0405, 0.001);
}

TEST_CASE("parallel candidate expansion is deterministic") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const sailroute::RouteResult single = route_with_workers(router, 1U);
    const sailroute::RouteResult parallel = route_with_workers(router, 4U);
    const sailroute::RouteResult repeated = route_with_workers(router, 4U);
    const sailroute::RouteResult automatic = route_with_workers(router, 0U);

    REQUIRE(single.diagnostics.expanded_nodes > single.diagnostics.time_steps);
    REQUIRE(!single.isochrones.empty());
    std::size_t captured_points = 0U;
    sailroute::TimePoint previous_time{};
    for (const sailroute::Isochrone& isochrone : single.isochrones) {
        REQUIRE(!isochrone.points.empty());
        REQUIRE(previous_time == sailroute::TimePoint{} ||
                isochrone.time > previous_time);
        previous_time = isochrone.time;
        captured_points += isochrone.points.size();
    }
    REQUIRE(captured_points + 1U == single.diagnostics.retained_candidates);
    require_same_route(single, parallel);
    require_same_route(parallel, repeated);
    require_same_route(single, automatic);
}

TEST_CASE("progress callbacks stream deterministic provisional routes and isochrones") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    std::vector<sailroute::RoutingProgress> single_progress;
    const sailroute::RouteResult single = route_with_workers(
        router,
        1U,
        false,
        [&single_progress](const sailroute::RoutingProgress& progress) {
            single_progress.push_back(progress);
        });
    std::vector<sailroute::RoutingProgress> parallel_progress;
    const sailroute::RouteResult parallel = route_with_workers(
        router,
        4U,
        false,
        [&parallel_progress](const sailroute::RoutingProgress& progress) {
            parallel_progress.push_back(progress);
        });

    REQUIRE(single.isochrones.empty());
    REQUIRE(parallel.isochrones.empty());
    REQUIRE(!single_progress.empty());
    REQUIRE(single_progress.size() + 1U == single.diagnostics.time_steps);

    sailroute::TimePoint previous_time{};
    for (std::size_t index = 0U; index < single_progress.size(); ++index) {
        const sailroute::RoutingProgress& progress = single_progress[index];
        REQUIRE(progress.diagnostics.time_steps == index + 1U);
        REQUIRE(!progress.isochrone.points.empty());
        REQUIRE(!progress.provisional_route.empty());
        REQUIRE(previous_time == sailroute::TimePoint{} ||
                progress.isochrone.time > previous_time);
        previous_time = progress.isochrone.time;

        const sailroute::RoutePoint& route_start =
            progress.provisional_route.front();
        const sailroute::RoutePoint& route_end =
            progress.provisional_route.back();
        REQUIRE(route_start.position.latitude_degrees == 1.0);
        REQUIRE(route_start.position.longitude_degrees == 0.5);
        REQUIRE(route_end.time == progress.isochrone.time);

        sailroute::Coordinate closest = progress.isochrone.points.front();
        double closest_distance =
            sailroute::detail::great_circle_distance_nautical_miles(
                closest,
                {1.0, 1.0});
        for (const sailroute::Coordinate point : progress.isochrone.points) {
            const double distance =
                sailroute::detail::great_circle_distance_nautical_miles(
                    point,
                    {1.0, 1.0});
            if (distance < closest_distance) {
                closest = point;
                closest_distance = distance;
            }
        }
        REQUIRE(route_end.position.latitude_degrees == closest.latitude_degrees);
        REQUIRE(route_end.position.longitude_degrees == closest.longitude_degrees);
    }

    require_same_route(single, parallel);
    require_same_progress(single_progress, parallel_progress);
}

TEST_CASE("router produces scheduled points at five-minute intervals") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    const auto departure = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    REQUIRE(departure.has_value());

    sailroute::RouteRequest request;
    request.start = {1.0, 0.5};
    request.destination = {1.0, 0.7};
    request.departure_time = departure.value();
    request.options.routing_intervals = {
        {std::chrono::minutes{5}, std::nullopt},
    };
    request.options.use_routing_intervals = true;
    request.options.arrival_radius_nautical_miles = 0.5;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 3U;
    request.options.maximum_route_duration = std::chrono::hours{12};

    const auto route = router.optimize(request);
    if (!route.has_value()) {
        throw std::runtime_error(route.error().message);
    }
    REQUIRE(route.value().points.size() > 2U);
    for (std::size_t index = 1U;
         index + 1U < route.value().points.size();
         ++index) {
        REQUIRE(
            route.value().points[index].time -
                route.value().points[index - 1U].time ==
            std::chrono::minutes{5});
    }
    REQUIRE(
        route.value().points.back().time -
            route.value().points[route.value().points.size() - 2U].time <=
        std::chrono::minutes{5});
}

TEST_CASE("explicit departure outside forecast coverage is rejected") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    const auto departure = sailroute::parse_utc_time("2026-07-14T11:59:59Z");
    REQUIRE(departure.has_value());

    sailroute::RouteRequest request;
    request.start = {1.0, 0.5};
    request.destination = {1.0, 0.7};
    request.departure_time = departure.value();
    const auto route = router.optimize(request);
    REQUIRE(!route.has_value());
    REQUIRE(
        route.error().code ==
        sailroute::ErrorCode::departure_outside_forecast);
}

TEST_CASE("omitted departure falls back to forecast start") {
    const RoutingGribFixture fixture{20200101, 0};
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest request;
    request.start = {1.0, 0.5};
    request.destination = {1.0, 0.7};
    request.options.maximum_route_duration = std::chrono::hours{12};
    const auto route = router.optimize(request);
    if (!route.has_value()) {
        throw std::runtime_error(route.error().message);
    }
    const auto forecast_start = sailroute::parse_utc_time("2020-01-01T00:00:00Z");
    REQUIRE(forecast_start.has_value());
    REQUIRE(route.value().departure_time == forecast_start.value());
    REQUIRE(
        route.value().departure_source ==
        sailroute::DepartureSource::forecast_start_fallback);
    REQUIRE(route.value().isochrones.empty());
}
