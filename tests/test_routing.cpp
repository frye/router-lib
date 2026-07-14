#include "sailroute/router.hpp"
#include "sailroute/time.hpp"

#include "../src/routing/geodesy.hpp"
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
    std::size_t worker_count) {
    const auto departure = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    REQUIRE(departure.has_value());

    sailroute::RouteRequest request;
    request.start = {1.0, 0.5};
    request.destination = {1.0, 1.0};
    request.departure_time = departure.value();
    request.options.time_step = std::chrono::minutes{30};
    request.options.heading_step_degrees = 10.0;
    request.options.arrival_radius_nautical_miles = 0.5;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 3;
    request.options.worker_count = worker_count;
    request.options.maximum_route_duration = std::chrono::hours{12};
    request.options.capture_isochrones = true;

    auto result = router.optimize(request);
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
