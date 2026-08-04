#include "sailroute/front.hpp"
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
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
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
    explicit RoutingGribFixture(
        long data_date = 20260714,
        long data_time = 1200,
        long final_forecast_hour = 12)
        : path_(
              std::filesystem::current_path() /
              ("sailroute-routing-" +
               std::to_string(
                   std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".grib")) {
        write_message("10u", 0, 0.0, data_date, data_time, "w");
        write_message("10v", 0, -10.0, data_date, data_time, "a");
        write_message(
            "10u",
            final_forecast_hour,
            0.0,
            data_date,
            data_time,
            "a");
        write_message(
            "10v",
            final_forecast_hour,
            -10.0,
            data_date,
            data_time,
            "a");
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

sailroute::RouteRequest routing_request(
    std::size_t worker_count,
    bool capture_isochrones = true) {
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
    return request;
}

sailroute::RouteResult route_with_workers(
    const sailroute::Router& router,
    std::size_t worker_count,
    bool capture_isochrones = true,
    sailroute::RoutingProgressCallback on_progress = {}) {
    const sailroute::RouteRequest request =
        routing_request(worker_count, capture_isochrones);
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
    REQUIRE(left.completion == right.completion);
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

void require_same_front(
    const sailroute::IsochroneFront& left,
    const sailroute::IsochroneFront& right) {
    REQUIRE(left.points.size() == right.points.size());
    REQUIRE(left.segments.size() == right.segments.size());
    for (std::size_t index = 0U; index < left.points.size(); ++index) {
        REQUIRE(
            left.points[index].latitude_degrees ==
            right.points[index].latitude_degrees);
        REQUIRE(
            left.points[index].longitude_degrees ==
            right.points[index].longitude_degrees);
    }
    for (std::size_t index = 0U; index < left.segments.size(); ++index) {
        REQUIRE(
            left.segments[index].point_offset ==
            right.segments[index].point_offset);
        REQUIRE(
            left.segments[index].point_count ==
            right.segments[index].point_count);
    }
}

sailroute::IsochroneFront copy_front(
    const sailroute::IsochroneFrontView& view) {
    return sailroute::IsochroneFront{
        std::vector<sailroute::Coordinate>{
            view.points.begin(),
            view.points.end()},
        std::vector<sailroute::IsochroneFrontSegment>{
            view.segments.begin(),
            view.segments.end()}};
}

struct SegmentObservation {
    sailroute::RoutePoint parent;
    sailroute::RoutePoint candidate;
};

void require_same_route_point(
    const sailroute::RoutePoint& left,
    const sailroute::RoutePoint& right) {
    REQUIRE(
        left.position.latitude_degrees ==
        right.position.latitude_degrees);
    REQUIRE(
        left.position.longitude_degrees ==
        right.position.longitude_degrees);
    REQUIRE(left.time == right.time);
    REQUIRE(left.heading_degrees == right.heading_degrees);
    REQUIRE(left.boat_speed_knots == right.boat_speed_knots);
    REQUIRE(left.true_wind_speed_knots == right.true_wind_speed_knots);
    REQUIRE(
        left.true_wind_direction_degrees ==
        right.true_wind_direction_degrees);
    REQUIRE(
        left.cumulative_distance_nautical_miles ==
        right.cumulative_distance_nautical_miles);
}

void require_same_segment_observations(
    const std::vector<SegmentObservation>& left,
    const std::vector<SegmentObservation>& right) {
    REQUIRE(left.size() == right.size());
    for (std::size_t index = 0U; index < left.size(); ++index) {
        require_same_route_point(left[index].parent, right[index].parent);
        require_same_route_point(left[index].candidate, right[index].candidate);
    }
}

bool matches_rejected_point(
    sailroute::Coordinate position,
    sailroute::TimePoint time,
    const std::vector<sailroute::RoutePoint>& rejected) {
    return std::any_of(
        rejected.begin(),
        rejected.end(),
        [position, time](const sailroute::RoutePoint& point) {
            return point.time == time &&
                   point.position.latitude_degrees ==
                       position.latitude_degrees &&
                   point.position.longitude_degrees ==
                       position.longitude_degrees;
        });
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
    REQUIRE(
        sailroute::to_string(sailroute::RouteCompletion::destination_reached) ==
        std::string_view{"destination_reached"});
    REQUIRE(
        sailroute::to_string(sailroute::RouteCompletion::forecast_exhausted) ==
        std::string_view{"forecast_exhausted"});
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
    REQUIRE(options.progress.every_n_steps == 1U);
    REQUIRE(sailroute::has_payload(
        options.progress.payload,
        sailroute::RoutingProgressPayload::retained_points));
    REQUIRE(sailroute::has_payload(
        options.progress.payload,
        sailroute::RoutingProgressPayload::provisional_route));
    REQUIRE(!sailroute::has_payload(
        options.progress.payload,
        sailroute::RoutingProgressPayload::display_contours));
    REQUIRE(
        options.progress.destination_front.half_angle_degrees == 90.0);
    REQUIRE(
        options.progress.destination_front.segment_policy ==
        sailroute::DestinationFrontSegmentPolicy::provisional_component);
    REQUIRE(
        options.progress.destination_front.minimum_secondary_segment_points ==
        3U);
    REQUIRE(!options.segment_eligibility);
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

TEST_CASE("maneuver delay shifts the sailed midpoint time") {
    REQUIRE(
        sailroute::detail::sailing_midpoint_offset(
            std::chrono::hours{3},
            std::chrono::seconds::zero()) ==
        std::chrono::minutes{90});
    REQUIRE(
        sailroute::detail::sailing_midpoint_offset(
            std::chrono::hours{3},
            std::chrono::minutes{30}) ==
        std::chrono::minutes{105});
    REQUIRE(
        sailroute::detail::sailing_midpoint_offset(
            std::chrono::hours{3},
            std::chrono::hours{1}) ==
        std::chrono::hours{2});
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

    REQUIRE(
        single.completion ==
        sailroute::RouteCompletion::destination_reached);
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

TEST_CASE("accept-all segment eligibility preserves existing routing") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const sailroute::RouteResult baseline = route_with_workers(router, 4U);
    sailroute::RouteRequest request = routing_request(4U, true);
    std::size_t evaluated_segments = 0U;
    request.options.segment_eligibility =
        [&evaluated_segments](const sailroute::RouteSegmentView&) {
            ++evaluated_segments;
            return true;
        };
    const auto constrained = router.optimize(request);
    REQUIRE(constrained.has_value());
    REQUIRE(
        evaluated_segments ==
        constrained.value().diagnostics.generated_candidates);
    require_same_route(baseline, constrained.value());
}

TEST_CASE("rejected segments never enter retained or emitted routing state") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(4U, true);

    std::vector<sailroute::RoutePoint> rejected;
    request.options.segment_eligibility =
        [&rejected](const sailroute::RouteSegmentView& segment) {
            if (segment.candidate.heading_degrees == 180.0) {
                rejected.push_back(segment.candidate);
                return false;
            }
            return true;
        };
    request.options.progress.payload =
        sailroute::RoutingProgressPayload::retained_points |
        sailroute::RoutingProgressPayload::provisional_route |
        sailroute::RoutingProgressPayload::destination_front;
    request.options.progress.destination_front.segment_policy =
        sailroute::DestinationFrontSegmentPolicy::all_meaningful_components;
    std::size_t progress_updates = 0U;
    const auto result = router.optimize_view(
        request,
        [&progress_updates, &rejected](
            const sailroute::RoutingProgressView& progress) {
            ++progress_updates;
            for (const sailroute::Coordinate point : progress.retained_points) {
                REQUIRE(!matches_rejected_point(
                    point,
                    progress.time,
                    rejected));
            }
            for (const sailroute::Coordinate point :
                 progress.destination_front.points) {
                REQUIRE(!matches_rejected_point(
                    point,
                    progress.time,
                    rejected));
            }
            for (const sailroute::RoutePoint& point :
                 progress.provisional_route) {
                REQUIRE(!matches_rejected_point(
                    point.position,
                    point.time,
                    rejected));
            }
        });
    REQUIRE(result.has_value());
    REQUIRE(!rejected.empty());
    REQUIRE(progress_updates > 0U);

    for (const sailroute::Isochrone& isochrone : result.value().isochrones) {
        for (const sailroute::Coordinate point : isochrone.points) {
            REQUIRE(!matches_rejected_point(
                point,
                isochrone.time,
                rejected));
        }
    }
    for (const sailroute::RoutePoint& point : result.value().points) {
        REQUIRE(!matches_rejected_point(
            point.position,
            point.time,
            rejected));
    }
}

TEST_CASE("rejected shortened arrivals fall back to a later routing step") {
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
    request.options.arrival_radius_nautical_miles = 0.5;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 3U;
    request.options.worker_count = 4U;
    request.options.maximum_route_duration = std::chrono::hours{12};
    request.options.capture_isochrones = true;

    std::optional<sailroute::TimePoint> rejected_step;
    std::vector<SegmentObservation> rejected_arrivals;
    request.options.segment_eligibility =
        [&rejected_step, &rejected_arrivals](
            const sailroute::RouteSegmentView& segment) {
            const double travelled =
                segment.candidate.cumulative_distance_nautical_miles -
                segment.parent.cumulative_distance_nautical_miles;
            const double full_step_distance =
                segment.candidate.boat_speed_knots * (5.0 / 60.0);
            const bool shortened = travelled + 1.0e-9 < full_step_distance;
            if (!shortened) {
                return true;
            }
            if (!rejected_step.has_value()) {
                rejected_step = segment.parent.time;
            }
            if (segment.parent.time == *rejected_step) {
                rejected_arrivals.push_back(
                    SegmentObservation{segment.parent, segment.candidate});
                return false;
            }
            return true;
        };

    const auto result = router.optimize(request);
    REQUIRE(result.has_value());
    REQUIRE(rejected_step.has_value());
    REQUIRE(!rejected_arrivals.empty());
    REQUIRE(result.value().arrival_time > *rejected_step + std::chrono::minutes{5});
    for (const SegmentObservation& segment : rejected_arrivals) {
        REQUIRE(segment.candidate.time > segment.parent.time);
        REQUIRE(
            segment.candidate.time <=
            segment.parent.time + std::chrono::minutes{5});
        REQUIRE(
            segment.candidate.cumulative_distance_nautical_miles -
                segment.parent.cumulative_distance_nautical_miles <
            segment.candidate.boat_speed_knots * (5.0 / 60.0));
        for (const sailroute::RoutePoint& point : result.value().points) {
            REQUIRE(!matches_rejected_point(
                point.position,
                point.time,
                {segment.candidate}));
        }
    }
}

TEST_CASE("segment eligibility observations are deterministic across workers") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const auto route_with_eligibility =
        [&router](
            std::size_t worker_count,
            std::vector<SegmentObservation>& observations,
            std::vector<sailroute::RoutingProgress>& progress_updates) {
            sailroute::RouteRequest request =
                routing_request(worker_count, false);
            request.options.segment_eligibility =
                [&observations](const sailroute::RouteSegmentView& segment) {
                    observations.push_back(
                        SegmentObservation{segment.parent, segment.candidate});
                    return segment.candidate.heading_degrees != 0.0;
                };
            auto result = router.optimize(
                request,
                [&progress_updates](
                    const sailroute::RoutingProgress& progress) {
                    progress_updates.push_back(progress);
                });
            if (!result.has_value()) {
                throw std::runtime_error(result.error().message);
            }
            return std::move(result.value());
        };

    std::vector<SegmentObservation> single_observations;
    std::vector<sailroute::RoutingProgress> single_progress;
    const sailroute::RouteResult single = route_with_eligibility(
        1U,
        single_observations,
        single_progress);
    std::vector<SegmentObservation> parallel_observations;
    std::vector<sailroute::RoutingProgress> parallel_progress;
    const sailroute::RouteResult parallel = route_with_eligibility(
        4U,
        parallel_observations,
        parallel_progress);

    REQUIRE(
        single_observations.size() ==
        single.diagnostics.generated_candidates);
    REQUIRE(
        parallel_observations.size() ==
        parallel.diagnostics.generated_candidates);
    require_same_segment_observations(
        single_observations,
        parallel_observations);
    require_same_progress(single_progress, parallel_progress);
    require_same_route(single, parallel);
}

TEST_CASE("segment eligibility runs on the caller thread and propagates exceptions") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(4U, false);
    const std::thread::id caller_thread = std::this_thread::get_id();
    bool wrong_thread = false;
    std::size_t evaluated_segments = 0U;
    request.options.segment_eligibility =
        [caller_thread, &wrong_thread, &evaluated_segments](
            const sailroute::RouteSegmentView&) {
            wrong_thread =
                wrong_thread || std::this_thread::get_id() != caller_thread;
            ++evaluated_segments;
            if (evaluated_segments == 10U) {
                throw std::domain_error{"segment eligibility failure"};
            }
            return true;
        };

    bool caught_expected_exception = false;
    try {
        static_cast<void>(router.optimize(request));
    } catch (const std::domain_error& error) {
        caught_expected_exception =
            std::string_view{error.what()} == "segment eligibility failure";
    }
    REQUIRE(caught_expected_exception);
    REQUIRE(!wrong_thread);
    REQUIRE(evaluated_segments == 10U);

    const auto subsequent = router.optimize(routing_request(4U, false));
    REQUIRE(subsequent.has_value());
}

TEST_CASE("segment eligibility reports all-rejected candidates distinctly") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest rejected_request = routing_request(4U, false);
    std::size_t evaluated_segments = 0U;
    rejected_request.options.segment_eligibility =
        [&evaluated_segments](const sailroute::RouteSegmentView&) {
            ++evaluated_segments;
            return false;
        };
    const auto rejected = router.optimize(rejected_request);
    REQUIRE(!rejected.has_value());
    REQUIRE(rejected.error().code == sailroute::ErrorCode::no_route);
    REQUIRE(
        rejected.error().message ==
        "segment eligibility rejected every candidate at routing step 1");
    REQUIRE(evaluated_segments > 0U);

    sailroute::RouteRequest no_candidates = routing_request(4U, false);
    no_candidates.options.minimum_boat_speed_knots = 1'000.0;
    no_candidates.options.segment_eligibility =
        [](const sailroute::RouteSegmentView&) {
            throw std::runtime_error{"must not be invoked"};
            return false;
        };
    const auto speed_limited = router.optimize(no_candidates);
    REQUIRE(!speed_limited.has_value());
    REQUIRE(speed_limited.error().code == sailroute::ErrorCode::no_route);
    REQUIRE(
        speed_limited.error().message ==
        "no heading met the minimum boat speed at routing step 1");
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

TEST_CASE("forecast exhaustion returns the best supported partial route") {
    const RoutingGribFixture fixture{20260714, 1200, 1};
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest request = routing_request(1U, true);
    std::vector<sailroute::RoutePoint> final_callback_route;
    const auto result = router.optimize_view(
        request,
        [&final_callback_route](
            const sailroute::RoutingProgressView& progress) {
            final_callback_route.assign(
                progress.provisional_route.begin(),
                progress.provisional_route.end());
        });

    REQUIRE(result.has_value());
    const sailroute::RouteResult& route = result.value();
    REQUIRE(
        route.completion ==
        sailroute::RouteCompletion::forecast_exhausted);
    REQUIRE(route.points.size() > 1U);
    REQUIRE(route.arrival_time == route.points.back().time);
    REQUIRE(
        route.arrival_time ==
        route.departure_time + std::chrono::hours{1});
    REQUIRE(route.diagnostics.time_steps == 2U);
    REQUIRE(route.isochrones.size() == 2U);
    REQUIRE(route.points.size() == final_callback_route.size());
    for (std::size_t index = 0U; index < route.points.size(); ++index) {
        REQUIRE(
            route.points[index].position.latitude_degrees ==
            final_callback_route[index].position.latitude_degrees);
        REQUIRE(
            route.points[index].position.longitude_degrees ==
            final_callback_route[index].position.longitude_degrees);
        REQUIRE(route.points[index].time == final_callback_route[index].time);
    }
}

TEST_CASE("partial route ownership is independent of callbacks and payloads") {
    const RoutingGribFixture fixture{20260714, 1200, 1};
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest direct_request = routing_request(4U, false);
    const auto direct = router.optimize(direct_request);
    REQUIRE(direct.has_value());
    REQUIRE(direct.value().isochrones.empty());

    sailroute::RouteRequest view_request = routing_request(1U, false);
    view_request.options.progress.every_n_steps = 100U;
    view_request.options.progress.payload =
        sailroute::RoutingProgressPayload::none;
    std::size_t callback_count = 0U;
    const auto view = router.optimize_view(
        view_request,
        [&callback_count](const sailroute::RoutingProgressView&) {
            ++callback_count;
        });

    REQUIRE(view.has_value());
    REQUIRE(callback_count == 0U);
    REQUIRE(
        direct.value().completion ==
        sailroute::RouteCompletion::forecast_exhausted);
    require_same_route(direct.value(), view.value());
}

TEST_CASE("maximum route duration remains an error at the forecast boundary") {
    {
        const RoutingGribFixture fixture;
        const auto weather = sailroute::WeatherDataset::load(fixture.path());
        REQUIRE(weather.has_value());
        const sailroute::Router router{weather.value()};

        sailroute::RouteRequest request = routing_request(1U, false);
        request.options.maximum_route_duration = std::chrono::hours{1};
        const auto result = router.optimize(request);

        REQUIRE(!result.has_value());
        REQUIRE(result.error().code == sailroute::ErrorCode::no_route);
    }

    {
        const RoutingGribFixture fixture{20260714, 1200, 1};
        const auto weather = sailroute::WeatherDataset::load(fixture.path());
        REQUIRE(weather.has_value());
        const sailroute::Router router{weather.value()};

        sailroute::RouteRequest request = routing_request(1U, false);
        request.options.maximum_route_duration = std::chrono::hours{1};
        const auto result = router.optimize(request);

        REQUIRE(!result.has_value());
        REQUIRE(result.error().code == sailroute::ErrorCode::no_route);
    }
}

TEST_CASE("progress callbacks can cancel before the second routing step") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    std::size_t update_count = 0U;
    const auto result = router.optimize(
        routing_request(1U, false),
        [&update_count](const sailroute::RoutingProgress& progress) {
            ++update_count;
            REQUIRE(progress.diagnostics.time_steps == 1U);
            return sailroute::RoutingProgressDecision::cancel;
        });

    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::cancelled);
    REQUIRE(sailroute::to_string(result.error().code) == "cancelled");
    REQUIRE(
        result.error().message ==
        "routing cancelled after 1 time step and 1 expanded node");
    REQUIRE(update_count == 1U);
}

TEST_CASE("progress callbacks can cancel multi-worker routing after multiple updates") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    std::vector<std::size_t> callback_steps;
    const sailroute::RoutingControlCallback cancel_after_three_updates =
        [&callback_steps](const sailroute::RoutingProgress& progress) {
            callback_steps.push_back(progress.diagnostics.time_steps);
            return callback_steps.size() == 3U
                ? sailroute::RoutingProgressDecision::cancel
                : sailroute::RoutingProgressDecision::continue_routing;
        };
    const auto result = router.optimize(
        routing_request(4U, false),
        cancel_after_three_updates);

    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::cancelled);
    REQUIRE(callback_steps.size() == 3U);
    REQUIRE(callback_steps[0] == 1U);
    REQUIRE(callback_steps[1] == 2U);
    REQUIRE(callback_steps[2] == 3U);
}

TEST_CASE("progress views select payloads without changing routing") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(1U, false);
    request.options.progress.payload =
        sailroute::RoutingProgressPayload::display_contours;

    std::size_t update_count = 0U;
    const auto result = router.optimize_view(
        request,
        [&update_count](const sailroute::RoutingProgressView& progress) {
            ++update_count;
            REQUIRE(progress.retained_points.empty());
            REQUIRE(progress.provisional_route.empty());
            REQUIRE(!progress.display_contours.points.empty());
            REQUIRE(!progress.display_contours.segments.empty());
            REQUIRE(progress.time != sailroute::TimePoint{});
        });
    REQUIRE(result.has_value());
    REQUIRE(update_count + 1U == result.value().diagnostics.time_steps);
}

TEST_CASE("progress views build destination fronts from eligible pre-prune candidates") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(1U, false);
    request.start = {0.8, 0.8};
    request.destination = {1.2, 1.2};
    request.options.heading_step_degrees = 5.0;
    request.options.max_nodes_per_bucket = 1U;
    request.options.progress.payload =
        sailroute::RoutingProgressPayload::retained_points |
        sailroute::RoutingProgressPayload::provisional_route |
        sailroute::RoutingProgressPayload::destination_front;
    request.options.progress.destination_front.half_angle_degrees = 120.0;
    request.options.progress.destination_front.segment_policy =
        sailroute::DestinationFrontSegmentPolicy::all_meaningful_components;

    std::vector<sailroute::RoutePoint> eligible_candidates;
    request.options.segment_eligibility =
        [&eligible_candidates](const sailroute::RouteSegmentView& segment) {
            eligible_candidates.push_back(segment.candidate);
            return true;
        };
    std::size_t update_count = 0U;
    bool observed_pre_prune_point = false;
    const auto result = router.optimize_view(
        request,
        [&eligible_candidates, &observed_pre_prune_point, &update_count](
            const sailroute::RoutingProgressView& progress) {
            ++update_count;
            REQUIRE(!progress.provisional_route.empty());
            const sailroute::Coordinate anchor =
                progress.provisional_route.back().position;
            bool anchor_present = false;
            for (const sailroute::Coordinate point :
                 progress.destination_front.points) {
                REQUIRE(matches_rejected_point(
                    point,
                    progress.time,
                    eligible_candidates));
                anchor_present =
                    anchor_present ||
                    (point.latitude_degrees == anchor.latitude_degrees &&
                     point.longitude_degrees == anchor.longitude_degrees);
                const bool retained = std::any_of(
                    progress.retained_points.begin(),
                    progress.retained_points.end(),
                    [point](sailroute::Coordinate retained_point) {
                        return point.latitude_degrees ==
                                   retained_point.latitude_degrees &&
                            point.longitude_degrees ==
                                   retained_point.longitude_degrees;
                    });
                observed_pre_prune_point =
                    observed_pre_prune_point || !retained;
            }
            REQUIRE(anchor_present);
        });

    if (!result.has_value()) {
        throw std::runtime_error(result.error().message);
    }
    REQUIRE(update_count + 1U == result.value().diagnostics.time_steps);
    REQUIRE(observed_pre_prune_point);
}

TEST_CASE("default progress fronts preserve retained-front behavior") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(1U, false);
    request.options.progress.payload =
        sailroute::RoutingProgressPayload::retained_points |
        sailroute::RoutingProgressPayload::destination_front;

    std::size_t updates = 0U;
    const auto result = router.optimize_view(
        request,
        [&request, &updates](const sailroute::RoutingProgressView& progress) {
            ++updates;
            const auto expected = sailroute::build_destination_front(
                progress.retained_points,
                request.destination,
                request.options.spatial_bucket_nautical_miles,
                request.options.progress.destination_front);
            REQUIRE(expected.has_value());
            require_same_front(
                copy_front(progress.destination_front),
                expected.value());
        });

    REQUIRE(result.has_value());
    REQUIRE(updates + 1U == result.value().diagnostics.time_steps);
}

TEST_CASE("destination front construction does not change routing search") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest default_request = routing_request(1U, true);
    sailroute::RouteRequest front_request = default_request;
    front_request.options.progress.payload =
        sailroute::RoutingProgressPayload::destination_front;
    front_request.options.progress.destination_front.half_angle_degrees = 120.0;
    front_request.options.progress.destination_front.segment_policy =
        sailroute::DestinationFrontSegmentPolicy::all_meaningful_components;

    const auto default_result = router.optimize(default_request);
    std::size_t updates = 0U;
    const auto front_result = router.optimize_view(
        front_request,
        [&updates](const sailroute::RoutingProgressView& progress) {
            ++updates;
            REQUIRE(!progress.destination_front.points.empty());
        });

    REQUIRE(default_result.has_value());
    REQUIRE(front_result.has_value());
    REQUIRE(updates > 0U);
    require_same_route(default_result.value(), front_result.value());
}

TEST_CASE("destination fronts are deterministic across worker counts") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const auto route_and_fronts =
        [&router](std::size_t worker_count) {
            sailroute::RouteRequest request =
                routing_request(worker_count, false);
            request.options.progress.payload =
                sailroute::RoutingProgressPayload::destination_front;
            request.options.progress.destination_front.segment_policy =
                sailroute::DestinationFrontSegmentPolicy::
                    all_meaningful_components;
            std::vector<sailroute::IsochroneFront> fronts;
            auto result = router.optimize_view(
                request,
                [&fronts](const sailroute::RoutingProgressView& progress) {
                    fronts.push_back(copy_front(progress.destination_front));
                });
            if (!result.has_value()) {
                throw std::runtime_error(result.error().message);
            }
            return std::pair{
                std::move(result.value()),
                std::move(fronts)};
        };

    const auto [single_route, single_fronts] = route_and_fronts(1U);
    const auto [parallel_route, parallel_fronts] = route_and_fronts(4U);
    const auto [automatic_route, automatic_fronts] = route_and_fronts(0U);

    require_same_route(single_route, parallel_route);
    require_same_route(single_route, automatic_route);
    REQUIRE(single_fronts.size() == parallel_fronts.size());
    REQUIRE(single_fronts.size() == automatic_fronts.size());
    for (std::size_t index = 0U; index < single_fronts.size(); ++index) {
        require_same_front(single_fronts[index], parallel_fronts[index]);
        require_same_front(single_fronts[index], automatic_fronts[index]);
    }
}

TEST_CASE("progress view cadence throttles callbacks but not isochrone capture") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(1U, true);
    request.options.progress.every_n_steps = 2U;
    request.options.progress.payload =
        sailroute::RoutingProgressPayload::none;

    std::vector<std::size_t> delivered_steps;
    const auto result = router.optimize_view(
        request,
        [&delivered_steps](const sailroute::RoutingProgressView& progress) {
            REQUIRE(progress.retained_points.empty());
            REQUIRE(progress.provisional_route.empty());
            REQUIRE(progress.display_contours.points.empty());
            REQUIRE(progress.display_contours.segments.empty());
            delivered_steps.push_back(progress.diagnostics.time_steps);
        });
    REQUIRE(result.has_value());
    REQUIRE(!delivered_steps.empty());
    for (const std::size_t step : delivered_steps) {
        REQUIRE(step % 2U == 0U);
    }
    REQUIRE(
        result.value().isochrones.size() + 1U ==
        result.value().diagnostics.time_steps);
    REQUIRE(
        delivered_steps.size() ==
        (result.value().diagnostics.time_steps - 1U) / 2U);
}

TEST_CASE("progress views preserve callback-scoped data through explicit copies") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(4U, false);

    std::vector<sailroute::RoutingProgress> copied;
    const std::thread::id caller_thread = std::this_thread::get_id();
    const auto result = router.optimize_view(
        request,
        [&copied, caller_thread](
            const sailroute::RoutingProgressView& progress) {
            REQUIRE(std::this_thread::get_id() == caller_thread);
            copied.push_back(sailroute::RoutingProgress{
                sailroute::Isochrone{
                    progress.time,
                    std::vector<sailroute::Coordinate>{
                        progress.retained_points.begin(),
                        progress.retained_points.end()}},
                std::vector<sailroute::RoutePoint>{
                    progress.provisional_route.begin(),
                    progress.provisional_route.end()},
                sailroute::IsochroneFront{},
                progress.diagnostics});
        });
    REQUIRE(result.has_value());
    REQUIRE(!copied.empty());
    REQUIRE(copied.back().diagnostics.time_steps == copied.size());
    REQUIRE(!copied.back().isochrone.points.empty());
    REQUIRE(!copied.back().provisional_route.empty());
}

TEST_CASE("progress view callbacks use the existing cancellation decision") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const auto result = router.optimize_view(
        routing_request(1U, false),
        [](const sailroute::RoutingProgressView& progress) {
            REQUIRE(progress.diagnostics.time_steps == 1U);
            return sailroute::RoutingProgressDecision::cancel;
        });
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::cancelled);
}

TEST_CASE("progress options reject invalid construction values") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(1U, false);
    request.options.progress.every_n_steps = 0U;
    auto result = router.optimize_view(
        request,
        sailroute::RoutingProgressViewCallback{});
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);

    request.options.progress.every_n_steps = 1U;
    request.options.progress.display_contours.alpha_nautical_miles = -1.0;
    result = router.optimize_view(
        request,
        sailroute::RoutingProgressViewCallback{});
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);

    request.options.progress.display_contours.alpha_nautical_miles.reset();
    const std::array<double, 6> invalid_angles{
        0.0,
        -1.0,
        180.0001,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    for (const double half_angle : invalid_angles) {
        request.options.progress.destination_front.half_angle_degrees =
            half_angle;
        result = router.optimize_view(
            request,
            sailroute::RoutingProgressViewCallback{});
        REQUIRE(!result.has_value());
        REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
    }

    request.options.progress.destination_front.half_angle_degrees = 90.0;
    request.options.progress.destination_front.segment_policy =
        static_cast<sailroute::DestinationFrontSegmentPolicy>(99);
    result = router.optimize_view(
        request,
        sailroute::RoutingProgressViewCallback{});
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
}

TEST_CASE("bearing-sector pruning rejects invalid sector widths") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(1U, false);
    request.options.pruning_strategy =
        sailroute::PruningStrategy::bearing_sectors;

    // sector_bucket_for divides a bearing by this width and casts the quotient
    // to an integer, so a zero or non-finite width is undefined behavior rather
    // than merely an odd search.
    const std::array<double, 6> invalid_sectors{
        0.0,
        -1.0,
        180.0001,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
    };
    for (const double sector : invalid_sectors) {
        request.options.pruning_sector_degrees = sector;
        const auto result = router.optimize(request);
        REQUIRE(!result.has_value());
        REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
    }

    request.options.pruning_sector_degrees = 2.0;
    REQUIRE(router.optimize(request).has_value());

    // The width is only consulted by bearing-sector pruning, so the default
    // grid strategy stays unaffected by a stale value.
    request.options.pruning_strategy =
        sailroute::PruningStrategy::destination_distance_grid;
    request.options.pruning_sector_degrees = 0.0;
    REQUIRE(router.optimize(request).has_value());
}

TEST_CASE("accuracy options reject invalid library values") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};
    sailroute::RouteRequest request = routing_request(1U, false);
    const auto require_invalid = [&router](const sailroute::RouteRequest& value) {
        const auto result = router.optimize(value);
        REQUIRE(!result.has_value());
        REQUIRE(result.error().code == sailroute::ErrorCode::invalid_argument);
    };

    request.options.maneuver.tack_penalty = std::chrono::seconds{-1};
    require_invalid(request);
    request.options.maneuver.tack_penalty = std::chrono::seconds::zero();
    request.options.maneuver.gybe_penalty = std::chrono::seconds{-1};
    require_invalid(request);
    request.options.maneuver.gybe_penalty = std::chrono::seconds::zero();

    for (const double angle :
         {-1.0,
          180.0001,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()}) {
        request.options.maneuver.downwind_true_wind_angle_degrees = angle;
        require_invalid(request);
    }
    request.options.maneuver.downwind_true_wind_angle_degrees = 150.0;

    request.options.midpoint_wind_sampling_threshold =
        std::chrono::minutes{-1};
    require_invalid(request);
    request.options.midpoint_wind_sampling_threshold =
        std::chrono::minutes::zero();

    for (const double wind_speed :
         {0.0,
          -1.0,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()}) {
        request.options.maximum_true_wind_speed_knots = wind_speed;
        require_invalid(request);
    }
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

namespace {

// The routing fixture blows a steady 10 m/s from the north, so a leg due north
// can only be sailed by beating and is the scenario where maneuver penalties,
// velocity-made-good headings, and board-aware pruning all matter.
sailroute::RouteRequest upwind_request(std::size_t worker_count = 0U) {
    const auto departure = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    REQUIRE(departure.has_value());

    sailroute::RouteRequest request;
    request.start = {0.4, 1.0};
    request.destination = {1.2, 1.0};
    request.departure_time = departure.value();
    request.options.time_step = std::chrono::minutes{30};
    request.options.use_routing_intervals = false;
    request.options.heading_step_degrees = 10.0;
    request.options.arrival_radius_nautical_miles = 1.0;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 4;
    request.options.worker_count = worker_count;
    request.options.maximum_route_duration = std::chrono::hours{12};
    return request;
}

std::size_t count_board_changes(const sailroute::RouteResult& route) {
    std::size_t changes = 0U;
    int previous_board = 0;
    for (const sailroute::RoutePoint& point : route.points) {
        double relative = std::fmod(
            point.heading_degrees - point.true_wind_direction_degrees, 360.0);
        if (relative < 0.0) {
            relative += 360.0;
        }
        const int board = relative > 0.0 && relative < 180.0
            ? 1
            : (relative > 180.0 ? -1 : 0);
        if (board != 0 && previous_board != 0 && board != previous_board) {
            ++changes;
        }
        if (board != 0) {
            previous_board = board;
        }
    }
    return changes;
}

sailroute::RouteResult must_route(
    const sailroute::Router& router,
    const sailroute::RouteRequest& request) {
    auto result = router.optimize(request);
    if (!result.has_value()) {
        throw std::runtime_error(result.error().message);
    }
    return std::move(result.value());
}

}  // namespace

TEST_CASE("accuracy options default to the unmodified search") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const sailroute::RouteResult baseline = must_route(router, upwind_request());

    sailroute::RouteRequest explicit_defaults = upwind_request();
    explicit_defaults.options.maneuver = sailroute::ManeuverPenalties{};
    explicit_defaults.options.heading_augmentation =
        sailroute::HeadingAugmentation::none;
    explicit_defaults.options.wind_sampling = sailroute::WindSampling::segment_start;
    explicit_defaults.options.polar_angle_interpolation =
        sailroute::PolarAngleInterpolation::linear;
    explicit_defaults.options.above_polar_range =
        sailroute::AbovePolarRangePolicy::clamp;
    explicit_defaults.options.pruning_strategy =
        sailroute::PruningStrategy::destination_distance_grid;

    require_same_route(baseline, must_route(router, explicit_defaults));
}

TEST_CASE("zero maneuver penalties leave the route untouched") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest zero_penalties = upwind_request();
    zero_penalties.options.maneuver.tack_penalty = std::chrono::seconds{0};
    zero_penalties.options.maneuver.gybe_penalty = std::chrono::seconds{0};

    require_same_route(
        must_route(router, upwind_request()),
        must_route(router, zero_penalties));
}

TEST_CASE("maneuver penalties suppress free tacking") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const sailroute::RouteResult unpenalised = must_route(router, upwind_request());

    sailroute::RouteRequest penalised = upwind_request();
    penalised.options.maneuver.tack_penalty = std::chrono::seconds{600};
    penalised.options.maneuver.gybe_penalty = std::chrono::seconds{600};
    const sailroute::RouteResult expensive = must_route(router, penalised);

    REQUIRE(count_board_changes(expensive) <= count_board_changes(unpenalised));
}

TEST_CASE("maneuver penalties are charged against the step") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    constexpr std::chrono::seconds penalty{300};
    sailroute::RouteRequest penalised = upwind_request();
    penalised.options.maneuver.tack_penalty = penalty;
    penalised.options.maneuver.gybe_penalty = penalty;
    const sailroute::RouteResult route = must_route(router, penalised);

    const auto board_of = [](const sailroute::RoutePoint& point) {
        double relative = std::fmod(
            point.heading_degrees - point.true_wind_direction_degrees, 360.0);
        if (relative < 0.0) {
            relative += 360.0;
        }
        return relative > 0.0 && relative < 180.0
            ? 1
            : (relative > 180.0 ? -1 : 0);
    };

    std::size_t penalised_legs = 0U;
    for (std::size_t index = 1U; index < route.points.size(); ++index) {
        const sailroute::RoutePoint& previous = route.points[index - 1U];
        const sailroute::RoutePoint& current = route.points[index];
        const double elapsed_seconds =
            std::chrono::duration<double>(current.time - previous.time).count();
        const double distance =
            current.cumulative_distance_nautical_miles -
            previous.cumulative_distance_nautical_miles;
        const double sailing_seconds =
            distance / current.boat_speed_knots * 3600.0;

        // Distance is always speed times the time actually spent sailing, so
        // any surplus in the leg is exactly the maneuver charge.
        const double surplus = elapsed_seconds - sailing_seconds;
        REQUIRE(surplus > -1.0);

        const bool changed_board = index > 1U && board_of(current) != 0 &&
            board_of(previous) != 0 &&
            board_of(current) != board_of(previous);
        if (changed_board) {
            ++penalised_legs;
            REQUIRE(std::abs(surplus - static_cast<double>(penalty.count())) < 1.5);
        } else {
            REQUIRE(surplus < 1.0);
        }
    }

    // The fixture is a dead beat, so the route has to change boards at least
    // once; otherwise this test would assert nothing.
    REQUIRE(penalised_legs > 0U);
}

TEST_CASE("maneuver penalties keep both boards through pruning") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest penalised = upwind_request();
    penalised.options.maneuver.tack_penalty = std::chrono::seconds{120};
    const sailroute::RouteResult board_aware = must_route(router, penalised);

    // Splitting each bucket by board retains strictly more of the frontier.
    REQUIRE(
        board_aware.diagnostics.retained_candidates >=
        must_route(router, upwind_request()).diagnostics.retained_candidates);
}

TEST_CASE("heading augmentation widens the evaluated heading set") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const sailroute::RouteResult plain = must_route(router, upwind_request());

    for (const auto mode : {
             sailroute::HeadingAugmentation::destination_bearing,
             sailroute::HeadingAugmentation::velocity_made_good,
             sailroute::HeadingAugmentation::
                 destination_bearing_and_velocity_made_good}) {
        sailroute::RouteRequest augmented = upwind_request();
        augmented.options.heading_augmentation = mode;
        const sailroute::RouteResult route = must_route(router, augmented);
        REQUIRE(
            route.diagnostics.generated_candidates >
            plain.diagnostics.generated_candidates);
        REQUIRE(route.completion == sailroute::RouteCompletion::destination_reached);
    }
}

TEST_CASE("velocity made good headings are sailed off the fixed grid") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest augmented = upwind_request();
    augmented.options.heading_augmentation =
        sailroute::HeadingAugmentation::velocity_made_good;
    const sailroute::RouteResult route = must_route(router, augmented);

    bool off_grid = false;
    for (const sailroute::RoutePoint& point : route.points) {
        const double remainder = std::fmod(point.heading_degrees, 10.0);
        if (std::abs(remainder) > 1e-9 && std::abs(remainder - 10.0) > 1e-9) {
            off_grid = true;
        }
    }
    REQUIRE(off_grid);
}

TEST_CASE("midpoint wind sampling produces a usable route") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest midpoint = upwind_request();
    midpoint.options.wind_sampling = sailroute::WindSampling::midpoint;
    const sailroute::RouteResult route = must_route(router, midpoint);
    REQUIRE(route.completion == sailroute::RouteCompletion::destination_reached);

    // The threshold suppresses the second sample on shorter steps, which in this
    // fixture leaves the first-order route untouched.
    sailroute::RouteRequest thresholded = midpoint;
    thresholded.options.midpoint_wind_sampling_threshold =
        std::chrono::minutes{600};
    require_same_route(
        must_route(router, upwind_request()),
        must_route(router, thresholded));
}

TEST_CASE("monotone cubic polar interpolation routes to the destination") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest cubic = upwind_request();
    cubic.options.polar_angle_interpolation =
        sailroute::PolarAngleInterpolation::monotone_cubic;
    const sailroute::RouteResult route = must_route(router, cubic);
    REQUIRE(route.completion == sailroute::RouteCompletion::destination_reached);
    REQUIRE(route.points.size() > 1U);
}

TEST_CASE("wind speed envelope stops the vessel sailing") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest capped = upwind_request();
    capped.options.maximum_true_wind_speed_knots = 1.0;
    const auto route = router.optimize(capped);
    REQUIRE(!route.has_value());

    // A limit above the forecast wind changes nothing.
    sailroute::RouteRequest generous = upwind_request();
    generous.options.maximum_true_wind_speed_knots = 200.0;
    require_same_route(
        must_route(router, upwind_request()),
        must_route(router, generous));
}

TEST_CASE("above polar range policy only bites past the last column") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    // The fixture's roughly 19 knot wind is inside the default polar, so
    // refusing to extrapolate leaves the route unchanged.
    sailroute::RouteRequest refuse = upwind_request();
    refuse.options.above_polar_range = sailroute::AbovePolarRangePolicy::no_speed;
    require_same_route(
        must_route(router, upwind_request()),
        must_route(router, refuse));
}

TEST_CASE("bearing sector pruning reaches the destination") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    sailroute::RouteRequest sectors = upwind_request();
    sectors.options.pruning_strategy = sailroute::PruningStrategy::bearing_sectors;
    sectors.options.pruning_sector_degrees = 5.0;
    const sailroute::RouteResult route = must_route(router, sectors);
    REQUIRE(route.completion == sailroute::RouteCompletion::destination_reached);
    REQUIRE(route.points.front().position.latitude_degrees == 0.4);
}

TEST_CASE("accuracy options stay deterministic across worker counts") {
    const RoutingGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const sailroute::Router router{weather.value()};

    const auto configure = [](sailroute::RouteRequest& request) {
        request.options.maneuver.tack_penalty = std::chrono::seconds{90};
        request.options.maneuver.gybe_penalty = std::chrono::seconds{45};
        request.options.heading_augmentation = sailroute::HeadingAugmentation::
            destination_bearing_and_velocity_made_good;
        request.options.wind_sampling = sailroute::WindSampling::midpoint;
        request.options.polar_angle_interpolation =
            sailroute::PolarAngleInterpolation::monotone_cubic;
        request.options.pruning_strategy =
            sailroute::PruningStrategy::bearing_sectors;
        request.options.pruning_sector_degrees = 4.0;
    };

    sailroute::RouteRequest serial = upwind_request(0U);
    configure(serial);
    const sailroute::RouteResult reference = must_route(router, serial);

    for (const std::size_t workers : {std::size_t{1}, std::size_t{2}, std::size_t{4}}) {
        sailroute::RouteRequest parallel = upwind_request(workers);
        configure(parallel);
        require_same_route(reference, must_route(router, parallel));
    }
}
