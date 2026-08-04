#include "sailroute/router.hpp"
#include "sailroute/serialization.hpp"
#include "sailroute/time.hpp"

#include <eccodes.h>

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;

void require_codes(int status, const char* operation) {
    if (status != CODES_SUCCESS) {
        throw std::runtime_error(
            std::string{operation} + ": " + codes_get_error_message(status));
    }
}

struct GridSpec {
    std::string filename;
    long longitude_count{};
    long latitude_count{};
    double first_latitude{};
    double first_longitude{};
    double last_latitude{};
    double last_longitude{};
    double longitude_increment{};
    double latitude_increment{};
    long final_forecast_hour{};
};

class CorpusGrib {
public:
    explicit CorpusGrib(GridSpec spec)
        : path_(spec.filename) {
        write_message(spec, "10u", 0L, 0.0, "w");
        write_message(spec, "10v", 0L, -10.0, "a");
        write_message(spec, "10u", spec.final_forecast_hour, 0.0, "a");
        write_message(spec, "10v", spec.final_forecast_hour, -10.0, "a");
    }

    ~CorpusGrib() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    CorpusGrib(const CorpusGrib&) = delete;
    CorpusGrib& operator=(const CorpusGrib&) = delete;

    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    static void write_message(
        const GridSpec& spec,
        const char* short_name,
        long forecast_hour,
        double value,
        const char* mode) {
        codes_handle* handle =
            codes_grib_handle_new_from_samples(nullptr, "regular_ll_sfc_grib2");
        if (handle == nullptr) {
            throw std::runtime_error("unable to create compatibility GRIB");
        }

        try {
            require_codes(
                codes_set_long(handle, "Ni", spec.longitude_count),
                "set Ni");
            require_codes(
                codes_set_long(handle, "Nj", spec.latitude_count),
                "set Nj");
            require_codes(
                codes_set_double(
                    handle,
                    "latitudeOfFirstGridPointInDegrees",
                    spec.first_latitude),
                "set first latitude");
            require_codes(
                codes_set_double(
                    handle,
                    "longitudeOfFirstGridPointInDegrees",
                    spec.first_longitude),
                "set first longitude");
            require_codes(
                codes_set_double(
                    handle,
                    "latitudeOfLastGridPointInDegrees",
                    spec.last_latitude),
                "set last latitude");
            require_codes(
                codes_set_double(
                    handle,
                    "longitudeOfLastGridPointInDegrees",
                    spec.last_longitude),
                "set last longitude");
            require_codes(
                codes_set_double(
                    handle,
                    "iDirectionIncrementInDegrees",
                    spec.longitude_increment),
                "set longitude increment");
            require_codes(
                codes_set_double(
                    handle,
                    "jDirectionIncrementInDegrees",
                    spec.latitude_increment),
                "set latitude increment");
            require_codes(codes_set_long(handle, "iScansNegatively", 0), "set i scan");
            require_codes(codes_set_long(handle, "jScansPositively", 0), "set j scan");
            require_codes(codes_set_long(handle, "dataDate", 20260714), "set date");
            require_codes(codes_set_long(handle, "dataTime", 1200), "set time");
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

            const std::size_t value_count =
                static_cast<std::size_t>(
                    spec.longitude_count * spec.latitude_count);
            const std::vector<double> values(value_count, value);
            require_codes(
                codes_set_double_array(
                    handle,
                    "values",
                    values.data(),
                    values.size()),
                "set wind values");
            require_codes(
                codes_write_message(handle, spec.filename.c_str(), mode),
                "write compatibility GRIB");
        } catch (...) {
            codes_handle_delete(handle);
            throw;
        }
        codes_handle_delete(handle);
    }

    std::filesystem::path path_;
};

[[nodiscard]] sailroute::TimePoint departure_time() {
    const auto parsed = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    if (!parsed.has_value()) {
        throw std::runtime_error(parsed.error().message);
    }
    return parsed.value();
}

[[nodiscard]] sailroute::RouteRequest request(
    sailroute::Coordinate start,
    sailroute::Coordinate destination,
    std::size_t workers,
    std::chrono::hours maximum_duration = 24h) {
    sailroute::RouteRequest value;
    value.start = start;
    value.destination = destination;
    value.departure_time = departure_time();
    value.options.time_step = 30min;
    value.options.use_routing_intervals = false;
    value.options.heading_step_degrees = 10.0;
    value.options.arrival_radius_nautical_miles = 0.5;
    value.options.spatial_bucket_nautical_miles = 3.0;
    value.options.max_nodes_per_bucket = 3U;
    value.options.worker_count = workers;
    value.options.maximum_route_duration = maximum_duration;
    value.options.capture_isochrones = true;
    return value;
}

[[nodiscard]] sailroute::Router load_router(const CorpusGrib& grib) {
    auto weather = sailroute::WeatherDataset::load(grib.path());
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }
    return sailroute::Router{std::move(weather.value())};
}

void print_result(
    std::string_view name,
    const sailroute::Result<sailroute::RouteResult>& result) {
    std::cout << "scenario=" << name << '\n';
    if (!result.has_value()) {
        std::cout << "error=" << sailroute::to_string(result.error().code)
                  << ':' << result.error().message << '\n';
        return;
    }
    const auto json = sailroute::route_to_json(result.value());
    if (!json.has_value()) {
        throw std::runtime_error(json.error().message);
    }
    std::cout << json.value() << '\n';
}

void print_progress(
    std::string_view name,
    const sailroute::RoutingProgress& progress) {
    std::cout << "progress=" << name << ':'
              << progress.isochrone.time.time_since_epoch().count() << ':'
              << progress.isochrone.points.size() << ':'
              << progress.provisional_route.size() << ':'
              << progress.destination_front.points.size() << ':'
              << progress.destination_front.segments.size() << ':'
              << progress.diagnostics.expanded_nodes << ':'
              << progress.diagnostics.generated_candidates << ':'
              << progress.diagnostics.retained_candidates << ':'
              << progress.diagnostics.time_steps << '\n';
}

void run_regional_corpus(const CorpusGrib& regional) {
    const sailroute::Router router = load_router(regional);

    for (const std::size_t workers : {1U, 4U, 0U}) {
        auto value = request({1.0, 0.5}, {1.0, 1.0}, workers);
        print_result(
            workers == 1U ? "regional-workers-1"
                          : workers == 4U ? "regional-workers-4"
                                          : "regional-workers-auto",
            router.optimize(value));
    }

    auto scheduled = request({1.0, 0.5}, {1.0, 1.0}, 1U);
    scheduled.options.use_routing_intervals = true;
    scheduled.options.routing_intervals = {
        {20min, 1h},
        {40min, std::nullopt},
    };
    print_result("scheduled-intervals", router.optimize(scheduled));

    auto accuracy = request({1.0, 0.5}, {1.0, 1.0}, 1U);
    accuracy.options.maneuver.tack_penalty = 15s;
    accuracy.options.maneuver.gybe_penalty = 10s;
    accuracy.options.heading_augmentation =
        sailroute::HeadingAugmentation::destination_bearing;
    accuracy.options.wind_sampling = sailroute::WindSampling::midpoint;
    accuracy.options.polar_angle_interpolation =
        sailroute::PolarAngleInterpolation::monotone_cubic;
    accuracy.options.pruning_strategy = sailroute::PruningStrategy::bearing_sectors;
    accuracy.options.pruning_sector_degrees = 5.0;
    print_result("accuracy-controls", router.optimize(accuracy));

    std::size_t eligibility_calls = 0U;
    auto eligible = request({1.0, 0.5}, {1.0, 1.0}, 4U);
    eligible.options.segment_eligibility =
        [&eligibility_calls](const sailroute::RouteSegmentView&) {
            ++eligibility_calls;
            return true;
        };
    const auto eligible_result = router.optimize(eligible);
    std::cout << "eligibility-calls=" << eligibility_calls << '\n';
    print_result("segment-eligibility", eligible_result);

    auto progress_request = request({1.0, 0.5}, {1.0, 1.0}, 1U);
    print_result(
        "progress-order",
        router.optimize(
            progress_request,
            [](const sailroute::RoutingProgress& progress) {
                print_progress("regional", progress);
            }));

    std::size_t cancellation_updates = 0U;
    auto cancelled = router.optimize(
        progress_request,
        [&cancellation_updates](const sailroute::RoutingProgress&) {
            ++cancellation_updates;
            return sailroute::RoutingProgressDecision::cancel;
        });
    std::cout << "cancellation-updates=" << cancellation_updates << '\n';
    print_result("cancellation", cancelled);
}

void run_global_corpus(const CorpusGrib& global) {
    const sailroute::Router router = load_router(global);
    print_result(
        "antimeridian",
        router.optimize(request({0.0, 179.5}, {0.0, -179.5}, 1U)));
    print_result(
        "high-latitude",
        router.optimize(request({70.0, 0.0}, {70.0, 2.0}, 1U)));
}

void run_exhaustion_corpus(const CorpusGrib& short_forecast) {
    const sailroute::Router router = load_router(short_forecast);
    print_result(
        "forecast-exhaustion",
        router.optimize(
            request({1.0, 0.25}, {1.0, 1.75}, 1U, 12h)));
}

}  // namespace

int main() {
    try {
        std::cout << "baseline=v0.3.2@cd99342cdaeb6725639f6ae53a384db50b0e0ad0\n";
        const CorpusGrib regional{
            GridSpec{
                "sailroute-compat-regional.grib",
                3L,
                3L,
                2.0,
                0.0,
                0.0,
                2.0,
                1.0,
                1.0,
                12L,
            }};
        const CorpusGrib global{
            GridSpec{
                "sailroute-compat-global.grib",
                8L,
                5L,
                80.0,
                0.0,
                -80.0,
                315.0,
                45.0,
                40.0,
                24L,
            }};
        const CorpusGrib short_forecast{
            GridSpec{
                "sailroute-compat-short.grib",
                3L,
                3L,
                2.0,
                0.0,
                0.0,
                2.0,
                1.0,
                1.0,
                1L,
            }};

        run_regional_corpus(regional);
        run_global_corpus(global);
        run_exhaustion_corpus(short_forecast);
    } catch (const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
    return 0;
}
