#include "strategic_benchmark.hpp"

#include "sailroute/time.hpp"

#include <eccodes.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace sailroute::benchmarks {
namespace {

// Both tacks see 8 kt TWS for five hours. The initially losing side can enter
// the diagonal 32 kt corridor after hour five, beyond the initial 3h probe.
class DelayedCorridorFixture {
public:
    explicit DelayedCorridorFixture(bool mirrored)
        : reflection_(mirrored ? -1.0 : 1.0),
          stem_("sailroute-strategic-" + std::to_string(
              std::chrono::steady_clock::now().time_since_epoch().count())),
          grib_path_(std::filesystem::current_path() / (stem_ + ".grib")),
          polar_path_(std::filesystem::current_path() / (stem_ + ".csv")) {
        try {
            bool first = true;
            for (const long hour : {0L, 5L, 6L, 18L}) {
                for (const char* component : {"10u", "10v"}) {
                    write_wind(component, hour, first ? "w" : "a");
                    first = false;
                }
            }
            std::ofstream polar(polar_path_);
            // Exact diagonal actions sail at 6 kt in 8 kt TWS and 24 kt in
            // 32 kt TWS. Cardinal actions have zero speed, not clamped speed.
            polar << "TWA/TWS,0,8,32,50\n"
                     "0,0,0,0,0\n44,0,0,0,0\n45,0,6,24,24\n46,0,0,0,0\n"
                     "134,0,0,0,0\n135,0,6,24,24\n136,0,0,0,0\n180,0,0,0,0\n";
            if (!polar) {
                throw std::runtime_error("cannot write strategic polar");
            }
        } catch (...) {
            remove_files();
            throw;
        }
    }

    DelayedCorridorFixture(const DelayedCorridorFixture&) = delete;
    DelayedCorridorFixture& operator=(const DelayedCorridorFixture&) = delete;
    ~DelayedCorridorFixture() { remove_files(); }

    StrategicScenario load(bool with_island) const {
        auto weather = WeatherDataset::load(grib_path_);
        if (!weather) throw std::runtime_error(weather.error().message);
        auto polar = VesselPolar::load(polar_path_);
        if (!polar) throw std::runtime_error(polar.error().message);
        RouteRequest request;
        request.start = {0.0, 0.0};
        request.destination = {reflection_ * 0.35, 1.0};
        request.departure_time =
            parse_utc_time("2026-07-14T00:00:00Z").value();
        auto& options = request.options;
        options.time_step = std::chrono::minutes{60};
        options.use_routing_intervals = false;
        options.heading_step_degrees = 45.0;
        options.heading_augmentation = HeadingAugmentation::none;
        options.maximum_integration_step = std::chrono::minutes{15};
        options.wind_sampling = WindSampling::midpoint;
        options.boat_speed_factor = 1.0;
        options.above_polar_range = AbovePolarRangePolicy::no_speed;
        options.arrival_radius_nautical_miles = 2.0;
        options.spatial_bucket_nautical_miles = 100.0;
        options.max_nodes_per_bucket = 3U;
        options.maximum_route_duration = std::chrono::hours{18};
        options.worker_count = 1U;
        options.strategic_retention = true;
        // With westward travel excluded, the full feasible fixed heading grid
        // is exactly {45,135}. This makes a genuinely exhaustive test affordable.
        options.segment_eligibility = [](const RouteSegmentView& leg) {
            return leg.candidate.position.longitude_degrees >=
                leg.parent.position.longitude_degrees;
        };
        RoutingEnvironment environment;
        if (with_island) {
            ExclusionZone island;
            island.identifier = "synthetic-island";
            island.source = "strategic regression";
            island.polygons.push_back(ExclusionPolygon{
                ExclusionRing{{
                    {-0.04, 0.12}, {-0.04, 0.55}, {0.04, 0.55}, {0.04, 0.12}}},
                {}});
            auto zones = ExclusionZoneSet::create(
                {std::move(island)}, {"synthetic island", "strategic regression", "1"});
            if (!zones) throw std::runtime_error(zones.error().message);
            environment.exclusions.zones = std::move(zones.value());
        }
        return {std::move(weather.value()), std::move(polar.value()),
                std::move(environment), std::move(request)};
    }

private:
    void remove_files() const noexcept {
        std::error_code ignored;
        std::filesystem::remove(grib_path_, ignored);
        std::filesystem::remove(polar_path_, ignored);
    }

    static void check(int status, const char* operation) {
        if (status != CODES_SUCCESS) {
            throw std::runtime_error(
                std::string{operation} + ": " + codes_get_error_message(status));
        }
    }

    void write_wind(const char* component, long hour, const char* mode) const {
        codes_handle* handle =
            codes_grib_handle_new_from_samples(nullptr, "regular_ll_sfc_grib2");
        if (handle == nullptr) throw std::runtime_error("cannot create strategic GRIB handle");
        try {
            constexpr long count = 121L;
            constexpr double spacing = 0.025;
            check(codes_set_long(handle, "Ni", count), "set Ni");
            check(codes_set_long(handle, "Nj", count), "set Nj");
            check(codes_set_long(handle, "iScansNegatively", 0L), "set i scan");
            check(codes_set_long(handle, "jScansPositively", 0L), "set j scan");
            check(codes_set_double(handle, "latitudeOfFirstGridPointInDegrees", 1.5),
                  "set first latitude");
            check(codes_set_double(handle, "latitudeOfLastGridPointInDegrees", -1.5),
                  "set last latitude");
            check(codes_set_double(handle, "longitudeOfFirstGridPointInDegrees", -0.5),
                  "set first longitude");
            check(codes_set_double(handle, "longitudeOfLastGridPointInDegrees", 2.5),
                  "set last longitude");
            check(codes_set_double(handle, "iDirectionIncrementInDegrees", spacing),
                  "set longitude increment");
            check(codes_set_double(handle, "jDirectionIncrementInDegrees", spacing),
                  "set latitude increment");
            check(codes_set_long(handle, "dataDate", 20260714L), "set date");
            check(codes_set_long(handle, "dataTime", 0L), "set time");
            check(codes_set_long(handle, "forecastTime", hour), "set hour");
            std::size_t size = std::char_traits<char>::length(component);
            check(codes_set_string(handle, "shortName", component, &size), "set component");
            check(codes_set_long(handle, "level", 10L), "set height");
            std::vector<double> values;
            values.reserve(static_cast<std::size_t>(count * count));
            for (long row = 0L; row < count; ++row) {
                const double latitude = reflection_ *
                    (1.5 - static_cast<double>(row) * spacing);
                for (long column = 0L; column < count; ++column) {
                    const double longitude = -0.5 + static_cast<double>(column) * spacing;
                    const double boundary = -0.25 + std::max(0.0, longitude - 0.30);
                    // Include the exact diagonal grid boundary symmetrically.
                    const double knots =
                        hour >= 6L && latitude <= boundary + 1e-9 ? 32.0 : 8.0;
                    values.push_back(component[2] == 'u' ? -knots * 1852.0 / 3600.0 : 0.0);
                }
            }
            check(codes_set_double_array(handle, "values", values.data(), values.size()),
                  "set wind values");
            check(codes_write_message(handle, grib_path_.string().c_str(), mode), "write GRIB");
        } catch (...) {
            codes_handle_delete(handle);
            throw;
        }
        codes_handle_delete(handle);
    }

    double reflection_;
    std::string stem_;
    std::filesystem::path grib_path_;
    std::filesystem::path polar_path_;
};

}  // namespace

StrategicScenario make_strategic_scenario(bool mirrored, bool with_island) {
    return DelayedCorridorFixture(mirrored).load(with_island);
}

void report_strategic_benchmark(
    std::ostream& output, bool mirrored, bool with_island, std::size_t iterations,
    bool replay_waypoints) {
    if (iterations == 0U) throw std::invalid_argument("strategic iterations must be positive");
    const auto scenario = make_strategic_scenario(mirrored, with_island);
    const Router router(scenario.weather, scenario.polar, scenario.environment);
    struct Policy {
        const char* name;
        bool strategic;
        std::size_t nodes_per_bucket;
    };
    const std::array policies{
        Policy{"greedy", false, 1U}, Policy{"heading-only", false, 3U},
        Policy{"strategic", true, 3U}, Policy{"strategic-wide", true, 12U}};
    output << "scenario,policy,iteration,operation,status,elapsed_seconds,wall_ms,"
              "generated_candidates,retained_candidates,expanded_nodes,detail\n";
    const std::string name = std::string{with_island ? "island-" : "corridor-"} +
        (mirrored ? "north" : "south");
    for (const auto& policy : policies) {
        auto request = scenario.request;
        request.options.strategic_retention = policy.strategic;
        request.options.max_nodes_per_bucket = policy.nodes_per_bucket;
        for (std::size_t iteration = 0U; iteration < iterations; ++iteration) {
            const auto write = [&](const Result<RouteResult>& result, const char* operation,
                                   std::chrono::steady_clock::time_point started) {
                output << name << ',' << policy.name << ',' << iteration << ',' << operation << ',';
                const double milliseconds = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - started).count();
                if (!result) {
                    output << to_string(result.error().code) << ",," << milliseconds << ",,,,\"";
                    for (const char character : result.error().message) {
                        if (character == '"') output << '"';
                        output << character;
                    }
                    output << "\"\n";
                } else {
                    const auto& route = result.value();
                    output << (route.completion == RouteCompletion::destination_reached
                        ? "arrived" : "incomplete") << ','
                           << (route.arrival_time - route.departure_time).count() << ','
                           << milliseconds << ',' << route.diagnostics.generated_candidates << ','
                           << route.diagnostics.retained_candidates << ','
                           << route.diagnostics.expanded_nodes << ",\n";
                }
            };
            const auto started = std::chrono::steady_clock::now();
            const auto route = router.optimize(request);
            write(route, "search", started);
            if (route && route.value().completion == RouteCompletion::destination_reached) {
                std::vector<SailingAction> actions;
                const auto& points = route.value().points;
                for (auto time = route.value().departure_time;
                     time < route.value().arrival_time; time += request.options.time_step) {
                    const auto first = std::upper_bound(
                        points.begin(), points.end(), time,
                        [](TimePoint boundary, const RoutePoint& point) {
                            return boundary < point.time;
                        });
                    if (first == points.end()) {
                        throw std::runtime_error("missing strategic action boundary");
                    }
                    // Preserve the original one-hour decision, including the
                    // final action. The common evaluator determines early arrival.
                    actions.push_back({first->heading_degrees, request.options.time_step});
                }
                const auto replay_started = std::chrono::steady_clock::now();
                const auto replay = router.evaluate_actions(request, actions);
                write(replay, "action-replay", replay_started);
                if (replay_waypoints) {
                    std::vector<Coordinate> waypoints;
                    for (std::size_t index = 1U; index < points.size(); ++index) {
                        waypoints.push_back(points[index].position);
                    }
                    const auto waypoint_started = std::chrono::steady_clock::now();
                    const auto geometric = router.evaluate_route(request, waypoints);
                    write(geometric, "waypoint-replay", waypoint_started);
                }
            }
        }
    }
}

}  // namespace sailroute::benchmarks
