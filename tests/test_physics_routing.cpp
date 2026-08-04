#include "sailroute/environment.hpp"
#include "sailroute/router.hpp"
#include "sailroute/serialization.hpp"
#include "sailroute/time.hpp"

#include "../src/routing/geodesy.hpp"
#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using sailroute::Coordinate;
using sailroute::CurrentVector;
using sailroute::EnvironmentGridSpec;
using sailroute::ErrorCode;
using sailroute::ExclusionPolygon;
using sailroute::ExclusionRing;
using sailroute::ExclusionZone;
using sailroute::ExclusionZoneSet;
using sailroute::LandmaskMetadata;
using sailroute::MissingDataPolicy;
using sailroute::ProviderMetadata;
using sailroute::RoutePoint;
using sailroute::RouteRequest;
using sailroute::RouteResult;
using sailroute::Router;
using sailroute::RoutingEnvironment;
using sailroute::RoutingSolver;
using sailroute::SeaStateInput;
using sailroute::SeaStatePerformanceModel;
using sailroute::SignedDistanceLandmask;
using sailroute::WaveState;
using sailroute::WeatherDataset;

WeatherDataset load_weather(
    const sailroute::test::ConstantWindGribFixture& fixture) {
    auto weather = WeatherDataset::load(fixture.path());
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }
    return std::move(weather.value());
}

RouteRequest base_request(
    std::size_t worker_count = 1U,
    RoutingSolver solver = RoutingSolver::isochrone_beam) {
    const auto departure = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    REQUIRE(departure.has_value());

    RouteRequest request;
    request.start = {1.0, 0.5};
    request.destination = {1.0, 1.0};
    request.departure_time = departure.value();
    request.options.solver = solver;
    request.options.time_step = std::chrono::minutes{30};
    request.options.use_routing_intervals = false;
    request.options.heading_step_degrees = 10.0;
    request.options.arrival_radius_nautical_miles = 0.5;
    request.options.spatial_bucket_nautical_miles = 3.0;
    request.options.max_nodes_per_bucket = 3;
    request.options.worker_count = worker_count;
    request.options.maximum_route_duration = std::chrono::hours{12};
    if (solver == RoutingSolver::time_dependent_lattice) {
        request.options.lattice.subdivision_level = 6U;
        request.options.lattice.refinement_levels = 0U;
        request.options.lattice.corridor_width_nautical_miles = 200.0;
    }
    return request;
}

RouteResult route_or_throw(const Router& router, const RouteRequest& request) {
    auto result = router.optimize(request);
    if (!result.has_value()) {
        throw std::runtime_error(
            std::string{sailroute::to_string(result.error().code)} + ": " +
            result.error().message);
    }
    return std::move(result.value());
}

void require_identical_route(const RouteResult& left, const RouteResult& right) {
    REQUIRE(left.departure_time == right.departure_time);
    REQUIRE(left.arrival_time == right.arrival_time);
    REQUIRE(left.completion == right.completion);
    REQUIRE(left.diagnostics.expanded_nodes == right.diagnostics.expanded_nodes);
    REQUIRE(
        left.diagnostics.generated_candidates ==
        right.diagnostics.generated_candidates);
    REQUIRE(
        left.diagnostics.retained_candidates ==
        right.diagnostics.retained_candidates);
    REQUIRE(left.diagnostics.time_steps == right.diagnostics.time_steps);
    REQUIRE(left.points.size() == right.points.size());
    for (std::size_t index = 0U; index < left.points.size(); ++index) {
        const RoutePoint& first = left.points[index];
        const RoutePoint& second = right.points[index];
        REQUIRE(
            first.position.latitude_degrees == second.position.latitude_degrees);
        REQUIRE(
            first.position.longitude_degrees ==
            second.position.longitude_degrees);
        REQUIRE(first.time == second.time);
        REQUIRE(first.heading_degrees == second.heading_degrees);
        REQUIRE(first.boat_speed_knots == second.boat_speed_knots);
        REQUIRE(first.true_wind_speed_knots == second.true_wind_speed_knots);
        REQUIRE(
            first.true_wind_direction_degrees ==
            second.true_wind_direction_degrees);
        REQUIRE(
            first.cumulative_distance_nautical_miles ==
            second.cumulative_distance_nautical_miles);
    }
}

ProviderMetadata make_metadata(std::string name) {
    return ProviderMetadata{std::move(name), "unit test", "1"};
}

class RecordingSeaStateModel final : public SeaStatePerformanceModel {
public:
    RecordingSeaStateModel() : metadata_(make_metadata("recording")) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }

    [[nodiscard]] double derated_speed_knots(
        const SeaStateInput& input) const override {
        std::lock_guard lock(mutex_);
        inputs_.push_back(input);
        return input.flat_water_speed_knots;
    }

    [[nodiscard]] std::vector<SeaStateInput> inputs() const {
        std::lock_guard lock(mutex_);
        return inputs_;
    }

private:
    ProviderMetadata metadata_;
    mutable std::mutex mutex_;
    mutable std::vector<SeaStateInput> inputs_;
};

RoutingEnvironment current_environment(
    CurrentVector current,
    MissingDataPolicy policy = MissingDataPolicy::fail_route) {
    RoutingEnvironment environment;
    auto provider = sailroute::make_uniform_current_provider(
        current, make_metadata("current"));
    REQUIRE(provider.has_value());
    environment.currents.provider = provider.value();
    environment.currents.missing_data_policy = policy;
    return environment;
}

RoutingEnvironment wave_environment(WaveState wave) {
    RoutingEnvironment environment;
    auto provider =
        sailroute::make_uniform_wave_provider(wave, make_metadata("wave"));
    REQUIRE(provider.has_value());
    auto model = sailroute::make_wave_height_derating_model();
    REQUIRE(model.has_value());
    environment.waves.provider = provider.value();
    environment.waves.model = model.value();
    return environment;
}

// A landmask whose only land is a meridian strip, optionally limited in
// latitude so a route can still get round it.
SignedDistanceLandmask barrier_landmask(
    double land_longitude,
    double half_width_degrees,
    std::optional<double> latitude_half_extent = std::nullopt) {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = -2.0;
    spec.west_longitude_degrees = -2.0;
    spec.latitude_step_degrees = 0.05;
    spec.longitude_step_degrees = 0.05;
    spec.latitude_count = 121U;
    spec.longitude_count = 121U;

    std::vector<double> distances(
        spec.latitude_count * spec.longitude_count, 0.0);
    for (std::size_t row = 0U; row < spec.latitude_count; ++row) {
        const double latitude = spec.south_latitude_degrees +
            spec.latitude_step_degrees * static_cast<double>(row);
        for (std::size_t column = 0U; column < spec.longitude_count; ++column) {
            const double longitude = spec.west_longitude_degrees +
                spec.longitude_step_degrees * static_cast<double>(column);
            double distance =
                (std::abs(longitude - land_longitude) - half_width_degrees) * 60.0;
            if (latitude_half_extent.has_value()) {
                distance = std::max(
                    distance,
                    (std::abs(latitude - 1.0) - *latitude_half_extent) * 60.0);
            }
            distances[row * spec.longitude_count + column] = distance;
        }
    }
    LandmaskMetadata metadata;
    metadata.provider = make_metadata("barrier_mask");
    metadata.resolution_nautical_miles = 3.0;
    metadata.interpolation_error_nautical_miles = 0.0;
    auto mask =
        SignedDistanceLandmask::create(spec, std::move(distances), metadata);
    REQUIRE(mask.has_value());
    return std::move(mask.value());
}

ExclusionZoneSet barrier_zone(
    double south,
    double north,
    double west,
    double east,
    std::optional<sailroute::TimePoint> active_from = std::nullopt) {
    ExclusionZone zone;
    zone.identifier = "barrier";
    zone.source = "unit test";
    zone.revision = 4U;
    zone.active_from = active_from;
    zone.polygons.push_back(ExclusionPolygon{
        ExclusionRing{{
            Coordinate{south, west},
            Coordinate{south, east},
            Coordinate{north, east},
            Coordinate{north, west},
        }},
        {}});
    auto zones =
        ExclusionZoneSet::create({std::move(zone)}, make_metadata("zones"));
    REQUIRE(zones.has_value());
    return std::move(zones.value());
}

}  // namespace

TEST_CASE("an unconfigured environment reproduces the no-provider route exactly") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    for (const RoutingSolver solver :
         {RoutingSolver::isochrone_beam,
          RoutingSolver::time_dependent_lattice}) {
        const Router legacy{weather};
        const Router configured{weather, polar, RoutingEnvironment{}};
        const RouteRequest request = base_request(1U, solver);
        const RouteResult expected = route_or_throw(legacy, request);
        const RouteResult actual = route_or_throw(configured, request);
        require_identical_route(expected, actual);
        REQUIRE(!expected.environment.has_value());
        REQUIRE(!expected.environment_diagnostics.has_value());
        REQUIRE(!actual.environment.has_value());
        REQUIRE(!actual.environment_diagnostics.has_value());
        for (const RoutePoint& point : actual.points) {
            REQUIRE(!point.environment.has_value());
        }
    }
}

TEST_CASE("geometry-only environments do not invent point physics audit data") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    RoutingEnvironment environment;
    environment.exclusions.zones = barrier_zone(10.0, 11.0, 10.0, 11.0);
    const Router router{weather, polar, environment};

    for (const RoutingSolver solver :
         {RoutingSolver::isochrone_beam,
          RoutingSolver::time_dependent_lattice}) {
        const RouteResult result =
            route_or_throw(router, base_request(1U, solver));
        REQUIRE(result.environment.has_value());
        REQUIRE(result.environment->exclusions.has_value());
        for (const RoutePoint& point : result.points) {
            REQUIRE(!point.environment.has_value());
        }
    }
}

TEST_CASE("zero current is exactly equivalent to configuring no current") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    const Router legacy{weather};
    const Router still{weather, polar, current_environment(CurrentVector{0.0, 0.0})};

    for (const RoutingSolver solver :
         {RoutingSolver::isochrone_beam,
          RoutingSolver::time_dependent_lattice}) {
        const RouteRequest request = base_request(1U, solver);
        const RouteResult expected = route_or_throw(legacy, request);
        const RouteResult actual = route_or_throw(still, request);
        require_identical_route(expected, actual);
        REQUIRE(actual.environment.has_value());
        REQUIRE(actual.environment_diagnostics.has_value());
        REQUIRE(actual.environment_diagnostics->current_samples > 0U);
        REQUIRE(actual.environment_diagnostics->current_rejections == 0U);
    }
}

TEST_CASE("ground motion is the vector sum of water velocity and current") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const CurrentVector current{1.5, -0.75};
    const Router router{
        weather,
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        current_environment(current)};

    const RouteResult route = route_or_throw(router, base_request());
    REQUIRE(route.points.size() > 1U);
    bool crabbed = false;
    for (std::size_t index = 1U; index < route.points.size(); ++index) {
        const RoutePoint& point = route.points[index];
        REQUIRE(point.environment.has_value());
        REQUIRE(point.environment->current_east_knots == current.east_knots);
        REQUIRE(point.environment->current_north_knots == current.north_knots);

        const double heading = point.heading_degrees * std::numbers::pi / 180.0;
        const double east =
            point.boat_speed_knots * std::sin(heading) + current.east_knots;
        const double north =
            point.boat_speed_knots * std::cos(heading) + current.north_knots;
        REQUIRE_NEAR(
            point.environment->speed_over_ground_knots,
            std::hypot(east, north),
            1e-9);
        REQUIRE_NEAR(
            sailroute::detail::angular_difference_degrees(
                point.environment->course_over_ground_degrees,
                std::atan2(east, north) * 180.0 / std::numbers::pi),
            0.0,
            1e-9);
        if (sailroute::detail::angular_difference_degrees(
                point.environment->course_over_ground_degrees,
                point.heading_degrees) > 1.0) {
            crabbed = true;
        }
    }
    // The water frame is what the polar sees, so the ground track has to
    // genuinely differ from the steered heading somewhere on this route.
    REQUIRE(crabbed);
}

TEST_CASE("a following current arrives sooner and an opposing one later") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    // The route runs east, so an eastward current helps and a westward one
    // hinders.
    const Router still{weather};
    const Router helped{
        weather, polar, current_environment(CurrentVector{2.5, 0.0})};
    const Router hindered{
        weather, polar, current_environment(CurrentVector{-2.5, 0.0})};

    const RouteRequest request = base_request();
    const RouteResult neutral = route_or_throw(still, request);
    const RouteResult faster = route_or_throw(helped, request);
    const RouteResult slower = route_or_throw(hindered, request);

    REQUIRE(faster.arrival_time < neutral.arrival_time);
    REQUIRE(slower.arrival_time > neutral.arrival_time);
}

TEST_CASE("the lattice solver steers a heading that holds the ground track") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    const CurrentVector current{0.0, 2.0};
    const Router router{weather, polar, current_environment(current)};

    const RouteResult route = route_or_throw(
        router, base_request(1U, RoutingSolver::time_dependent_lattice));
    REQUIRE(route.points.size() > 1U);
    bool crabbed = false;
    for (std::size_t index = 1U; index < route.points.size(); ++index) {
        const RoutePoint& point = route.points[index];
        if (!point.environment.has_value()) {
            continue;  // A wait transition makes no progress to steer.
        }
        const double heading = point.heading_degrees * std::numbers::pi / 180.0;
        const double east =
            point.boat_speed_knots * std::sin(heading) + current.east_knots;
        const double north =
            point.boat_speed_knots * std::cos(heading) + current.north_knots;
        REQUIRE_NEAR(
            sailroute::detail::angular_difference_degrees(
                std::atan2(east, north) * 180.0 / std::numbers::pi,
                point.environment->course_over_ground_degrees),
            0.0,
            1e-6);
        REQUIRE_NEAR(
            point.environment->speed_over_ground_knots,
            std::hypot(east, north),
            1e-6);
        if (sailroute::detail::angular_difference_degrees(
                point.heading_degrees,
                point.environment->course_over_ground_degrees) > 1.0) {
            crabbed = true;
        }
    }
    REQUIRE(crabbed);
}

TEST_CASE("a calm sea reproduces the flat-water route and waves slow it down") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    const Router flat{weather};
    const Router calm{weather, polar, wave_environment(WaveState{0.0, 8.0, 0.0})};
    const Router seaway{
        weather, polar, wave_environment(WaveState{3.5, 7.0, 0.0})};

    const RouteRequest request = base_request();
    const RouteResult expected = route_or_throw(flat, request);
    const RouteResult unchanged = route_or_throw(calm, request);
    require_identical_route(expected, unchanged);

    const RouteResult derated = route_or_throw(seaway, request);
    REQUIRE(derated.arrival_time > expected.arrival_time);
    REQUIRE(derated.environment_diagnostics.has_value());
    REQUIRE(derated.environment_diagnostics->sea_state_evaluations > 0U);
    REQUIRE(derated.environment_diagnostics->wave_samples > 0U);
    for (std::size_t index = 1U; index < derated.points.size(); ++index) {
        const RoutePoint& point = derated.points[index];
        REQUIRE(point.environment.has_value());
        REQUIRE(point.environment->significant_wave_height_metres == 3.5);
        REQUIRE(
            point.boat_speed_knots <=
            point.environment->flat_water_speed_knots + 1e-12);
        REQUIRE(point.environment->relative_wave_angle_degrees >= 0.0);
        REQUIRE(point.environment->relative_wave_angle_degrees <= 180.0);
    }
}

TEST_CASE("a landmask barrier across the route removes every candidate") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);

    RoutingEnvironment environment;
    environment.land.landmask = barrier_landmask(0.75, 0.1);
    environment.land.missing_data_policy = MissingDataPolicy::reject_transition;
    const Router router{
        weather,
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        environment};

    const auto blocked = router.optimize(base_request());
    REQUIRE(!blocked.has_value());
    REQUIRE(blocked.error().code == ErrorCode::no_route);
}

TEST_CASE("a landmask island forces a longer route around it") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    RoutingEnvironment environment;
    environment.land.landmask = barrier_landmask(0.75, 0.05, 0.15);
    environment.land.missing_data_policy = MissingDataPolicy::reject_transition;
    const Router open{weather};
    const Router obstructed{weather, polar, environment};

    const RouteRequest request = base_request();
    const RouteResult direct = route_or_throw(open, request);
    const RouteResult around = route_or_throw(obstructed, request);

    REQUIRE(around.arrival_time >= direct.arrival_time);
    REQUIRE(around.environment_diagnostics.has_value());
    REQUIRE(around.environment_diagnostics->land_checks > 0U);
    REQUIRE(around.environment_diagnostics->land_rejections > 0U);
    REQUIRE(around.environment_diagnostics->land_distance_queries > 0U);

    // Every retained leg must itself be certified clear of the island.
    const SignedDistanceLandmask& mask = *environment.land.landmask;
    for (std::size_t index = 1U; index < around.points.size(); ++index) {
        REQUIRE(mask
                    .certify_segment(
                        around.points[index - 1U].position,
                        around.points[index].position,
                        0.0,
                        environment.land.maximum_subdivision_depth)
                    .clear);
    }
}

TEST_CASE("an exclusion zone across the route is respected and audited") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    RoutingEnvironment environment;
    environment.exclusions.zones = barrier_zone(0.9, 1.1, 0.65, 0.85);
    const Router open{weather};
    const Router constrained{weather, polar, environment};

    const RouteRequest request = base_request();
    const RouteResult direct = route_or_throw(open, request);
    const RouteResult avoiding = route_or_throw(constrained, request);

    REQUIRE(avoiding.arrival_time >= direct.arrival_time);
    REQUIRE(avoiding.environment.has_value());
    REQUIRE(avoiding.environment->exclusion_zone_count == 1U);
    REQUIRE(avoiding.environment->exclusion_revision == 4U);
    REQUIRE(avoiding.environment_diagnostics->exclusion_checks > 0U);
    REQUIRE(avoiding.environment_diagnostics->exclusion_rejections > 0U);

    const ExclusionZoneSet& zones = *environment.exclusions.zones;
    for (std::size_t index = 1U; index < avoiding.points.size(); ++index) {
        REQUIRE(!zones
                     .intersects_segment(
                         avoiding.points[index - 1U].position,
                         avoiding.points[index - 1U].time,
                         avoiding.points[index].position,
                         avoiding.points[index].time,
                         sailroute::ExclusionBoundaryPolicy::boundary_excluded)
                     .violated);
    }
}

TEST_CASE("an exclusion window that opens later leaves a short route untouched") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    const auto departure = sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    REQUIRE(departure.has_value());

    RoutingEnvironment environment;
    environment.exclusions.zones = barrier_zone(
        0.9, 1.1, 0.65, 0.85, departure.value() + std::chrono::hours{8});
    const Router open{weather};
    const Router constrained{weather, polar, environment};

    const RouteRequest request = base_request();
    const RouteResult direct = route_or_throw(open, request);
    const RouteResult later = route_or_throw(constrained, request);
    REQUIRE(later.arrival_time == direct.arrival_time);
    REQUIRE(later.environment_diagnostics->exclusion_rejections == 0U);
}

TEST_CASE("missing environment data follows the configured policy") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    // A grid current field that covers nothing the route can reach.
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = 40.0;
    spec.west_longitude_degrees = 40.0;
    spec.latitude_step_degrees = 1.0;
    spec.longitude_step_degrees = 1.0;
    spec.latitude_count = 2U;
    spec.longitude_count = 2U;
    auto elsewhere = sailroute::make_grid_current_provider(
        spec,
        {0.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0},
        make_metadata("elsewhere"));
    REQUIRE(elsewhere.has_value());

    for (const RoutingSolver solver :
         {RoutingSolver::isochrone_beam,
          RoutingSolver::time_dependent_lattice}) {
        RoutingEnvironment failing;
        failing.currents.provider = elsewhere.value();
        failing.currents.missing_data_policy = MissingDataPolicy::fail_route;
        const Router strict{weather, polar, failing};
        const auto failed = strict.optimize(base_request(1U, solver));
        REQUIRE(!failed.has_value());
        REQUIRE(failed.error().code == ErrorCode::environment_data_unavailable);

        RoutingEnvironment rejecting = failing;
        rejecting.currents.missing_data_policy =
            MissingDataPolicy::reject_transition;
        const Router lenient{weather, polar, rejecting};
        const auto rejected = lenient.optimize(base_request(1U, solver));
        REQUIRE(!rejected.has_value());
        REQUIRE(rejected.error().code == ErrorCode::no_route);
    }
}

TEST_CASE("a router rejects contradictory environment configuration") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    RoutingEnvironment unpaired;
    unpaired.waves.provider = sailroute::make_uniform_wave_provider(
                                  WaveState{2.0, 8.0, 0.0}, make_metadata("wave"))
                                  .value();
    const Router router{weather, polar, unpaired};
    const auto result = router.optimize(base_request());
    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == ErrorCode::invalid_environment);

    RoutingEnvironment stray_clearance;
    stray_clearance.currents.provider =
        sailroute::make_uniform_current_provider(
            CurrentVector{1.0, 0.0}, make_metadata("current"))
            .value();
    stray_clearance.land.clearance_nautical_miles = 5.0;
    const Router clearance_router{weather, polar, stray_clearance};
    const auto clearance_result = clearance_router.optimize(base_request());
    REQUIRE(!clearance_result.has_value());
    REQUIRE(clearance_result.error().code == ErrorCode::invalid_environment);
}

TEST_CASE("every provider combined stays deterministic across worker counts") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    RoutingEnvironment environment;
    environment.currents.provider =
        sailroute::make_uniform_current_provider(
            CurrentVector{1.0, 0.5}, make_metadata("current"))
            .value();
    environment.waves.provider = sailroute::make_uniform_wave_provider(
                                     WaveState{2.0, 9.0, 20.0}, make_metadata("wave"))
                                     .value();
    environment.waves.model =
        sailroute::make_wave_height_derating_model().value();
    environment.land.landmask = barrier_landmask(0.75, 0.05, 0.1);
    environment.land.clearance_nautical_miles = 0.5;
    environment.land.missing_data_policy = MissingDataPolicy::reject_transition;
    environment.exclusions.zones = barrier_zone(1.15, 1.25, 0.6, 0.9);

    const Router router{weather, polar, environment};
    const RouteResult single = route_or_throw(router, base_request(1U));
    const RouteResult many = route_or_throw(router, base_request(4U));
    require_identical_route(single, many);

    REQUIRE(single.environment_diagnostics.has_value());
    REQUIRE(many.environment_diagnostics.has_value());
    const auto& first = *single.environment_diagnostics;
    const auto& second = *many.environment_diagnostics;
    REQUIRE(first.current_samples == second.current_samples);
    REQUIRE(first.wave_samples == second.wave_samples);
    REQUIRE(first.sea_state_evaluations == second.sea_state_evaluations);
    REQUIRE(first.land_checks == second.land_checks);
    REQUIRE(first.land_rejections == second.land_rejections);
    REQUIRE(first.exclusion_checks == second.exclusion_checks);
    REQUIRE(first.exclusion_geometry_tests == second.exclusion_geometry_tests);

    REQUIRE(single.environment->current_provider->name == "current");
    REQUIRE(single.environment->wave_provider->name == "wave");
    REQUIRE(single.environment->sea_state_model->name == "wave_height_derating");
    REQUIRE(single.environment->landmask->name == "barrier_mask");
    REQUIRE(single.environment->exclusions->name == "zones");
    REQUIRE(single.environment->land_clearance_nautical_miles == 0.5);
    REQUIRE(single.environment->land_resolution_nautical_miles == 3.0);
}

TEST_CASE("land rejection happens before the caller eligibility callback") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);

    RoutingEnvironment environment;
    environment.land.landmask = barrier_landmask(0.75, 0.05, 0.15);
    environment.land.missing_data_policy = MissingDataPolicy::reject_transition;
    const Router router{
        weather,
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        environment};

    const SignedDistanceLandmask& mask = *environment.land.landmask;
    std::size_t observed = 0U;
    RouteRequest request = base_request();
    request.options.segment_eligibility =
        [&](const sailroute::RouteSegmentView& segment) {
            ++observed;
            // The callback must never be shown a leg the landmask rejected.
            REQUIRE(mask
                        .certify_segment(
                            segment.parent.position,
                            segment.candidate.position,
                            0.0,
                            environment.land.maximum_subdivision_depth)
                        .clear);
            return true;
        };
    const RouteResult route = route_or_throw(router, request);
    REQUIRE(observed > 0U);
    REQUIRE(route.points.size() > 1U);
}

TEST_CASE("eligibility callback exceptions still propagate with providers") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const Router router{
        weather,
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        current_environment(CurrentVector{1.0, 0.0})};

    RouteRequest request = base_request(4U);
    request.options.segment_eligibility =
        [](const sailroute::RouteSegmentView&) -> bool {
        throw std::runtime_error("eligibility exploded");
    };
    bool caught = false;
    try {
        const auto ignored = router.optimize(request);
        (void)ignored;
    } catch (const std::runtime_error& error) {
        caught = std::string{error.what()} == "eligibility exploded";
    }
    REQUIRE(caught);
}

TEST_CASE("midpoint environment sampling stays deterministic") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    RoutingEnvironment environment =
        current_environment(CurrentVector{1.0, 0.0});
    environment.sampling = sailroute::EnvironmentSampling::midpoint;
    const Router router{weather, polar, environment};

    RouteRequest request = base_request();
    request.options.wind_sampling = sailroute::WindSampling::midpoint;
    const RouteResult first = route_or_throw(router, request);
    const RouteResult second = route_or_throw(router, request);
    require_identical_route(first, second);
    REQUIRE(
        first.environment->sampling == sailroute::EnvironmentSampling::midpoint);
    // Midpoint sampling asks the provider a second time per candidate, so the
    // sample count outgrows the one-per-parent segment-start count.
    REQUIRE(
        first.environment_diagnostics->current_samples >
        first.diagnostics.expanded_nodes);
}

TEST_CASE("midpoint sea-state evaluation uses midpoint wind inputs") {
    sailroute::test::ConstantWindGribFixture::Options forecast;
    forecast.north_metres_per_second = -5.0;
    forecast.final_east_metres_per_second = 5.0;
    forecast.final_north_metres_per_second = -15.0;
    const sailroute::test::ConstantWindGribFixture fixture{forecast};
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    RoutingEnvironment environment;
    auto provider = sailroute::make_uniform_wave_provider(
        WaveState{1.0, 8.0, 0.0}, make_metadata("wave"));
    REQUIRE(provider.has_value());
    const auto model = std::make_shared<RecordingSeaStateModel>();
    environment.waves.provider = provider.value();
    environment.waves.model = model;
    environment.sampling = sailroute::EnvironmentSampling::midpoint;
    const Router router{weather, polar, environment};

    RouteRequest request = base_request();
    request.options.wind_sampling = sailroute::WindSampling::midpoint;
    (void)route_or_throw(router, request);

    const std::vector<SeaStateInput> inputs = model->inputs();
    REQUIRE(inputs.size() >= 2U);
    bool found_refined_pair = false;
    for (std::size_t index = 1U; index < inputs.size(); ++index) {
        const SeaStateInput& start = inputs[index - 1U];
        const SeaStateInput& midpoint = inputs[index];
        if (start.heading_degrees == midpoint.heading_degrees &&
            (start.true_wind_speed_knots != midpoint.true_wind_speed_knots ||
             start.true_wind_angle_degrees !=
                 midpoint.true_wind_angle_degrees)) {
            found_refined_pair = true;
            break;
        }
    }
    REQUIRE(found_refined_pair);
}

TEST_CASE("serialization adds environment data only when it exists") {
    const sailroute::test::ConstantWindGribFixture fixture;
    const WeatherDataset weather = load_weather(fixture);
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();

    const Router plain{weather};
    const RouteResult without = route_or_throw(plain, base_request());
    const auto plain_json = sailroute::route_to_json(without);
    REQUIRE(plain_json.has_value());
    REQUIRE(plain_json.value().find("\"environment\"") == std::string::npos);
    REQUIRE(
        plain_json.value().find("\"environmentDiagnostics\"") ==
        std::string::npos);
    const auto plain_gpx = sailroute::route_to_gpx(without);
    REQUIRE(plain_gpx.has_value());
    REQUIRE(
        plain_gpx.value().find("sailroute:speedOverGroundKnots") ==
        std::string::npos);

    RoutingEnvironment environment =
        current_environment(CurrentVector{1.0, 0.5});
    environment.exclusions.zones = barrier_zone(1.5, 1.6, 1.5, 1.6);
    const Router configured{weather, polar, environment};
    const RouteResult with = route_or_throw(configured, base_request());

    const auto json = sailroute::route_to_json(with);
    REQUIRE(json.has_value());
    REQUIRE(json.value().find("\"environmentDiagnostics\"") != std::string::npos);
    REQUIRE(json.value().find("\"currentSamples\"") != std::string::npos);
    REQUIRE(json.value().find("\"currentProvider\"") != std::string::npos);
    REQUIRE(json.value().find("\"exclusionBoundaryPolicy\"") != std::string::npos);
    REQUIRE(json.value().find("\"boundary_excluded\"") != std::string::npos);
    REQUIRE(json.value().find("\"speedOverGroundKnots\"") != std::string::npos);
    REQUIRE(json.value().find("\"waveProvider\":null") != std::string::npos);

    const auto gpx = sailroute::route_to_gpx(with);
    REQUIRE(gpx.has_value());
    REQUIRE(
        gpx.value().find(
            "<sailroute:currentProviderName>current</sailroute:currentProviderName>") !=
        std::string::npos);
    REQUIRE(gpx.value().find("sailroute:speedOverGroundKnots") != std::string::npos);
    REQUIRE(gpx.value().find("sailroute:exclusionZoneCount") != std::string::npos);
    REQUIRE(
        gpx.value().find("sailroute:landClearanceNauticalMiles") ==
        std::string::npos);
}
