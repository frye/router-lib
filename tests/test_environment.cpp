#include "sailroute/environment.hpp"

#include "../src/routing/environment_context.hpp"
#include "test_support.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

using sailroute::Coordinate;
using sailroute::CurrentVector;
using sailroute::EnvironmentDiagnostics;
using sailroute::EnvironmentGridSpec;
using sailroute::EnvironmentSample;
using sailroute::EnvironmentSampleStatus;
using sailroute::ErrorCode;
using sailroute::MissingDataPolicy;
using sailroute::ProviderMetadata;
using sailroute::RoutingEnvironment;
using sailroute::SeaStateInput;
using sailroute::SeaStatePerformanceModel;
using sailroute::TimePoint;
using sailroute::WaveState;

ProviderMetadata make_metadata(std::string name) {
    return ProviderMetadata{std::move(name), "unit test", "1"};
}

/// Reports a fixed non-value so policy handling can be exercised directly.
class FailingCurrentProvider final : public sailroute::CurrentProvider {
public:
    explicit FailingCurrentProvider(EnvironmentSampleStatus status)
        : status_(status), metadata_(make_metadata("failing_current")) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }
    [[nodiscard]] sailroute::EnvironmentCoverage coverage() const override {
        return {};
    }
    [[nodiscard]] EnvironmentSample<CurrentVector> sample(
        Coordinate,
        TimePoint) const override {
        return EnvironmentSample<CurrentVector>::without_value(status_);
    }

private:
    EnvironmentSampleStatus status_;
    ProviderMetadata metadata_;
};

/// Claims `available` while handing back a value no consumer may trust.
class NonFiniteCurrentProvider final : public sailroute::CurrentProvider {
public:
    NonFiniteCurrentProvider() : metadata_(make_metadata("non_finite_current")) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }
    [[nodiscard]] sailroute::EnvironmentCoverage coverage() const override {
        return {};
    }
    [[nodiscard]] EnvironmentSample<CurrentVector> sample(
        Coordinate,
        TimePoint) const override {
        return EnvironmentSample<CurrentVector>::available(
            CurrentVector{std::numeric_limits<double>::quiet_NaN(), 0.0});
    }

private:
    ProviderMetadata metadata_;
};

/// A model that breaks the contract by making the vessel faster in waves.
class AcceleratingModel final : public SeaStatePerformanceModel {
public:
    AcceleratingModel() : metadata_(make_metadata("accelerating")) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }
    [[nodiscard]] double derated_speed_knots(
        const SeaStateInput& input) const override {
        return input.flat_water_speed_knots + 1.0;
    }

private:
    ProviderMetadata metadata_;
};

TimePoint epoch() {
    return TimePoint{std::chrono::seconds{1'800'000'000}};
}

}  // namespace

TEST_CASE("current vectors report speed and set in the oceanographic frame") {
    const CurrentVector east{2.0, 0.0};
    REQUIRE_NEAR(east.speed_knots(), 2.0, 1e-12);
    REQUIRE_NEAR(east.set_toward_degrees(), 90.0, 1e-9);

    const CurrentVector north{0.0, 3.0};
    REQUIRE_NEAR(north.set_toward_degrees(), 0.0, 1e-9);

    const CurrentVector diagonal{1.0, -1.0};
    REQUIRE_NEAR(diagonal.speed_knots(), std::sqrt(2.0), 1e-12);
    REQUIRE_NEAR(diagonal.set_toward_degrees(), 135.0, 1e-9);

    const CurrentVector still{0.0, 0.0};
    REQUIRE(still.set_toward_degrees() == 0.0);
}

TEST_CASE("uniform providers require metadata and finite values") {
    REQUIRE(!sailroute::make_uniform_current_provider(
                 CurrentVector{1.0, 0.0}, ProviderMetadata{"", "source", "1"})
                 .has_value());
    REQUIRE(!sailroute::make_uniform_current_provider(
                 CurrentVector{1.0, 0.0}, ProviderMetadata{"name", "", "1"})
                 .has_value());
    const auto non_finite = sailroute::make_uniform_current_provider(
        CurrentVector{std::numeric_limits<double>::infinity(), 0.0},
        make_metadata("bad"));
    REQUIRE(!non_finite.has_value());
    REQUIRE(non_finite.error().code == ErrorCode::invalid_environment);

    const auto negative_height = sailroute::make_uniform_wave_provider(
        WaveState{-1.0, 8.0, 0.0}, make_metadata("bad"));
    REQUIRE(!negative_height.has_value());

    const auto good = sailroute::make_uniform_current_provider(
        CurrentVector{1.5, -0.5}, make_metadata("uniform"));
    REQUIRE(good.has_value());
    const auto sampled = good.value()->sample(Coordinate{10.0, 20.0}, epoch());
    REQUIRE(sampled.has_value());
    REQUIRE(sampled.value.east_knots == 1.5);
    REQUIRE(sampled.value.north_knots == -0.5);
    REQUIRE(good.value()->coverage().global_longitude_coverage);

    const auto invalid_position = good.value()->sample(
        Coordinate{std::numeric_limits<double>::quiet_NaN(), 0.0}, epoch());
    REQUIRE(!invalid_position.has_value());
    REQUIRE(invalid_position.status == EnvironmentSampleStatus::outside_coverage);
}

TEST_CASE("grid current fields interpolate bilinearly and bound coverage") {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = 0.0;
    spec.west_longitude_degrees = 0.0;
    spec.latitude_step_degrees = 1.0;
    spec.longitude_step_degrees = 1.0;
    spec.latitude_count = 2U;
    spec.longitude_count = 2U;

    const auto provider = sailroute::make_grid_current_provider(
        spec,
        {0.0, 2.0, 0.0, 2.0},
        {0.0, 0.0, 4.0, 4.0},
        make_metadata("grid_current"));
    REQUIRE(provider.has_value());

    const auto corner = provider.value()->sample(Coordinate{0.0, 0.0}, epoch());
    REQUIRE(corner.has_value());
    REQUIRE_NEAR(corner.value.east_knots, 0.0, 1e-12);
    REQUIRE_NEAR(corner.value.north_knots, 0.0, 1e-12);

    const auto centre = provider.value()->sample(Coordinate{0.5, 0.5}, epoch());
    REQUIRE(centre.has_value());
    REQUIRE_NEAR(centre.value.east_knots, 1.0, 1e-12);
    REQUIRE_NEAR(centre.value.north_knots, 2.0, 1e-12);

    const auto outside = provider.value()->sample(Coordinate{5.0, 0.5}, epoch());
    REQUIRE(!outside.has_value());
    REQUIRE(outside.status == EnvironmentSampleStatus::outside_coverage);
    REQUIRE(
        !provider.value()->sample(Coordinate{0.5, 40.0}, epoch()).has_value());
}

TEST_CASE("grid fields reject malformed specifications and sample counts") {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = 0.0;
    spec.west_longitude_degrees = 0.0;
    spec.latitude_step_degrees = 1.0;
    spec.longitude_step_degrees = 1.0;
    spec.latitude_count = 2U;
    spec.longitude_count = 2U;

    REQUIRE(!sailroute::make_grid_current_provider(
                 spec, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, make_metadata("grid"))
                 .has_value());
    REQUIRE(!sailroute::make_grid_current_provider(
                 spec,
                 {0.0, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN()},
                 {0.0, 0.0, 0.0, 0.0},
                 make_metadata("grid"))
                 .has_value());

    EnvironmentGridSpec degenerate = spec;
    degenerate.latitude_count = 1U;
    REQUIRE(!sailroute::make_grid_current_provider(
                 degenerate, {0.0, 0.0}, {0.0, 0.0}, make_metadata("grid"))
                 .has_value());

    EnvironmentGridSpec negative_step = spec;
    negative_step.longitude_step_degrees = -1.0;
    REQUIRE(!sailroute::make_grid_current_provider(
                 negative_step,
                 {0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0},
                 make_metadata("grid"))
                 .has_value());

    EnvironmentGridSpec beyond_pole = spec;
    beyond_pole.south_latitude_degrees = 89.5;
    beyond_pole.latitude_count = 4U;
    REQUIRE(!sailroute::make_grid_current_provider(
                 beyond_pole,
                 std::vector<double>(8U, 0.0),
                 std::vector<double>(8U, 0.0),
                 make_metadata("grid"))
                 .has_value());

    EnvironmentGridSpec bad_global = spec;
    bad_global.global_longitude_coverage = true;
    REQUIRE(!sailroute::make_grid_current_provider(
                 bad_global,
                 {0.0, 0.0, 0.0, 0.0},
                 {0.0, 0.0, 0.0, 0.0},
                 make_metadata("grid"))
                 .has_value());
}

TEST_CASE("a globally covering grid wraps across the antimeridian") {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = -1.0;
    spec.west_longitude_degrees = -180.0;
    spec.latitude_step_degrees = 1.0;
    spec.longitude_step_degrees = 90.0;
    spec.latitude_count = 3U;
    spec.longitude_count = 4U;
    spec.global_longitude_coverage = true;

    // East components are 1 at -180/0 and 3 at -90/+90, so the wrap-around
    // interval between +90 and -180 must interpolate rather than clamp.
    const std::vector<double> east{
        1.0, 3.0, 1.0, 3.0,
        1.0, 3.0, 1.0, 3.0,
        1.0, 3.0, 1.0, 3.0};
    const auto provider = sailroute::make_grid_current_provider(
        spec, east, std::vector<double>(12U, 0.0), make_metadata("global"));
    REQUIRE(provider.has_value());

    const auto west_edge =
        provider.value()->sample(Coordinate{0.0, -180.0}, epoch());
    REQUIRE(west_edge.has_value());
    REQUIRE_NEAR(west_edge.value.east_knots, 1.0, 1e-12);

    const auto wrapped =
        provider.value()->sample(Coordinate{0.0, 135.0}, epoch());
    REQUIRE(wrapped.has_value());
    REQUIRE_NEAR(wrapped.value.east_knots, 2.0, 1e-12);

    const auto far_east =
        provider.value()->sample(Coordinate{0.0, 179.999}, epoch());
    REQUIRE(far_east.has_value());
    REQUIRE_NEAR(far_east.value.east_knots, 1.0, 1e-3);
}

TEST_CASE("grid wave directions blend as vectors across the compass wrap") {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = 0.0;
    spec.west_longitude_degrees = 0.0;
    spec.latitude_step_degrees = 1.0;
    spec.longitude_step_degrees = 1.0;
    spec.latitude_count = 2U;
    spec.longitude_count = 2U;

    const auto provider = sailroute::make_grid_wave_provider(
        spec,
        {2.0, 2.0, 2.0, 2.0},
        {8.0, 8.0, 8.0, 8.0},
        {350.0, 10.0, 350.0, 10.0},
        make_metadata("grid_wave"));
    REQUIRE(provider.has_value());

    const auto middle = provider.value()->sample(Coordinate{0.5, 0.5}, epoch());
    REQUIRE(middle.has_value());
    REQUIRE_NEAR(middle.value.direction_from_degrees, 0.0, 1e-9);
    REQUIRE_NEAR(middle.value.significant_height_metres, 2.0, 1e-12);
    REQUIRE_NEAR(middle.value.peak_period_seconds, 8.0, 1e-12);
}

TEST_CASE("the built-in derating model is calm-exact, bounded, and directional") {
    const auto model = sailroute::make_wave_height_derating_model();
    REQUIRE(model.has_value());
    REQUIRE(model.value()->metadata().name == "wave_height_derating");

    SeaStateInput input;
    input.flat_water_speed_knots = 8.25;
    input.true_wind_speed_knots = 18.0;
    input.true_wind_angle_degrees = 60.0;
    input.wave = WaveState{0.0, 8.0, 0.0};
    input.relative_wave_angle_degrees = 180.0;
    REQUIRE(model.value()->derated_speed_knots(input) == 8.25);

    input.wave.significant_height_metres = 2.5;
    input.relative_wave_angle_degrees = 180.0;
    const double head = model.value()->derated_speed_knots(input);
    input.relative_wave_angle_degrees = 90.0;
    const double beam = model.value()->derated_speed_knots(input);
    input.relative_wave_angle_degrees = 0.0;
    const double following = model.value()->derated_speed_knots(input);
    REQUIRE(head < beam);
    REQUIRE(beam < following);
    REQUIRE(following < input.flat_water_speed_knots);
    REQUIRE(head > 0.0);

    // The loss saturates, so even an extreme sea cannot stop the vessel.
    input.wave.significant_height_metres = 40.0;
    input.relative_wave_angle_degrees = 180.0;
    const double extreme = model.value()->derated_speed_knots(input);
    REQUIRE_NEAR(extreme, input.flat_water_speed_knots * 0.4, 1e-9);
}

TEST_CASE("derating coefficients are validated and period sensitivity applies") {
    sailroute::WaveHeightDeratingCoefficients coefficients;
    coefficients.maximum_loss_fraction = 1.5;
    REQUIRE(!sailroute::make_wave_height_derating_model(coefficients).has_value());

    coefficients = {};
    coefficients.height_exponent = 0.0;
    REQUIRE(!sailroute::make_wave_height_derating_model(coefficients).has_value());

    coefficients = {};
    coefficients.period_sensitivity = 1.0;
    coefficients.reference_period_seconds = 8.0;
    const auto model = sailroute::make_wave_height_derating_model(coefficients);
    REQUIRE(model.has_value());

    SeaStateInput input;
    input.flat_water_speed_knots = 8.0;
    input.wave = WaveState{3.0, 4.0, 0.0};
    input.relative_wave_angle_degrees = 180.0;
    const double steep = model.value()->derated_speed_knots(input);
    input.wave.peak_period_seconds = 14.0;
    const double swell = model.value()->derated_speed_knots(input);
    REQUIRE(steep < swell);
}

TEST_CASE("environment validation rejects contradictory configurations") {
    RoutingEnvironment environment;
    REQUIRE(!sailroute::validate_environment(environment).has_value());
    REQUIRE(!environment.active());
    REQUIRE(!sailroute::describe_environment(environment).has_value());

    const auto waves =
        sailroute::make_uniform_wave_provider(WaveState{2.0, 8.0, 0.0}, make_metadata("waves"));
    REQUIRE(waves.has_value());
    environment.waves.provider = waves.value();
    const auto unpaired = sailroute::validate_environment(environment);
    REQUIRE(unpaired.has_value());
    REQUIRE(unpaired->code == ErrorCode::invalid_environment);

    environment.waves.provider.reset();
    environment.waves.model =
        sailroute::make_wave_height_derating_model().value();
    REQUIRE(sailroute::validate_environment(environment).has_value());

    environment.waves.provider = waves.value();
    REQUIRE(!sailroute::validate_environment(environment).has_value());
    REQUIRE(environment.active());

    environment.land.clearance_nautical_miles = 1.0;
    REQUIRE(sailroute::validate_environment(environment).has_value());
    environment.land.clearance_nautical_miles = 0.0;

    RoutingEnvironment sampling_only;
    sampling_only.sampling = sailroute::EnvironmentSampling::midpoint;
    REQUIRE(sailroute::validate_environment(sampling_only).has_value());
}

TEST_CASE("environment metadata records every configured source and policy") {
    RoutingEnvironment environment;
    environment.currents.provider =
        sailroute::make_uniform_current_provider(
            CurrentVector{1.0, 0.0}, make_metadata("current"))
            .value();
    environment.currents.missing_data_policy =
        MissingDataPolicy::reject_transition;
    environment.waves.provider =
        sailroute::make_uniform_wave_provider(
            WaveState{1.0, 7.0, 180.0}, make_metadata("wave"))
            .value();
    environment.waves.model =
        sailroute::make_wave_height_derating_model().value();

    const auto described = sailroute::describe_environment(environment);
    REQUIRE(described.has_value());
    REQUIRE(described->current_provider.has_value());
    REQUIRE(described->current_provider->name == "current");
    REQUIRE(described->wave_provider->name == "wave");
    REQUIRE(described->sea_state_model->name == "wave_height_derating");
    REQUIRE(!described->landmask.has_value());
    REQUIRE(!described->exclusions.has_value());
    REQUIRE(described->current_policy == MissingDataPolicy::reject_transition);
    REQUIRE(described->wave_policy == MissingDataPolicy::fail_route);
    REQUIRE(described->exclusion_zone_count == 0U);
}

TEST_CASE("water velocity translates into the ground frame by vector sum") {
    using sailroute::detail::ground_velocity;

    const auto still = ground_velocity(45.0, 6.0, CurrentVector{0.0, 0.0});
    REQUIRE_NEAR(still.course_degrees, 45.0, 1e-12);
    REQUIRE_NEAR(still.speed_knots, 6.0, 1e-12);

    const auto following = ground_velocity(0.0, 10.0, CurrentVector{0.0, 5.0});
    REQUIRE_NEAR(following.course_degrees, 0.0, 1e-9);
    REQUIRE_NEAR(following.speed_knots, 15.0, 1e-12);

    const auto opposing = ground_velocity(0.0, 10.0, CurrentVector{0.0, -4.0});
    REQUIRE_NEAR(opposing.course_degrees, 0.0, 1e-9);
    REQUIRE_NEAR(opposing.speed_knots, 6.0, 1e-12);

    const auto crossing = ground_velocity(0.0, 10.0, CurrentVector{5.0, 0.0});
    REQUIRE_NEAR(crossing.course_degrees, 26.565051177078, 1e-9);
    REQUIRE_NEAR(crossing.speed_knots, std::sqrt(125.0), 1e-12);

    const auto stalled = ground_velocity(180.0, 4.0, CurrentVector{0.0, 4.0});
    REQUIRE_NEAR(stalled.speed_knots, 0.0, 1e-12);
}

TEST_CASE("the course to steer cancels the cross-track current exactly") {
    using sailroute::detail::ground_velocity;
    using sailroute::detail::water_heading_offset_degrees;

    const CurrentVector setting_east{5.0, 0.0};
    const auto offset = water_heading_offset_degrees(0.0, 10.0, setting_east);
    REQUIRE(offset.has_value());
    REQUIRE_NEAR(*offset, -30.0, 1e-9);

    const auto resulting = ground_velocity(*offset, 10.0, setting_east);
    REQUIRE_NEAR(resulting.course_degrees, 0.0, 1e-9);
    REQUIRE_NEAR(resulting.speed_knots, std::sqrt(75.0), 1e-9);

    // A pure along-track current needs no correction at all.
    const auto no_offset =
        water_heading_offset_degrees(90.0, 8.0, CurrentVector{3.0, 0.0});
    REQUIRE(no_offset.has_value());
    REQUIRE_NEAR(*no_offset, 0.0, 1e-12);

    // A cross-track current stronger than the vessel cannot be held.
    REQUIRE(!water_heading_offset_degrees(0.0, 2.0, CurrentVector{5.0, 0.0})
                 .has_value());
    REQUIRE(!water_heading_offset_degrees(0.0, 0.0, CurrentVector{1.0, 0.0})
                 .has_value());
}

TEST_CASE("relative wave angle measures from a following sea") {
    using sailroute::detail::relative_wave_angle_degrees;
    // Waves from the north travel south, so sailing south is a following sea.
    REQUIRE_NEAR(relative_wave_angle_degrees(180.0, 0.0), 0.0, 1e-9);
    REQUIRE_NEAR(relative_wave_angle_degrees(0.0, 0.0), 180.0, 1e-9);
    REQUIRE_NEAR(relative_wave_angle_degrees(90.0, 0.0), 90.0, 1e-9);
    REQUIRE_NEAR(relative_wave_angle_degrees(270.0, 0.0), 90.0, 1e-9);
    // High longitude and the compass wrap are ordinary angles here.
    REQUIRE_NEAR(relative_wave_angle_degrees(10.0, 190.0), 0.0, 1e-9);
    REQUIRE_NEAR(relative_wave_angle_degrees(350.0, 170.0), 0.0, 1e-9);
}

TEST_CASE("missing current data follows the configured policy") {
    RoutingEnvironment environment;
    environment.currents.provider = std::make_shared<const FailingCurrentProvider>(
        EnvironmentSampleStatus::outside_coverage);
    environment.currents.missing_data_policy = MissingDataPolicy::fail_route;

    EnvironmentDiagnostics diagnostics;
    auto failing = sailroute::detail::sample_environment(
        environment, Coordinate{0.0, 0.0}, epoch(), diagnostics);
    REQUIRE(failing.outcome == sailroute::detail::EnvironmentOutcome::failed);
    REQUIRE(failing.error.has_value());
    REQUIRE(failing.error->code == ErrorCode::environment_data_unavailable);
    REQUIRE(diagnostics.current_samples == 1U);
    REQUIRE(diagnostics.current_rejections == 1U);

    environment.currents.missing_data_policy = MissingDataPolicy::reject_transition;
    auto rejecting = sailroute::detail::sample_environment(
        environment, Coordinate{0.0, 0.0}, epoch(), diagnostics);
    REQUIRE(rejecting.outcome == sailroute::detail::EnvironmentOutcome::rejected);
    REQUIRE(!rejecting.error.has_value());
    REQUIRE(diagnostics.current_rejections == 2U);

    // A provider that claims success while returning a non-finite component is
    // treated as invalid data rather than as zero current.
    environment.currents.provider =
        std::make_shared<const NonFiniteCurrentProvider>();
    environment.currents.missing_data_policy = MissingDataPolicy::fail_route;
    auto non_finite = sailroute::detail::sample_environment(
        environment, Coordinate{0.0, 0.0}, epoch(), diagnostics);
    REQUIRE(non_finite.outcome == sailroute::detail::EnvironmentOutcome::failed);
}

TEST_CASE("a sea-state model that accelerates the vessel is a hard error") {
    RoutingEnvironment environment;
    environment.waves.provider =
        sailroute::make_uniform_wave_provider(
            WaveState{2.0, 8.0, 0.0}, make_metadata("wave"))
            .value();
    environment.waves.model = std::make_shared<const AcceleratingModel>();

    EnvironmentDiagnostics diagnostics;
    const auto applied = sailroute::detail::apply_sea_state(
        environment,
        7.0,
        18.0,
        90.0,
        45.0,
        WaveState{2.0, 8.0, 0.0},
        diagnostics);
    REQUIRE(!applied.has_value());
    REQUIRE(applied.error().code == ErrorCode::invalid_environment);
    REQUIRE(diagnostics.sea_state_evaluations == 1U);
}

TEST_CASE("environment diagnostics merge as order-independent sums") {
    EnvironmentDiagnostics left;
    left.current_samples = 3U;
    left.land_rejections = 1U;
    EnvironmentDiagnostics right;
    right.current_samples = 4U;
    right.exclusion_checks = 2U;

    EnvironmentDiagnostics forward = left;
    sailroute::detail::merge(forward, right);
    EnvironmentDiagnostics backward = right;
    sailroute::detail::merge(backward, left);

    REQUIRE(forward.current_samples == backward.current_samples);
    REQUIRE(forward.current_samples == 7U);
    REQUIRE(forward.land_rejections == backward.land_rejections);
    REQUIRE(forward.exclusion_checks == backward.exclusion_checks);
}
