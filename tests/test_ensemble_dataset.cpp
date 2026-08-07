#include "sailroute/ensemble.hpp"
#include "sailroute/sailroute.hpp"

#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <chrono>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using sailroute::EnsembleAlignmentMode;
using sailroute::EnsembleDataset;
using sailroute::EnsembleLoadOptions;
using sailroute::EnsembleMemberInput;
using sailroute::EnsembleRunMetadata;
using sailroute::ErrorCode;
using sailroute::ProviderMetadata;
using sailroute::RoutingEnvironment;
using sailroute::test::ConstantWindGribFixture;

EnsembleRunMetadata run_metadata() {
    return EnsembleRunMetadata{
        "cycle-20260714T1200Z",
        "test-model",
        sailroute::TimePoint{std::chrono::seconds{1'784'035'200}},
        "generated ecCodes fixture",
        1U};
}

EnsembleMemberInput member_input(
    std::string identifier,
    double weight,
    const ConstantWindGribFixture& fixture,
    RoutingEnvironment environment = {}) {
    EnsembleMemberInput input;
    input.identifier = std::move(identifier);
    input.weight = weight;
    input.grib_path = fixture.path();
    input.environment = std::move(environment);
    return input;
}

ProviderMetadata provider_metadata(std::string name) {
    return ProviderMetadata{std::move(name), "ensemble unit test", "1"};
}

}  // namespace

TEST_CASE("ensemble load canonicalizes members and preserves both weight forms") {
    const ConstantWindGribFixture alpha_fixture(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 2.0,
            .north_metres_per_second = -1.0});
    const ConstantWindGribFixture beta_fixture(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 4.0,
            .north_metres_per_second = -2.0});

    const auto ensemble = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("beta", 3.0, beta_fixture),
            member_input("alpha", 1.0, alpha_fixture),
        });
    REQUIRE(ensemble.has_value());
    REQUIRE(ensemble.value().member_count() == 2U);
    REQUIRE(ensemble.value().members()[0].identifier == "alpha");
    REQUIRE(ensemble.value().members()[1].identifier == "beta");
    REQUIRE(ensemble.value().members()[0].original_weight == 1.0);
    REQUIRE(ensemble.value().members()[1].original_weight == 3.0);
    REQUIRE_NEAR(ensemble.value().members()[0].normalized_weight, 0.25, 1e-15);
    REQUIRE_NEAR(ensemble.value().members()[1].normalized_weight, 0.75, 1e-15);
    REQUIRE(ensemble.value().alignment().shared_search_compatible());

    const auto reversed = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("alpha", 1.0, alpha_fixture),
            member_input("beta", 3.0, beta_fixture),
        });
    REQUIRE(reversed.has_value());
    REQUIRE(
        reversed.value().members()[0].identifier ==
        ensemble.value().members()[0].identifier);
    REQUIRE(
        reversed.value().members()[1].normalized_weight ==
        ensemble.value().members()[1].normalized_weight);
}

TEST_CASE("weather metadata preserves initialization before first lead time") {
    const ConstantWindGribFixture fixture(
        ConstantWindGribFixture::Options{
            .initial_forecast_hour = 6,
            .final_forecast_hour = 24});
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto initialization =
        sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    const auto first_valid =
        sailroute::parse_utc_time("2026-07-14T18:00:00Z");
    REQUIRE(initialization.has_value());
    REQUIRE(first_valid.has_value());
    REQUIRE(
        weather.value().metadata().initialization_time ==
        initialization.value());
    REQUIRE(
        weather.value().metadata().first_valid_time ==
        first_valid.value());
}

TEST_CASE("ensemble weight and identifier validation rejects ambiguous inputs") {
    const ConstantWindGribFixture fixture;

    const auto empty = EnsembleDataset::load(run_metadata(), {});
    REQUIRE(!empty.has_value());
    REQUIRE(empty.error().code == ErrorCode::invalid_argument);

    const auto empty_identifier = EnsembleDataset::load(
        run_metadata(),
        {member_input("", 1.0, fixture)});
    REQUIRE(!empty_identifier.has_value());

    const auto duplicate = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("same", 1.0, fixture),
            member_input("same", 2.0, fixture),
        });
    REQUIRE(!duplicate.has_value());

    const auto negative = EnsembleDataset::load(
        run_metadata(),
        {member_input("negative", -1.0, fixture)});
    REQUIRE(!negative.has_value());

    const auto non_finite = EnsembleDataset::load(
        run_metadata(),
        {member_input(
            "infinite",
            std::numeric_limits<double>::infinity(),
            fixture)});
    REQUIRE(!non_finite.has_value());

    const auto zero_total = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("a", 0.0, fixture),
            member_input("b", 0.0, fixture),
        });
    REQUIRE(!zero_total.has_value());

    const auto zero_member = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("a", 0.0, fixture),
            member_input("b", 2.0, fixture),
        });
    REQUIRE(zero_member.has_value());
    REQUIRE(zero_member.value().members()[0].normalized_weight == 0.0);
    REQUIRE(zero_member.value().members()[1].normalized_weight == 1.0);

    const auto erased = EnsembleDataset::load(
        run_metadata(),
        {
            member_input(
                "large",
                std::numeric_limits<double>::max(),
                fixture),
            member_input(
                "tiny",
                std::numeric_limits<double>::denorm_min(),
                fixture),
        });
    REQUIRE(!erased.has_value());
    REQUIRE(erased.error().message.find("erased") != std::string::npos);

    const auto large = EnsembleDataset::load(
        run_metadata(),
        {
            member_input(
                "a",
                std::numeric_limits<double>::max(),
                fixture),
            member_input(
                "b",
                std::numeric_limits<double>::max(),
                fixture),
        });
    REQUIRE(large.has_value());
    REQUIRE(large.value().members()[0].normalized_weight == 0.5);
    REQUIRE(large.value().members()[1].normalized_weight == 0.5);
}

TEST_CASE("weather exposes exact valid-time and canonical grid coverage identity") {
    const ConstantWindGribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    REQUIRE(weather.value().valid_times().size() == 2U);
    REQUIRE(
        weather.value().valid_times().front() ==
        weather.value().metadata().first_valid_time);
    REQUIRE(
        weather.value().valid_times().back() ==
        weather.value().metadata().last_valid_time);
    REQUIRE(weather.value().grid_identity().latitude_count == 3U);
    REQUIRE(weather.value().grid_identity().longitude_count == 3U);
    REQUIRE(weather.value().grid_identity().south_latitude_degrees == 0.0);
    REQUIRE(weather.value().grid_identity().west_longitude_degrees == 0.0);
    REQUIRE(!weather.value().grid_identity().interpolation_bounds.has_value());
}

TEST_CASE("strict alignment rejects missing slices and mismatched axes or grids") {
    const ConstantWindGribFixture reference;
    const ConstantWindGribFixture short_horizon(
        ConstantWindGribFixture::Options{.final_forecast_hour = 12});
    const ConstantWindGribFixture shifted_grid(
        ConstantWindGribFixture::Options{.west_longitude_degrees = 1.0});
    const ConstantWindGribFixture missing_slice(
        ConstantWindGribFixture::Options{.write_final_north = false});

    const auto unequal_time = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("reference", 1.0, reference),
            member_input("short", 1.0, short_horizon),
        });
    REQUIRE(!unequal_time.has_value());
    REQUIRE(unequal_time.error().code == ErrorCode::incomplete_forecast);

    const auto unequal_grid = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("reference", 1.0, reference),
            member_input("shifted", 1.0, shifted_grid),
        });
    REQUIRE(!unequal_grid.has_value());
    REQUIRE(unequal_grid.error().code == ErrorCode::incomplete_forecast);

    const auto missing = EnsembleDataset::load(
        run_metadata(),
        {member_input("missing", 1.0, missing_slice)});
    REQUIRE(!missing.has_value());
    REQUIRE(missing.error().code == ErrorCode::incomplete_forecast);
}

TEST_CASE("permissive alignment reports unequal horizon and partial coverage") {
    const ConstantWindGribFixture reference;
    const ConstantWindGribFixture short_shifted(
        ConstantWindGribFixture::Options{
            .west_longitude_degrees = 1.0,
            .final_forecast_hour = 12});

    const auto ensemble = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("reference", 1.0, reference),
            member_input("short-shifted", 1.0, short_shifted),
        },
        EnsembleLoadOptions{EnsembleAlignmentMode::permissive_member_local});
    REQUIRE(ensemble.has_value());
    REQUIRE(!ensemble.value().alignment().exact_wind_valid_times);
    REQUIRE(!ensemble.value().alignment().equal_wind_horizon);
    REQUIRE(!ensemble.value().alignment().exact_wind_grid_geometry);
    REQUIRE(!ensemble.value().alignment().exact_wind_coverage);
    REQUIRE(ensemble.value().alignment().partial_wind_coverage);
    REQUIRE(!ensemble.value().alignment().shared_search_compatible());
}

TEST_CASE("strict alignment canonicalizes equivalent antimeridian bounds") {
    const ConstantWindGribFixture fixture(
        ConstantWindGribFixture::Options{
            .west_longitude_degrees = 179.0});

    EnsembleMemberInput negative_bound =
        member_input("negative-bound", 1.0, fixture);
    negative_bound.bounds =
        sailroute::GeographicBounds{0.0, -180.0, 2.0, -179.0};
    EnsembleMemberInput positive_bound =
        member_input("positive-bound", 1.0, fixture);
    positive_bound.bounds =
        sailroute::GeographicBounds{0.0, 180.0, 2.0, -179.0};

    const auto ensemble = EnsembleDataset::load(
        run_metadata(),
        {std::move(positive_bound), std::move(negative_bound)});
    REQUIRE(ensemble.has_value());
    REQUIRE(ensemble.value().alignment().exact_wind_grid_geometry);
    REQUIRE(ensemble.value().alignment().exact_wind_coverage);
    REQUIRE(ensemble.value().alignment().shared_search_compatible());
}

TEST_CASE("ensemble sampling remains canonical and matches independent members") {
    const ConstantWindGribFixture alpha_fixture(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 2.0,
            .north_metres_per_second = -1.0,
            .final_east_metres_per_second = 6.0});
    const ConstantWindGribFixture beta_fixture(
        ConstantWindGribFixture::Options{
            .east_metres_per_second = 4.0,
            .north_metres_per_second = -2.0,
            .final_east_metres_per_second = 8.0});
    const auto ensemble = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("beta", 1.0, beta_fixture),
            member_input("alpha", 1.0, alpha_fixture),
        });
    REQUIRE(ensemble.has_value());

    const sailroute::TimePoint first =
        ensemble.value().members()[0].wind_valid_times.front();
    const sailroute::TimePoint last =
        ensemble.value().members()[0].wind_valid_times.back();
    const auto common_sampler = ensemble.value().sampler_at(first);
    REQUIRE(common_sampler.has_value());
    REQUIRE(common_sampler.value().valid());
    const auto common = common_sampler.value().sample({1.0, 1.0});
    REQUIRE(common.size() == 2U);
    REQUIRE(common[0].member_identifier == "alpha");
    REQUIRE(common[1].member_identifier == "beta");
    for (std::size_t index = 0U; index < common.size(); ++index) {
        REQUIRE(common[index].wind.has_value());
        const auto independent =
            ensemble.value().member_weather(index)->interpolate({1.0, 1.0}, first);
        REQUIRE(independent.has_value());
        REQUIRE(
            common[index].wind.value().east_mps ==
            independent.value().east_mps);
        REQUIRE(
            common[index].wind.value().north_mps ==
            independent.value().north_mps);
    }

    const std::vector<sailroute::TimePoint> times{first, last};
    const auto member_sampler = ensemble.value().sampler_at(times);
    REQUIRE(member_sampler.has_value());
    const std::vector<sailroute::Coordinate> coordinates{
        {0.5, 0.5},
        {1.5, 1.5},
    };
    const auto member_samples = member_sampler.value().sample(coordinates);
    REQUIRE(member_samples.has_value());
    REQUIRE(member_samples.value().size() == 2U);
    REQUIRE(member_samples.value()[0].wind.value().east_mps == 2.0);
    REQUIRE(member_samples.value()[1].wind.value().east_mps == 8.0);
}

TEST_CASE("permissive sampling reports an outcome for every member-local failure") {
    const ConstantWindGribFixture long_fixture;
    const ConstantWindGribFixture short_fixture(
        ConstantWindGribFixture::Options{.final_forecast_hour = 12});
    const auto ensemble = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("long", 1.0, long_fixture),
            member_input("short", 1.0, short_fixture),
        },
        EnsembleLoadOptions{EnsembleAlignmentMode::permissive_member_local});
    REQUIRE(ensemble.has_value());

    const sailroute::TimePoint sample_time =
        ensemble.value().members()[0].wind_valid_times.front() +
        std::chrono::hours{18};
    const auto sampler = ensemble.value().sampler_at(sample_time);
    REQUIRE(sampler.has_value());
    REQUIRE(!sampler.value().valid());
    const auto samples = sampler.value().sample({1.0, 1.0});
    REQUIRE(samples.size() == 2U);
    REQUIRE(samples[0].member_identifier == "long");
    REQUIRE(samples[0].wind.has_value());
    REQUIRE(samples[1].member_identifier == "short");
    REQUIRE(!samples[1].wind.has_value());
    REQUIRE(
        samples[1].wind.error().code ==
        ErrorCode::departure_outside_forecast);
}

TEST_CASE("ensemble validates Stage 3 categories and preserves member providers") {
    const ConstantWindGribFixture alpha_fixture;
    const ConstantWindGribFixture beta_fixture;

    RoutingEnvironment alpha_environment;
    alpha_environment.currents.provider =
        sailroute::make_uniform_current_provider(
            {1.0, 0.0},
            provider_metadata("alpha-current"))
            .value();
    RoutingEnvironment beta_environment;
    beta_environment.currents.provider =
        sailroute::make_uniform_current_provider(
            {-1.0, 0.0},
            provider_metadata("beta-current"))
            .value();

    const auto ensemble = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("beta", 1.0, beta_fixture, beta_environment),
            member_input("alpha", 1.0, alpha_fixture, alpha_environment),
        });
    REQUIRE(ensemble.has_value());
    REQUIRE(ensemble.value().members()[0].configured_variables.currents);
    REQUIRE(
        ensemble.value().members()[0].environment->current_provider->name ==
        "alpha-current");
    REQUIRE(
        ensemble.value().members()[1].environment->current_provider->name ==
        "beta-current");
    REQUIRE(
        ensemble.value().members()[0].current_coverage->
            global_longitude_coverage);

    const auto mismatched_categories = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("alpha", 1.0, alpha_fixture, alpha_environment),
            member_input("beta", 1.0, beta_fixture),
        });
    REQUIRE(!mismatched_categories.has_value());
    REQUIRE(mismatched_categories.error().code == ErrorCode::invalid_environment);

    const auto permissive_categories = EnsembleDataset::load(
        run_metadata(),
        {
            member_input("alpha", 1.0, alpha_fixture, alpha_environment),
            member_input("beta", 1.0, beta_fixture),
        },
        EnsembleLoadOptions{EnsembleAlignmentMode::permissive_member_local});
    REQUIRE(permissive_categories.has_value());
    REQUIRE(
        !permissive_categories.value().alignment().
            aligned_environment_categories);
    REQUIRE(
        permissive_categories.value().members()[0].
            configured_variables.currents);
    REQUIRE(
        !permissive_categories.value().members()[1].
            configured_variables.currents);
    REQUIRE(
        !permissive_categories.value().alignment().
            shared_search_compatible());

    RoutingEnvironment invalid_environment;
    invalid_environment.waves.provider =
        sailroute::make_uniform_wave_provider(
            {1.0, 8.0, 0.0},
            provider_metadata("wave"))
            .value();
    const auto invalid = EnsembleDataset::load(
        run_metadata(),
        {member_input("invalid", 1.0, alpha_fixture, invalid_environment)});
    REQUIRE(!invalid.has_value());
    REQUIRE(invalid.error().code == ErrorCode::invalid_environment);
}
