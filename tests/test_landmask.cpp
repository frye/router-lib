#include "sailroute/environment.hpp"

#include "../src/routing/geodesy.hpp"
#include "test_support.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

using sailroute::Coordinate;
using sailroute::EnvironmentGridSpec;
using sailroute::EnvironmentSampleStatus;
using sailroute::ErrorCode;
using sailroute::LandmaskMetadata;
using sailroute::ProviderMetadata;
using sailroute::SignedDistanceLandmask;

LandmaskMetadata mask_metadata(
    double resolution = 6.0,
    double interpolation_error = 0.0) {
    LandmaskMetadata metadata;
    metadata.provider = ProviderMetadata{"test_mask", "unit test", "1"};
    metadata.resolution_nautical_miles = resolution;
    metadata.interpolation_error_nautical_miles = interpolation_error;
    return metadata;
}

// One degree of latitude is 60 nautical miles, so a signed distance built from
// the distance to a meridian of longitude has a known analytic value at every
// point and stays 1-Lipschitz, which is what segment certification relies on.
SignedDistanceLandmask meridian_channel(
    double land_longitude_degrees,
    double half_width_degrees,
    double interpolation_error = 0.05) {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = -2.0;
    spec.west_longitude_degrees = -4.0;
    spec.latitude_step_degrees = 0.1;
    spec.longitude_step_degrees = 0.1;
    spec.latitude_count = 41U;
    spec.longitude_count = 81U;

    std::vector<double> distances(
        spec.latitude_count * spec.longitude_count, 0.0);
    for (std::size_t row = 0U; row < spec.latitude_count; ++row) {
        const double latitude = spec.south_latitude_degrees +
            spec.latitude_step_degrees * static_cast<double>(row);
        for (std::size_t column = 0U; column < spec.longitude_count; ++column) {
            const double longitude = spec.west_longitude_degrees +
                spec.longitude_step_degrees * static_cast<double>(column);
            const double offset =
                std::abs(longitude - land_longitude_degrees) - half_width_degrees;
            // Convert the longitude offset to nautical miles at this latitude.
            distances[row * spec.longitude_count + column] =
                offset * 60.0 * std::cos(latitude * 3.14159265358979323846 / 180.0);
        }
    }
    auto mask = SignedDistanceLandmask::create(
        spec, std::move(distances), mask_metadata(6.0, interpolation_error));
    REQUIRE(mask.has_value());
    return std::move(mask.value());
}

}  // namespace

TEST_CASE("landmask construction validates metadata, grid, and samples") {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = 0.0;
    spec.west_longitude_degrees = 0.0;
    spec.latitude_step_degrees = 1.0;
    spec.longitude_step_degrees = 1.0;
    spec.latitude_count = 2U;
    spec.longitude_count = 2U;

    REQUIRE(!SignedDistanceLandmask::create(
                 spec,
                 {1.0, 1.0, 1.0, 1.0},
                 LandmaskMetadata{ProviderMetadata{"", "s", "1"}, 1.0, 0.0})
                 .has_value());

    LandmaskMetadata bad_resolution = mask_metadata();
    bad_resolution.resolution_nautical_miles = 0.0;
    REQUIRE(!SignedDistanceLandmask::create(
                 spec, {1.0, 1.0, 1.0, 1.0}, bad_resolution)
                 .has_value());

    LandmaskMetadata bad_error = mask_metadata();
    bad_error.interpolation_error_nautical_miles = -1.0;
    REQUIRE(
        !SignedDistanceLandmask::create(spec, {1.0, 1.0, 1.0, 1.0}, bad_error)
             .has_value());

    REQUIRE(!SignedDistanceLandmask::create(spec, {1.0, 1.0, 1.0}, mask_metadata())
                 .has_value());

    const auto non_finite = SignedDistanceLandmask::create(
        spec,
        {1.0, 1.0, 1.0, std::numeric_limits<double>::quiet_NaN()},
        mask_metadata());
    REQUIRE(!non_finite.has_value());
    REQUIRE(non_finite.error().code == ErrorCode::invalid_environment);
}

TEST_CASE("landmask point queries are signed, bilinear, and coverage bounded") {
    const SignedDistanceLandmask mask = meridian_channel(0.0, 0.5, 0.0);
    REQUIRE(mask.metadata().resolution_nautical_miles == 6.0);
    REQUIRE(mask.grid().latitude_count == 41U);

    const auto on_land = mask.signed_distance_nautical_miles(Coordinate{0.0, 0.0});
    REQUIRE(on_land.has_value());
    REQUIRE(on_land.value < 0.0);
    REQUIRE_NEAR(on_land.value, -30.0, 1e-6);

    const auto in_water = mask.signed_distance_nautical_miles(Coordinate{0.0, 2.0});
    REQUIRE(in_water.has_value());
    REQUIRE_NEAR(in_water.value, 90.0, 1e-6);

    const auto coastline =
        mask.signed_distance_nautical_miles(Coordinate{0.0, 0.5});
    REQUIRE(coastline.has_value());
    REQUIRE_NEAR(coastline.value, 0.0, 1e-6);

    const auto outside =
        mask.signed_distance_nautical_miles(Coordinate{40.0, 0.0});
    REQUIRE(!outside.has_value());
    REQUIRE(outside.status == EnvironmentSampleStatus::outside_coverage);

    const SignedDistanceLandmask empty;
    const auto unconfigured =
        empty.signed_distance_nautical_miles(Coordinate{0.0, 0.0});
    REQUIRE(!unconfigured.has_value());
    REQUIRE(unconfigured.status == EnvironmentSampleStatus::unavailable);
}

TEST_CASE("segment certification rejects a crossing between water endpoints") {
    const SignedDistanceLandmask mask = meridian_channel(0.0, 0.5, 0.0);

    // Both endpoints are 90 nautical miles offshore, but the straight line
    // between them runs straight over the island.
    const auto crossing =
        mask.certify_segment({0.0, -2.0}, {0.0, 2.0}, 0.0, 12U);
    REQUIRE(!crossing.clear);
    REQUIRE(crossing.status == EnvironmentSampleStatus::available);
    REQUIRE(crossing.distance_queries > 2U);

    // The same pair of endpoints joined by a leg that stays west of the island
    // is certified.
    const auto clear = mask.certify_segment({0.0, -2.0}, {1.0, -1.5}, 0.0, 12U);
    REQUIRE(clear.clear);

    // An endpoint on land is rejected immediately.
    const auto from_land =
        mask.certify_segment({0.0, 0.0}, {0.0, 2.0}, 0.0, 12U);
    REQUIRE(!from_land.clear);
    REQUIRE(from_land.distance_queries <= 2U);
}

TEST_CASE("certification honours clearance and the mask's own error bound") {
    const SignedDistanceLandmask mask = meridian_channel(0.0, 0.5, 0.0);

    // A leg 12 nautical miles off the coast passes with no clearance but not
    // with a 20 nautical mile clearance.
    const Coordinate near_shore_south{-0.5, 0.7};
    const Coordinate near_shore_north{0.5, 0.7};
    REQUIRE(
        mask.certify_segment(near_shore_south, near_shore_north, 0.0, 12U).clear);
    REQUIRE(
        !mask.certify_segment(near_shore_south, near_shore_north, 20.0, 12U)
             .clear);

    // A mask that admits a 20 nautical mile interpolation error must reject the
    // same leg even when no clearance is requested, because the error is added
    // to the requirement rather than rounded toward accepting land.
    const SignedDistanceLandmask coarse = meridian_channel(0.0, 0.5, 20.0);
    REQUIRE(
        !coarse.certify_segment(near_shore_south, near_shore_north, 0.0, 12U)
             .clear);

    const auto invalid_clearance =
        mask.certify_segment(near_shore_south, near_shore_north, -1.0, 12U);
    REQUIRE(!invalid_clearance.clear);
    REQUIRE(invalid_clearance.status == EnvironmentSampleStatus::invalid_data);
}

TEST_CASE("a narrow channel closes as the required clearance grows") {
    // Two islands 0.4 degrees apart leave a channel roughly 24 nautical miles
    // wide, so its centre line is about 12 nautical miles from either shore.
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = -1.0;
    spec.west_longitude_degrees = -2.0;
    spec.latitude_step_degrees = 0.05;
    spec.longitude_step_degrees = 0.05;
    spec.latitude_count = 41U;
    spec.longitude_count = 81U;

    std::vector<double> distances(
        spec.latitude_count * spec.longitude_count, 0.0);
    for (std::size_t row = 0U; row < spec.latitude_count; ++row) {
        for (std::size_t column = 0U; column < spec.longitude_count; ++column) {
            const double longitude = spec.west_longitude_degrees +
                spec.longitude_step_degrees * static_cast<double>(column);
            const double west_island = std::abs(longitude + 0.5) - 0.3;
            const double east_island = std::abs(longitude - 0.5) - 0.3;
            distances[row * spec.longitude_count + column] =
                std::min(west_island, east_island) * 60.0;
        }
    }
    const auto mask = SignedDistanceLandmask::create(
        spec, std::move(distances), mask_metadata(3.0, 0.0));
    REQUIRE(mask.has_value());

    const Coordinate south{-0.9, 0.0};
    const Coordinate north{0.9, 0.0};
    REQUIRE(mask.value().certify_segment(south, north, 0.0, 14U).clear);
    REQUIRE(mask.value().certify_segment(south, north, 6.0, 14U).clear);
    REQUIRE(!mask.value().certify_segment(south, north, 18.0, 14U).clear);
}

TEST_CASE("leaving the mask is reported rather than assumed to be open water") {
    const SignedDistanceLandmask mask = meridian_channel(0.0, 0.5, 0.0);
    const auto leaving = mask.certify_segment({0.0, -2.0}, {30.0, -2.0}, 0.0, 12U);
    REQUIRE(!leaving.clear);
    REQUIRE(leaving.status == EnvironmentSampleStatus::outside_coverage);
}

TEST_CASE("certification works across the antimeridian and near the pole") {
    EnvironmentGridSpec spec;
    spec.south_latitude_degrees = -1.0;
    spec.west_longitude_degrees = 178.0;
    spec.latitude_step_degrees = 0.5;
    spec.longitude_step_degrees = 0.5;
    spec.latitude_count = 5U;
    spec.longitude_count = 9U;

    // Land occupies a strip centred exactly on the antimeridian.
    std::vector<double> distances(
        spec.latitude_count * spec.longitude_count, 0.0);
    for (std::size_t row = 0U; row < spec.latitude_count; ++row) {
        for (std::size_t column = 0U; column < spec.longitude_count; ++column) {
            const double offset =
                spec.longitude_step_degrees * static_cast<double>(column) - 2.0;
            distances[row * spec.longitude_count + column] =
                (std::abs(offset) - 0.5) * 60.0;
        }
    }
    const auto mask = SignedDistanceLandmask::create(
        spec, std::move(distances), mask_metadata(30.0, 0.0));
    REQUIRE(mask.has_value());

    const auto on_meridian =
        mask.value().signed_distance_nautical_miles(Coordinate{0.0, -180.0});
    REQUIRE(on_meridian.has_value());
    REQUIRE(on_meridian.value < 0.0);

    const auto crossing =
        mask.value().certify_segment({0.0, 178.5}, {0.0, -178.5}, 0.0, 12U);
    REQUIRE(!crossing.clear);

    const auto clear =
        mask.value().certify_segment({-0.5, 178.5}, {0.5, 178.5}, 0.0, 12U);
    REQUIRE(clear.clear);

    // A polar mask covers every longitude, so a leg over the top of the world
    // stays inside it.
    EnvironmentGridSpec polar;
    polar.south_latitude_degrees = 85.0;
    polar.west_longitude_degrees = -180.0;
    polar.latitude_step_degrees = 1.0;
    polar.longitude_step_degrees = 45.0;
    polar.latitude_count = 6U;
    polar.longitude_count = 8U;
    polar.global_longitude_coverage = true;
    const auto polar_mask = SignedDistanceLandmask::create(
        polar,
        std::vector<double>(48U, 200.0),
        mask_metadata(60.0, 0.0));
    REQUIRE(polar_mask.has_value());
    REQUIRE(
        polar_mask.value().certify_segment({88.0, 0.0}, {88.0, 180.0}, 10.0, 12U)
            .clear);
}

TEST_CASE("certification concedes rather than guessing when the budget runs out") {
    const SignedDistanceLandmask mask = meridian_channel(0.0, 0.5, 0.0);
    // One subdivision cannot resolve a long leg past a narrow island, so the
    // conservative answer is to reject.
    const auto shallow =
        mask.certify_segment({-1.5, -3.0}, {1.5, 3.0}, 0.0, 1U);
    REQUIRE(!shallow.clear);
    REQUIRE(shallow.status == EnvironmentSampleStatus::available);
}
