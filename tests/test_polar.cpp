#include "sailroute/polar.hpp"

#include "test_support.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace {

class PolarFixture {
public:
    PolarFixture(std::string name, std::string contents)
        : path_(std::filesystem::current_path() / std::move(name)) {
        std::ofstream output(path_);
        output << contents;
        if (!output) {
            throw std::runtime_error("unable to create polar fixture");
        }
    }

    ~PolarFixture() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    std::filesystem::path path_;
};

}  // namespace

TEST_CASE("CSV polar loads and interpolates with folded angles") {
    const PolarFixture fixture{
        "test_polar_matrix.csv",
        "# TWS columns and TWA rows, all speeds in knots\n"
        "TWA/TWS,0,10,20\n"
        "0,0,0,0\n"
        "90,0,10,20\n"
        "180,0,5,10\n"};

    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    REQUIRE_NEAR(loaded.value().boat_speed_knots(5.0, 45.0), 2.5, 1e-12);
    REQUIRE_NEAR(loaded.value().boat_speed_knots(15.0, -90.0), 15.0, 1e-12);
    REQUIRE_NEAR(loaded.value().boat_speed_knots(30.0, 270.0), 20.0, 1e-12);
    REQUIRE_NEAR(loaded.value().boat_speed_knots(10.0, 360.0), 0.0, 1e-12);
    REQUIRE_NEAR(loaded.value().boat_speed_knots(0.0, 90.0), 0.0, 1e-12);
}

TEST_CASE("Expedition polar accepts comments and common delimiters") {
    const PolarFixture fixture{
        "test_expedition.pol",
        "! Exported Expedition-style polar\n"
        "; semicolon comment\n"
        "Polar version 1\n"
        "TWA\\TWS ; 6 ; 12 ; 18\n"
        "30 ; 0 ; 0 ; 0\n"
        "60 ; 4 ; 6 ; 8\n"
        "120 ; 5 ; 7 ; 9\n"
        "180 ; 3 ; 5 ; 7\n"};

    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    REQUIRE_NEAR(loaded.value().boat_speed_knots(3.0, 60.0), 2.0, 1e-12);
    REQUIRE_NEAR(loaded.value().boat_speed_knots(9.0, 90.0), 5.5, 1e-12);
    REQUIRE(loaded.value().source().find("test_expedition.pol") != std::string::npos);
}

TEST_CASE("numeric Expedition headers support zero TWS and zero corner variants") {
    const PolarFixture zero_tws{
        "test_zero_tws.pol",
        "0 10 20\n"
        "0 0 0 0\n"
        "90 0 10 20\n"};
    const auto first = sailroute::VesselPolar::load(zero_tws.path());
    REQUIRE(first.has_value());
    REQUIRE_NEAR(first.value().boat_speed_knots(5.0, 90.0), 5.0, 1e-12);

    const PolarFixture zero_corner{
        "test_zero_corner.pol",
        "0\t10\t20\n"
        "0\t0\t0\n"
        "90\t10\t20\n"};
    const auto second = sailroute::VesselPolar::load(zero_corner.path());
    REQUIRE(second.has_value());
    REQUIRE_NEAR(second.value().boat_speed_knots(15.0, 45.0), 7.5, 1e-12);
}

TEST_CASE("invalid polar diagnostics identify bad rows") {
    const PolarFixture fixture{
        "test_invalid_polar.csv",
        "TWA/TWS,6,12\n"
        "30,4,5\n"
        "20,4,5\n"};

    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(!loaded.has_value());
    REQUIRE(loaded.error().code == sailroute::ErrorCode::invalid_polar);
    REQUIRE(loaded.error().message.find("line 3") != std::string::npos);
    REQUIRE(loaded.error().message.find("strictly increasing") != std::string::npos);
}

TEST_CASE("built-in polar is conservative and clearly identified") {
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    REQUIRE(polar.source().find("Built-in approximate") != std::string::npos);
    REQUIRE(polar.source().find("not manufacturer") != std::string::npos);
    REQUIRE_NEAR(polar.boat_speed_knots(20.0, 30.0), 0.0, 1e-12);
    REQUIRE(polar.boat_speed_knots(20.0, 90.0) > 0.0);
    REQUIRE_NEAR(
        polar.boat_speed_knots(
            std::numeric_limits<double>::quiet_NaN(), 90.0),
        0.0,
        1e-12);
}

TEST_CASE("sample First 44-class polar loads") {
    const auto path =
        std::filesystem::path{SAILROUTE_SOURCE_DIR} / "samples" / "sample.pol";
    const auto loaded = sailroute::VesselPolar::load(path);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded.value().boat_speed_knots(12.0, 45.0) > 7.0);
    REQUIRE(loaded.value().boat_speed_knots(20.0, 135.0) > 10.0);
}

TEST_CASE("monotone cubic polar interpolation passes through tabulated rows") {
    const PolarFixture fixture{
        "test_polar_pchip.csv",
        "TWA/TWS,0,10\n"
        "0,0,0\n"
        "40,0,5\n"
        "60,0,7\n"
        "90,0,8\n"
        "120,0,9\n"
        "150,0,6\n"
        "180,0,4\n"};

    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    const sailroute::PolarSlice cubic = loaded.value().slice_at(
        10.0, sailroute::PolarAngleInterpolation::monotone_cubic);
    REQUIRE(cubic.valid());

    // Interpolation must reproduce every tabulated row exactly.
    REQUIRE_NEAR(cubic.speed_knots(40.0), 5.0, 1e-12);
    REQUIRE_NEAR(cubic.speed_knots(90.0), 8.0, 1e-12);
    REQUIRE_NEAR(cubic.speed_knots(120.0), 9.0, 1e-12);
    REQUIRE_NEAR(cubic.speed_knots(180.0), 4.0, 1e-12);
}

TEST_CASE("monotone cubic polar interpolation does not overshoot") {
    const PolarFixture fixture{
        "test_polar_pchip_shape.csv",
        "TWA/TWS,0,10\n"
        "0,0,0\n"
        "40,0,5\n"
        "60,0,7\n"
        "90,0,8\n"
        "120,0,9\n"
        "150,0,6\n"
        "180,0,4\n"};

    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    const sailroute::PolarSlice cubic = loaded.value().slice_at(
        10.0, sailroute::PolarAngleInterpolation::monotone_cubic);

    // A shape-preserving scheme stays inside the bracketing rows on every
    // monotone stretch, unlike an unconstrained spline.
    for (int step = 0; step <= 100; ++step) {
        const double angle = 40.0 + 0.2 * static_cast<double>(step);
        const double speed = cubic.speed_knots(angle);
        REQUIRE(speed >= 5.0 - 1e-12);
        REQUIRE(speed <= 8.0 + 1e-12);
    }
    for (int step = 0; step <= 100; ++step) {
        const double angle = 150.0 + 0.3 * static_cast<double>(step);
        const double speed = cubic.speed_knots(angle);
        REQUIRE(speed >= 4.0 - 1e-12);
        REQUIRE(speed <= 6.0 + 1e-12);
    }
}

TEST_CASE("polar slice matches direct lookup and reports wind range") {
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    const double maximum = polar.maximum_tabulated_wind_speed_knots();
    REQUIRE(maximum > 0.0);

    for (const double wind : {2.0, 6.5, 12.0, 18.0}) {
        const sailroute::PolarSlice slice = polar.slice_at(wind);
        REQUIRE(!slice.above_tabulated_wind_speed());
        for (const double angle : {0.0, 35.0, 52.5, 90.0, 135.0, 175.0, 180.0}) {
            REQUIRE(slice.speed_knots(angle) == polar.boat_speed_knots(wind, angle));
        }
    }

    const sailroute::PolarSlice above = polar.slice_at(maximum + 25.0);
    REQUIRE(above.above_tabulated_wind_speed());
    REQUIRE(above.speed_knots(90.0) == polar.boat_speed_knots(maximum, 90.0));
}

TEST_CASE("velocity made good angles bracket the polar optima") {
    const auto polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    const sailroute::PolarSlice slice = polar.slice_at(12.0);
    const sailroute::VelocityMadeGoodAngles optima =
        slice.velocity_made_good_angles();
    REQUIRE(optima.valid);
    REQUIRE(optima.upwind_degrees > 0.0);
    REQUIRE(optima.upwind_degrees < 90.0);
    REQUIRE(optima.downwind_degrees > 90.0);
    REQUIRE(optima.downwind_degrees < 180.0);

    // The reported angles must beat a one-degree sweep of every alternative.
    const double best_upwind =
        slice.speed_knots(optima.upwind_degrees) *
        std::cos(optima.upwind_degrees * 3.14159265358979323846 / 180.0);
    const double best_downwind =
        -slice.speed_knots(optima.downwind_degrees) *
        std::cos(optima.downwind_degrees * 3.14159265358979323846 / 180.0);
    for (int angle = 1; angle < 180; ++angle) {
        const double radians =
            static_cast<double>(angle) * 3.14159265358979323846 / 180.0;
        const double made_good =
            slice.speed_knots(static_cast<double>(angle)) * std::cos(radians);
        REQUIRE(made_good <= best_upwind + 1e-6);
        REQUIRE(-made_good <= best_downwind + 1e-6);
    }
}
