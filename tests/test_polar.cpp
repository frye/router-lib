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

TEST_CASE("polar support tolerates geodesic roundoff without bridging forbidden sectors") {
    const PolarFixture fixture{
        "test_polar_roundoff.csv",
        "TWA/TWS,8,32\n"
        "0,0,0\n44,0,0\n45,6,24\n46,0,0\n"
        "134,0,0\n135,6,24\n136,0,0\n180,0,0\n"};
    const auto polar = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(polar.has_value());
    const auto slice = polar.value().slice_at(16.0);
    REQUIRE(slice.supports_sailing_angle(135.0 + 1.0e-10));
    REQUIRE(slice.supports_sailing_angle(45.0 - 1.0e-10));
    REQUIRE(!slice.supports_sailing_angle(135.0 + 1.0e-4));
    REQUIRE(!slice.supports_sailing_angle(45.0 - 1.0e-4));
    REQUIRE(!slice.supports_sailing_angle(0.0));
}

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
    const auto light_wind = polar.slice_at(4.0);
    REQUIRE(light_wind.supports_sailing_angle(45.0));
    REQUIRE(!light_wind.supports_sailing_angle(30.0));
    REQUIRE_NEAR(light_wind.speed_knots(45.0), 4.1 * 4.0 / 6.0, 1e-12);
    REQUIRE(!polar.slice_at(0.0).supports_sailing_angle(45.0));
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
    for (const double wind : {0.1, 2.0, 4.0}) {
        const auto slice = loaded.value().slice_at(wind);
        REQUIRE(slice.supports_sailing_angle(35.0));
        REQUIRE(slice.supports_sailing_angle(90.0));
        REQUIRE(!slice.supports_sailing_angle(34.99));
        REQUIRE(slice.speed_knots(90.0) > 0.0);
    }
    REQUIRE(!loaded.value().slice_at(0.0).supports_sailing_angle(90.0));
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

TEST_CASE("polar imports native Expedition rows with irregular angle support") {
    const PolarFixture fixture{
        "test_polar_native_irregular.pol",
        "\xEF\xBB\xBF! Native Expedition polar\n"
        "Polar version 1\n"
        "6\t40 4\t90 6 150 5 # three pairs\n"
        "12 50 5 80 8 120 9 170 7\n"};
    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    const auto& polar = loaded.value();
    REQUIRE_NEAR(polar.boat_speed_knots(6.0, 90.0), 6.0, 1e-12);
    REQUIRE_NEAR(polar.boat_speed_knots(12.0, 80.0), 8.0, 1e-12);
    REQUIRE_NEAR(polar.boat_speed_knots(9.0, 90.0), 7.125, 1e-12);
    REQUIRE_NEAR(polar.boat_speed_knots(3.0, 90.0), 3.0, 1e-12);
    REQUIRE_NEAR(polar.maximum_boat_speed_knots(), 9.0, 1e-12);
    REQUIRE_NEAR(polar.maximum_tabulated_wind_speed_knots(), 12.0, 1e-12);
    for (const auto interpolation : {
             sailroute::PolarAngleInterpolation::linear,
             sailroute::PolarAngleInterpolation::monotone_cubic}) {
        const auto low = polar.slice_at(6.0, interpolation);
        const auto between = polar.slice_at(9.0, interpolation);
        const auto high = polar.slice_at(12.0, interpolation);
        REQUIRE_NEAR(low.minimum_sailing_angle_degrees(), 40.0, 1e-12);
        REQUIRE_NEAR(low.maximum_sailing_angle_degrees(), 150.0, 1e-12);
        REQUIRE_NEAR(between.minimum_sailing_angle_degrees(), 50.0, 1e-12);
        REQUIRE_NEAR(between.maximum_sailing_angle_degrees(), 150.0, 1e-12);
        REQUIRE_NEAR(high.minimum_sailing_angle_degrees(), 50.0, 1e-12);
        REQUIRE_NEAR(high.maximum_sailing_angle_degrees(), 170.0, 1e-12);
        REQUIRE(low.supports_sailing_angle(40.0));
        REQUIRE(!between.supports_sailing_angle(40.0));
        REQUIRE(between.supports_sailing_angle(50.0));
        REQUIRE(between.supports_sailing_angle(-90.0));
        REQUIRE(between.supports_sailing_angle(270.0));
        REQUIRE(between.supports_sailing_angle(150.0));
        REQUIRE(!between.supports_sailing_angle(150.01));
        REQUIRE(!low.supports_sailing_angle(160.0));
        REQUIRE(high.supports_sailing_angle(160.0));
        REQUIRE_NEAR(low.speed_knots(90.0), 6.0, 1e-12);
        REQUIRE_NEAR(high.speed_knots(80.0), 8.0, 1e-12);
        const auto optima = between.velocity_made_good_angles();
        REQUIRE(optima.valid);
        REQUIRE(between.supports_sailing_angle(optima.upwind_degrees));
        REQUIRE(between.supports_sailing_angle(optima.downwind_degrees));
    }
    REQUIRE(polar.slice_at(0.1).supports_sailing_angle(40.0));
    REQUIRE(!polar.slice_at(0.0).supports_sailing_angle(90.0));
    REQUIRE(polar.slice_at(20.0).supports_sailing_angle(170.0));
    REQUIRE(!polar.slice_at(20.0).supports_sailing_angle(175.0));
}

TEST_CASE("polar rejects ambiguous numeric formats unless explicitly selected") {
    const PolarFixture fixture{
        "test_polar_ambiguous.pol",
        "0 30 40 90 100\n"
        "10 40 50 100 110\n"
        "50 50 60 110 120\n"};
    const auto ambiguous = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(!ambiguous.has_value());
    REQUIRE(ambiguous.error().code == sailroute::ErrorCode::invalid_polar);
    REQUIRE(ambiguous.error().message.find("ambiguous") != std::string::npos);
    REQUIRE(ambiguous.error().message.find("line 1") != std::string::npos);
    sailroute::PolarLoadOptions options;
    options.format = sailroute::PolarFormat::matrix;
    const auto matrix = sailroute::VesselPolar::load(fixture.path(), options);
    REQUIRE(matrix.has_value());
    REQUIRE_NEAR(matrix.value().boat_speed_knots(30.0, 10.0), 40.0, 1e-12);
    options.format = sailroute::PolarFormat::expedition;
    const auto native = sailroute::VesselPolar::load(fixture.path(), options);
    REQUIRE(native.has_value());
    REQUIRE_NEAR(native.value().boat_speed_knots(10.0, 40.0), 50.0, 1e-12);
}

TEST_CASE("polar sailing support never clamps absent head-to-wind or downwind sectors") {
    const PolarFixture fixture{
        "test_polar_limited_domain.csv",
        "TWA/TWS,6,12\n"
        "45,5,7\n"
        "90,7,9\n"
        "150,6,8\n"};
    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    for (const auto interpolation : {
             sailroute::PolarAngleInterpolation::linear,
             sailroute::PolarAngleInterpolation::monotone_cubic}) {
        const auto slice = loaded.value().slice_at(9.0, interpolation);
        // Raw interpolation remains compatible, but never establishes legality.
        REQUIRE(slice.speed_knots(0.0) > 0.0);
        REQUIRE(!slice.supports_sailing_angle(0.0));
        REQUIRE(!slice.supports_sailing_angle(360.0));
        REQUIRE(!slice.supports_sailing_angle(44.99));
        REQUIRE(slice.supports_sailing_angle(45.0));
        REQUIRE(slice.supports_sailing_angle(-45.0));
        REQUIRE(slice.supports_sailing_angle(150.0));
        REQUIRE(!slice.supports_sailing_angle(150.01));
        REQUIRE(!slice.supports_sailing_angle(180.0));
        REQUIRE(!slice.supports_sailing_angle(std::numeric_limits<double>::quiet_NaN()));
        REQUIRE(!slice.supports_sailing_angle(std::numeric_limits<double>::infinity()));
    }
}

TEST_CASE("polar zero samples do not authorize interpolated no-go or interior sectors") {
    const PolarFixture fixture{
        "test_polar_zero_sectors.csv",
        "TWA/TWS,0,6,12\n"
        "0,0,0,0\n"
        "45,0,5,7\n"
        "60,0,6,8\n"
        "90,0,0,0\n"
        "120,0,7,9\n"
        "150,0,6,8\n"
        "180,0,0,0\n"};
    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    for (const auto interpolation : {
             sailroute::PolarAngleInterpolation::linear,
             sailroute::PolarAngleInterpolation::monotone_cubic}) {
        for (const double wind : {0.5, 6.0, 9.0, 12.0}) {
            const auto slice = loaded.value().slice_at(wind, interpolation);
            REQUIRE_NEAR(slice.minimum_sailing_angle_degrees(), 45.0, 1e-12);
            REQUIRE_NEAR(slice.maximum_sailing_angle_degrees(), 150.0, 1e-12);
            for (const double angle : {0.0, 20.0, 44.99, 60.01, 75.0, 90.0, 110.0, 160.0, 180.0}) {
                REQUIRE(!slice.supports_sailing_angle(angle));
            }
            for (const double angle : {45.0, 50.0, 60.0, 120.0, 135.0, 150.0}) {
                REQUIRE(slice.supports_sailing_angle(angle));
            }
            REQUIRE(slice.speed_knots(20.0) > 0.0);
            REQUIRE_NEAR(slice.speed_knots(90.0), 0.0, 1e-12);
        }
    }
}

TEST_CASE("polar native normalization preserves original zero-adjacent forbidden sectors") {
    const PolarFixture fixture{
        "test_polar_native_zero_sectors.pol",
        "6 30 0 50 5 90 8 120 0 150 6 180 5\n"
        "12 40 4 60 6 80 8 100 9 140 7 160 6 180 5\n"};
    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    for (const auto interpolation : {
             sailroute::PolarAngleInterpolation::linear,
             sailroute::PolarAngleInterpolation::monotone_cubic}) {
        const auto slice = loaded.value().slice_at(9.0, interpolation);
        for (const double angle : {35.0, 40.0, 45.0, 95.0, 100.0, 120.0, 140.0, 145.0}) {
            REQUIRE(!slice.supports_sailing_angle(angle));
        }
        for (const double angle : {50.0, 60.0, 90.0, 150.0, 160.0, 180.0}) {
            REQUIRE(slice.supports_sailing_angle(angle));
        }
        REQUIRE(slice.speed_knots(45.0) > 0.0);
        REQUIRE(loaded.value().slice_at(12.0, interpolation).supports_sailing_angle(100.0));
    }
}

TEST_CASE("polar load minimum sailing angle only restricts supplied support") {
    const PolarFixture fixture{
        "test_polar_explicit_minimum.csv",
        "TWA/TWS,6,12\n"
        "0,0,0\n"
        "40,5,7\n"
        "60,7,9\n"
        "150,6,8\n"};
    sailroute::PolarLoadOptions options;
    options.minimum_sailing_angle_degrees = 47.5;
    const auto loaded = sailroute::VesselPolar::load(fixture.path(), options);
    REQUIRE(loaded.has_value());
    const auto slice = loaded.value().slice_at(9.0);
    REQUIRE_NEAR(slice.minimum_sailing_angle_degrees(), 47.5, 1e-12);
    REQUIRE(!slice.supports_sailing_angle(47.49));
    REQUIRE(slice.supports_sailing_angle(47.5));
    REQUIRE(slice.supports_sailing_angle(60.0));
    REQUIRE(!slice.supports_sailing_angle(160.0));
    const auto optima = slice.velocity_made_good_angles();
    REQUIRE(optima.valid);
    REQUIRE(slice.supports_sailing_angle(optima.upwind_degrees));
    options.minimum_sailing_angle_degrees = 20.0;
    const auto lower = sailroute::VesselPolar::load(fixture.path(), options);
    REQUIRE(lower.has_value());
    REQUIRE_NEAR(lower.value().slice_at(9.0).minimum_sailing_angle_degrees(), 40.0, 1e-12);
    options.minimum_sailing_angle_degrees = 160.0;
    const auto empty = sailroute::VesselPolar::load(fixture.path(), options);
    REQUIRE(empty.has_value());
    REQUIRE(std::isnan(empty.value().slice_at(9.0).minimum_sailing_angle_degrees()));
    REQUIRE(!empty.value().slice_at(9.0).supports_sailing_angle(170.0));
    REQUIRE(!empty.value().slice_at(9.0).velocity_made_good_angles().valid);
    for (const double invalid : {0.0, -1.0, 180.01,
             std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity()}) {
        options.minimum_sailing_angle_degrees = invalid;
        const auto rejected = sailroute::VesselPolar::load(fixture.path(), options);
        REQUIRE(!rejected.has_value());
        REQUIRE(rejected.error().code == sailroute::ErrorCode::invalid_polar);
    }
}

TEST_CASE("polar absent support includes empty columns disjoint curves and invalid slices") {
    const PolarFixture fixture{
        "test_polar_empty_column.csv",
        "TWA/TWS,0,6,12\n"
        "45,0,0,5\n"
        "90,0,0,8\n"
        "150,0,0,6\n"};
    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    for (const double wind : {0.0, 3.0, 6.0, 9.0}) {
        const auto slice = loaded.value().slice_at(wind);
        REQUIRE(!slice.supports_sailing_angle(90.0));
        REQUIRE(std::isnan(slice.minimum_sailing_angle_degrees()));
        REQUIRE(std::isnan(slice.maximum_sailing_angle_degrees()));
        REQUIRE(!slice.velocity_made_good_angles().valid);
    }
    REQUIRE(loaded.value().slice_at(12.0).supports_sailing_angle(90.0));
    const PolarFixture disjoint{
        "test_polar_disjoint.pol",
        "6 30 4 60 6\n"
        "12 120 8 150 6\n"};
    const auto native = sailroute::VesselPolar::load(disjoint.path());
    REQUIRE(native.has_value());
    REQUIRE(native.value().slice_at(6.0).supports_sailing_angle(45.0));
    REQUIRE(native.value().slice_at(12.0).supports_sailing_angle(135.0));
    REQUIRE(!native.value().slice_at(9.0).supports_sailing_angle(90.0));
    REQUIRE(std::isnan(native.value().slice_at(9.0).minimum_sailing_angle_degrees()));
    REQUIRE(!sailroute::PolarSlice{}.supports_sailing_angle(90.0));
    REQUIRE(std::isnan(sailroute::PolarSlice{}.minimum_sailing_angle_degrees()));
    REQUIRE(!loaded.value().slice_at(-1.0).supports_sailing_angle(90.0));
}

TEST_CASE("polar malformed native rows fail with line-specific diagnostics") {
    const std::string first = "6 40 4 90 6 150 5\n";
    const std::string invalid_rows[]{
        "12 50 5 90 7 150\n",
        "12 50 5\n",
        "6 50 5 90 7 150 6\n",
        "-12 50 5 90 7 150 6\n",
        "12 90 5 50 7 150 6\n",
        "12 50 5 50 7 150 6\n",
        "12 50 5 90 7 181 6\n",
        "12 -1 5 90 7 150 6\n",
        "12 50 -5 90 7 150 6\n",
        "12 50 nan 90 7 150 6\n",
        "inf 50 5 90 7 150 6\n",
        "12 50 5 90 missing 150 6\n",
        "12,50,5,90,7,150,\n",
        "12 50 5 70 7\n",
        "not a numeric row\n",
    };
    for (const auto& row : invalid_rows) {
        const PolarFixture fixture{"test_polar_native_malformed.pol", first + row};
        for (const auto format : {sailroute::PolarFormat::automatic, sailroute::PolarFormat::expedition}) {
            sailroute::PolarLoadOptions options;
            options.format = format;
            const auto loaded = sailroute::VesselPolar::load(fixture.path(), options);
            REQUIRE(!loaded.has_value());
            REQUIRE(loaded.error().code == sailroute::ErrorCode::invalid_polar);
            REQUIRE(loaded.error().message.find("line 2") != std::string::npos);
        }
    }
}

TEST_CASE("polar malformed matrix cells are not silently skipped or treated as zero") {
    for (const std::string row : {
             "90,,8\n", "90,7,\n", "90,7\n", "90,nan,8\n",
             "90,-1,8\n", "missing,7,8\n", "unrecognized row\n"}) {
        const PolarFixture fixture{
            "test_polar_missing_matrix.csv",
            "TWA/TWS,6,12\n45,5,7\n" + row + "150,6,8\n"};
        const auto loaded = sailroute::VesselPolar::load(fixture.path());
        REQUIRE(!loaded.has_value());
        REQUIRE(loaded.error().code == sailroute::ErrorCode::invalid_polar);
        REQUIRE(loaded.error().message.find("line 3") != std::string::npos);
    }
}

TEST_CASE("polar positive head-to-wind samples cannot create sailing support") {
    const PolarFixture fixture{
        "test_polar_positive_headwind.csv",
        "TWA/TWS,6,12\n"
        "0,5,7\n"
        "45,5,7\n"
        "90,7,9\n"};
    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    const auto slice = loaded.value().slice_at(9.0);
    REQUIRE_NEAR(slice.minimum_sailing_angle_degrees(), 45.0, 1e-12);
    REQUIRE(!slice.supports_sailing_angle(0.0));
    REQUIRE(!slice.supports_sailing_angle(30.0));
    REQUIRE(slice.supports_sailing_angle(45.0));
}

TEST_CASE("polar malformed headers cannot drop missing wind columns or skip numeric data") {
    for (const std::string header : {
             "TWA/TWS,,6,12\n", "TWA/TWS,missing,6,12\n",
             "TWA/TWS,nan,12\n", "TWA/TWS,6,12,\n", "TWA/TWS\n",
             "1e999,6,12\n", "bad,6,12\n"}) {
        const PolarFixture fixture{
            "test_polar_missing_header.csv", header + "45,5,7\n90,7,9\n150,6,8\n"};
        const auto loaded = sailroute::VesselPolar::load(fixture.path());
        REQUIRE(!loaded.has_value());
        REQUIRE(loaded.error().code == sailroute::ErrorCode::invalid_polar);
        REQUIRE(loaded.error().message.find("line 1") != std::string::npos);
    }
}

TEST_CASE("polar native parser accepts delimited quoted values and preserves isolated support") {
    const PolarFixture fixture{
        "test_polar_native_quoted.pol",
        "! native points include a measured zero on each side of a single sailing point\n"
        "\"6\";30;0;60;5;90;0;150;6;180;5\n"
        "\"12\";30;0;60;7;90;0;150;8;180;7\n"};
    const auto loaded = sailroute::VesselPolar::load(fixture.path());
    REQUIRE(loaded.has_value());
    const auto slice = loaded.value().slice_at(9.0);
    REQUIRE(slice.supports_sailing_angle(60.0));
    REQUIRE(!slice.supports_sailing_angle(59.99));
    REQUIRE(!slice.supports_sailing_angle(60.01));
    REQUIRE(!slice.supports_sailing_angle(120.0));
    REQUIRE(slice.supports_sailing_angle(150.0));
    REQUIRE(slice.supports_sailing_angle(170.0));
}
