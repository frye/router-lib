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
