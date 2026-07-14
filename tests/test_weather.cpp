#include "sailroute/time.hpp"
#include "sailroute/weather.hpp"

#include "test_support.hpp"

#include <eccodes.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

void require_codes(int status, const char* operation) {
    if (status != CODES_SUCCESS) {
        throw std::runtime_error(
            std::string{operation} + ": " + codes_get_error_message(status));
    }
}

class GribFixture {
public:
    explicit GribFixture(const char* sample_name = "regular_ll_sfc_grib2")
        : path_(
              std::filesystem::temp_directory_path() /
              ("sailroute-weather-" +
               std::to_string(
                   std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".grib")) {
        write_message(
            sample_name, "10u", 0, {12, 14, 16, 10, 12, 14, 8, 10, 12}, "w");
        write_message(
            sample_name, "10v", 0, {4, 4, 4, 4, 4, 4, 4, 4, 4}, "a");
        write_message(
            sample_name, "10u", 6, {18, 20, 22, 16, 18, 20, 14, 16, 18}, "a");
        write_message(
            sample_name, "10v", 6, {6, 6, 6, 6, 6, 6, 6, 6, 6}, "a");
    }

    ~GribFixture() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    void write_message(
        const char* sample_name,
        const char* short_name,
        long forecast_hour,
        const std::array<double, 9>& values,
        const char* mode) {
        codes_handle* handle =
            codes_grib_handle_new_from_samples(nullptr, sample_name);
        if (handle == nullptr) {
            throw std::runtime_error("unable to create ecCodes GRIB sample");
        }

        try {
            require_codes(codes_set_long(handle, "Ni", 3), "set Ni");
            require_codes(codes_set_long(handle, "Nj", 3), "set Nj");
            require_codes(
                codes_set_double(handle, "latitudeOfFirstGridPointInDegrees", 2.0),
                "set first latitude");
            require_codes(
                codes_set_double(handle, "longitudeOfFirstGridPointInDegrees", 0.0),
                "set first longitude");
            require_codes(
                codes_set_double(handle, "latitudeOfLastGridPointInDegrees", 0.0),
                "set last latitude");
            require_codes(
                codes_set_double(handle, "longitudeOfLastGridPointInDegrees", 2.0),
                "set last longitude");
            require_codes(
                codes_set_double(handle, "iDirectionIncrementInDegrees", 1.0),
                "set longitude increment");
            require_codes(
                codes_set_double(handle, "jDirectionIncrementInDegrees", 1.0),
                "set latitude increment");
            require_codes(codes_set_long(handle, "iScansNegatively", 0), "set i scan");
            require_codes(codes_set_long(handle, "jScansPositively", 0), "set j scan");
            require_codes(codes_set_long(handle, "dataDate", 20260714), "set date");
            require_codes(codes_set_long(handle, "dataTime", 0), "set time");
            const char* forecast_key =
                std::string_view{sample_name}.ends_with("grib1") ? "P1" : "forecastTime";
            require_codes(
                codes_set_long(handle, forecast_key, forecast_hour),
                "set forecast time");
            if (std::string_view{sample_name}.ends_with("grib1")) {
                require_codes(
                    codes_set_long(handle, "indicatorOfTypeOfLevel", 105),
                    "set GRIB1 level type");
            }
            std::size_t short_name_size = std::char_traits<char>::length(short_name);
            require_codes(
                codes_set_string(handle, "shortName", short_name, &short_name_size),
                "set wind component");
            require_codes(codes_set_long(handle, "level", 10), "set wind level");
            require_codes(
                codes_set_double_array(handle, "values", values.data(), values.size()),
                "set values");
            require_codes(
                codes_write_message(handle, path_.string().c_str(), mode),
                "write GRIB message");
        } catch (...) {
            codes_handle_delete(handle);
            throw;
        }
        codes_handle_delete(handle);
    }

    std::filesystem::path path_;
};

}  // namespace

TEST_CASE("GRIB2 weather loads paired winds and interpolates space and time") {
    const GribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    REQUIRE(weather.value().metadata().latitude_count == 3);
    REQUIRE(weather.value().metadata().longitude_count == 3);
    REQUIRE(!weather.value().metadata().global_longitude_coverage);

    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());
    const auto wind = weather.value().interpolate(
        {1.0, 1.0},
        start.value() + std::chrono::hours{3});
    REQUIRE(wind.has_value());
    REQUIRE_NEAR(wind.value().east_mps, 15.0, 1e-9);
    REQUIRE_NEAR(wind.value().north_mps, 5.0, 1e-9);
}

TEST_CASE("weather rejects coordinates and times outside forecast coverage") {
    const GribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());

    const auto coordinate_error =
        weather.value().interpolate({10.0, 1.0}, start.value());
    REQUIRE(!coordinate_error.has_value());
    REQUIRE(
        coordinate_error.error().code ==
        sailroute::ErrorCode::coordinate_outside_forecast);

    const auto time_error =
        weather.value().interpolate({1.0, 1.0}, start.value() - std::chrono::seconds{1});
    REQUIRE(!time_error.has_value());
    REQUIRE(
        time_error.error().code ==
        sailroute::ErrorCode::departure_outside_forecast);
}

TEST_CASE("bounded weather load retains an interpolation subgrid and enforces bounds") {
    const GribFixture fixture;
    const sailroute::GeographicBounds bounds{
        0.25,
        0.25,
        0.75,
        0.75};
    const auto weather = sailroute::WeatherDataset::load(fixture.path(), bounds);
    REQUIRE(weather.has_value());
    REQUIRE(weather.value().metadata().latitude_count == 2);
    REQUIRE(weather.value().metadata().longitude_count == 2);
    REQUIRE(!weather.value().metadata().global_longitude_coverage);

    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());
    const auto wind = weather.value().interpolate(
        {0.5, 0.5},
        start.value() + std::chrono::hours{3});
    REQUIRE(wind.has_value());
    REQUIRE_NEAR(wind.value().east_mps, 13.0, 1e-9);
    REQUIRE_NEAR(wind.value().north_mps, 5.0, 1e-9);

    const auto outside = weather.value().interpolate(
        {0.8, 0.5},
        start.value());
    REQUIRE(!outside.has_value());
    REQUIRE(
        outside.error().code ==
        sailroute::ErrorCode::coordinate_outside_forecast);
}

TEST_CASE("bounded weather load rejects invalid or uncovered bounds") {
    const GribFixture fixture;
    const auto invalid = sailroute::WeatherDataset::load(
        fixture.path(),
        sailroute::GeographicBounds{1.0, 0.0, 0.0, 1.0});
    REQUIRE(!invalid.has_value());
    REQUIRE(invalid.error().code == sailroute::ErrorCode::invalid_argument);

    const auto uncovered = sailroute::WeatherDataset::load(
        fixture.path(),
        sailroute::GeographicBounds{0.0, 0.0, 2.1, 1.0});
    REQUIRE(!uncovered.has_value());
    REQUIRE(
        uncovered.error().code ==
        sailroute::ErrorCode::coordinate_outside_forecast);
}

TEST_CASE("GRIB1 paired winds are supported") {
    const GribFixture fixture{"regular_ll_sfc_grib1"};
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }
    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());
    const auto wind = weather.value().interpolate({1.0, 1.0}, start.value());
    REQUIRE(wind.has_value());
    REQUIRE_NEAR(wind.value().east_mps, 12.0, 1e-9);
    REQUIRE_NEAR(wind.value().north_mps, 4.0, 1e-9);
}
