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

namespace {

/// Describes one regular_ll GRIB message (a single spatial tile) to write.
struct TileMessage {
    const char* short_name{};
    long forecast_hour{};
    long ni{};
    long nj{};
    double first_latitude{};
    double first_longitude{};
    double longitude_step{};
    double latitude_step{};
    std::vector<double> values;
    // GRIB decimal precision (number of preserved decimal places). Zero leaves
    // the ecCodes sample default; a positive value quantizes the tile, which
    // lets a test reproduce independently packed tiles whose shared edge
    // disagrees slightly.
    long decimal_precision{0};
};

/// Fills a north-to-south, west-to-east scan for value function f(lat, lon).
template <typename ValueFn>
std::vector<double> scan_values(
    long ni,
    long nj,
    double first_latitude,
    double first_longitude,
    double longitude_step,
    double latitude_step,
    ValueFn value_at) {
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(ni * nj));
    for (long row = 0; row < nj; ++row) {
        const double latitude =
            first_latitude - static_cast<double>(row) * latitude_step;
        for (long column = 0; column < ni; ++column) {
            const double longitude =
                first_longitude + static_cast<double>(column) * longitude_step;
            values.push_back(value_at(latitude, longitude));
        }
    }
    return values;
}

/// Writes a collection of tiles into one GRIB2 file so a single valid time can
/// carry several messages per wind component (spatial tiles/mosaics).
class MosaicFixture {
public:
    explicit MosaicFixture(const std::vector<TileMessage>& tiles)
        : path_(
              std::filesystem::temp_directory_path() /
              ("sailroute-mosaic-" +
               std::to_string(
                   std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".grib")) {
        for (std::size_t index = 0; index < tiles.size(); ++index) {
            write_tile(tiles[index], index == 0 ? "w" : "a");
        }
    }

    ~MosaicFixture() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    void write_tile(const TileMessage& tile, const char* mode) {
        codes_handle* handle =
            codes_grib_handle_new_from_samples(nullptr, "regular_ll_sfc_grib2");
        if (handle == nullptr) {
            throw std::runtime_error("unable to create ecCodes GRIB sample");
        }

        try {
            const double last_latitude =
                tile.first_latitude -
                static_cast<double>(tile.nj - 1) * tile.latitude_step;
            const double last_longitude =
                tile.first_longitude +
                static_cast<double>(tile.ni - 1) * tile.longitude_step;
            require_codes(codes_set_long(handle, "Ni", tile.ni), "set Ni");
            require_codes(codes_set_long(handle, "Nj", tile.nj), "set Nj");
            require_codes(
                codes_set_double(
                    handle, "latitudeOfFirstGridPointInDegrees",
                    tile.first_latitude),
                "set first latitude");
            require_codes(
                codes_set_double(
                    handle, "longitudeOfFirstGridPointInDegrees",
                    tile.first_longitude),
                "set first longitude");
            require_codes(
                codes_set_double(
                    handle, "latitudeOfLastGridPointInDegrees", last_latitude),
                "set last latitude");
            require_codes(
                codes_set_double(
                    handle, "longitudeOfLastGridPointInDegrees", last_longitude),
                "set last longitude");
            require_codes(
                codes_set_double(
                    handle, "iDirectionIncrementInDegrees", tile.longitude_step),
                "set longitude increment");
            require_codes(
                codes_set_double(
                    handle, "jDirectionIncrementInDegrees", tile.latitude_step),
                "set latitude increment");
            require_codes(codes_set_long(handle, "iScansNegatively", 0), "set i scan");
            require_codes(codes_set_long(handle, "jScansPositively", 0), "set j scan");
            require_codes(codes_set_long(handle, "dataDate", 20260714), "set date");
            require_codes(codes_set_long(handle, "dataTime", 0), "set time");
            require_codes(
                codes_set_long(handle, "forecastTime", tile.forecast_hour),
                "set forecast time");
            std::size_t short_name_size =
                std::char_traits<char>::length(tile.short_name);
            require_codes(
                codes_set_string(
                    handle, "shortName", tile.short_name, &short_name_size),
                "set wind component");
            require_codes(codes_set_long(handle, "level", 10), "set wind level");
            if (tile.decimal_precision != 0) {
                require_codes(
                    codes_set_long(
                        handle, "decimalPrecision", tile.decimal_precision),
                    "set decimal precision");
            }
            require_codes(
                codes_set_double_array(
                    handle, "values", tile.values.data(), tile.values.size()),
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

double sample_u(double latitude, double longitude) {
    return 100.0 * latitude + longitude;
}

double sample_v(double latitude, double longitude) {
    return 10.0 * latitude + longitude;
}

TileMessage make_tile(
    const char* short_name,
    long ni,
    long nj,
    double first_latitude,
    double first_longitude,
    double longitude_step,
    double latitude_step,
    double (*value_at)(double, double)) {
    TileMessage tile{
        short_name, 0, ni, nj, first_latitude, first_longitude,
        longitude_step, latitude_step, {}};
    tile.values = scan_values(
        ni, nj, first_latitude, first_longitude, longitude_step, latitude_step,
        value_at);
    return tile;
}

/// A value function whose shared-edge (latitude 1) samples have fractional
/// parts that quantize differently at coarse versus fine decimal precision,
/// producing a small but non-zero decoded disagreement.
double sample_seam(double latitude, double longitude) {
    return 3.0 * latitude + longitude + 0.033 * (longitude + 1.0);
}

TileMessage make_tile_prec(
    const char* short_name,
    long ni,
    long nj,
    double first_latitude,
    double first_longitude,
    double longitude_step,
    double latitude_step,
    double (*value_at)(double, double),
    long decimal_precision) {
    TileMessage tile = make_tile(
        short_name, ni, nj, first_latitude, first_longitude, longitude_step,
        latitude_step, value_at);
    tile.decimal_precision = decimal_precision;
    return tile;
}

}  // namespace

TEST_CASE("weather mosaics adjacent latitude tiles sharing an edge") {
    // Two U/V tiles per valid time: a northern band (lat 2..1) and a southern
    // band (lat 1..0) that share the lat=1 row.
    const MosaicFixture fixture({
        make_tile("10u", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_v),
        make_tile("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_v),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }
    REQUIRE(weather.value().metadata().latitude_count == 3);
    REQUIRE(weather.value().metadata().longitude_count == 3);

    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());
    const auto wind = weather.value().interpolate({0.5, 0.5}, start.value());
    REQUIRE(wind.has_value());
    REQUIRE_NEAR(wind.value().east_mps, 50.5, 1e-9);
    REQUIRE_NEAR(wind.value().north_mps, 5.5, 1e-9);
}

TEST_CASE("bounded load spans multiple latitude tiles no single tile covers") {
    // Requested bounds straddle the shared boundary between the two bands, so
    // the mosaic must be assembled before the crop is applied.
    const MosaicFixture fixture({
        make_tile("10u", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_v),
        make_tile("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_v),
    });

    const sailroute::GeographicBounds bounds{0.5, 0.25, 1.5, 0.75};
    const auto weather =
        sailroute::WeatherDataset::load(fixture.path(), bounds);
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }

    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());
    const auto wind = weather.value().interpolate({1.0, 0.5}, start.value());
    REQUIRE(wind.has_value());
    REQUIRE_NEAR(wind.value().east_mps, 100.5, 1e-9);
    REQUIRE_NEAR(wind.value().north_mps, 10.5, 1e-9);
}

TEST_CASE("weather mosaics adjacent longitude tiles sharing an edge") {
    // Western tile (lon 0..2) and eastern tile (lon 2..4) share the lon=2
    // column.
    const MosaicFixture fixture({
        make_tile("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_v),
        make_tile("10u", 3, 2, 1.0, 2.0, 1.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 2.0, 1.0, 1.0, sample_v),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }
    REQUIRE(weather.value().metadata().latitude_count == 2);
    REQUIRE(weather.value().metadata().longitude_count == 5);

    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());
    const auto wind = weather.value().interpolate({0.5, 3.5}, start.value());
    REQUIRE(wind.has_value());
    REQUIRE_NEAR(wind.value().east_mps, 53.5, 1e-9);
    REQUIRE_NEAR(wind.value().north_mps, 8.5, 1e-9);
}

TEST_CASE("weather mosaics tiles spanning the antimeridian") {
    // Tiles at lon 170..180 and 180..190 (i.e. crossing 180 degrees) share the
    // lon=180 column and assemble into a continuous 170..190 grid.
    const MosaicFixture fixture({
        make_tile("10u", 3, 2, 1.0, 170.0, 5.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 170.0, 5.0, 1.0, sample_v),
        make_tile("10u", 3, 2, 1.0, 180.0, 5.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 180.0, 5.0, 1.0, sample_v),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }
    REQUIRE(weather.value().metadata().longitude_count == 5);

    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());
    // Sample at 182.5 degrees east, which is also -177.5 degrees.
    const auto east_form = weather.value().interpolate({0.0, 182.5}, start.value());
    REQUIRE(east_form.has_value());
    REQUIRE_NEAR(east_form.value().east_mps, 182.5, 1e-9);
    const auto west_form = weather.value().interpolate({0.0, -177.5}, start.value());
    REQUIRE(west_form.has_value());
    REQUIRE_NEAR(west_form.value().east_mps, 182.5, 1e-9);
}

TEST_CASE("weather rejects tiles with mismatched resolution") {
    const MosaicFixture fixture({
        make_tile("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10u", 5, 2, 1.0, 2.0, 0.5, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_v),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(!weather.has_value());
    REQUIRE(
        weather.error().code == sailroute::ErrorCode::unsupported_grib);
}

TEST_CASE("weather rejects tiles that leave a gap") {
    // Northern tile is one column narrower, so the union rectangle has an
    // uncovered corner.
    const MosaicFixture fixture({
        make_tile("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10u", 2, 2, 2.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10v", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_v),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(!weather.has_value());
    REQUIRE(
        weather.error().code == sailroute::ErrorCode::incomplete_forecast);
}

TEST_CASE("weather mosaics tiles that disagree within GRIB packing error") {
    // Reproduces the real NOAA GFS assembly: adjacent tiles are packed
    // independently (here a coarse and a fine decimal precision), so their
    // shared lat=1 edge decodes to slightly different values. The overlap is a
    // legitimate quantization artifact, not a conflict, so the mosaic must be
    // accepted. Under a fixed 1e-6 epsilon this load failed with
    // "10 m V wind tiles disagree where they overlap".
    const MosaicFixture fixture({
        make_tile_prec("10u", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_seam, 1),
        make_tile_prec("10v", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_seam, 1),
        make_tile_prec("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_seam, 3),
        make_tile_prec("10v", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_seam, 3),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    if (!weather.has_value()) {
        throw std::runtime_error(weather.error().message);
    }
    REQUIRE(weather.value().metadata().latitude_count == 3);
    REQUIRE(weather.value().metadata().longitude_count == 3);
}

TEST_CASE("weather still rejects overlaps beyond GRIB packing error") {
    // Both tiles are packed at the same precision but the southern band's
    // shared row is offset far beyond any quantization error, so this remains a
    // genuine conflict even with the precision-aware tolerance.
    TileMessage northern =
        make_tile_prec("10u", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_seam, 3);
    TileMessage southern =
        make_tile_prec("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_seam, 3);
    for (double& value : southern.values) {
        value += 0.5;
    }
    const MosaicFixture fixture({
        northern,
        southern,
        make_tile_prec("10v", 3, 3, 2.0, 0.0, 1.0, 1.0, sample_seam, 3),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(!weather.has_value());
    REQUIRE(weather.error().code == sailroute::ErrorCode::grib_decode);
}

TEST_CASE("weather rejects tiles that conflict where they overlap") {
    // Both tiles cover the lat=1 row with different values.
    TileMessage northern =
        make_tile("10u", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_u);
    TileMessage southern =
        make_tile("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_u);
    for (double& value : southern.values) {
        value += 1.0;
    }
    const MosaicFixture fixture({
        northern,
        southern,
        make_tile("10v", 3, 3, 2.0, 0.0, 1.0, 1.0, sample_v),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(!weather.has_value());
    REQUIRE(weather.error().code == sailroute::ErrorCode::grib_decode);
}

TEST_CASE("weather rejects tiles with incomplete U/V coverage") {
    // Two U bands but no V message at the valid time.
    const MosaicFixture fixture({
        make_tile("10u", 3, 2, 2.0, 0.0, 1.0, 1.0, sample_u),
        make_tile("10u", 3, 2, 1.0, 0.0, 1.0, 1.0, sample_u),
    });

    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(!weather.has_value());
    REQUIRE(
        weather.error().code == sailroute::ErrorCode::incomplete_forecast);
}

TEST_CASE("time sampler reproduces interpolate exactly") {
    const GribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());

    for (const int minutes : {0, 37, 90, 180, 271, 360}) {
        const auto when = start.value() + std::chrono::minutes{minutes};
        const auto sampler = weather.value().sampler_at(when);
        REQUIRE(sampler.has_value());
        for (const double latitude : {0.0, 0.25, 1.0, 1.75, 2.0}) {
            for (const double longitude : {0.0, 0.5, 1.0, 1.5, 2.0}) {
                const sailroute::Coordinate coordinate{latitude, longitude};
                const auto direct = weather.value().interpolate(coordinate, when);
                const auto sampled = sampler.value().sample(coordinate);
                REQUIRE(direct.has_value() == sampled.has_value());
                if (direct.has_value()) {
                    REQUIRE(direct.value().east_mps == sampled.value().east_mps);
                    REQUIRE(direct.value().north_mps == sampled.value().north_mps);
                }
            }
        }
    }
}

TEST_CASE("time sampler matches interpolate for out-of-range times") {
    const GribFixture fixture;
    const auto weather = sailroute::WeatherDataset::load(fixture.path());
    REQUIRE(weather.has_value());
    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());

    // Errors are user-visible: the router reports the last interpolation failure
    // when a step yields no candidates, so the sampler has to fail in exactly the
    // same order and with the same code as interpolate.
    for (const auto when :
         {start.value() - std::chrono::seconds{1},
          start.value() + std::chrono::hours{24}}) {
        const auto sampler = weather.value().sampler_at(when);
        REQUIRE(sampler.has_value());
        for (const sailroute::Coordinate coordinate :
             {sailroute::Coordinate{1.0, 1.0}, sailroute::Coordinate{10.0, 1.0}}) {
            const auto direct = weather.value().interpolate(coordinate, when);
            const auto sampled = sampler.value().sample(coordinate);
            REQUIRE(!direct.has_value());
            REQUIRE(!sampled.has_value());
            REQUIRE(direct.error().code == sampled.error().code);
            REQUIRE(direct.error().message == sampled.error().message);
        }
    }
}

TEST_CASE("bounded time sampler reports the coordinate problem first") {
    const GribFixture fixture;
    const sailroute::GeographicBounds bounds{0.25, 0.25, 0.75, 0.75};
    const auto weather = sailroute::WeatherDataset::load(fixture.path(), bounds);
    REQUIRE(weather.has_value());
    const auto start = sailroute::parse_utc_time("2026-07-14T00:00:00Z");
    REQUIRE(start.has_value());

    // Both the coordinate and the time are invalid here. interpolate checks
    // bounds first, and the sampler must defer its time error to match.
    const auto when = start.value() - std::chrono::seconds{1};
    const auto sampler = weather.value().sampler_at(when);
    REQUIRE(sampler.has_value());

    const auto outside = sampler.value().sample({0.8, 0.5});
    REQUIRE(!outside.has_value());
    REQUIRE(
        outside.error().code == sailroute::ErrorCode::coordinate_outside_forecast);

    const auto inside = sampler.value().sample({0.5, 0.5});
    REQUIRE(!inside.has_value());
    REQUIRE(
        inside.error().code == sailroute::ErrorCode::departure_outside_forecast);
}
