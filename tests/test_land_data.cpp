#include "sailroute/land_data.hpp"
#include "sailroute/router.hpp"

#include "../src/environment/spherical.hpp"
#include "../src/routing/geodesy.hpp"
#include "test_support.hpp"
#include "grib_fixture.hpp"

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace {

using Bytes = std::vector<unsigned char>;
using sailroute::Coordinate;
using sailroute::ErrorCode;
using sailroute::GeographicBounds;
using sailroute::LandDataOptions;
using sailroute::load_gshhg_landmask;

void word(Bytes& bytes, std::int32_t signed_value) {
    const std::uint32_t value = std::bit_cast<std::uint32_t>(signed_value);
    for (unsigned int shift : {24U, 16U, 8U, 0U}) {
        bytes.push_back(static_cast<unsigned char>((value >> shift) & 255U));
    }
}

void replace_word(Bytes& bytes, std::size_t index, std::int32_t value) {
    Bytes encoded;
    word(encoded, value);
    std::copy(encoded.begin(), encoded.end(), bytes.begin() +
        static_cast<std::ptrdiff_t>(index * 4U));
}

std::int32_t microdegrees(double value) {
    return static_cast<std::int32_t>(std::llround(value * 1000000.0));
}

Bytes record(
    std::int32_t id,
    unsigned int level,
    std::int32_t parent,
    std::vector<Coordinate> points,
    unsigned int version = 15U) {
    REQUIRE(points.size() >= 3U);
    double west = points.front().longitude_degrees;
    double east = west;
    double south = points.front().latitude_degrees;
    double north = south;
    for (const Coordinate point : points) {
        west = std::min(west, point.longitude_degrees);
        east = std::max(east, point.longitude_degrees);
        south = std::min(south, point.latitude_degrees);
        north = std::max(north, point.latitude_degrees);
    }
    points.push_back(points.front());
    Bytes bytes;
    word(bytes, id);
    word(bytes, static_cast<std::int32_t>(points.size()));
    word(bytes, static_cast<std::int32_t>(level | (version << 8U) |
        (west < 0.0 && east > 0.0 ? (1U << 16U) : 0U)));
    word(bytes, microdegrees(west));
    word(bytes, microdegrees(east));
    word(bytes, microdegrees(south));
    word(bytes, microdegrees(north));
    word(bytes, 100);
    word(bytes, 100);
    word(bytes, parent);
    word(bytes, -1);
    for (const Coordinate point : points) {
        double longitude = std::fmod(point.longitude_degrees, 360.0);
        if (longitude < 0.0) {
            longitude += 360.0;
        }
        word(bytes, microdegrees(longitude));
        word(bytes, microdegrees(point.latitude_degrees));
    }
    return bytes;
}

Bytes rectangle(
    std::int32_t id,
    unsigned int level,
    std::int32_t parent,
    GeographicBounds bounds) {
    if (bounds.east_longitude_degrees < bounds.west_longitude_degrees) {
        bounds.east_longitude_degrees += 360.0;
    }
    return record(id, level, parent, {
        {bounds.south_latitude_degrees, bounds.west_longitude_degrees},
        {bounds.south_latitude_degrees, bounds.east_longitude_degrees},
        {bounds.north_latitude_degrees, bounds.east_longitude_degrees},
        {bounds.north_latitude_degrees, bounds.west_longitude_degrees}});
}

Bytes together(std::initializer_list<Bytes> records) {
    Bytes bytes;
    for (const auto& item : records) {
        bytes.insert(bytes.end(), item.begin(), item.end());
    }
    return bytes;
}

class Fixture {
public:
    explicit Fixture(const Bytes& bytes) {
        static std::atomic<unsigned int> sequence{};
        // Generated fixtures stay in the test working directory, not a
        // machine-global temporary directory or a downloaded dataset.
        path_ = "sailroute-gshhg-synthetic-" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
            "-" + std::to_string(sequence.fetch_add(1U)) + ".b";
        std::ofstream stream(path_, std::ios::binary);
        stream.write(reinterpret_cast<const char*>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        REQUIRE(stream.good());
    }

    ~Fixture() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    Fixture(const Fixture&) = delete;
    Fixture& operator=(const Fixture&) = delete;

    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

LandDataOptions options(GeographicBounds bounds = {-2.0, -2.0, 2.0, 2.0}) {
    LandDataOptions result;
    result.bounds = bounds;
    result.resolution_nautical_miles = 3.0;
    return result;
}

double sample(const sailroute::SignedDistanceLandmask& mask, Coordinate point) {
    const auto value = mask.signed_distance_nautical_miles(point);
    REQUIRE(value.has_value());
    return value.value;
}

}  // namespace

TEST_CASE("GSHHG CLI loads a local island and returns a land-aware arrived route") {
    const Fixture island(rectangle(1, 1U, -1, {0.94, 0.68, 1.06, 0.82}));
    const sailroute::test::ConstantWindGribFixture weather;
    const auto quote = [](const std::filesystem::path& path) {
#ifdef _WIN32
        return "\"" + path.string() + "\"";
#else
        std::string result = "'";
        for (const char character : path.string()) {
            result.append(character == '\'' ? "'\\''" : std::string(1, character));
        }
        return result + "'";
#endif
    };
    const std::string command = quote(SAILROUTE_CLI_PATH) +
        " --grib " + quote(weather.path()) + " --demo-polar" +
        " --start 1,0.5 --destination 1,1 --departure 2026-07-14T12:00:00Z" +
        " --bounds 0,0,2,2 --land " + quote(island.path()) +
        " --land-resolution-nm 1 --land-clearance-nm 0.25";
#ifdef _WIN32
    FILE* pipe = _popen(command.c_str(), "r");
#else
    FILE* pipe = popen(command.c_str(), "r");
#endif
    REQUIRE(pipe != nullptr);
    std::string output;
    char buffer[4096];
    while (std::fgets(buffer, sizeof(buffer), pipe)) output.append(buffer);
#ifdef _WIN32
    const int status = _pclose(pipe);
#else
    const int status = pclose(pipe);
#endif
    REQUIRE(status == 0);
    REQUIRE(output.find("\"completion\":\"destination_reached\"") != std::string::npos);
    REQUIRE(output.find("\"landAvoidance\":true") != std::string::npos);
    REQUIRE(output.find("\"name\":\"gshhg\"") != std::string::npos);
    REQUIRE(output.find("land avoidance disabled") == std::string::npos);
}

TEST_CASE("GSHHG big-endian local island preserves source and bounded coverage") {
    const Fixture file(rectangle(0, 1U, -1, {-0.5, -0.5, 0.5, 0.5}));
    const auto loaded = load_gshhg_landmask(file.path(), options());
    REQUIRE(loaded.has_value());
    const auto& mask = loaded.value();
    REQUIRE(sample(mask, {0.0, 0.0}) < 0.0);
    REQUIRE(sample(mask, {0.0, 1.5}) > 0.0);
    REQUIRE_NEAR(sample(mask, {0.0, 1.5}), 60.04046, 0.1);
    REQUIRE(mask.metadata().provider.name == "gshhg");
    REQUIRE(mask.metadata().provider.revision == "native-15");
    REQUIRE(mask.metadata().provider.source.find(file.path().string()) != std::string::npos);
    REQUIRE(mask.metadata().provider.source.find("Wessel and Smith") != std::string::npos);
    REQUIRE(mask.metadata().interpolation_error_nautical_miles > 0.0);
    REQUIRE(mask.metadata().resolution_nautical_miles == 3.0);
    REQUIRE(!mask.certify_segment({0.0, -1.5}, {0.0, 1.5}, 0.0, 12U).clear);
    REQUIRE(mask.certify_segment({-1.5, -1.5}, {1.5, -1.5}, 0.0, 12U).clear);
    REQUIRE(!mask.certify_segment({-0.4, 0.8}, {0.4, 0.8}, 20.0, 12U).clear);
    REQUIRE(mask.signed_distance_nautical_miles({3.0, 0.0}).status ==
        sailroute::EnvironmentSampleStatus::outside_coverage);
    const auto outside = mask.certify_segment({0.0, 1.5}, {0.0, 3.0}, 0.0, 12U);
    REQUIRE(!outside.clear);
    REQUIRE(outside.status == sailroute::EnvironmentSampleStatus::outside_coverage);
}

TEST_CASE("GSHHG levels preserve lakes islands and ponds in arbitrary record order") {
    const Fixture file(together({
        rectangle(3, 4U, 2, {-0.1, -0.1, 0.1, 0.1}),
        rectangle(2, 3U, 1, {-0.4, -0.4, 0.4, 0.4}),
        rectangle(1, 2U, 0, {-1.0, -1.0, 1.0, 1.0}),
        rectangle(0, 1U, -1, {-2.0, -2.0, 2.0, 2.0})}));
    const auto loaded = load_gshhg_landmask(file.path(), options({-3.0, -3.0, 3.0, 3.0}));
    REQUIRE(loaded.has_value());
    REQUIRE(sample(loaded.value(), {0.0, 1.5}) < 0.0);
    REQUIRE(sample(loaded.value(), {0.0, 0.7}) > 0.0);
    REQUIRE(sample(loaded.value(), {0.0, 0.25}) < 0.0);
    REQUIRE(sample(loaded.value(), {0.0, 0.0}) > 0.0);
    REQUIRE(sample(loaded.value(), {0.0, 2.5}) > 0.0);
}

TEST_CASE("GSHHG antimeridian rings and grid are continuous across both longitude spellings") {
    const Fixture file(rectangle(0, 1U, -1, {-0.5, 179.5, 0.5, -179.5}));
    const auto loaded = load_gshhg_landmask(file.path(), options({-2.0, 178.0, 2.0, -178.0}));
    REQUIRE(loaded.has_value());
    REQUIRE(sample(loaded.value(), {0.0, 180.0}) < 0.0);
    REQUIRE_NEAR(
        sample(loaded.value(), {0.0, 180.0}),
        sample(loaded.value(), {0.0, -180.0}), 1.0e-8);
    REQUIRE(sample(loaded.value(), {0.0, 179.0}) > 0.0);
    REQUIRE(sample(loaded.value(), {0.0, -179.0}) > 0.0);
    REQUIRE(!loaded.value().certify_segment({0.0, 179.0}, {0.0, -179.0}, 0.0, 12U).clear);
    REQUIRE(loaded.value().certify_segment({1.5, 179.0}, {1.5, -179.0}, 0.0, 12U).clear);
    REQUIRE(loaded.value().signed_distance_nautical_miles({0.0, 177.0}).status ==
        sailroute::EnvironmentSampleStatus::outside_coverage);
}

TEST_CASE("GSHHG Greenwich wrapping and reversed winding do not invert land") {
    std::vector<Coordinate> vertices{
        {-0.5, -0.5}, {-0.5, 0.5}, {0.5, 0.5}, {0.5, -0.5}};
    const Fixture forward(record(0, 1U, -1, vertices));
    std::reverse(vertices.begin(), vertices.end());
    const Fixture reverse(record(0, 1U, -1, vertices));
    const auto first = load_gshhg_landmask(forward.path(), options());
    const auto second = load_gshhg_landmask(reverse.path(), options());
    REQUIRE(first.has_value());
    REQUIRE(second.has_value());
    for (const Coordinate point : std::vector<Coordinate>{{0.0, 0.0}, {0.0, -1.0}, {1.0, 0.0}}) {
        REQUIRE_NEAR(sample(first.value(), point), sample(second.value(), point), 1.0e-8);
    }
}

TEST_CASE("GSHHG tiny island between all-water grid nodes cannot be certified away") {
    const Fixture file(rectangle(0, 1U, -1, {0.35, 0.35, 0.45, 0.45}));
    auto coarse = options({-1.0, -1.0, 1.0, 1.0});
    coarse.resolution_nautical_miles = 120.0;
    coarse.distance_cap_nautical_miles = 300.0;
    const auto loaded = load_gshhg_landmask(file.path(), coarse);
    REQUIRE(loaded.has_value());
    const auto& mask = loaded.value();
    const auto& grid = mask.grid();
    for (std::size_t row = 0U; row < grid.latitude_count; ++row) {
        for (std::size_t column = 0U; column < grid.longitude_count; ++column) {
            REQUIRE(sample(mask, {
                grid.south_latitude_degrees + static_cast<double>(row) * grid.latitude_step_degrees,
                grid.west_longitude_degrees + static_cast<double>(column) * grid.longitude_step_degrees}) > 0.0);
        }
    }
    const Coordinate center{0.4, 0.4};
    const double interpolated = sample(mask, center);
    REQUIRE(interpolated > 0.0);
    const double true_distance = -sailroute::detail::earth_radius_nautical_miles *
        sailroute::environment_detail::distance_to_arc(
            sailroute::environment_detail::to_unit_vector(center),
            sailroute::environment_detail::to_unit_vector({0.35, 0.35}),
            sailroute::environment_detail::to_unit_vector({0.45, 0.35}));
    REQUIRE(std::abs(interpolated - true_distance) <=
        mask.metadata().interpolation_error_nautical_miles);
    REQUIRE(!mask.certify_segment({0.4, -0.5}, {0.4, 0.9}, 0.0, 12U).clear);
    REQUIRE(!mask.certify_segment(center, center, 0.0, 12U).clear);
    coarse.resolution_nautical_miles = 1.0;
    const auto fine = load_gshhg_landmask(file.path(), coarse);
    REQUIRE(fine.has_value());
    REQUIRE(sample(fine.value(), center) < 0.0);
}

TEST_CASE("GSHHG validates every byte through the final record without partial fallback") {
    const Bytes valid = rectangle(0, 1U, -1, {-0.5, -0.5, 0.5, 0.5});
    for (std::size_t length = 0U; length < valid.size(); ++length) {
        const Fixture file(Bytes(valid.begin(), valid.begin() + static_cast<std::ptrdiff_t>(length)));
        const auto loaded = load_gshhg_landmask(file.path(), options());
        REQUIRE(!loaded.has_value());
        REQUIRE(loaded.error().code == ErrorCode::invalid_environment);
    }
    Bytes with_tail = valid;
    with_tail.push_back(0U);
    const Fixture trailing(with_tail);
    REQUIRE(!load_gshhg_landmask(trailing.path(), options()).has_value());
    const auto missing = load_gshhg_landmask("sailroute-no-such-gshhg-file.b", options());
    REQUIRE(!missing.has_value());
    REQUIRE(missing.error().code == ErrorCode::file_io);
}

TEST_CASE("GSHHG malformed headers coordinates and encodings are explicit errors") {
    const Bytes valid = rectangle(0, 1U, -1, {-0.5, -0.5, 0.5, 0.5});
    const std::vector<std::pair<std::size_t, std::int32_t>> corruptions{
        {0U, -1}, {1U, -1}, {1U, 2}, {1U, 2000000000},
        {2U, 1}, {2U, (17 << 8) | 1}, {2U, (15 << 8) | 7},
        {2U, (15 << 8) | 1 | (1 << 22)},
        {3U, 1000000}, {4U, 361000000}, {5U, -91000000},
        {6U, -1000000}, {7U, -1}, {8U, -1}, {9U, 0}, {10U, -2},
        {11U, -1}, {11U, 360000001}, {12U, 90000001}, {12U, 1000000}};
    for (const auto& [index, value] : corruptions) {
        Bytes bytes = valid;
        replace_word(bytes, index, value);
        const Fixture file(bytes);
        REQUIRE(!load_gshhg_landmask(file.path(), options()).has_value());
    }
    Bytes little_endian = valid;
    for (std::size_t index = 0U; index < little_endian.size(); index += 4U) {
        std::reverse(little_endian.begin() + static_cast<std::ptrdiff_t>(index),
            little_endian.begin() + static_cast<std::ptrdiff_t>(index + 4U));
    }
    const Fixture file(little_endian);
    REQUIRE(!load_gshhg_landmask(file.path(), options()).has_value());
}

TEST_CASE("GSHHG source identifiers and complete parent chains are validated globally") {
    const Bytes outer = rectangle(0, 1U, -1, {-1.0, -1.0, 1.0, 1.0});
    const std::vector<Bytes> invalid{
        together({outer, outer}),
        rectangle(1, 2U, 9, {-0.5, -0.5, 0.5, 0.5}),
        together({outer, rectangle(1, 3U, 0, {-0.5, -0.5, 0.5, 0.5})}),
        together({outer, rectangle(1, 2U, 0, {1.4, 1.4, 1.8, 1.8})}),
        together({outer, rectangle(1, 2U, 99, {40.0, 40.0, 41.0, 41.0})}),
        together({outer, record(2, 1U, -1, {{40.0, 40.0}, {40.0, 41.0}, {41.0, 40.0}}, 14U)})};
    for (const auto& bytes : invalid) {
        const Fixture file(bytes);
        REQUIRE(!load_gshhg_landmask(file.path(), options()).has_value());
    }
}

TEST_CASE("GSHHG degenerate rings fail rather than inventing water") {
    const std::vector<std::vector<Coordinate>> invalid{
        {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}},
        {{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}},
        {{0.0, 0.0}, {0.0, 180.0}, {1.0, 0.0}}};
    for (const auto& vertices : invalid) {
        const Fixture file(record(0, 1U, -1, vertices));
        REQUIRE(!load_gshhg_landmask(file.path(), options()).has_value());
    }
}

TEST_CASE("GSHHG option and resource bounds are checked before allocation") {
    const Fixture file(rectangle(0, 1U, -1, {-0.5, -0.5, 0.5, 0.5}));
    const std::vector<GeographicBounds> bad_bounds{
        {2.0, -2.0, -2.0, 2.0}, {-2.0, 0.0, 2.0, 0.0},
        {-86.0, 0.0, -84.0, 2.0}, {84.0, 0.0, 86.0, 2.0},
        {-2.0, -181.0, 2.0, 2.0}, {-2.0, 0.0, 2.0, 181.0},
        {-70.0, 0.0, 70.0, 2.0}, {-2.0, -80.0, 2.0, 80.0},
        {-2.0, -180.0, 2.0, 180.0},
        {std::numeric_limits<double>::quiet_NaN(), 0.0, 2.0, 2.0}};
    for (const auto& bounds : bad_bounds) {
        const auto loaded = load_gshhg_landmask(file.path(), options(bounds));
        REQUIRE(!loaded.has_value());
        REQUIRE(loaded.error().code == ErrorCode::invalid_argument);
    }
    for (const double resolution : {0.0, -1.0, 0.001, 121.0,
             std::numeric_limits<double>::infinity()}) {
        auto settings = options();
        settings.resolution_nautical_miles = resolution;
        REQUIRE(!load_gshhg_landmask(file.path(), settings).has_value());
    }
    for (const double cap : {0.0, 601.0, std::numeric_limits<double>::quiet_NaN()}) {
        auto settings = options();
        settings.distance_cap_nautical_miles = cap;
        REQUIRE(!load_gshhg_landmask(file.path(), settings).has_value());
    }
    for (int budget = 0; budget < 3; ++budget) {
        auto settings = options();
        if (budget == 0) settings.maximum_grid_nodes = 4U;
        if (budget == 1) settings.maximum_source_points = 3U;
        if (budget == 2) settings.maximum_geometry_tests = 1U;
        const auto loaded = load_gshhg_landmask(file.path(), settings);
        REQUIRE(!loaded.has_value());
        REQUIRE(loaded.error().code == ErrorCode::resource_limit);
    }
    auto settings = options();
    settings.maximum_grid_nodes = 250001U;
    REQUIRE(!load_gshhg_landmask(file.path(), settings).has_value());
    settings = options();
    settings.maximum_source_points = 10000001U;
    REQUIRE(!load_gshhg_landmask(file.path(), settings).has_value());
    settings = options();
    settings.maximum_geometry_tests = 100000001U;
    REQUIRE(!load_gshhg_landmask(file.path(), settings).has_value());
    settings = options();
    settings.distance_cap_nautical_miles = 1.0;
    REQUIRE(!load_gshhg_landmask(file.path(), settings).has_value());
    settings = options({83.0, 0.0, 84.0, 1.0});
    settings.distance_cap_nautical_miles = 600.0;
    REQUIRE(!load_gshhg_landmask(file.path(), settings).has_value());
}

TEST_CASE("GSHHG polar and Antarctica representations are rejected only when relevant") {
    for (const unsigned int level : {5U, 6U}) {
        const Fixture file(together({
            rectangle(0, 1U, -1, {-0.5, -0.5, 0.5, 0.5}),
            rectangle(1, level, -1, {-80.0, 0.0, -70.0, 10.0})}));
        REQUIRE(load_gshhg_landmask(file.path(), options()).has_value());
        const auto polar = load_gshhg_landmask(file.path(), options({-76.0, 1.0, -72.0, 4.0}));
        REQUIRE(!polar.has_value());
        REQUIRE(polar.error().message.find("polar") != std::string::npos);
    }
    const Fixture file(record(0, 1U, -1, {{80.0, 0.0}, {80.0, 120.0}, {80.0, 240.0}}));
    REQUIRE(load_gshhg_landmask(file.path(), options()).has_value());
    const auto north = load_gshhg_landmask(file.path(), options({82.0, 1.0, 84.0, 3.0}));
    REQUIRE(!north.has_value());
    REQUIRE(north.error().message.find("polar") != std::string::npos);
}

TEST_CASE("GSHHG large nonpolar enclosing land is retained even with no nearby shoreline") {
    const Fixture file(record(0, 1U, -1, {
        {-10.0, -100.0}, {-10.0, 0.0}, {-10.0, 100.0},
        {10.0, 100.0}, {10.0, 0.0}, {10.0, -100.0}}));
    const auto loaded = load_gshhg_landmask(file.path(), options({-1.0, -1.0, 1.0, 1.0}));
    REQUIRE(loaded.has_value());
    REQUIRE_NEAR(sample(loaded.value(), {0.0, 0.0}), -120.0, 1.0e-8);
}

TEST_CASE("GSHHG source selection includes great-circle bulges beyond native latitude boxes") {
    const Fixture file(rectangle(0, 1U, -1, {50.0, -20.0, 60.0, 20.0}));
    auto settings = options({61.0, -0.1, 62.0, 0.1});
    settings.resolution_nautical_miles = 0.1;
    settings.distance_cap_nautical_miles = 1.0;
    const auto loaded = load_gshhg_landmask(file.path(), settings);
    REQUIRE(loaded.has_value());
    REQUIRE(sample(loaded.value(), {61.2, 0.0}) < 0.0);
    REQUIRE(sample(loaded.value(), {62.0, 0.0}) > 0.0);
}

TEST_CASE("GSHHG far offshore distances saturate without extrapolating regional coverage") {
    const Fixture file(rectangle(0, 1U, -1, {40.0, 40.0, 41.0, 41.0}));
    const auto loaded = load_gshhg_landmask(file.path(), options());
    REQUIRE(loaded.has_value());
    REQUIRE_NEAR(sample(loaded.value(), {0.0, 0.0}), 120.0, 1.0e-8);
    REQUIRE(loaded.value().certify_segment({0.0, 0.0}, {0.0, 1.0}, 5.0, 12U).clear);
    REQUIRE(!loaded.value().certify_segment({0.0, 0.0}, {0.0, 3.0}, 0.0, 12U).clear);
}

TEST_CASE("GSHHG channel clearance uses both shorelines and the interpolation allowance") {
    const Fixture file(together({
        rectangle(0, 1U, -1, {-1.0, -1.0, 1.0, -0.2}),
        rectangle(1, 1U, -1, {-1.0, 0.2, 1.0, 1.0})}));
    auto settings = options({-1.5, -1.5, 1.5, 1.5});
    settings.resolution_nautical_miles = 1.0;
    const auto loaded = load_gshhg_landmask(file.path(), settings);
    REQUIRE(loaded.has_value());
    REQUIRE(loaded.value().certify_segment({-0.8, 0.0}, {0.8, 0.0}, 5.0, 14U).clear);
    REQUIRE(!loaded.value().certify_segment({-0.8, 0.0}, {0.8, 0.0}, 14.0, 14U).clear);
}

TEST_CASE("GSHHG distance halo retains nearby land entirely outside requested grid bounds") {
    const Fixture file(rectangle(0, 1U, -1, {-0.5, 1.1, 0.5, 1.3}));
    auto settings = options({-1.0, -1.0, 1.0, 1.0});
    settings.resolution_nautical_miles = 1.0;
    const auto loaded = load_gshhg_landmask(file.path(), settings);
    REQUIRE(loaded.has_value());
    REQUIRE_NEAR(sample(loaded.value(), {0.0, 0.95}), 9.006, 0.05);
    REQUIRE(!loaded.value().certify_segment({-0.1, 0.95}, {0.1, 0.95}, 10.0, 12U).clear);
}
