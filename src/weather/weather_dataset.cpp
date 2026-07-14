#include "sailroute/weather.hpp"

#include <eccodes.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace sailroute {
namespace {

constexpr double kLongitudePeriod = 360.0;
constexpr double kKnotsToMetresPerSecond = 0.5144444444444445;

enum class WindComponent {
    east,
    north,
};

struct Grid {
    std::size_t longitude_count{};
    std::size_t latitude_count{};
    double west_longitude_degrees{};
    double south_latitude_degrees{};
    double longitude_step_degrees{};
    double latitude_step_degrees{};
    bool global_longitude_coverage{};
    bool duplicate_longitude_endpoint{};
};

struct DecodedField {
    Grid grid;
    std::vector<double> values;
};

struct PendingTimeSlice {
    std::optional<DecodedField> east;
    std::optional<DecodedField> north;
};

[[nodiscard]] std::optional<Error> validate_bounds(
    const GeographicBounds& bounds) {
    if (!std::isfinite(bounds.south_latitude_degrees) ||
        !std::isfinite(bounds.west_longitude_degrees) ||
        !std::isfinite(bounds.north_latitude_degrees) ||
        !std::isfinite(bounds.east_longitude_degrees) ||
        bounds.south_latitude_degrees < -90.0 ||
        bounds.north_latitude_degrees > 90.0 ||
        bounds.south_latitude_degrees > bounds.north_latitude_degrees ||
        bounds.west_longitude_degrees < -180.0 ||
        bounds.west_longitude_degrees > 180.0 ||
        bounds.east_longitude_degrees < -180.0 ||
        bounds.east_longitude_degrees > 180.0) {
        return Error{
            ErrorCode::invalid_argument,
            "weather bounds must contain finite south/north latitudes in [-90, 90] "
            "with south <= north and west/east longitudes in [-180, 180]"};
    }
    return std::nullopt;
}

struct FileCloser {
    void operator()(std::FILE* file) const noexcept {
        if (file != nullptr) {
            std::fclose(file);
        }
    }
};

struct HandleDeleter {
    void operator()(codes_handle* handle) const noexcept {
        if (handle != nullptr) {
            codes_handle_delete(handle);
        }
    }
};

using FilePtr = std::unique_ptr<std::FILE, FileCloser>;
using HandlePtr = std::unique_ptr<codes_handle, HandleDeleter>;

[[nodiscard]] Error codes_error(
    ErrorCode code,
    std::string_view operation,
    int status) {
    std::string message{operation};
    message += ": ";
    message += codes_get_error_message(status);
    return Error{code, std::move(message)};
}

[[nodiscard]] Result<long> required_long(codes_handle* handle, const char* key) {
    long value = 0;
    const int status = codes_get_long(handle, key, &value);
    if (status != CODES_SUCCESS) {
        return codes_error(
            ErrorCode::grib_decode,
            std::string{"cannot read GRIB key "} + key,
            status);
    }
    return value;
}

[[nodiscard]] Result<double> required_double(codes_handle* handle, const char* key) {
    double value = 0.0;
    const int status = codes_get_double(handle, key, &value);
    if (status != CODES_SUCCESS) {
        return codes_error(
            ErrorCode::grib_decode,
            std::string{"cannot read GRIB key "} + key,
            status);
    }
    return value;
}

[[nodiscard]] Result<std::string> required_string(codes_handle* handle, const char* key) {
    std::array<char, 256> buffer{};
    std::size_t size = buffer.size();
    const int status = codes_get_string(handle, key, buffer.data(), &size);
    if (status != CODES_SUCCESS) {
        return codes_error(
            ErrorCode::grib_decode,
            std::string{"cannot read GRIB key "} + key,
            status);
    }
    return std::string{buffer.data()};
}

[[nodiscard]] bool optional_long(codes_handle* handle, const char* key, long& value) {
    return codes_get_long(handle, key, &value) == CODES_SUCCESS;
}

[[nodiscard]] bool optional_double(codes_handle* handle, const char* key, double& value) {
    return codes_get_double(handle, key, &value) == CODES_SUCCESS;
}

[[nodiscard]] bool optional_string(
    codes_handle* handle,
    const char* key,
    std::string& value) {
    std::array<char, 256> buffer{};
    std::size_t size = buffer.size();
    if (codes_get_string(handle, key, buffer.data(), &size) != CODES_SUCCESS) {
        return false;
    }
    value = buffer.data();
    return true;
}

[[nodiscard]] double normalize_longitude(double longitude_degrees) noexcept {
    double normalized = std::fmod(longitude_degrees, kLongitudePeriod);
    if (normalized < 0.0) {
        normalized += kLongitudePeriod;
    }
    if (normalized >= kLongitudePeriod) {
        normalized -= kLongitudePeriod;
    }
    return normalized;
}

[[nodiscard]] double circular_longitude_difference(double lhs, double rhs) noexcept {
    double difference = std::abs(normalize_longitude(lhs) - normalize_longitude(rhs));
    if (difference > kLongitudePeriod / 2.0) {
        difference = kLongitudePeriod - difference;
    }
    return difference;
}

[[nodiscard]] bool nearly_equal(double lhs, double rhs, double scale = 1.0) noexcept {
    const double tolerance = 1.0e-8 * std::max({1.0, std::abs(scale), std::abs(lhs), std::abs(rhs)});
    return std::abs(lhs - rhs) <= tolerance;
}

[[nodiscard]] std::string canonical_unit(std::string_view unit) {
    std::string result;
    result.reserve(unit.size());
    for (const char character : unit) {
        const unsigned char byte = static_cast<unsigned char>(character);
        if (character != ' ' && character != '\t' && character != '.' &&
            character != '_') {
            if (byte >= static_cast<unsigned char>('A') &&
                byte <= static_cast<unsigned char>('Z')) {
                result.push_back(static_cast<char>(
                    byte - static_cast<unsigned char>('A') +
                    static_cast<unsigned char>('a')));
            } else {
                result.push_back(character);
            }
        }
    }
    return result;
}

[[nodiscard]] Result<double> unit_conversion_factor(codes_handle* handle) {
    auto units_result = required_string(handle, "units");
    if (!units_result) {
        return units_result.error();
    }

    const std::string units = canonical_unit(units_result.value());
    if (units == "ms**-1" || units == "ms^-1" || units == "ms-1" ||
        units == "m/s" || units == "m/s**-1" || units == "m/s^-1" ||
        units == "m/s-1") {
        return 1.0;
    }
    if (units == "knot" || units == "knots" || units == "kt" || units == "kts") {
        return kKnotsToMetresPerSecond;
    }
    if (units == "kmh**-1" || units == "kmh^-1" || units == "kmh-1" ||
        units == "km/h" || units == "kph") {
        return 1.0 / 3.6;
    }
    if (units == "cms**-1" || units == "cms^-1" || units == "cms-1" ||
        units == "cm/s") {
        return 0.01;
    }

    return Error{
        ErrorCode::unsupported_grib,
        "unsupported wind units '" + units_result.value() + "'"};
}

[[nodiscard]] std::optional<WindComponent> wind_component(codes_handle* handle) {
    std::string short_name;
    const bool has_short_name = optional_string(handle, "shortName", short_name);

    long parameter_id = 0;
    const bool has_parameter_id = optional_long(handle, "paramId", parameter_id);

    if ((has_short_name && (short_name == "10u" || short_name == "u10")) ||
        (has_parameter_id && parameter_id == 165)) {
        return WindComponent::east;
    }
    if ((has_short_name && (short_name == "10v" || short_name == "v10")) ||
        (has_parameter_id && parameter_id == 166)) {
        return WindComponent::north;
    }

    if (!has_short_name || (short_name != "u" && short_name != "v")) {
        return std::nullopt;
    }

    std::string level_type;
    long level = 0;
    if (!optional_string(handle, "typeOfLevel", level_type) ||
        !optional_long(handle, "level", level) ||
        level_type != "heightAboveGround" || level != 10) {
        return std::nullopt;
    }
    return short_name == "u" ? WindComponent::east : WindComponent::north;
}

[[nodiscard]] Result<bool> validate_wind_level(codes_handle* handle) {
    auto level_type_result = required_string(handle, "typeOfLevel");
    if (!level_type_result) {
        return level_type_result.error();
    }
    auto level_result = required_long(handle, "level");
    if (!level_result) {
        return level_result.error();
    }
    if (level_type_result.value() != "heightAboveGround" || level_result.value() != 10) {
        return Error{
            ErrorCode::unsupported_grib,
            "10 m wind message does not use typeOfLevel=heightAboveGround and level=10"};
    }
    return true;
}

[[nodiscard]] Result<TimePoint> valid_time(codes_handle* handle) {
    auto date_result = required_long(handle, "validityDate");
    if (!date_result) {
        return date_result.error();
    }
    auto time_result = required_long(handle, "validityTime");
    if (!time_result) {
        return time_result.error();
    }

    const long encoded_date = date_result.value();
    const long encoded_time = time_result.value();
    const int year = static_cast<int>(encoded_date / 10000L);
    const unsigned month = static_cast<unsigned>((encoded_date / 100L) % 100L);
    const unsigned day = static_cast<unsigned>(encoded_date % 100L);
    const int hour = static_cast<int>(encoded_time / 100L);
    const int minute = static_cast<int>(encoded_time % 100L);

    const std::chrono::year_month_day date{
        std::chrono::year{year},
        std::chrono::month{month},
        std::chrono::day{day}};
    if (!date.ok() || hour < 0 || hour > 23 || minute < 0 || minute > 59) {
        return Error{ErrorCode::grib_decode, "GRIB message has an invalid validity time"};
    }

    return TimePoint{std::chrono::sys_days{date}.time_since_epoch()} +
           std::chrono::hours{hour} + std::chrono::minutes{minute};
}

[[nodiscard]] Result<Grid> read_grid(codes_handle* handle) {
    auto grid_type_result = required_string(handle, "gridType");
    if (!grid_type_result) {
        return grid_type_result.error();
    }
    if (grid_type_result.value() != "regular_ll") {
        return Error{
            ErrorCode::unsupported_grib,
            "unsupported wind grid type '" + grid_type_result.value() +
                "'; only regular_ll is supported"};
    }

    auto ni_result = required_long(handle, "Ni");
    auto nj_result = required_long(handle, "Nj");
    if (!ni_result) {
        return ni_result.error();
    }
    if (!nj_result) {
        return nj_result.error();
    }
    if (ni_result.value() < 2 || nj_result.value() < 2) {
        return Error{
            ErrorCode::unsupported_grib,
            "regular_ll wind grids must contain at least two latitudes and longitudes"};
    }

    const auto ni = static_cast<std::size_t>(ni_result.value());
    const auto nj = static_cast<std::size_t>(nj_result.value());
    if (ni > std::numeric_limits<std::size_t>::max() / nj) {
        return Error{ErrorCode::grib_decode, "GRIB grid dimensions overflow addressable memory"};
    }

    auto first_latitude_result =
        required_double(handle, "latitudeOfFirstGridPointInDegrees");
    auto first_longitude_result =
        required_double(handle, "longitudeOfFirstGridPointInDegrees");
    auto last_latitude_result =
        required_double(handle, "latitudeOfLastGridPointInDegrees");
    auto last_longitude_result =
        required_double(handle, "longitudeOfLastGridPointInDegrees");
    if (!first_latitude_result) {
        return first_latitude_result.error();
    }
    if (!first_longitude_result) {
        return first_longitude_result.error();
    }
    if (!last_latitude_result) {
        return last_latitude_result.error();
    }
    if (!last_longitude_result) {
        return last_longitude_result.error();
    }

    long i_scans_negatively = 0;
    long j_scans_positively = 0;
    if (!optional_long(handle, "iScansNegatively", i_scans_negatively) ||
        !optional_long(handle, "jScansPositively", j_scans_positively)) {
        return Error{
            ErrorCode::grib_decode,
            "regular_ll wind grid is missing scan-direction metadata"};
    }

    double longitude_step = 0.0;
    if (!optional_double(handle, "iDirectionIncrementInDegrees", longitude_step) ||
        !std::isfinite(longitude_step) || longitude_step <= 0.0) {
        double directed_span =
            last_longitude_result.value() - first_longitude_result.value();
        if (i_scans_negatively != 0) {
            directed_span = -directed_span;
        }
        while (directed_span < 0.0) {
            directed_span += kLongitudePeriod;
        }
        longitude_step = directed_span / static_cast<double>(ni - 1U);
    }

    double latitude_step = 0.0;
    if (!optional_double(handle, "jDirectionIncrementInDegrees", latitude_step) ||
        !std::isfinite(latitude_step) || latitude_step <= 0.0) {
        latitude_step =
            std::abs(last_latitude_result.value() - first_latitude_result.value()) /
            static_cast<double>(nj - 1U);
    }
    longitude_step = std::abs(longitude_step);
    latitude_step = std::abs(latitude_step);
    if (!std::isfinite(longitude_step) || longitude_step <= 0.0 ||
        !std::isfinite(latitude_step) || latitude_step <= 0.0) {
        return Error{
            ErrorCode::unsupported_grib,
            "regular_ll wind grid has invalid coordinate increments"};
    }

    const double longitude_span = longitude_step * static_cast<double>(ni - 1U);
    const double latitude_span = latitude_step * static_cast<double>(nj - 1U);
    const double expected_last_latitude =
        first_latitude_result.value() +
        (j_scans_positively != 0 ? latitude_span : -latitude_span);
    if (!nearly_equal(
            expected_last_latitude,
            last_latitude_result.value(),
            latitude_span)) {
        return Error{
            ErrorCode::unsupported_grib,
            "regular_ll latitude metadata is inconsistent with its dimensions"};
    }

    const double expected_last_longitude =
        first_longitude_result.value() +
        (i_scans_negatively != 0 ? -longitude_span : longitude_span);
    if (circular_longitude_difference(
            expected_last_longitude,
            last_longitude_result.value()) >
        1.0e-8 * std::max(1.0, longitude_span)) {
        return Error{
            ErrorCode::unsupported_grib,
            "regular_ll longitude metadata is inconsistent with its dimensions"};
    }

    const double south_latitude =
        j_scans_positively != 0
            ? first_latitude_result.value()
            : first_latitude_result.value() - latitude_span;
    const double west_longitude =
        normalize_longitude(
            i_scans_negatively != 0
                ? first_longitude_result.value() - longitude_span
                : first_longitude_result.value());

    const bool duplicate_endpoint = nearly_equal(longitude_span, kLongitudePeriod);
    const bool global_coverage =
        duplicate_endpoint ||
        nearly_equal(longitude_step * static_cast<double>(ni), kLongitudePeriod);

    return Grid{
        ni,
        nj,
        west_longitude,
        south_latitude,
        longitude_step,
        latitude_step,
        global_coverage,
        duplicate_endpoint};
}

[[nodiscard]] bool matching_grid(const Grid& lhs, const Grid& rhs) noexcept {
    return lhs.longitude_count == rhs.longitude_count &&
           lhs.latitude_count == rhs.latitude_count &&
           lhs.global_longitude_coverage == rhs.global_longitude_coverage &&
           lhs.duplicate_longitude_endpoint == rhs.duplicate_longitude_endpoint &&
           nearly_equal(
               lhs.south_latitude_degrees,
               rhs.south_latitude_degrees,
               lhs.latitude_step_degrees *
                   static_cast<double>(lhs.latitude_count - 1U)) &&
           circular_longitude_difference(
               lhs.west_longitude_degrees,
               rhs.west_longitude_degrees) <=
               1.0e-8 * std::max(1.0, lhs.longitude_step_degrees) &&
           nearly_equal(
               lhs.longitude_step_degrees,
               rhs.longitude_step_degrees,
               lhs.longitude_step_degrees) &&
           nearly_equal(
               lhs.latitude_step_degrees,
               rhs.latitude_step_degrees,
               lhs.latitude_step_degrees);
}

[[nodiscard]] Result<DecodedField> decode_field(codes_handle* handle) {
    auto grid_result = read_grid(handle);
    if (!grid_result) {
        return grid_result.error();
    }
    const Grid grid = grid_result.value();

    std::size_t value_count = 0;
    int status = codes_get_size(handle, "values", &value_count);
    if (status != CODES_SUCCESS) {
        return codes_error(ErrorCode::grib_decode, "cannot read GRIB value count", status);
    }
    const std::size_t expected_count = grid.longitude_count * grid.latitude_count;
    if (value_count != expected_count) {
        return Error{
            ErrorCode::grib_decode,
            "GRIB value count does not match regular_ll grid dimensions"};
    }

    std::vector<double> scanned_values(value_count);
    status = codes_get_double_array(
        handle,
        "values",
        scanned_values.data(),
        &value_count);
    if (status != CODES_SUCCESS) {
        return codes_error(ErrorCode::grib_decode, "cannot decode GRIB values", status);
    }

    auto conversion_result = unit_conversion_factor(handle);
    if (!conversion_result) {
        return conversion_result.error();
    }
    const double conversion = conversion_result.value();

    long i_scans_negatively = 0;
    long j_scans_positively = 0;
    long j_points_are_consecutive = 0;
    long alternative_row_scanning = 0;
    if (!optional_long(handle, "iScansNegatively", i_scans_negatively) ||
        !optional_long(handle, "jScansPositively", j_scans_positively) ||
        !optional_long(handle, "jPointsAreConsecutive", j_points_are_consecutive)) {
        return Error{
            ErrorCode::grib_decode,
            "regular_ll wind grid is missing scan-order metadata"};
    }
    static_cast<void>(
        optional_long(handle, "alternativeRowScanning", alternative_row_scanning));

    double missing_value = std::numeric_limits<double>::quiet_NaN();
    static_cast<void>(optional_double(handle, "missingValue", missing_value));

    std::vector<double> normalized_values(
        expected_count,
        std::numeric_limits<double>::quiet_NaN());
    for (std::size_t raw_index = 0; raw_index < expected_count; ++raw_index) {
        std::size_t scan_i = 0;
        std::size_t scan_j = 0;
        if (j_points_are_consecutive == 0) {
            scan_j = raw_index / grid.longitude_count;
            scan_i = raw_index % grid.longitude_count;
            if (alternative_row_scanning != 0 && (scan_j % 2U) != 0U) {
                scan_i = grid.longitude_count - 1U - scan_i;
            }
        } else {
            scan_i = raw_index / grid.latitude_count;
            scan_j = raw_index % grid.latitude_count;
            if (alternative_row_scanning != 0 && (scan_i % 2U) != 0U) {
                scan_j = grid.latitude_count - 1U - scan_j;
            }
        }

        const std::size_t normalized_i =
            i_scans_negatively != 0
                ? grid.longitude_count - 1U - scan_i
                : scan_i;
        const std::size_t normalized_j =
            j_scans_positively != 0
                ? scan_j
                : grid.latitude_count - 1U - scan_j;

        const double value = scanned_values[raw_index];
        const bool missing =
            !std::isfinite(value) ||
            (std::isfinite(missing_value) && value == missing_value);
        normalized_values[
            normalized_j * grid.longitude_count + normalized_i] =
            missing ? std::numeric_limits<double>::quiet_NaN() : value * conversion;
    }

    return DecodedField{grid, std::move(normalized_values)};
}

struct CropWindow {
    std::size_t first_latitude{};
    std::size_t last_latitude{};
    std::size_t first_longitude{};
    std::size_t last_longitude{};
};

[[nodiscard]] std::pair<std::size_t, std::size_t> enclosing_indices(
    double first_position,
    double last_position,
    std::size_t count) noexcept {
    std::size_t first = static_cast<std::size_t>(std::floor(first_position));
    std::size_t last = static_cast<std::size_t>(std::ceil(last_position));
    first = std::min(first, count - 1U);
    last = std::min(last, count - 1U);
    if (first == last) {
        if (last + 1U < count) {
            ++last;
        } else {
            --first;
        }
    }
    return {first, last};
}

[[nodiscard]] Result<CropWindow> crop_window(
    const Grid& grid,
    const GeographicBounds& bounds) {
    const double north_latitude =
        grid.south_latitude_degrees +
        grid.latitude_step_degrees *
            static_cast<double>(grid.latitude_count - 1U);
    const double latitude_tolerance =
        1.0e-8 * std::max(1.0, north_latitude - grid.south_latitude_degrees);
    if (bounds.south_latitude_degrees <
            grid.south_latitude_degrees - latitude_tolerance ||
        bounds.north_latitude_degrees > north_latitude + latitude_tolerance) {
        return Error{
            ErrorCode::coordinate_outside_forecast,
            "requested weather bounds extend outside forecast latitude coverage"};
    }

    const double south_position =
        (std::clamp(
             bounds.south_latitude_degrees,
             grid.south_latitude_degrees,
             north_latitude) -
         grid.south_latitude_degrees) /
        grid.latitude_step_degrees;
    const double north_position =
        (std::clamp(
             bounds.north_latitude_degrees,
             grid.south_latitude_degrees,
             north_latitude) -
         grid.south_latitude_degrees) /
        grid.latitude_step_degrees;
    const auto [first_latitude, last_latitude] =
        enclosing_indices(south_position, north_position, grid.latitude_count);

    double longitude_span =
        bounds.east_longitude_degrees - bounds.west_longitude_degrees;
    if (longitude_span < 0.0) {
        longitude_span += kLongitudePeriod;
    }
    if (bounds.west_longitude_degrees == -180.0 &&
        bounds.east_longitude_degrees == 180.0) {
        longitude_span = kLongitudePeriod;
    }

    double west_delta =
        normalize_longitude(bounds.west_longitude_degrees) -
        grid.west_longitude_degrees;
    while (west_delta < 0.0) {
        west_delta += kLongitudePeriod;
    }
    while (west_delta >= kLongitudePeriod) {
        west_delta -= kLongitudePeriod;
    }

    const std::size_t period_count =
        grid.duplicate_longitude_endpoint
            ? grid.longitude_count - 1U
            : grid.longitude_count;
    const double grid_span =
        grid.global_longitude_coverage
            ? kLongitudePeriod
            : grid.longitude_step_degrees *
                  static_cast<double>(grid.longitude_count - 1U);
    const double longitude_tolerance =
        1.0e-8 * std::max(1.0, grid_span);
    if ((!grid.global_longitude_coverage &&
         (west_delta > grid_span + longitude_tolerance ||
          west_delta + longitude_span > grid_span + longitude_tolerance)) ||
        longitude_span > grid_span + longitude_tolerance) {
        return Error{
            ErrorCode::coordinate_outside_forecast,
            "requested weather bounds extend outside forecast longitude coverage"};
    }

    if (grid.global_longitude_coverage &&
        nearly_equal(longitude_span, kLongitudePeriod)) {
        return CropWindow{
            first_latitude,
            last_latitude,
            0U,
            grid.longitude_count - 1U};
    }

    const double west_position = west_delta / grid.longitude_step_degrees;
    const double east_position =
        (west_delta + longitude_span) / grid.longitude_step_degrees;
    std::size_t first_longitude =
        static_cast<std::size_t>(std::floor(west_position));
    std::size_t last_longitude =
        static_cast<std::size_t>(std::ceil(east_position));
    if (first_longitude == last_longitude) {
        ++last_longitude;
    }
    if (!grid.global_longitude_coverage) {
        first_longitude = std::min(first_longitude, grid.longitude_count - 2U);
        last_longitude = std::min(
            std::max(last_longitude, first_longitude + 1U),
            grid.longitude_count - 1U);
    } else if (last_longitude - first_longitude + 1U > period_count) {
        last_longitude = first_longitude + period_count - 1U;
    }

    return CropWindow{
        first_latitude,
        last_latitude,
        first_longitude,
        last_longitude};
}

[[nodiscard]] Result<DecodedField> crop_field(
    DecodedField field,
    const GeographicBounds& bounds) {
    auto window_result = crop_window(field.grid, bounds);
    if (!window_result) {
        return window_result.error();
    }
    const CropWindow window = window_result.value();
    const std::size_t latitude_count =
        window.last_latitude - window.first_latitude + 1U;
    const std::size_t longitude_count =
        window.last_longitude - window.first_longitude + 1U;
    if (latitude_count == field.grid.latitude_count &&
        longitude_count == field.grid.longitude_count &&
        window.first_latitude == 0U &&
        window.first_longitude == 0U) {
        return field;
    }

    const std::size_t source_longitude_period =
        field.grid.duplicate_longitude_endpoint
            ? field.grid.longitude_count - 1U
            : field.grid.longitude_count;
    std::vector<double> cropped_values;
    cropped_values.reserve(latitude_count * longitude_count);
    for (std::size_t latitude = window.first_latitude;
         latitude <= window.last_latitude;
         ++latitude) {
        for (std::size_t longitude = window.first_longitude;
             longitude <= window.last_longitude;
             ++longitude) {
            const std::size_t source_longitude =
                field.grid.global_longitude_coverage
                    ? longitude % source_longitude_period
                    : longitude;
            cropped_values.push_back(
                field.values[
                    latitude * field.grid.longitude_count +
                    source_longitude]);
        }
    }

    Grid cropped_grid{
        longitude_count,
        latitude_count,
        normalize_longitude(
            field.grid.west_longitude_degrees +
            field.grid.longitude_step_degrees *
                static_cast<double>(window.first_longitude)),
        field.grid.south_latitude_degrees +
            field.grid.latitude_step_degrees *
                static_cast<double>(window.first_latitude),
        field.grid.longitude_step_degrees,
        field.grid.latitude_step_degrees,
        false,
        false};
    return DecodedField{cropped_grid, std::move(cropped_values)};
}

[[nodiscard]] std::string component_name(WindComponent component) {
    return component == WindComponent::east ? "U" : "V";
}

struct SpatialBracket {
    std::size_t latitude0{};
    std::size_t latitude1{};
    std::size_t longitude0{};
    std::size_t longitude1{};
    double latitude_fraction{};
    double longitude_fraction{};
};

[[nodiscard]] std::pair<std::size_t, double> bounded_axis_bracket(
    double position,
    std::size_t count) noexcept {
    if (position <= 0.0) {
        return {0U, 0.0};
    }
    const double final_position = static_cast<double>(count - 1U);
    if (position >= final_position) {
        return {count - 2U, 1.0};
    }
    const auto lower = static_cast<std::size_t>(std::floor(position));
    return {lower, position - static_cast<double>(lower)};
}

[[nodiscard]] Result<SpatialBracket> spatial_bracket(
    const Grid& grid,
    Coordinate coordinate) {
    if (!std::isfinite(coordinate.latitude_degrees) ||
        !std::isfinite(coordinate.longitude_degrees) ||
        coordinate.latitude_degrees < -90.0 ||
        coordinate.latitude_degrees > 90.0) {
        return Error{
            ErrorCode::invalid_argument,
            "weather interpolation coordinate must contain a finite latitude in [-90, 90] "
            "and a finite longitude"};
    }

    const double north_latitude =
        grid.south_latitude_degrees +
        grid.latitude_step_degrees *
            static_cast<double>(grid.latitude_count - 1U);
    const double latitude_tolerance =
        1.0e-8 * std::max(1.0, north_latitude - grid.south_latitude_degrees);
    if (coordinate.latitude_degrees < grid.south_latitude_degrees - latitude_tolerance ||
        coordinate.latitude_degrees > north_latitude + latitude_tolerance) {
        return Error{
            ErrorCode::coordinate_outside_forecast,
            "latitude is outside forecast grid coverage"};
    }
    const double clamped_latitude =
        std::clamp(
            coordinate.latitude_degrees,
            grid.south_latitude_degrees,
            north_latitude);
    const double latitude_position =
        (clamped_latitude - grid.south_latitude_degrees) /
        grid.latitude_step_degrees;
    const auto [latitude0, latitude_fraction] =
        bounded_axis_bracket(latitude_position, grid.latitude_count);

    std::size_t longitude0 = 0;
    std::size_t longitude1 = 0;
    double longitude_fraction = 0.0;
    const double normalized_query =
        normalize_longitude(coordinate.longitude_degrees);

    if (grid.global_longitude_coverage) {
        double delta =
            std::fmod(
                normalized_query - grid.west_longitude_degrees,
                kLongitudePeriod);
        if (delta < 0.0) {
            delta += kLongitudePeriod;
        }
        const double longitude_position = delta / grid.longitude_step_degrees;
        if (grid.duplicate_longitude_endpoint) {
            const auto bracket =
                bounded_axis_bracket(longitude_position, grid.longitude_count);
            longitude0 = bracket.first;
            longitude1 = longitude0 + 1U;
            longitude_fraction = bracket.second;
        } else {
            longitude0 =
                static_cast<std::size_t>(std::floor(longitude_position)) %
                grid.longitude_count;
            longitude1 = (longitude0 + 1U) % grid.longitude_count;
            longitude_fraction =
                longitude_position - std::floor(longitude_position);
        }
    } else {
        const double east_longitude =
            grid.west_longitude_degrees +
            grid.longitude_step_degrees *
                static_cast<double>(grid.longitude_count - 1U);
        double unwrapped_query = normalized_query;
        while (unwrapped_query < grid.west_longitude_degrees) {
            unwrapped_query += kLongitudePeriod;
        }
        while (unwrapped_query - kLongitudePeriod >= grid.west_longitude_degrees) {
            unwrapped_query -= kLongitudePeriod;
        }
        const double longitude_tolerance =
            1.0e-8 * std::max(1.0, east_longitude - grid.west_longitude_degrees);
        if (unwrapped_query < grid.west_longitude_degrees - longitude_tolerance ||
            unwrapped_query > east_longitude + longitude_tolerance) {
            return Error{
                ErrorCode::coordinate_outside_forecast,
                "longitude is outside forecast grid coverage"};
        }
        unwrapped_query =
            std::clamp(
                unwrapped_query,
                grid.west_longitude_degrees,
                east_longitude);
        const double longitude_position =
            (unwrapped_query - grid.west_longitude_degrees) /
            grid.longitude_step_degrees;
        const auto bracket =
            bounded_axis_bracket(longitude_position, grid.longitude_count);
        longitude0 = bracket.first;
        longitude1 = longitude0 + 1U;
        longitude_fraction = bracket.second;
    }

    return SpatialBracket{
        latitude0,
        latitude0 + 1U,
        longitude0,
        longitude1,
        latitude_fraction,
        longitude_fraction};
}

[[nodiscard]] Result<double> bilinear_sample(
    const std::vector<double>& values,
    const Grid& grid,
    std::size_t time_index,
    const SpatialBracket& bracket,
    std::string_view component) {
    const std::size_t plane_size =
        grid.longitude_count * grid.latitude_count;
    const std::size_t base = time_index * plane_size;
    const auto at = [&](std::size_t latitude, std::size_t longitude) {
        return values[base + latitude * grid.longitude_count + longitude];
    };

    const double value00 = at(bracket.latitude0, bracket.longitude0);
    const double value01 = at(bracket.latitude0, bracket.longitude1);
    const double value10 = at(bracket.latitude1, bracket.longitude0);
    const double value11 = at(bracket.latitude1, bracket.longitude1);
    const double west_fraction = 1.0 - bracket.longitude_fraction;
    const double south_fraction = 1.0 - bracket.latitude_fraction;
    const std::array<double, 4> values_at_corners{
        value00,
        value01,
        value10,
        value11};
    const std::array<double, 4> weights{
        south_fraction * west_fraction,
        south_fraction * bracket.longitude_fraction,
        bracket.latitude_fraction * west_fraction,
        bracket.latitude_fraction * bracket.longitude_fraction};

    double result = 0.0;
    for (std::size_t index = 0; index < weights.size(); ++index) {
        if (weights[index] == 0.0) {
            continue;
        }
        if (!std::isfinite(values_at_corners[index])) {
            return Error{
                ErrorCode::incomplete_forecast,
                std::string{"missing "} + std::string{component} +
                    " wind value in interpolation stencil"};
        }
        result += values_at_corners[index] * weights[index];
    }
    return result;
}

}  // namespace

struct WeatherDataset::Impl {
    ForecastMetadata metadata;
    Grid grid;
    std::vector<TimePoint> times;
    std::vector<double> east_values;
    std::vector<double> north_values;
    std::optional<GeographicBounds> bounds;
};

WeatherDataset::WeatherDataset() : impl_(std::make_shared<Impl>()) {}

WeatherDataset::~WeatherDataset() = default;

WeatherDataset::WeatherDataset(const WeatherDataset&) = default;

WeatherDataset::WeatherDataset(WeatherDataset&&) noexcept = default;

WeatherDataset& WeatherDataset::operator=(const WeatherDataset&) = default;

WeatherDataset& WeatherDataset::operator=(WeatherDataset&&) noexcept = default;

WeatherDataset::WeatherDataset(std::shared_ptr<const Impl> impl)
    : impl_(std::move(impl)) {}

Result<WeatherDataset> WeatherDataset::load(const std::filesystem::path& path) {
    return load_impl(path, std::nullopt);
}

Result<WeatherDataset> WeatherDataset::load(
    const std::filesystem::path& path,
    GeographicBounds bounds) {
    return load_impl(path, bounds);
}

Result<WeatherDataset> WeatherDataset::load_impl(
    const std::filesystem::path& path,
    std::optional<GeographicBounds> bounds) {
    if (path.empty()) {
        return Error{ErrorCode::invalid_argument, "GRIB path must not be empty"};
    }
    if (bounds) {
        if (const auto error = validate_bounds(*bounds)) {
            return *error;
        }
    }

    const auto utf8_path = path.u8string();
    const std::string display_path{
        reinterpret_cast<const char*>(utf8_path.data()),
        utf8_path.size()};
    errno = 0;
#ifdef _WIN32
    FilePtr file{_wfopen(path.c_str(), L"rb")};
#else
    FilePtr file{std::fopen(path.c_str(), "rb")};
#endif
    if (!file) {
        const int open_error = errno;
        std::string message = "cannot open GRIB file '" + display_path + "'";
        if (open_error != 0) {
            message += ": ";
            message += std::strerror(open_error);
        }
        return Error{ErrorCode::file_io, std::move(message)};
    }

    std::map<TimePoint, PendingTimeSlice> pending;
    std::size_t grib_message_count = 0;
    std::size_t wind_message_count = 0;
    int decode_status = CODES_SUCCESS;

    while (true) {
        HandlePtr handle{
            codes_handle_new_from_file(
                nullptr,
                file.get(),
                PRODUCT_GRIB,
                &decode_status)};
        if (!handle) {
            break;
        }
        ++grib_message_count;

        auto edition_result = required_long(handle.get(), "edition");
        if (!edition_result) {
            return edition_result.error();
        }
        if (edition_result.value() != 1 && edition_result.value() != 2) {
            return Error{
                ErrorCode::unsupported_grib,
                "unsupported GRIB edition " + std::to_string(edition_result.value())};
        }

        const std::optional<WindComponent> component =
            wind_component(handle.get());
        if (!component) {
            continue;
        }
        ++wind_message_count;

        auto level_result = validate_wind_level(handle.get());
        if (!level_result) {
            return level_result.error();
        }
        auto time_result = valid_time(handle.get());
        if (!time_result) {
            return time_result.error();
        }
        auto field_result = decode_field(handle.get());
        if (!field_result) {
            return field_result.error();
        }
        if (bounds) {
            field_result = crop_field(
                std::move(field_result.value()),
                *bounds);
            if (!field_result) {
                return field_result.error();
            }
        }

        PendingTimeSlice& slice = pending[time_result.value()];
        std::optional<DecodedField>& destination =
            *component == WindComponent::east ? slice.east : slice.north;
        if (destination) {
            return Error{
                ErrorCode::incomplete_forecast,
                "ambiguous forecast: multiple 10 m " +
                    component_name(*component) +
                    " wind messages share one valid time"};
        }
        destination = std::move(field_result.value());
    }

    if (decode_status != CODES_SUCCESS) {
        return codes_error(
            ErrorCode::grib_decode,
            "failed while scanning GRIB messages",
            decode_status);
    }
    if (grib_message_count == 0) {
        return Error{
            ErrorCode::grib_decode,
            "file contains no decodable GRIB1 or GRIB2 messages"};
    }
    if (wind_message_count == 0 || pending.empty()) {
        return Error{
            ErrorCode::incomplete_forecast,
            "forecast contains no supported 10 m U/V wind messages"};
    }

    std::optional<Grid> common_grid;
    for (const auto& [time, slice] : pending) {
        static_cast<void>(time);
        if (!slice.east || !slice.north) {
            return Error{
                ErrorCode::incomplete_forecast,
                "forecast does not contain paired 10 m U/V wind messages at every valid time"};
        }
        if (!matching_grid(slice.east->grid, slice.north->grid)) {
            return Error{
                ErrorCode::incomplete_forecast,
                "paired 10 m U/V wind messages use different grids"};
        }
        if (!common_grid) {
            common_grid = slice.east->grid;
        } else if (!matching_grid(*common_grid, slice.east->grid)) {
            return Error{
                ErrorCode::incomplete_forecast,
                "10 m wind grid changes between forecast valid times"};
        }
    }

    const Grid grid = *common_grid;
    const std::size_t plane_size =
        grid.longitude_count * grid.latitude_count;
    if (pending.size() > std::numeric_limits<std::size_t>::max() / plane_size) {
        return Error{
            ErrorCode::grib_decode,
            "decoded forecast dimensions overflow addressable memory"};
    }
    const std::size_t total_size = pending.size() * plane_size;

    auto impl = std::make_shared<Impl>();
    impl->grid = grid;
    impl->bounds = bounds;
    impl->times.reserve(pending.size());
    impl->east_values.reserve(total_size);
    impl->north_values.reserve(total_size);
    for (auto& [time, slice] : pending) {
        impl->times.push_back(time);
        impl->east_values.insert(
            impl->east_values.end(),
            std::make_move_iterator(slice.east->values.begin()),
            std::make_move_iterator(slice.east->values.end()));
        impl->north_values.insert(
            impl->north_values.end(),
            std::make_move_iterator(slice.north->values.begin()),
            std::make_move_iterator(slice.north->values.end()));
    }

    impl->metadata = ForecastMetadata{
        impl->times.front(),
        impl->times.back(),
        grid.latitude_count,
        grid.longitude_count,
        grid.global_longitude_coverage,
        display_path};
    return WeatherDataset{std::move(impl)};
}

const ForecastMetadata& WeatherDataset::metadata() const {
    if (impl_) {
        return impl_->metadata;
    }
    static const ForecastMetadata empty_metadata{};
    return empty_metadata;
}

Result<Wind> WeatherDataset::interpolate(
    Coordinate coordinate,
    TimePoint time) const {
    if (!impl_ || impl_->times.empty()) {
        return Error{
            ErrorCode::incomplete_forecast,
            "weather dataset is empty"};
    }
    if (impl_->bounds) {
        const GeographicBounds& bounds = *impl_->bounds;
        double longitude_span =
            bounds.east_longitude_degrees - bounds.west_longitude_degrees;
        if (longitude_span < 0.0) {
            longitude_span += kLongitudePeriod;
        }
        if (bounds.west_longitude_degrees == -180.0 &&
            bounds.east_longitude_degrees == 180.0) {
            longitude_span = kLongitudePeriod;
        }
        double longitude_delta =
            normalize_longitude(coordinate.longitude_degrees) -
            normalize_longitude(bounds.west_longitude_degrees);
        while (longitude_delta < 0.0) {
            longitude_delta += kLongitudePeriod;
        }
        const double tolerance = 1.0e-8;
        if (!std::isfinite(coordinate.latitude_degrees) ||
            !std::isfinite(coordinate.longitude_degrees) ||
            coordinate.latitude_degrees <
                bounds.south_latitude_degrees - tolerance ||
            coordinate.latitude_degrees >
                bounds.north_latitude_degrees + tolerance ||
            longitude_delta > longitude_span + tolerance) {
            return Error{
                ErrorCode::coordinate_outside_forecast,
                "coordinate is outside the requested weather bounds"};
        }
    }

    const auto upper =
        std::lower_bound(impl_->times.begin(), impl_->times.end(), time);
    std::size_t time0 = 0;
    std::size_t time1 = 0;
    double time_fraction = 0.0;
    if (upper == impl_->times.end()) {
        if (time != impl_->times.back()) {
            return Error{
                ErrorCode::departure_outside_forecast,
                "requested time is after forecast coverage"};
        }
        time0 = impl_->times.size() - 1U;
        time1 = time0;
    } else if (*upper == time) {
        time0 = static_cast<std::size_t>(
            std::distance(impl_->times.begin(), upper));
        time1 = time0;
    } else {
        if (upper == impl_->times.begin()) {
            return Error{
                ErrorCode::departure_outside_forecast,
                "requested time is before forecast coverage"};
        }
        time1 = static_cast<std::size_t>(
            std::distance(impl_->times.begin(), upper));
        time0 = time1 - 1U;
        const auto interval = impl_->times[time1] - impl_->times[time0];
        const auto elapsed = time - impl_->times[time0];
        const double interval_seconds =
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::seconds>(interval).count());
        const double elapsed_seconds =
            static_cast<double>(
                std::chrono::duration_cast<std::chrono::seconds>(elapsed).count());
        if (interval_seconds <= 0.0) {
            return Error{
                ErrorCode::incomplete_forecast,
                "forecast valid times are not strictly increasing"};
        }
        time_fraction = elapsed_seconds / interval_seconds;
    }

    auto bracket_result = spatial_bracket(impl_->grid, coordinate);
    if (!bracket_result) {
        return bracket_result.error();
    }
    const SpatialBracket bracket = bracket_result.value();

    auto east0_result =
        bilinear_sample(
            impl_->east_values,
            impl_->grid,
            time0,
            bracket,
            "U");
    if (!east0_result) {
        return east0_result.error();
    }
    auto north0_result =
        bilinear_sample(
            impl_->north_values,
            impl_->grid,
            time0,
            bracket,
            "V");
    if (!north0_result) {
        return north0_result.error();
    }

    if (time0 == time1) {
        return Wind{east0_result.value(), north0_result.value()};
    }

    auto east1_result =
        bilinear_sample(
            impl_->east_values,
            impl_->grid,
            time1,
            bracket,
            "U");
    if (!east1_result) {
        return east1_result.error();
    }
    auto north1_result =
        bilinear_sample(
            impl_->north_values,
            impl_->grid,
            time1,
            bracket,
            "V");
    if (!north1_result) {
        return north1_result.error();
    }

    return Wind{
        east0_result.value() +
            (east1_result.value() - east0_result.value()) * time_fraction,
        north0_result.value() +
            (north1_result.value() - north0_result.value()) * time_fraction};
}

}  // namespace sailroute
