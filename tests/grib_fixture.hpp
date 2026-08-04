#pragma once

#include <eccodes.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace sailroute::test {

/// Writes a small constant-wind GRIB2 file for tests that need a real dataset.
///
/// The wind is uniform in space and constant in time, so any routing difference
/// a test observes comes from the option or provider under test rather than
/// from the forecast.
class ConstantWindGribFixture {
public:
    struct Options {
        double east_metres_per_second{0.0};
        double north_metres_per_second{-10.0};
        double south_latitude_degrees{0.0};
        double west_longitude_degrees{0.0};
        double step_degrees{1.0};
        long point_count{3};
        long data_date{20260714};
        long data_time{1200};
        long final_forecast_hour{24};
    };

    ConstantWindGribFixture() : ConstantWindGribFixture(Options{}) {}

    explicit ConstantWindGribFixture(Options options)
        : options_(options),
          path_(
              std::filesystem::current_path() /
              ("sailroute-environment-" +
               std::to_string(
                   std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".grib")) {
        write("10u", 0, options_.east_metres_per_second, "w");
        write("10v", 0, options_.north_metres_per_second, "a");
        write(
            "10u",
            options_.final_forecast_hour,
            options_.east_metres_per_second,
            "a");
        write(
            "10v",
            options_.final_forecast_hour,
            options_.north_metres_per_second,
            "a");
    }

    ConstantWindGribFixture(const ConstantWindGribFixture&) = delete;
    ConstantWindGribFixture& operator=(const ConstantWindGribFixture&) = delete;

    ~ConstantWindGribFixture() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    static void check(int status, const char* operation) {
        if (status != CODES_SUCCESS) {
            throw std::runtime_error(
                std::string{operation} + ": " + codes_get_error_message(status));
        }
    }

    void write(
        const char* short_name,
        long forecast_hour,
        double value,
        const char* mode) {
        codes_handle* handle =
            codes_grib_handle_new_from_samples(nullptr, "regular_ll_sfc_grib2");
        if (handle == nullptr) {
            throw std::runtime_error("unable to create GRIB fixture handle");
        }
        try {
            const double span = options_.step_degrees *
                static_cast<double>(options_.point_count - 1);
            check(codes_set_long(handle, "Ni", options_.point_count), "set Ni");
            check(codes_set_long(handle, "Nj", options_.point_count), "set Nj");
            check(
                codes_set_double(
                    handle,
                    "latitudeOfFirstGridPointInDegrees",
                    options_.south_latitude_degrees + span),
                "set first latitude");
            check(
                codes_set_double(
                    handle,
                    "longitudeOfFirstGridPointInDegrees",
                    options_.west_longitude_degrees),
                "set first longitude");
            check(
                codes_set_double(
                    handle,
                    "latitudeOfLastGridPointInDegrees",
                    options_.south_latitude_degrees),
                "set last latitude");
            check(
                codes_set_double(
                    handle,
                    "longitudeOfLastGridPointInDegrees",
                    options_.west_longitude_degrees + span),
                "set last longitude");
            check(
                codes_set_double(
                    handle,
                    "iDirectionIncrementInDegrees",
                    options_.step_degrees),
                "set longitude increment");
            check(
                codes_set_double(
                    handle,
                    "jDirectionIncrementInDegrees",
                    options_.step_degrees),
                "set latitude increment");
            check(
                codes_set_long(handle, "dataDate", options_.data_date),
                "set date");
            check(
                codes_set_long(handle, "dataTime", options_.data_time),
                "set time");
            check(
                codes_set_long(handle, "forecastTime", forecast_hour),
                "set forecast time");
            std::size_t name_size = std::char_traits<char>::length(short_name);
            check(
                codes_set_string(handle, "shortName", short_name, &name_size),
                "set wind component");
            check(codes_set_long(handle, "level", 10), "set wind level");
            const std::vector<double> values(
                static_cast<std::size_t>(
                    options_.point_count * options_.point_count),
                value);
            check(
                codes_set_double_array(
                    handle, "values", values.data(), values.size()),
                "set wind values");
            check(
                codes_write_message(handle, path_.string().c_str(), mode),
                "write GRIB fixture");
        } catch (...) {
            codes_handle_delete(handle);
            throw;
        }
        codes_handle_delete(handle);
    }

    Options options_;
    std::filesystem::path path_;
};

}  // namespace sailroute::test
