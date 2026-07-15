#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace sailroute {

enum class ErrorCode {
    invalid_argument,
    file_io,
    grib_decode,
    unsupported_grib,
    incomplete_forecast,
    invalid_polar,
    departure_outside_forecast,
    coordinate_outside_forecast,
    no_route,
    forecast_exhausted,
    output_error,
    cancelled,
};

struct Error {
    ErrorCode code;
    std::string message;
};

std::string_view to_string(ErrorCode code) noexcept;

template <typename T>
class Result {
public:
    Result(T value) : value_(std::move(value)) {}
    Result(Error error) : value_(std::move(error)) {}

    [[nodiscard]] bool has_value() const noexcept {
        return std::holds_alternative<T>(value_);
    }

    explicit operator bool() const noexcept {
        return has_value();
    }

    [[nodiscard]] T& value() {
        return std::get<T>(value_);
    }

    [[nodiscard]] const T& value() const {
        return std::get<T>(value_);
    }

    [[nodiscard]] Error& error() {
        return std::get<Error>(value_);
    }

    [[nodiscard]] const Error& error() const {
        return std::get<Error>(value_);
    }

private:
    std::variant<T, Error> value_;
};

}  // namespace sailroute
