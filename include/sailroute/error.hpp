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

/// A machine-readable error category and human-readable diagnostic.
struct Error {
    ErrorCode code;
    std::string message;
};

/// Returns the stable snake_case name of an error category.
std::string_view to_string(ErrorCode code) noexcept;

/// Value-or-error return type used by all fallible public operations.
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
