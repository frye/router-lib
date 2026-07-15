#include "sailroute/error.hpp"

#include <string_view>

namespace sailroute {

std::string_view to_string(ErrorCode code) noexcept {
    switch (code) {
        case ErrorCode::invalid_argument: return "invalid_argument";
        case ErrorCode::file_io: return "file_io";
        case ErrorCode::grib_decode: return "grib_decode";
        case ErrorCode::unsupported_grib: return "unsupported_grib";
        case ErrorCode::incomplete_forecast: return "incomplete_forecast";
        case ErrorCode::invalid_polar: return "invalid_polar";
        case ErrorCode::departure_outside_forecast: return "departure_outside_forecast";
        case ErrorCode::coordinate_outside_forecast: return "coordinate_outside_forecast";
        case ErrorCode::no_route: return "no_route";
        case ErrorCode::cancelled: return "cancelled";
        case ErrorCode::forecast_exhausted: return "forecast_exhausted";
        case ErrorCode::output_error: return "output_error";
    }
    return "unknown";
}

}  // namespace sailroute
