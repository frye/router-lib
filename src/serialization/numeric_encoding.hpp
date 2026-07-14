#pragma once

#include <charconv>
#include <cmath>
#include <limits>
#include <string>

namespace sailroute::serialization_detail {

inline bool append_number(std::string& output, double value) {
    if (!std::isfinite(value)) {
        return false;
    }
    char buffer[64]{};
    const auto converted = std::to_chars(
        buffer,
        buffer + sizeof(buffer),
        value,
        std::chars_format::general,
        std::numeric_limits<double>::max_digits10);
    if (converted.ec != std::errc{}) {
        return false;
    }
    output.append(buffer, converted.ptr);
    return true;
}

inline bool append_coordinate_number(std::string& output, double value) {
    if (!std::isfinite(value)) {
        return false;
    }
    if (value == 0.0) {
        output.push_back('0');
        return true;
    }
    char buffer[64]{};
    const auto converted = std::to_chars(
        buffer,
        buffer + sizeof(buffer),
        value,
        std::chars_format::fixed,
        10);
    if (converted.ec != std::errc{}) {
        return false;
    }
    char* end = converted.ptr;
    while (end > buffer && end[-1] == '0') {
        --end;
    }
    if (end > buffer && end[-1] == '.') {
        --end;
    }
    output.append(buffer, end);
    return true;
}

}  // namespace sailroute::serialization_detail
