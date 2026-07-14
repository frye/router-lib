#include "sailroute/time.hpp"

#include <array>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <ctime>

namespace sailroute {
namespace {

bool parse_fixed_int(std::string_view value, std::size_t offset, std::size_t count, int& result) {
    if (offset + count > value.size()) {
        return false;
    }
    for (std::size_t index = offset; index < offset + count; ++index) {
        if (value[index] < '0' || value[index] > '9') {
            return false;
        }
    }
    const char* begin = value.data() + offset;
    const char* end = begin + count;
    const auto parsed = std::from_chars(begin, end, result);
    return parsed.ec == std::errc{} && parsed.ptr == end;
}

}  // namespace

Result<TimePoint> parse_utc_time(std::string_view value) {
    if (value.size() != 20 || value[4] != '-' || value[7] != '-' || value[10] != 'T' ||
        value[13] != ':' || value[16] != ':' || value[19] != 'Z') {
        return Error{ErrorCode::invalid_argument, "departure must use YYYY-MM-DDTHH:MM:SSZ"};
    }

    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    int second = 0;
    if (!parse_fixed_int(value, 0, 4, year) || !parse_fixed_int(value, 5, 2, month) ||
        !parse_fixed_int(value, 8, 2, day) || !parse_fixed_int(value, 11, 2, hour) ||
        !parse_fixed_int(value, 14, 2, minute) || !parse_fixed_int(value, 17, 2, second)) {
        return Error{ErrorCode::invalid_argument, "departure contains non-numeric date fields"};
    }

    const std::chrono::year_month_day date{
        std::chrono::year{year},
        std::chrono::month{static_cast<unsigned>(month)},
        std::chrono::day{static_cast<unsigned>(day)}};
    if (!date.ok() || hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
        second < 0 || second > 59) {
        return Error{ErrorCode::invalid_argument, "departure is not a valid UTC timestamp"};
    }

    return TimePoint{std::chrono::sys_days{date}.time_since_epoch()} +
           std::chrono::hours{hour} + std::chrono::minutes{minute} +
           std::chrono::seconds{second};
}

std::string format_utc_time(TimePoint value) {
    const auto days = std::chrono::floor<std::chrono::days>(value);
    const std::chrono::year_month_day date{days};
    const auto time = value - days;
    const auto hours = std::chrono::duration_cast<std::chrono::hours>(time);
    const auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time - hours);
    const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time - hours - minutes);

    std::array<char, 21> buffer{};
    std::snprintf(
        buffer.data(),
        buffer.size(),
        "%04d-%02u-%02uT%02lld:%02lld:%02lldZ",
        static_cast<int>(date.year()),
        static_cast<unsigned>(date.month()),
        static_cast<unsigned>(date.day()),
        static_cast<long long>(hours.count()),
        static_cast<long long>(minutes.count()),
        static_cast<long long>(seconds.count()));
    return buffer.data();
}

}  // namespace sailroute
