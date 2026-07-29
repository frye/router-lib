#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <string>
#include <string_view>

namespace sailroute {

/// Parses an exact UTC timestamp in YYYY-MM-DDTHH:MM:SSZ form.
Result<TimePoint> parse_utc_time(std::string_view value);
/// Formats a UTC timestamp at whole-second precision.
std::string format_utc_time(TimePoint value);

}  // namespace sailroute
