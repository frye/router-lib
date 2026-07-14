#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <string>
#include <string_view>

namespace sailroute {

Result<TimePoint> parse_utc_time(std::string_view value);
std::string format_utc_time(TimePoint value);

}  // namespace sailroute
