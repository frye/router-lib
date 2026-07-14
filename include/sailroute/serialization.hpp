#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <string>

namespace sailroute {

Result<std::string> route_to_json(const RouteResult& route);
Result<std::string> route_to_gpx(const RouteResult& route);
Result<std::string> isochrones_to_json(const RouteResult& route);
Result<std::string> isochrones_to_gpx(const RouteResult& route);

}  // namespace sailroute
