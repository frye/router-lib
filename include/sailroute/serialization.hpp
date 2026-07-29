#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <string>

namespace sailroute {

/// Serializes route metadata, diagnostics, and points as JSON.
Result<std::string> route_to_json(const RouteResult& route);
/// Serializes a route as a GPX 1.1 track with sailroute extensions.
Result<std::string> route_to_gpx(const RouteResult& route);
/// Serializes captured isochrone display contours as GeoJSON.
Result<std::string> isochrones_to_json(const RouteResult& route);
/// Serializes captured isochrones as separate GPX 1.1 tracks.
Result<std::string> isochrones_to_gpx(const RouteResult& route);

}  // namespace sailroute
