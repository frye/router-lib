#pragma once

#include "sailroute/ensemble.hpp"
#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <cstddef>
#include <string>
#include <string_view>

namespace sailroute {

/// Hard parser limits for the isolated ensemble JSON formats.
inline constexpr std::size_t ensemble_json_max_input_bytes =
    64U * 1024U * 1024U;
inline constexpr std::size_t ensemble_json_max_nesting_depth = 128U;

/// Serializes route metadata, diagnostics, and points as JSON.
Result<std::string> route_to_json(const RouteResult& route);
/// Serializes a route as a GPX 1.1 track with sailroute extensions.
Result<std::string> route_to_gpx(const RouteResult& route);
/// Serializes captured isochrone display contours as GeoJSON.
Result<std::string> isochrones_to_json(const RouteResult& route);
/// Serializes captured isochrones as separate GPX 1.1 tracks.
Result<std::string> isochrones_to_gpx(const RouteResult& route);

/// Combined run metadata and ensemble route result for round-trip serialization.
struct EnsembleRouteDocument {
    EnsembleRunMetadata metadata;
    /// Canonically ordered forecast and environment attribution for every member.
    std::vector<EnsembleMemberMetadata> member_metadata;
    EnsembleRouteResult result;
};

/// Serializes an ensemble route result and its run metadata as versioned JSON.
///
/// Uses schema version "ensemble_route_result_v1". Isolated from the
/// deterministic route_to_json serializer; existing deterministic output is
/// not affected.
[[nodiscard]] Result<std::string> ensemble_route_to_json(
    const EnsembleRouteDocument& doc);

/// Parses a versioned ensemble route JSON document.
///
/// Rejects: malformed JSON, unknown schema version, missing/unknown fields,
/// wrong types, invalid enum names, non-finite numeric values, and
/// inconsistent topology or member identities.
[[nodiscard]] Result<EnsembleRouteDocument> ensemble_route_from_json(
    std::string_view json);

/// Strict rival outcomes document parsed from the ensemble_rival_outcomes_v1
/// JSON format used by --ensemble-rival CLI input.
struct EnsembleRivalOutcomesDocument {
    std::vector<EnsembleMemberOutcome> member_outcomes;
};

/// Serializes rival outcomes as versioned JSON (schema ensemble_rival_outcomes_v1).
[[nodiscard]] Result<std::string> ensemble_rival_outcomes_to_json(
    const EnsembleRivalOutcomesDocument& doc);

/// Parses a rival outcomes JSON document (schema ensemble_rival_outcomes_v1).
[[nodiscard]] Result<EnsembleRivalOutcomesDocument>
ensemble_rival_outcomes_from_json(std::string_view json);

}  // namespace sailroute
