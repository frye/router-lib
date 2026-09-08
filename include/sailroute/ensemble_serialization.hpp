#pragma once

#include "sailroute/ensemble.hpp"
#include "sailroute/error.hpp"

#include <cstddef>
#include <string>
#include <string_view>

namespace sailroute {

inline constexpr std::size_t ensemble_json_max_input_bytes =
    64U * 1024U * 1024U;
inline constexpr std::size_t ensemble_json_max_nesting_depth = 128U;

/// Metadata and selected common-plan/member-risk results for round trips.
struct EnsembleRouteDocument {
    EnsembleRunMetadata metadata;
    std::vector<EnsembleMemberMetadata> member_metadata;
    EnsembleRouteResult result;
};

/// Writes ensemble_route_result_v2 for compact results or polar-wind audit data,
/// with optional retained-alternative diagnostics. Legacy-compatible graph
/// results without the new audit data continue to use ensemble_route_result_v1.
/// Neither format represents an executable adaptive policy.
[[nodiscard]] Result<std::string> ensemble_route_to_json(
    const EnsembleRouteDocument& doc);

/// Parses strict v1/v2 documents, rejecting unknown fields/versions, invalid
/// values, inconsistent member identities, and malformed alternative topology.
[[nodiscard]] Result<EnsembleRouteDocument> ensemble_route_from_json(
    std::string_view json);

struct EnsembleRivalOutcomesDocument {
    std::vector<EnsembleMemberOutcome> member_outcomes;
};

/// Writes the ensemble_rival_outcomes_v1 format.
[[nodiscard]] Result<std::string> ensemble_rival_outcomes_to_json(
    const EnsembleRivalOutcomesDocument& doc);

/// Parses the strict ensemble_rival_outcomes_v1 format.
[[nodiscard]] Result<EnsembleRivalOutcomesDocument>
ensemble_rival_outcomes_from_json(std::string_view json);

}  // namespace sailroute
