#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <optional>
#include <span>
#include <vector>

namespace sailroute::detail {

// Writes the destination-facing front into pre-allocated output vectors.
// Clears the output vectors before populating them.
[[nodiscard]] std::optional<Error> build_destination_front_into(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles,
    const DestinationFrontOptions& options,
    std::vector<Coordinate>& front_points,
    std::vector<IsochroneFrontSegment>& segments);

// Uses the route search's actual provisional endpoint to identify the principal
// component while selecting display points from the full eligible candidate set.
[[nodiscard]] std::optional<Error> build_destination_front_into(
    std::span<const Coordinate> candidate_points,
    Coordinate destination,
    Coordinate anchor,
    double band_width_nautical_miles,
    const DestinationFrontOptions& options,
    std::vector<Coordinate>& front_points,
    std::vector<IsochroneFrontSegment>& segments);

}  // namespace sailroute::detail
