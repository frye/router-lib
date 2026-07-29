#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <span>

namespace sailroute {

// Builds the destination-facing equal-time reachable front from a set of
// retained frontier points produced by a single routing time step.
//
// The algorithm operates in a local geodesic frame centered on the frontier
// centroid and oriented toward the destination:
//
//   - Only the destination-facing half (points with positive along-track
//     projection onto the destination bearing) is considered.
//   - Points are assigned to cross-track bands of width
//     band_width_nautical_miles.  The point with the greatest along-track
//     progress (i.e. closest to the destination along the forward axis) is
//     selected per band; ties are broken lexicographically by (latitude,
//     longitude) for determinism.
//   - The selected set is trimmed to the contiguous band run (no gaps in band
//     index) that contains the provisional best point (the input point with the
//     smallest great-circle distance to the destination).
//   - The trimmed set is ordered port-to-starboard (ascending cross-track
//     offset, equivalently ascending band index).
//   - Antimeridian crossings between adjacent ordered points produce a new
//     segment boundary; no coordinates are interpolated or synthesised.
//
// A single retained point produces a one-point, one-segment front.
// An empty input produces an empty front without error.
//
// Returns an error if any input coordinate is invalid or if
// band_width_nautical_miles is not finite and positive.
[[nodiscard]] Result<IsochroneFront> build_destination_front(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles);

}  // namespace sailroute
