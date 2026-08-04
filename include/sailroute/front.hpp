#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <span>

namespace sailroute {

// Builds a destination-facing equal-time reachable front from retained points.
// The default mode preserves the v0.3 principal-component algorithm. The
// eligible_pre_prune mode enables the Stage 2 anchor/component rules; Router
// supplies the pre-prune population and explicit anchor for that mode.
//
// The algorithm operates in a local geodesic frame centered on the frontier
// centroid and oriented toward the destination:
//
//   - The destination-facing aperture keeps points whose absolute bearing
//     difference from the destination bearing is no greater than
//     options.half_angle_degrees. A point at the centroid itself is retained.
//     If no non-centroid point lies within the aperture, all points are
//     retained and the per-band selection picks the best available ones.
//   - Points are assigned to cross-track bands of width
//     band_width_nautical_miles.  The point with the greatest along-track
//     progress (i.e. closest to the destination along the forward axis) is
//     selected per band; ties are broken lexicographically by (latitude,
//     longitude) for determinism.
//   - The principal contiguous band run contains the derived provisional point.
//     In eligible_pre_prune mode, segment_policy may retain additional runs with
//     at least minimum_secondary_segment_points as separate open segments.
//   - The trimmed set is ordered port-to-starboard (ascending cross-track
//     offset, equivalently ascending band index).
//   - Antimeridian crossings between adjacent ordered points produce a new
//     segment boundary; no coordinates are interpolated or synthesised.
//
// A single retained point produces a one-point, one-segment front.
// An empty input produces an empty front without error.
//
// Every returned coordinate is an input coordinate; none are synthesized.
// Returns an error if any input coordinate is invalid or if
// band_width_nautical_miles is not finite and positive, or if
// options.half_angle_degrees is not finite and in (0, 180], or if
// options.minimum_secondary_segment_points is zero.
[[nodiscard]] Result<IsochroneFront> build_destination_front(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles,
    const DestinationFrontOptions& options);

// Builds the same front using the original 90-degree half-angle.
// This overload preserves the existing public symbol and behavior.
[[nodiscard]] Result<IsochroneFront> build_destination_front(
    std::span<const Coordinate> retained_points,
    Coordinate destination,
    double band_width_nautical_miles);

}  // namespace sailroute
