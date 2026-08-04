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
//   - The destination-facing aperture keeps points whose absolute bearing
//     difference from the destination bearing is no greater than
//     options.half_angle_degrees. A point at the centroid itself is retained.
//     If no non-centroid point lies within the aperture, all points are
//     retained and the per-band selection picks the best available ones.
//   - Points are assigned to cross-track bands of width
//     band_width_nautical_miles. The legacy provisional-component policy
//     selects greatest along-track progress per band. The opt-in
//     all-meaningful-components policy selects smallest remaining great-circle
//     distance, followed by greatest along-track progress. Both use
//     lexicographic (latitude, longitude) tie-breaks.
//   - Missing bands split the winners into open components. The component
//     containing the provisional best point is always retained. With
//     all_meaningful_components, secondary components meeting
//     minimum_secondary_segment_points are retained as well.
//   - Retained components and their points are ordered port-to-starboard
//     (ascending cross-track band).
//   - Antimeridian crossings between adjacent ordered points produce a new
//     segment boundary; no coordinates are interpolated or synthesised.
//
// Defaults preserve the original single provisional-component behavior.
// A single retained point produces a one-point, one-segment front.
// An empty input produces an empty front without error.
//
// Returns an error if any input coordinate is invalid or if
// band_width_nautical_miles is not finite and positive, or if
// options.half_angle_degrees is not finite and in (0, 180].
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
