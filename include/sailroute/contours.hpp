#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <span>

namespace sailroute {

// Builds deterministic display segments without forcing disconnected or
// degenerate frontiers into one closed ring. Antimeridian crossings are split
// into canonical-coordinate open segments.
[[nodiscard]] Result<DisplayContours> build_display_contours(
    std::span<const Coordinate> points,
    const DisplayContourOptions& options = {});

}  // namespace sailroute
