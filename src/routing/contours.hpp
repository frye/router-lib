#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <optional>
#include <span>
#include <vector>

namespace sailroute::detail {

[[nodiscard]] std::optional<Error> build_display_contours_into(
    std::span<const Coordinate> points,
    const DisplayContourOptions& options,
    std::vector<Coordinate>& contour_points,
    std::vector<DisplayContourSegment>& segments);

}  // namespace sailroute::detail
