#include "sailroute/contours.hpp"

#include "test_support.hpp"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

namespace {

std::span<const sailroute::Coordinate> segment_points(
    const sailroute::DisplayContours& contours,
    const sailroute::DisplayContourSegment& segment) {
    return std::span<const sailroute::Coordinate>{contours.points}.subspan(
        segment.point_offset,
        segment.point_count);
}

void require_same_contours(
    const sailroute::DisplayContours& left,
    const sailroute::DisplayContours& right) {
    REQUIRE(left.points.size() == right.points.size());
    REQUIRE(left.segments.size() == right.segments.size());
    for (std::size_t index = 0U; index < left.points.size(); ++index) {
        REQUIRE(
            left.points[index].latitude_degrees ==
            right.points[index].latitude_degrees);
        REQUIRE(
            left.points[index].longitude_degrees ==
            right.points[index].longitude_degrees);
    }
    for (std::size_t index = 0U; index < left.segments.size(); ++index) {
        REQUIRE(
            left.segments[index].point_offset ==
            right.segments[index].point_offset);
        REQUIRE(
            left.segments[index].point_count ==
            right.segments[index].point_count);
        REQUIRE(left.segments[index].closed == right.segments[index].closed);
    }
}

}  // namespace

TEST_CASE("display contours are deterministic for non-convex input") {
    const std::vector<sailroute::Coordinate> points{
        {0.0, 0.0},
        {0.0, 3.0},
        {1.0, 3.0},
        {1.0, 1.0},
        {2.0, 1.0},
        {2.0, 3.0},
        {3.0, 3.0},
        {3.0, 0.0},
    };
    auto forward = sailroute::build_display_contours(points);
    REQUIRE(forward.has_value());
    REQUIRE(!forward.value().segments.empty());

    std::vector<sailroute::Coordinate> reversed = points;
    std::reverse(reversed.begin(), reversed.end());
    auto backward = sailroute::build_display_contours(reversed);
    REQUIRE(backward.has_value());
    require_same_contours(forward.value(), backward.value());

    bool retained_concavity = false;
    for (const sailroute::Coordinate point : forward.value().points) {
        retained_concavity =
            retained_concavity ||
            (point.latitude_degrees == 1.0 &&
             point.longitude_degrees == 1.0);
    }
    REQUIRE(retained_concavity);
}

TEST_CASE("display contours preserve disconnected components") {
    const std::vector<sailroute::Coordinate> points{
        {0.0, 0.0},
        {0.0, 1.0},
        {1.0, 1.0},
        {1.0, 0.0},
        {0.0, 10.0},
        {0.0, 11.0},
        {1.0, 11.0},
        {1.0, 10.0},
    };
    const sailroute::DisplayContourOptions options{
        .alpha_nautical_miles = 50.0,
    };
    auto contours = sailroute::build_display_contours(points, options);
    REQUIRE(contours.has_value());
    REQUIRE(contours.value().segments.size() == 2U);
    for (const sailroute::DisplayContourSegment& segment :
         contours.value().segments) {
        REQUIRE(segment.closed);
        REQUIRE(segment.point_count == 4U);
    }
}

TEST_CASE("display contours use open chains for collinear frontiers") {
    const std::vector<sailroute::Coordinate> points{
        {0.0, 2.0},
        {0.0, 0.0},
        {0.0, 1.0},
    };
    auto contours = sailroute::build_display_contours(points);
    REQUIRE(contours.has_value());
    REQUIRE(contours.value().segments.size() == 1U);
    const sailroute::DisplayContourSegment& segment =
        contours.value().segments.front();
    REQUIRE(!segment.closed);
    REQUIRE(segment.point_count == 3U);
    const auto ordered = segment_points(contours.value(), segment);
    REQUIRE(ordered.front().longitude_degrees == 0.0);
    REQUIRE(ordered.back().longitude_degrees == 2.0);
}

TEST_CASE("display contours split antimeridian crossings") {
    const std::vector<sailroute::Coordinate> points{
        {-1.0, 179.0},
        {-1.0, -179.0},
        {1.0, -179.0},
        {1.0, 179.0},
    };
    auto contours = sailroute::build_display_contours(points);
    REQUIRE(contours.has_value());
    REQUIRE(!contours.value().segments.empty());

    bool saw_positive_boundary = false;
    bool saw_negative_boundary = false;
    for (const sailroute::DisplayContourSegment& segment :
         contours.value().segments) {
        REQUIRE(!segment.closed);
        const auto ordered = segment_points(contours.value(), segment);
        for (std::size_t index = 0U; index < ordered.size(); ++index) {
            saw_positive_boundary =
                saw_positive_boundary ||
                ordered[index].longitude_degrees == 180.0;
            saw_negative_boundary =
                saw_negative_boundary ||
                ordered[index].longitude_degrees == -180.0;
            if (index != 0U) {
                REQUIRE(
                    std::abs(
                        ordered[index].longitude_degrees -
                        ordered[index - 1U].longitude_degrees) <=
                    180.0);
            }
        }
    }
    REQUIRE(saw_positive_boundary);
    REQUIRE(saw_negative_boundary);
}

TEST_CASE("display contours split edges that start on the antimeridian") {
    const std::vector<sailroute::Coordinate> points{
        {0.0, 180.0},
        {0.0, -179.0},
        {1.0, -179.0},
    };
    auto contours = sailroute::build_display_contours(points);
    REQUIRE(contours.has_value());
    REQUIRE(!contours.value().segments.empty());
    for (const sailroute::DisplayContourSegment& segment :
         contours.value().segments) {
        const auto ordered = segment_points(contours.value(), segment);
        for (std::size_t index = 1U; index < ordered.size(); ++index) {
            REQUIRE(
                std::abs(
                    ordered[index].longitude_degrees -
                    ordered[index - 1U].longitude_degrees) <=
                180.0);
        }
    }
}

TEST_CASE("display contours deduplicate antimeridian aliases") {
    auto contours = sailroute::build_display_contours(
        std::vector<sailroute::Coordinate>{
            {0.0, -180.0},
            {0.0, 180.0},
        });
    REQUIRE(contours.has_value());
    REQUIRE(contours.value().points.size() == 1U);
    REQUIRE(std::isfinite(
        contours.value().points.front().latitude_degrees));
    REQUIRE(std::isfinite(
        contours.value().points.front().longitude_degrees));
    REQUIRE(
        contours.value().points.front().longitude_degrees == -180.0);
}

TEST_CASE("display contour fallback meridians are input-order independent") {
    const std::vector<sailroute::Coordinate> points{
        {-1.0, -90.0},
        {1.0, -90.0},
        {-1.0, 90.0},
        {1.0, 90.0},
    };
    auto forward = sailroute::build_display_contours(points);
    REQUIRE(forward.has_value());

    std::vector<sailroute::Coordinate> reversed = points;
    std::reverse(reversed.begin(), reversed.end());
    auto backward = sailroute::build_display_contours(reversed);
    REQUIRE(backward.has_value());
    require_same_contours(forward.value(), backward.value());
}

TEST_CASE("display contours split shortest dateline edges with a Greenwich center") {
    const std::vector<sailroute::Coordinate> points{
        {-2.0, -179.0},
        {2.0, -179.0},
        {-2.0, 179.0},
        {2.0, 179.0},
        {-3.0, 0.0},
        {-1.5, 0.0},
        {-0.5, 0.0},
        {0.5, 0.0},
        {1.5, 0.0},
        {3.0, 0.0},
    };
    auto contours = sailroute::build_display_contours(
        points,
        sailroute::DisplayContourOptions{
            .alpha_nautical_miles = 20'000.0,
        });
    REQUIRE(contours.has_value());
    for (const sailroute::DisplayContourSegment& segment :
         contours.value().segments) {
        const auto ordered = segment_points(contours.value(), segment);
        for (std::size_t index = 1U; index < ordered.size(); ++index) {
            REQUIRE(
                std::abs(
                    ordered[index].longitude_degrees -
                    ordered[index - 1U].longitude_degrees) <=
                180.0);
        }
    }
}

TEST_CASE("display contours reject invalid coordinates and alpha") {
    auto contours = sailroute::build_display_contours(
        std::vector<sailroute::Coordinate>{{0.0, 181.0}});
    REQUIRE(!contours.has_value());
    REQUIRE(contours.error().code == sailroute::ErrorCode::invalid_argument);

    contours = sailroute::build_display_contours(
        std::vector<sailroute::Coordinate>{{0.0, 0.0}},
        sailroute::DisplayContourOptions{.alpha_nautical_miles = 0.0});
    REQUIRE(!contours.has_value());
    REQUIRE(contours.error().code == sailroute::ErrorCode::invalid_argument);
}
