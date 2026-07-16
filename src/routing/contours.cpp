#include "sailroute/contours.hpp"

#include "routing/contours.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace sailroute {
namespace {

struct PlanarPoint {
    double x{};
    double y{};
    double unwrapped_longitude{};
    Coordinate coordinate;
};

struct Edge {
    std::size_t first{};
    std::size_t second{};

    Edge(std::size_t left, std::size_t right)
        : first(std::min(left, right)), second(std::max(left, right)) {}

    friend bool operator<(const Edge& left, const Edge& right) noexcept {
        return std::tie(left.first, left.second) <
               std::tie(right.first, right.second);
    }
};

struct Triangle {
    std::size_t first{};
    std::size_t second{};
    std::size_t third{};
};

double normalized_longitude_delta(double longitude, double origin) noexcept {
    return std::fmod(longitude - origin + 540.0, 360.0) - 180.0;
}

Coordinate canonical_coordinate(Coordinate coordinate) noexcept {
    if (coordinate.longitude_degrees == 180.0) {
        coordinate.longitude_degrees = -180.0;
    }
    return coordinate;
}

long double orientation(
    const PlanarPoint& first,
    const PlanarPoint& second,
    const PlanarPoint& third) noexcept {
    return
        (static_cast<long double>(second.x) - first.x) *
            (static_cast<long double>(third.y) - first.y) -
        (static_cast<long double>(second.y) - first.y) *
            (static_cast<long double>(third.x) - first.x);
}

std::optional<Triangle> make_triangle(
    const std::vector<PlanarPoint>& points,
    std::size_t first,
    std::size_t second,
    std::size_t third) {
    const long double area =
        orientation(points[first], points[second], points[third]);
    if (area == 0.0L) {
        return std::nullopt;
    }
    if (area < 0.0L) {
        std::swap(first, second);
    }
    return Triangle{first, second, third};
}

bool circumcircle_contains(
    const std::vector<PlanarPoint>& points,
    const Triangle& triangle,
    const PlanarPoint& point) noexcept {
    const PlanarPoint& first = points[triangle.first];
    const PlanarPoint& second = points[triangle.second];
    const PlanarPoint& third = points[triangle.third];
    const long double ax = static_cast<long double>(first.x) - point.x;
    const long double ay = static_cast<long double>(first.y) - point.y;
    const long double bx = static_cast<long double>(second.x) - point.x;
    const long double by = static_cast<long double>(second.y) - point.y;
    const long double cx = static_cast<long double>(third.x) - point.x;
    const long double cy = static_cast<long double>(third.y) - point.y;
    const long double determinant =
        (ax * ax + ay * ay) * (bx * cy - by * cx) -
        (bx * bx + by * by) * (ax * cy - ay * cx) +
        (cx * cx + cy * cy) * (ax * by - ay * bx);
    return determinant > 0.0L;
}

double distance(const PlanarPoint& left, const PlanarPoint& right) noexcept {
    return std::hypot(left.x - right.x, left.y - right.y);
}

double circumradius(
    const std::vector<PlanarPoint>& points,
    const Triangle& triangle) noexcept {
    const PlanarPoint& first = points[triangle.first];
    const PlanarPoint& second = points[triangle.second];
    const PlanarPoint& third = points[triangle.third];
    const long double twice_area =
        std::abs(orientation(first, second, third));
    if (twice_area == 0.0L) {
        return std::numeric_limits<double>::infinity();
    }
    const long double product =
        static_cast<long double>(distance(first, second)) *
        distance(second, third) *
        distance(third, first);
    return static_cast<double>(product / (2.0L * twice_area));
}

std::vector<Triangle> triangulate(
    const std::vector<PlanarPoint>& input_points) {
    if (input_points.size() < 3U) {
        return {};
    }

    std::vector<PlanarPoint> points = input_points;
    double minimum_x = input_points.front().x;
    double maximum_x = minimum_x;
    double minimum_y = input_points.front().y;
    double maximum_y = minimum_y;
    for (const PlanarPoint& point : input_points) {
        minimum_x = std::min(minimum_x, point.x);
        maximum_x = std::max(maximum_x, point.x);
        minimum_y = std::min(minimum_y, point.y);
        maximum_y = std::max(maximum_y, point.y);
    }
    const double midpoint_x = (minimum_x + maximum_x) / 2.0;
    const double midpoint_y = (minimum_y + maximum_y) / 2.0;
    const double extent =
        std::max({maximum_x - minimum_x, maximum_y - minimum_y, 1.0});
    const std::size_t super_first = points.size();
    points.push_back(PlanarPoint{
        midpoint_x - 32.0 * extent,
        midpoint_y - extent,
        0.0,
        {}});
    const std::size_t super_second = points.size();
    points.push_back(PlanarPoint{
        midpoint_x,
        midpoint_y + 32.0 * extent,
        0.0,
        {}});
    const std::size_t super_third = points.size();
    points.push_back(PlanarPoint{
        midpoint_x + 32.0 * extent,
        midpoint_y - extent,
        0.0,
        {}});

    std::vector<Triangle> triangles;
    const auto super_triangle =
        make_triangle(points, super_first, super_second, super_third);
    if (!super_triangle.has_value()) {
        return {};
    }
    triangles.push_back(*super_triangle);

    for (std::size_t point_index = 0U;
         point_index < input_points.size();
         ++point_index) {
        std::map<Edge, std::size_t> edge_counts;
        std::vector<Triangle> retained;
        retained.reserve(triangles.size());
        for (const Triangle& triangle : triangles) {
            if (!circumcircle_contains(points, triangle, points[point_index])) {
                retained.push_back(triangle);
                continue;
            }
            ++edge_counts[Edge{triangle.first, triangle.second}];
            ++edge_counts[Edge{triangle.second, triangle.third}];
            ++edge_counts[Edge{triangle.third, triangle.first}];
        }
        triangles = std::move(retained);
        for (const auto& [edge, count] : edge_counts) {
            if (count != 1U) {
                continue;
            }
            const auto triangle =
                make_triangle(points, edge.first, edge.second, point_index);
            if (triangle.has_value()) {
                triangles.push_back(*triangle);
            }
        }
    }

    std::erase_if(
        triangles,
        [input_size = input_points.size()](const Triangle& triangle) {
            return triangle.first >= input_size ||
                   triangle.second >= input_size ||
                   triangle.third >= input_size;
        });
    return triangles;
}

double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2U;
    if (values.size() % 2U != 0U) {
        return values[middle];
    }
    return (values[middle - 1U] + values[middle]) / 2.0;
}

double automatic_alpha(
    const std::vector<PlanarPoint>& points,
    const std::vector<Triangle>& triangles) {
    std::vector<double> radii;
    radii.reserve(triangles.size());
    for (const Triangle& triangle : triangles) {
        const double radius = circumradius(points, triangle);
        if (std::isfinite(radius) && radius > 0.0) {
            radii.push_back(radius);
        }
    }
    if (!radii.empty()) {
        return median(std::move(radii)) * 1.05;
    }

    std::vector<double> adjacent_distances;
    adjacent_distances.reserve(points.size());
    for (std::size_t index = 1U; index < points.size(); ++index) {
        const double separation = distance(points[index - 1U], points[index]);
        if (separation > 0.0) {
            adjacent_distances.push_back(separation);
        }
    }
    return median(std::move(adjacent_distances));
}

std::vector<std::vector<std::size_t>> trace_boundary_paths(
    const std::vector<PlanarPoint>& points,
    const std::set<Edge>& edges,
    std::vector<bool>& boundary_vertices,
    std::vector<bool>& closed_paths) {
    std::vector<std::vector<std::size_t>> adjacency(points.size());
    for (const Edge& edge : edges) {
        adjacency[edge.first].push_back(edge.second);
        adjacency[edge.second].push_back(edge.first);
        boundary_vertices[edge.first] = true;
        boundary_vertices[edge.second] = true;
    }
    for (auto& neighbors : adjacency) {
        std::sort(neighbors.begin(), neighbors.end());
    }

    std::set<Edge> unused = edges;
    std::vector<std::vector<std::size_t>> paths;
    const auto trace_from =
        [&adjacency, &unused](
            std::size_t start,
            std::size_t next,
            bool allow_cycle) {
            std::vector<std::size_t> path{start};
            std::size_t previous = start;
            std::size_t current = next;
            unused.erase(Edge{start, next});
            while (true) {
                path.push_back(current);
                if (current == start) {
                    break;
                }
                if (!allow_cycle && adjacency[current].size() != 2U) {
                    break;
                }
                std::optional<std::size_t> following;
                for (const std::size_t candidate : adjacency[current]) {
                    if (candidate == previous) {
                        continue;
                    }
                    if (unused.contains(Edge{current, candidate})) {
                        following = candidate;
                        break;
                    }
                }
                if (!following.has_value()) {
                    break;
                }
                previous = current;
                current = *following;
                unused.erase(Edge{previous, current});
            }
            return path;
        };

    for (std::size_t vertex = 0U; vertex < adjacency.size(); ++vertex) {
        if (adjacency[vertex].empty() || adjacency[vertex].size() == 2U) {
            continue;
        }
        for (const std::size_t neighbor : adjacency[vertex]) {
            if (!unused.contains(Edge{vertex, neighbor})) {
                continue;
            }
            paths.push_back(trace_from(vertex, neighbor, false));
            closed_paths.push_back(false);
        }
    }
    while (!unused.empty()) {
        const Edge first = *unused.begin();
        std::vector<std::size_t> path =
            trace_from(first.first, first.second, true);
        const bool closed =
            path.size() > 2U && path.front() == path.back();
        if (closed) {
            path.pop_back();
        }
        paths.push_back(std::move(path));
        closed_paths.push_back(closed);
    }
    return paths;
}

void normalize_path(
    const std::vector<PlanarPoint>& points,
    std::vector<std::size_t>& path,
    bool closed) {
    if (path.size() < 2U) {
        return;
    }
    if (!closed) {
        if (path.back() < path.front()) {
            std::reverse(path.begin(), path.end());
        }
        return;
    }

    long double signed_twice_area = 0.0L;
    for (std::size_t index = 0U; index < path.size(); ++index) {
        const PlanarPoint& current = points[path[index]];
        const PlanarPoint& next = points[path[(index + 1U) % path.size()]];
        signed_twice_area +=
            static_cast<long double>(current.x) * next.y -
            static_cast<long double>(next.x) * current.y;
    }
    if (signed_twice_area < 0.0L) {
        std::reverse(path.begin(), path.end());
    }
    const auto minimum =
        std::min_element(path.begin(), path.end());
    std::rotate(path.begin(), minimum, path.end());
}

void append_segment(
    std::span<const Coordinate> points,
    bool closed,
    std::vector<Coordinate>& output,
    std::vector<DisplayContourSegment>& segments) {
    if (points.empty()) {
        return;
    }
    const std::size_t offset = output.size();
    output.insert(output.end(), points.begin(), points.end());
    segments.push_back(DisplayContourSegment{offset, points.size(), closed});
}

void append_antimeridian_safe_path(
    const std::vector<PlanarPoint>& points,
    std::span<const std::size_t> path,
    bool closed,
    std::vector<Coordinate>& output,
    std::vector<DisplayContourSegment>& segments) {
    if (path.empty()) {
        return;
    }
    if (path.size() == 1U) {
        const Coordinate point = points[path.front()].coordinate;
        append_segment(
            std::span<const Coordinate>{&point, 1U},
            false,
            output,
            segments);
        return;
    }

    std::vector<Coordinate> current;
    current.reserve(path.size());
    current.push_back(points[path.front()].coordinate);
    const std::size_t edge_count = path.size() - 1U + (closed ? 1U : 0U);
    bool split = false;
    for (std::size_t edge_index = 0U; edge_index < edge_count; ++edge_index) {
        const PlanarPoint& first = points[path[edge_index % path.size()]];
        const PlanarPoint& second =
            points[path[(edge_index + 1U) % path.size()]];
        const double edge_second_longitude =
            first.unwrapped_longitude +
            normalized_longitude_delta(
                second.coordinate.longitude_degrees,
                first.coordinate.longitude_degrees);
        const double lower = std::min(
            first.unwrapped_longitude,
            edge_second_longitude);
        const double upper = std::max(
            first.unwrapped_longitude,
            edge_second_longitude);
        std::optional<double> boundary;
        if (std::abs(
                second.coordinate.longitude_degrees -
                first.coordinate.longitude_degrees) > 180.0) {
            for (const double candidate : {-180.0, 180.0}) {
                if (candidate >= lower && candidate <= upper) {
                    boundary = candidate;
                    break;
                }
            }
        }
        if (!boundary.has_value()) {
            if (!(closed && edge_index + 1U == edge_count)) {
                current.push_back(second.coordinate);
            }
            continue;
        }

        split = true;
        const double fraction =
            (*boundary - first.unwrapped_longitude) /
            (edge_second_longitude - first.unwrapped_longitude);
        const double latitude =
            first.coordinate.latitude_degrees +
            fraction *
                (second.coordinate.latitude_degrees -
                 first.coordinate.latitude_degrees);
        const bool eastward =
            edge_second_longitude > first.unwrapped_longitude;
        const Coordinate leaving{
            latitude,
            eastward ? 180.0 : -180.0};
        const Coordinate entering{
            latitude,
            eastward ? -180.0 : 180.0};
        current.push_back(leaving);
        append_segment(current, false, output, segments);
        current.clear();
        current.push_back(entering);
        if (*boundary != edge_second_longitude &&
            !(closed && edge_index + 1U == edge_count)) {
            current.push_back(second.coordinate);
        }
    }
    if (!current.empty()) {
        append_segment(current, closed && !split, output, segments);
    }
}

std::vector<std::vector<std::size_t>> degenerate_paths(
    const std::vector<PlanarPoint>& points,
    double alpha,
    bool explicit_alpha,
    std::vector<bool>& used) {
    std::vector<std::vector<std::size_t>> paths;
    if (points.empty()) {
        return paths;
    }
    if (points.size() == 1U) {
        used[0U] = true;
        paths.push_back({0U});
        return paths;
    }

    const double maximum_gap =
        alpha > 0.0 ? 2.0 * alpha : std::numeric_limits<double>::infinity();
    std::vector<std::size_t> current{0U};
    used[0U] = true;
    for (std::size_t index = 1U; index < points.size(); ++index) {
        if (explicit_alpha &&
            distance(points[index - 1U], points[index]) > maximum_gap) {
            paths.push_back(std::move(current));
            current.clear();
        }
        current.push_back(index);
        used[index] = true;
    }
    paths.push_back(std::move(current));
    return paths;
}

std::optional<Error> prepare_planar_points(
    std::span<const Coordinate> input,
    std::vector<PlanarPoint>& points) {
    points.clear();
    if (input.empty()) {
        return std::nullopt;
    }

    double latitude_center = 0.0;
    double longitude_sine = 0.0;
    double longitude_cosine = 0.0;
    for (const Coordinate input_coordinate : input) {
        if (!is_valid(input_coordinate)) {
            return Error{
                ErrorCode::invalid_argument,
                "display contour coordinates must be finite and within canonical bounds"};
        }
        const Coordinate coordinate =
            canonical_coordinate(input_coordinate);
        latitude_center += coordinate.latitude_degrees;
        const double radians =
            coordinate.longitude_degrees * std::numbers::pi / 180.0;
        longitude_sine += std::sin(radians);
        longitude_cosine += std::cos(radians);
    }
    latitude_center /= static_cast<double>(input.size());
    const auto canonical_origin = std::min_element(
        input.begin(),
        input.end(),
        [](Coordinate left, Coordinate right) {
            left = canonical_coordinate(left);
            right = canonical_coordinate(right);
            return std::tie(
                       left.longitude_degrees,
                       left.latitude_degrees) <
                   std::tie(
                       right.longitude_degrees,
                       right.latitude_degrees);
        });
    double longitude_center = canonical_origin->longitude_degrees;
    longitude_center =
        canonical_coordinate({0.0, longitude_center}).longitude_degrees;
    if (std::abs(longitude_sine) > 1.0e-12 ||
        std::abs(longitude_cosine) > 1.0e-12) {
        longitude_center =
            std::atan2(longitude_sine, longitude_cosine) *
            180.0 / std::numbers::pi;
    }
    const double longitude_scale =
        std::cos(latitude_center * std::numbers::pi / 180.0);

    points.reserve(input.size());
    for (const Coordinate input_coordinate : input) {
        const Coordinate coordinate =
            canonical_coordinate(input_coordinate);
        const double delta =
            normalized_longitude_delta(
                coordinate.longitude_degrees,
                longitude_center);
        points.push_back(PlanarPoint{
            delta * 60.0 * longitude_scale,
            (coordinate.latitude_degrees - latitude_center) * 60.0,
            longitude_center + delta,
            coordinate});
    }
    std::sort(
        points.begin(),
        points.end(),
        [](const PlanarPoint& left, const PlanarPoint& right) {
            return std::tie(
                       left.x,
                       left.y,
                       left.coordinate.latitude_degrees,
                       left.coordinate.longitude_degrees) <
                   std::tie(
                       right.x,
                       right.y,
                       right.coordinate.latitude_degrees,
                       right.coordinate.longitude_degrees);
        });
    points.erase(
        std::unique(
            points.begin(),
            points.end(),
            [](const PlanarPoint& left, const PlanarPoint& right) {
                return left.coordinate.latitude_degrees ==
                           right.coordinate.latitude_degrees &&
                       left.coordinate.longitude_degrees ==
                           right.coordinate.longitude_degrees;
            }),
        points.end());
    return std::nullopt;
}

}  // namespace

namespace detail {

std::optional<Error> build_display_contours_into(
    std::span<const Coordinate> input,
    const DisplayContourOptions& options,
    std::vector<Coordinate>& contour_points,
    std::vector<DisplayContourSegment>& segments) {
    contour_points.clear();
    segments.clear();
    if (options.alpha_nautical_miles.has_value() &&
        (!std::isfinite(*options.alpha_nautical_miles) ||
         *options.alpha_nautical_miles <= 0.0)) {
        return Error{
            ErrorCode::invalid_argument,
            "display contour alpha_nautical_miles must be finite and positive"};
    }

    std::vector<PlanarPoint> points;
    if (const auto error = prepare_planar_points(input, points);
        error.has_value()) {
        return error;
    }
    if (points.empty()) {
        return std::nullopt;
    }

    const std::vector<Triangle> triangles = triangulate(points);
    const double alpha = options.alpha_nautical_miles.value_or(
        automatic_alpha(points, triangles));
    std::map<Edge, std::size_t> edge_counts;
    std::vector<bool> alpha_vertices(points.size(), false);
    for (const Triangle& triangle : triangles) {
        const double radius = circumradius(points, triangle);
        if (!std::isfinite(radius) || radius > alpha) {
            continue;
        }
        alpha_vertices[triangle.first] = true;
        alpha_vertices[triangle.second] = true;
        alpha_vertices[triangle.third] = true;
        ++edge_counts[Edge{triangle.first, triangle.second}];
        ++edge_counts[Edge{triangle.second, triangle.third}];
        ++edge_counts[Edge{triangle.third, triangle.first}];
    }
    std::set<Edge> boundary_edges;
    for (const auto& [edge, count] : edge_counts) {
        if (count == 1U) {
            boundary_edges.insert(edge);
        }
    }

    std::vector<bool> emitted(points.size(), false);
    std::vector<bool> closed_paths;
    std::vector<std::vector<std::size_t>> paths;
    if (!boundary_edges.empty()) {
        paths = trace_boundary_paths(
            points,
            boundary_edges,
            emitted,
            closed_paths);
    } else {
        paths = degenerate_paths(
            points,
            alpha,
            options.alpha_nautical_miles.has_value(),
            emitted);
        closed_paths.assign(paths.size(), false);
    }

    for (std::size_t index = 0U; index < paths.size(); ++index) {
        normalize_path(points, paths[index], closed_paths[index]);
        append_antimeridian_safe_path(
            points,
            paths[index],
            closed_paths[index],
            contour_points,
            segments);
    }
    for (std::size_t index = 0U; index < points.size(); ++index) {
        if (emitted[index] || alpha_vertices[index]) {
            continue;
        }
        const std::array<std::size_t, 1U> singleton{index};
        append_antimeridian_safe_path(
            points,
            singleton,
            false,
            contour_points,
            segments);
    }
    return std::nullopt;
}

}  // namespace detail

Result<DisplayContours> build_display_contours(
    std::span<const Coordinate> points,
    const DisplayContourOptions& options) {
    DisplayContours contours;
    if (const auto error = detail::build_display_contours_into(
            points,
            options,
            contours.points,
            contours.segments);
        error.has_value()) {
        return *error;
    }
    return contours;
}

}  // namespace sailroute
