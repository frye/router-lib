#include "routing/mixed_lattice.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <utility>
#include <vector>

namespace sailroute::detail {
namespace {

constexpr double earth_radius_nautical_miles = 3440.065;

struct Vector3 {
    double x{};
    double y{};
    double z{};
};

Vector3 operator+(Vector3 left, Vector3 right) noexcept {
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vector3 operator-(Vector3 left, Vector3 right) noexcept {
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vector3 operator*(Vector3 vector, double scalar) noexcept {
    return {vector.x * scalar, vector.y * scalar, vector.z * scalar};
}

double dot(Vector3 left, Vector3 right) noexcept {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vector3 cross(Vector3 left, Vector3 right) noexcept {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x};
}

double length(Vector3 vector) noexcept {
    return std::sqrt(dot(vector, vector));
}

Vector3 normalized(Vector3 vector) noexcept {
    const double magnitude = length(vector);
    if (!(magnitude > 0.0)) {
        return {};
    }
    return vector * (1.0 / magnitude);
}

Vector3 vector_from(Coordinate coordinate) noexcept {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    const double latitude = coordinate.latitude_degrees * degrees_to_radians;
    const double longitude = coordinate.longitude_degrees * degrees_to_radians;
    const double cosine_latitude = std::cos(latitude);
    return {
        cosine_latitude * std::cos(longitude),
        cosine_latitude * std::sin(longitude),
        std::sin(latitude)};
}

Coordinate coordinate_from(Vector3 vector) noexcept {
    constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
    vector = normalized(vector);
    return {
        std::asin(std::clamp(vector.z, -1.0, 1.0)) * radians_to_degrees,
        std::atan2(vector.y, vector.x) * radians_to_degrees};
}

double angular_distance(Vector3 left, Vector3 right) noexcept {
    return std::acos(std::clamp(dot(left, right), -1.0, 1.0));
}

bool contains(
    Vector3 point,
    Vector3 first,
    Vector3 second,
    Vector3 third) noexcept {
    constexpr double boundary_tolerance = 1.0e-12;
    const auto same_side = [point](Vector3 start, Vector3 end, Vector3 opposite) {
        const Vector3 normal = cross(start, end);
        const double reference = dot(normal, opposite);
        const double candidate = dot(normal, point);
        return reference >= 0.0
            ? candidate >= -boundary_tolerance
            : candidate <= boundary_tolerance;
    };
    return same_side(first, second, third) &&
        same_side(second, third, first) &&
        same_side(third, first, second);
}

double point_to_segment_radians(
    Vector3 point,
    Vector3 start,
    Vector3 end) noexcept {
    const double segment_length = angular_distance(start, end);
    if (segment_length <= 1.0e-12) {
        return angular_distance(point, start);
    }

    const Vector3 unnormalized_normal = cross(start, end);
    if (length(unnormalized_normal) <= 1.0e-12) {
        return std::min(
            angular_distance(point, start),
            angular_distance(point, end));
    }
    const Vector3 normal = normalized(unnormalized_normal);

    const Vector3 projected =
        normalized(point - normal * dot(point, normal));
    double best = std::min(
        angular_distance(point, start),
        angular_distance(point, end));
    for (const Vector3 candidate :
         std::array{projected, projected * -1.0}) {
        const double first = angular_distance(start, candidate);
        const double second = angular_distance(candidate, end);
        if (std::abs(first + second - segment_length) <= 1.0e-10) {
            best = std::min(best, angular_distance(point, candidate));
        }
    }
    return best;
}

double distance_to_route_radians(
    Vector3 coordinate,
    std::span<const RoutePoint> route) noexcept {
    if (route.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    if (route.size() == 1U) {
        return angular_distance(
            coordinate, vector_from(route.front().position));
    }

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t index = 1U; index < route.size(); ++index) {
        best = std::min(
            best,
            point_to_segment_radians(
                coordinate,
                vector_from(route[index - 1U].position),
                vector_from(route[index].position)));
    }
    return best;
}

bool face_intersects_corridor(
    const GeodesicLattice::Face& face,
    std::span<const Coordinate> coordinates,
    std::span<const RoutePoint> route,
    double width_radians) noexcept {
    const Vector3 first = vector_from(coordinates[face[0U]]);
    const Vector3 second = vector_from(coordinates[face[1U]]);
    const Vector3 third = vector_from(coordinates[face[2U]]);
    const Vector3 center = normalized(first + second + third);
    const double radius = std::max({
        angular_distance(center, first),
        angular_distance(center, second),
        angular_distance(center, third)});
    return distance_to_route_radians(center, route) <= width_radians + radius;
}

std::pair<std::size_t, std::size_t> ordered_edge(
    std::size_t first,
    std::size_t second) noexcept {
    return {std::min(first, second), std::max(first, second)};
}

}  // namespace

double distance_to_route_nautical_miles(
    Coordinate coordinate,
    std::span<const RoutePoint> route) noexcept {
    return distance_to_route_radians(vector_from(coordinate), route) *
        earth_radius_nautical_miles;
}

Result<MixedResolutionLattice> MixedResolutionLattice::create(
    std::size_t coarse_level,
    std::size_t refinement_levels,
    std::span<const RoutePoint> route,
    double corridor_width_nautical_miles) {
    if (route.empty()) {
        return Error{
            ErrorCode::invalid_argument,
            "mixed-resolution corridor requires an incumbent route"};
    }
    if (coarse_level + refinement_levels >
        GeodesicLattice::maximum_subdivision_level) {
        return Error{
            ErrorCode::invalid_argument,
            "mixed-resolution lattice exceeds the supported maximum level"};
    }

    auto coarse = GeodesicLattice::create(coarse_level);
    if (!coarse) {
        return coarse.error();
    }

    MixedResolutionLattice mixed;
    mixed.subdivision_level_ = coarse_level + refinement_levels;
    mixed.coarse_vertex_count_ = coarse.value().vertex_count();
    mixed.coordinates_.reserve(coarse.value().vertex_count());
    for (CellIndex cell = 0U; cell < coarse.value().vertex_count(); ++cell) {
        mixed.coordinates_.push_back(coarse.value().coordinate(cell));
    }
    std::vector<Face> faces(
        coarse.value().faces().begin(),
        coarse.value().faces().end());
    std::map<std::pair<CellIndex, CellIndex>, CellIndex> midpoint_cache;
    std::set<std::pair<CellIndex, CellIndex>> split_edges;
    const double width_radians =
        corridor_width_nautical_miles / earth_radius_nautical_miles;

    for (std::size_t level = 0U; level < refinement_levels; ++level) {
        std::vector<Face> refined;
        refined.reserve(faces.size() * 2U);
        const auto midpoint = [&mixed, &midpoint_cache, &split_edges](
                                  CellIndex first,
                                  CellIndex second) {
            const auto edge = ordered_edge(first, second);
            if (const auto found = midpoint_cache.find(edge);
                found != midpoint_cache.end()) {
                return found->second;
            }
            const CellIndex index = mixed.coordinates_.size();
            mixed.coordinates_.push_back(coordinate_from(
                vector_from(mixed.coordinates_[first]) +
                vector_from(mixed.coordinates_[second])));
            midpoint_cache.emplace(edge, index);
            split_edges.insert(edge);
            return index;
        };

        for (const Face face : faces) {
            if (!face_intersects_corridor(
                    face, mixed.coordinates_, route, width_radians)) {
                refined.push_back(face);
                continue;
            }
            const CellIndex first_second = midpoint(face[0U], face[1U]);
            const CellIndex second_third = midpoint(face[1U], face[2U]);
            const CellIndex third_first = midpoint(face[2U], face[0U]);
            refined.push_back({face[0U], first_second, third_first});
            refined.push_back({face[1U], second_third, first_second});
            refined.push_back({face[2U], third_first, second_third});
            refined.push_back({first_second, second_third, third_first});
        }
        faces = std::move(refined);
    }

    std::set<std::pair<CellIndex, CellIndex>> edges;
    for (const Face face : faces) {
        edges.insert(ordered_edge(face[0U], face[1U]));
        edges.insert(ordered_edge(face[1U], face[2U]));
        edges.insert(ordered_edge(face[2U], face[0U]));
    }
    for (const auto& split : split_edges) {
        edges.erase(split);
    }

    mixed.neighbors_.resize(mixed.coordinates_.size());
    for (const auto [first, second] : edges) {
        mixed.neighbors_[first].push_back(second);
        mixed.neighbors_[second].push_back(first);
    }
    mixed.leaf_face_count_ = faces.size();
    mixed.faces_ = std::move(faces);
    return mixed;
}

std::size_t MixedResolutionLattice::subdivision_level() const noexcept {
    return subdivision_level_;
}

std::size_t MixedResolutionLattice::vertex_count() const noexcept {
    return coordinates_.size();
}

std::size_t MixedResolutionLattice::coarse_vertex_count() const noexcept {
    return coarse_vertex_count_;
}

std::size_t MixedResolutionLattice::leaf_face_count() const noexcept {
    return leaf_face_count_;
}

Coordinate MixedResolutionLattice::coordinate(CellIndex cell) const noexcept {
    return coordinates_[cell];
}

std::span<const MixedResolutionLattice::CellIndex>
MixedResolutionLattice::neighbors(CellIndex cell) const noexcept {
    return neighbors_[cell];
}

std::optional<MixedResolutionLattice::CellIndex>
MixedResolutionLattice::nearest_cell(Coordinate coordinate) const noexcept {
    if (!std::isfinite(coordinate.latitude_degrees) ||
        !std::isfinite(coordinate.longitude_degrees) ||
        coordinate.latitude_degrees < -90.0 ||
        coordinate.latitude_degrees > 90.0 ||
        coordinate.longitude_degrees < -180.0 ||
        coordinate.longitude_degrees > 180.0) {
        return std::nullopt;
    }
    const Vector3 query = vector_from(coordinate);
    CellIndex closest = 0U;
    double closest_dot = dot(query, vector_from(coordinates_.front()));
    for (CellIndex cell = 1U; cell < coordinates_.size(); ++cell) {
        const double candidate = dot(query, vector_from(coordinates_[cell]));
        if (candidate > closest_dot) {
            closest = cell;
            closest_dot = candidate;
        }
    }
    return closest;
}

std::optional<MixedResolutionLattice::Face>
MixedResolutionLattice::containing_face(Coordinate coordinate) const noexcept {
    if (!std::isfinite(coordinate.latitude_degrees) ||
        !std::isfinite(coordinate.longitude_degrees) ||
        coordinate.latitude_degrees < -90.0 ||
        coordinate.latitude_degrees > 90.0 ||
        coordinate.longitude_degrees < -180.0 ||
        coordinate.longitude_degrees > 180.0) {
        return std::nullopt;
    }
    const Vector3 query = vector_from(coordinate);
    for (const Face face : faces_) {
        if (contains(
                query,
                vector_from(coordinates_[face[0U]]),
                vector_from(coordinates_[face[1U]]),
                vector_from(coordinates_[face[2U]]))) {
            return face;
        }
    }
    return std::nullopt;
}

double MixedResolutionLattice::maximum_neighbor_edge_length_nautical_miles(
    CellIndex cell) const noexcept {
    double maximum = 0.0;
    const Vector3 source = vector_from(coordinates_[cell]);
    for (const CellIndex neighbor : neighbors_[cell]) {
        maximum = std::max(
            maximum,
            angular_distance(source, vector_from(coordinates_[neighbor])) *
                earth_radius_nautical_miles);
    }
    return maximum;
}

}  // namespace sailroute::detail
