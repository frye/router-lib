#include "routing/lattice.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace sailroute::detail {
namespace {

constexpr double earth_radius_nautical_miles = 3440.065;

struct Vector3 {
    double x;
    double y;
    double z;
};

double dot(Vector3 left, Vector3 right) noexcept {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

Vector3 cross(Vector3 left, Vector3 right) noexcept {
    return {
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x};
}

Vector3 normalized(Vector3 vector) noexcept {
    const double length = std::sqrt(
        vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
    return Vector3{vector.x / length, vector.y / length, vector.z / length};
}

Coordinate coordinate_from(Vector3 vector) noexcept {
    constexpr double radians_to_degrees = 180.0 / std::numbers::pi;
    const double latitude = std::asin(std::clamp(vector.z, -1.0, 1.0));
    double longitude = std::atan2(vector.y, vector.x) * radians_to_degrees;
    if (longitude > 180.0) {
        longitude -= 360.0;
    }
    if (longitude < -180.0) {
        longitude += 360.0;
    }
    return Coordinate{latitude * radians_to_degrees, longitude};
}

bool is_valid_coordinate(Coordinate coordinate) noexcept {
    return std::isfinite(coordinate.latitude_degrees) &&
           std::isfinite(coordinate.longitude_degrees) &&
           coordinate.latitude_degrees >= -90.0 &&
           coordinate.latitude_degrees <= 90.0 &&
           coordinate.longitude_degrees >= -180.0 &&
           coordinate.longitude_degrees <= 180.0;
}

Vector3 vector_from(Coordinate coordinate) noexcept {
    constexpr double degrees_to_radians = std::numbers::pi / 180.0;
    const double latitude = coordinate.latitude_degrees * degrees_to_radians;
    const double longitude = coordinate.longitude_degrees * degrees_to_radians;
    const double cosine_latitude = std::cos(latitude);
    return Vector3{
        cosine_latitude * std::cos(longitude),
        cosine_latitude * std::sin(longitude),
        std::sin(latitude)};
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

std::size_t vertex_count_for_level(std::size_t level) noexcept {
    return 10U * (std::size_t{1U} << (2U * level)) + 2U;
}

}  // namespace

Result<GeodesicLattice> GeodesicLattice::create(std::size_t subdivision_level) {
    if (subdivision_level > maximum_subdivision_level) {
        return Error{
            ErrorCode::invalid_argument,
            "geodesic lattice subdivision level exceeds the supported maximum of " +
                std::to_string(maximum_subdivision_level)};
    }
    return GeodesicLattice{subdivision_level};
}

GeodesicLattice::GeodesicLattice(std::size_t subdivision_level)
    : subdivision_level_(subdivision_level) {
    const double golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;
    std::vector<Vector3> vertices{
        {-1.0, golden_ratio, 0.0},
        {1.0, golden_ratio, 0.0},
        {-1.0, -golden_ratio, 0.0},
        {1.0, -golden_ratio, 0.0},
        {0.0, -1.0, golden_ratio},
        {0.0, 1.0, golden_ratio},
        {0.0, -1.0, -golden_ratio},
        {0.0, 1.0, -golden_ratio},
        {golden_ratio, 0.0, -1.0},
        {golden_ratio, 0.0, 1.0},
        {-golden_ratio, 0.0, -1.0},
        {-golden_ratio, 0.0, 1.0},
    };
    for (Vector3& vertex : vertices) {
        vertex = normalized(vertex);
    }

    std::vector<Face> faces{
        Face{0U, 11U, 5U}, Face{0U, 5U, 1U}, Face{0U, 1U, 7U},
        Face{0U, 7U, 10U},
        {0U, 10U, 11U}, {1U, 5U, 9U}, {5U, 11U, 4U}, {11U, 10U, 2U},
        {10U, 7U, 6U}, {7U, 1U, 8U}, {3U, 9U, 4U}, {3U, 4U, 2U},
        {3U, 2U, 6U}, {3U, 6U, 8U}, {3U, 8U, 9U}, {4U, 9U, 5U},
        {2U, 4U, 11U}, {6U, 2U, 10U}, {8U, 6U, 7U}, {9U, 8U, 1U},
    };

    for (std::size_t level = 0U; level < subdivision_level_; ++level) {
        std::map<std::pair<CellIndex, CellIndex>, CellIndex> midpoint_cache;
        std::vector<Face> subdivided_faces;
        subdivided_faces.reserve(faces.size() * 4U);
        const auto midpoint = [&vertices, &midpoint_cache](
                                  CellIndex first,
                                  CellIndex second) -> CellIndex {
            const std::pair<CellIndex, CellIndex> edge{
                std::min(first, second), std::max(first, second)};
            if (const auto found = midpoint_cache.find(edge);
                found != midpoint_cache.end()) {
                return found->second;
            }
            const Vector3& left = vertices[first];
            const Vector3& right = vertices[second];
            const CellIndex index = vertices.size();
            vertices.push_back(normalized(Vector3{
                left.x + right.x,
                left.y + right.y,
                left.z + right.z}));
            midpoint_cache.emplace(edge, index);
            return index;
        };

        for (const Face face : faces) {
            const CellIndex first_second = midpoint(face[0U], face[1U]);
            const CellIndex second_third = midpoint(face[1U], face[2U]);
            const CellIndex third_first = midpoint(face[2U], face[0U]);
            subdivided_faces.push_back({face[0U], first_second, third_first});
            subdivided_faces.push_back({face[1U], second_third, first_second});
            subdivided_faces.push_back({face[2U], third_first, second_third});
            subdivided_faces.push_back({first_second, second_third, third_first});
        }
        faces = std::move(subdivided_faces);
    }

    coordinates_.reserve(vertices.size());
    unit_vectors_.reserve(vertices.size() * 3U);
    for (const Vector3 vertex : vertices) {
        coordinates_.push_back(coordinate_from(vertex));
        unit_vectors_.push_back(vertex.x);
        unit_vectors_.push_back(vertex.y);
        unit_vectors_.push_back(vertex.z);
    }

    neighbors_.resize(vertices.size());
    const auto add_edge = [this](CellIndex first, CellIndex second) {
        neighbors_[first].push_back(second);
        neighbors_[second].push_back(first);
    };
    for (const Face face : faces) {
        add_edge(face[0U], face[1U]);
        add_edge(face[1U], face[2U]);
        add_edge(face[2U], face[0U]);
    }
    for (std::vector<CellIndex>& adjacent : neighbors_) {
        std::sort(adjacent.begin(), adjacent.end());
        adjacent.erase(std::unique(adjacent.begin(), adjacent.end()), adjacent.end());
    }

    for (CellIndex first = 0U; first < neighbors_.size(); ++first) {
        for (const CellIndex second : neighbors_[first]) {
            if (first >= second) {
                continue;
            }
            const double dot_product =
                unit_vectors_[3U * first] * unit_vectors_[3U * second] +
                unit_vectors_[3U * first + 1U] * unit_vectors_[3U * second + 1U] +
                unit_vectors_[3U * first + 2U] * unit_vectors_[3U * second + 2U];
            const double length = earth_radius_nautical_miles *
                std::acos(std::clamp(dot_product, -1.0, 1.0));
            maximum_neighbor_edge_length_nautical_miles_ =
                std::max(maximum_neighbor_edge_length_nautical_miles_, length);
        }
    }
    faces_ = std::move(faces);
}

std::size_t GeodesicLattice::subdivision_level() const noexcept {
    return subdivision_level_;
}

std::size_t GeodesicLattice::vertex_count() const noexcept {
    return coordinates_.size();
}

Coordinate GeodesicLattice::coordinate(CellIndex cell) const noexcept {
    return coordinates_[cell];
}

std::span<const GeodesicLattice::CellIndex> GeodesicLattice::neighbors(
    CellIndex cell) const noexcept {
    return neighbors_[cell];
}

std::span<const GeodesicLattice::Face> GeodesicLattice::faces() const noexcept {
    return faces_;
}

std::optional<GeodesicLattice::CellIndex> GeodesicLattice::nearest_cell(
    Coordinate coordinate) const noexcept {
    if (!is_valid_coordinate(coordinate)) {
        return std::nullopt;
    }
    const Vector3 query = vector_from(coordinate);
    CellIndex closest = 0U;
    double closest_dot_product =
        query.x * unit_vectors_[0U] + query.y * unit_vectors_[1U] +
        query.z * unit_vectors_[2U];
    for (CellIndex cell = 1U; cell < vertex_count(); ++cell) {
        const double dot_product =
            query.x * unit_vectors_[3U * cell] +
            query.y * unit_vectors_[3U * cell + 1U] +
            query.z * unit_vectors_[3U * cell + 2U];
        if (dot_product > closest_dot_product) {
            closest = cell;
            closest_dot_product = dot_product;
        }
    }
    return closest;
}

std::optional<GeodesicLattice::Face> GeodesicLattice::containing_face(
    Coordinate coordinate) const noexcept {
    if (!is_valid_coordinate(coordinate)) {
        return std::nullopt;
    }
    const Vector3 query = vector_from(coordinate);
    for (const Face face : faces_) {
        const auto vertex = [this](CellIndex cell) {
            return Vector3{
                unit_vectors_[3U * cell],
                unit_vectors_[3U * cell + 1U],
                unit_vectors_[3U * cell + 2U]};
        };
        if (contains(
                query,
                vertex(face[0U]),
                vertex(face[1U]),
                vertex(face[2U]))) {
            return face;
        }
    }
    return std::nullopt;
}

double GeodesicLattice::maximum_neighbor_edge_length_nautical_miles() const noexcept {
    return maximum_neighbor_edge_length_nautical_miles_;
}

double GeodesicLattice::maximum_neighbor_edge_length_nautical_miles(
    CellIndex cell) const noexcept {
    double maximum = 0.0;
    for (const CellIndex neighbor : neighbors_[cell]) {
        const double dot_product =
            unit_vectors_[3U * cell] * unit_vectors_[3U * neighbor] +
            unit_vectors_[3U * cell + 1U] * unit_vectors_[3U * neighbor + 1U] +
            unit_vectors_[3U * cell + 2U] * unit_vectors_[3U * neighbor + 2U];
        maximum = std::max(
            maximum,
            earth_radius_nautical_miles *
                std::acos(std::clamp(dot_product, -1.0, 1.0)));
    }
    return maximum;
}

std::optional<GeodesicLattice::CellIndex> GeodesicLattice::coincident_cell_at_level(
    CellIndex cell,
    std::size_t level) const noexcept {
    if (level > subdivision_level_ || cell >= vertex_count() ||
        cell >= vertex_count_for_level(level)) {
        return std::nullopt;
    }
    return cell;
}

}  // namespace sailroute::detail
