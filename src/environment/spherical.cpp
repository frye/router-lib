#include "environment/spherical.hpp"

#include "routing/geodesy.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace sailroute::environment_detail {
namespace {

// Directions closer than this are treated as the same point. One nanoradian is
// roughly a fifth of a millimetre on the earth's surface, far below any
// meaningful exclusion-zone tolerance and far above double rounding noise.
constexpr double coincident_radians = 1.0e-9;

bool on_minor_arc(
    UnitVector point,
    UnitVector from,
    UnitVector to,
    UnitVector normal) noexcept {
    if (angle_between(point, from) <= coincident_radians ||
        angle_between(point, to) <= coincident_radians) {
        return true;
    }
    return dot(cross(from, point), normal) >= 0.0 &&
        dot(cross(point, to), normal) >= 0.0;
}

}  // namespace

UnitVector to_unit_vector(Coordinate coordinate) noexcept {
    const double latitude =
        coordinate.latitude_degrees * std::numbers::pi / 180.0;
    const double longitude =
        coordinate.longitude_degrees * std::numbers::pi / 180.0;
    const double cos_latitude = std::cos(latitude);
    return UnitVector{
        cos_latitude * std::cos(longitude),
        cos_latitude * std::sin(longitude),
        std::sin(latitude)};
}

Coordinate to_coordinate(UnitVector vector) noexcept {
    const UnitVector unit = normalize(vector);
    const double latitude =
        std::asin(std::clamp(unit.z, -1.0, 1.0)) * 180.0 / std::numbers::pi;
    if (unit.x == 0.0 && unit.y == 0.0) {
        return Coordinate{latitude, 0.0};
    }
    const double longitude =
        std::atan2(unit.y, unit.x) * 180.0 / std::numbers::pi;
    return Coordinate{latitude, longitude};
}

UnitVector cross(UnitVector left, UnitVector right) noexcept {
    return UnitVector{
        left.y * right.z - left.z * right.y,
        left.z * right.x - left.x * right.z,
        left.x * right.y - left.y * right.x};
}

double dot(UnitVector left, UnitVector right) noexcept {
    return left.x * right.x + left.y * right.y + left.z * right.z;
}

double norm(UnitVector vector) noexcept {
    return std::sqrt(dot(vector, vector));
}

UnitVector normalize(UnitVector vector) noexcept {
    const double length = norm(vector);
    if (!(length > 0.0)) {
        return vector;
    }
    return UnitVector{vector.x / length, vector.y / length, vector.z / length};
}

UnitVector negate(UnitVector vector) noexcept {
    return UnitVector{-vector.x, -vector.y, -vector.z};
}

UnitVector add(UnitVector left, UnitVector right) noexcept {
    return UnitVector{
        left.x + right.x, left.y + right.y, left.z + right.z};
}

UnitVector scale(UnitVector vector, double factor) noexcept {
    return UnitVector{vector.x * factor, vector.y * factor, vector.z * factor};
}

double angle_between(UnitVector left, UnitVector right) noexcept {
    // atan2 of the cross and dot products stays accurate for both the small
    // and the near-antipodal angles that acos loses.
    return std::atan2(norm(cross(left, right)), dot(left, right));
}

UnitVector slerp(UnitVector from, UnitVector to, double fraction) noexcept {
    const double angle = angle_between(from, to);
    if (!(angle > coincident_radians)) {
        return from;
    }
    const double sine = std::sin(angle);
    const double first = std::sin((1.0 - fraction) * angle) / sine;
    const double second = std::sin(fraction * angle) / sine;
    return normalize(add(scale(from, first), scale(to, second)));
}

ArcIntersections intersect_arcs(
    UnitVector a,
    UnitVector b,
    UnitVector c,
    UnitVector d) noexcept {
    ArcIntersections found;
    const UnitVector first_normal = cross(a, b);
    const UnitVector second_normal = cross(c, d);
    if (norm(first_normal) <= coincident_radians ||
        norm(second_normal) <= coincident_radians) {
        return found;
    }

    const auto record = [&](UnitVector point) {
        for (std::size_t index = 0U; index < found.count; ++index) {
            if (angle_between(found.points[index], point) <=
                coincident_radians) {
                return;
            }
        }
        if (found.count < found.points.size()) {
            found.points[found.count++] = point;
        }
    };

    const UnitVector line = cross(first_normal, second_normal);
    if (norm(line) <= coincident_radians) {
        // The arcs share a great circle. Any endpoint of one lying on the
        // other is a genuine contact, which is what a shared boundary looks
        // like.
        if (on_minor_arc(c, a, b, first_normal)) {
            record(c);
        }
        if (on_minor_arc(d, a, b, first_normal)) {
            record(d);
        }
        if (on_minor_arc(a, c, d, second_normal)) {
            record(a);
        }
        if (on_minor_arc(b, c, d, second_normal)) {
            record(b);
        }
        return found;
    }

    const UnitVector candidate = normalize(line);
    for (const UnitVector point : {candidate, negate(candidate)}) {
        if (on_minor_arc(point, a, b, first_normal) &&
            on_minor_arc(point, c, d, second_normal)) {
            record(point);
        }
    }
    return found;
}

double distance_to_arc(
    UnitVector point,
    UnitVector from,
    UnitVector to) noexcept {
    const UnitVector normal = cross(from, to);
    if (norm(normal) <= coincident_radians) {
        return angle_between(point, from);
    }
    const UnitVector unit_normal = normalize(normal);
    // Foot of the perpendicular from the point onto the arc's great circle.
    const UnitVector projected =
        normalize(add(point, scale(unit_normal, -dot(point, unit_normal))));
    if (norm(projected) > 0.0 &&
        dot(cross(from, projected), normal) >= 0.0 &&
        dot(cross(projected, to), normal) >= 0.0) {
        return std::abs(std::asin(std::clamp(dot(point, unit_normal), -1.0, 1.0)));
    }
    return std::min(angle_between(point, from), angle_between(point, to));
}

}  // namespace sailroute::environment_detail
