#include "sailroute/environment.hpp"

#include "environment/spherical.hpp"
#include "routing/geodesy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace sailroute {
namespace {

using environment_detail::UnitVector;

// Contact tolerance for boundary tests, in radians of central angle. This is
// roughly two centimetres on the earth's surface, which is far below any
// meaningful exclusion tolerance and far above double rounding noise.
constexpr double boundary_tolerance_radians = 3.0e-9;

// A ring whose vertices average to nearly the origin bounds two hemispheres of
// equal claim, so its interior is undefined. Such rings are rejected instead of
// being resolved arbitrarily.
constexpr double minimum_centroid_norm = 1.0e-6;

// How far beyond a ring's bounding cap the containment ray's far end is placed.
// Keeping it off the cap centre's antipode means the ray from any enclosed
// point is a well-defined great-circle arc rather than a degenerate one.
constexpr double exterior_reference_margin_radians = 0.05;

// Deterministic rotations retried when a containment ray runs exactly through a
// ring vertex, where the crossing parity is ambiguous.
constexpr double reference_perturbations_radians[]{
    0.0, 0.013, 0.041, 0.097, 0.211};

struct PreparedRing {
    std::vector<UnitVector> vertices;
    /// Deterministically derived points known to lie outside the ring, used as
    /// the far end of the containment ray. Later entries are rotations of the
    /// first, tried only when a ray lands exactly on a vertex.
    std::vector<UnitVector> exterior_references;
    UnitVector cap_center;
    double cap_radius_radians{};
};

struct PreparedPolygon {
    PreparedRing outer;
    std::vector<PreparedRing> holes;
};

struct PreparedZone {
    std::vector<PreparedPolygon> polygons;
    UnitVector cap_center;
    double cap_radius_radians{};
    // Precomputed so the per-segment rejection below is a dot product and two
    // multiplications rather than an inverse trigonometric call per zone.
    double cap_cosine{1.0};
    double cap_sine{};
};

Error invalid_topology(std::string message) {
    return Error{ErrorCode::invalid_environment, std::move(message)};
}

Result<PreparedRing> prepare_ring(const ExclusionRing& ring) {
    std::vector<UnitVector> vertices;
    vertices.reserve(ring.vertices.size());
    for (const Coordinate vertex : ring.vertices) {
        if (!is_valid(vertex)) {
            return invalid_topology(
                "exclusion ring contains a coordinate outside canonical bounds");
        }
        const UnitVector unit = environment_detail::to_unit_vector(vertex);
        if (!vertices.empty() &&
            environment_detail::angle_between(vertices.back(), unit) <=
                boundary_tolerance_radians) {
            continue;
        }
        vertices.push_back(unit);
    }
    while (vertices.size() > 1U &&
           environment_detail::angle_between(vertices.front(), vertices.back()) <=
               boundary_tolerance_radians) {
        vertices.pop_back();
    }
    if (vertices.size() < 3U) {
        return invalid_topology(
            "exclusion ring must contain at least three distinct vertices");
    }

    UnitVector sum{};
    for (const UnitVector vertex : vertices) {
        sum = environment_detail::add(sum, vertex);
    }
    if (environment_detail::norm(sum) <
        minimum_centroid_norm * static_cast<double>(vertices.size())) {
        return invalid_topology(
            "exclusion ring vertices average to the sphere's centre, so the "
            "ring does not bound a well-defined interior");
    }

    PreparedRing prepared;
    prepared.cap_center = environment_detail::normalize(sum);
    double longest_edge = 0.0;
    for (std::size_t index = 0U; index < vertices.size(); ++index) {
        prepared.cap_radius_radians = std::max(
            prepared.cap_radius_radians,
            environment_detail::angle_between(
                prepared.cap_center, vertices[index]));
        longest_edge = std::max(
            longest_edge,
            environment_detail::angle_between(
                vertices[index], vertices[(index + 1U) % vertices.size()]));
    }
    // An edge bulges away from its endpoints, and no point on an arc is more
    // than half its length from the nearer endpoint, so this bounds the whole
    // ring rather than only its vertices.
    prepared.cap_radius_radians += longest_edge / 2.0;
    if (prepared.cap_radius_radians >=
        std::numbers::pi / 2.0 - exterior_reference_margin_radians) {
        return invalid_topology(
            "exclusion ring is not contained in a hemisphere, so its interior "
            "is ambiguous; split it into smaller components");
    }

    // The whole ring lies inside the cap, so everything outside the cap is on
    // the ring's exterior side. A reference just beyond the cap edge is
    // therefore outside the ring, and unlike the cap centre's antipode it is
    // never antipodal to an enclosed test point, which would collapse the
    // containment ray to a degenerate arc.
    UnitVector transverse = environment_detail::cross(
        prepared.cap_center, UnitVector{0.0, 0.0, 1.0});
    if (environment_detail::norm(transverse) < 1.0e-6) {
        transverse = environment_detail::cross(
            prepared.cap_center, UnitVector{1.0, 0.0, 0.0});
    }
    transverse = environment_detail::normalize(transverse);
    const UnitVector binormal = environment_detail::normalize(
        environment_detail::cross(transverse, prepared.cap_center));
    const double reference_angle =
        prepared.cap_radius_radians + exterior_reference_margin_radians;
    for (const double rotation : reference_perturbations_radians) {
        const UnitVector direction = environment_detail::add(
            environment_detail::scale(transverse, std::cos(rotation)),
            environment_detail::scale(binormal, std::sin(rotation)));
        prepared.exterior_references.push_back(environment_detail::normalize(
            environment_detail::add(
                environment_detail::scale(
                    prepared.cap_center, std::cos(reference_angle)),
                environment_detail::scale(direction, std::sin(reference_angle)))));
    }
    prepared.vertices = std::move(vertices);
    return prepared;
}

struct RingContainment {
    bool interior{false};
    bool boundary{false};
    std::size_t geometry_tests{};
};

RingContainment classify_point(const PreparedRing& ring, UnitVector point) {
    RingContainment containment;
    const std::size_t count = ring.vertices.size();
    for (std::size_t index = 0U; index < count; ++index) {
        const UnitVector from = ring.vertices[index];
        const UnitVector to = ring.vertices[(index + 1U) % count];
        ++containment.geometry_tests;
        if (environment_detail::distance_to_arc(point, from, to) <=
            boundary_tolerance_radians) {
            containment.boundary = true;
            return containment;
        }
    }
    if (environment_detail::angle_between(point, ring.cap_center) >
        ring.cap_radius_radians + boundary_tolerance_radians) {
        return containment;
    }

    // Even-odd containment along a ray to a point known to be outside. A ray
    // that lands exactly on a vertex has ambiguous parity, so a fixed sequence
    // of rotated references is tried until one of them is unambiguous.
    for (const UnitVector reference : ring.exterior_references) {
        std::size_t crossings = 0U;
        bool ambiguous = false;
        for (std::size_t index = 0U; index < count; ++index) {
            const UnitVector from = ring.vertices[index];
            const UnitVector to = ring.vertices[(index + 1U) % count];
            ++containment.geometry_tests;
            const environment_detail::ArcIntersections hits =
                environment_detail::intersect_arcs(point, reference, from, to);
            for (std::size_t hit = 0U; hit < hits.count; ++hit) {
                if (environment_detail::angle_between(
                        hits.points[hit], from) <= boundary_tolerance_radians ||
                    environment_detail::angle_between(hits.points[hit], to) <=
                        boundary_tolerance_radians) {
                    ambiguous = true;
                }
            }
            crossings += hits.count;
        }
        if (!ambiguous) {
            containment.interior = (crossings % 2U) == 1U;
            return containment;
        }
    }
    return containment;
}

struct PolygonContainment {
    bool interior{false};
    bool boundary{false};
    std::size_t geometry_tests{};
};

PolygonContainment classify_point(
    const PreparedPolygon& polygon,
    UnitVector point) {
    PolygonContainment containment;
    const RingContainment outer = classify_point(polygon.outer, point);
    containment.geometry_tests += outer.geometry_tests;
    if (outer.boundary) {
        containment.boundary = true;
        return containment;
    }
    if (!outer.interior) {
        return containment;
    }
    for (const PreparedRing& hole : polygon.holes) {
        const RingContainment inside_hole = classify_point(hole, point);
        containment.geometry_tests += inside_hole.geometry_tests;
        if (inside_hole.boundary) {
            containment.boundary = true;
            return containment;
        }
        if (inside_hole.interior) {
            return containment;
        }
    }
    containment.interior = true;
    return containment;
}

void collect_ring_crossings(
    const PreparedRing& ring,
    UnitVector from,
    UnitVector to,
    double segment_angle,
    std::vector<double>& parameters,
    std::size_t& geometry_tests) {
    const std::size_t count = ring.vertices.size();
    for (std::size_t index = 0U; index < count; ++index) {
        ++geometry_tests;
        const environment_detail::ArcIntersections hits =
            environment_detail::intersect_arcs(
                from,
                to,
                ring.vertices[index],
                ring.vertices[(index + 1U) % count]);
        for (std::size_t hit = 0U; hit < hits.count; ++hit) {
            const double along =
                environment_detail::angle_between(from, hits.points[hit]);
            parameters.push_back(
                segment_angle > 0.0 ? std::clamp(along / segment_angle, 0.0, 1.0)
                                    : 0.0);
        }
    }
}

}  // namespace

struct ExclusionZoneSet::Impl {
    std::vector<ExclusionZone> zones;
    std::vector<PreparedZone> prepared;
    ProviderMetadata metadata;
    std::uint64_t maximum_revision{};
};

ExclusionZoneSet::ExclusionZoneSet() = default;
ExclusionZoneSet::~ExclusionZoneSet() = default;
ExclusionZoneSet::ExclusionZoneSet(const ExclusionZoneSet&) = default;
ExclusionZoneSet::ExclusionZoneSet(ExclusionZoneSet&&) noexcept = default;
ExclusionZoneSet& ExclusionZoneSet::operator=(const ExclusionZoneSet&) = default;
ExclusionZoneSet& ExclusionZoneSet::operator=(ExclusionZoneSet&&) noexcept =
    default;

ExclusionZoneSet::ExclusionZoneSet(std::shared_ptr<const Impl> impl)
    : impl_(std::move(impl)) {}

Result<ExclusionZoneSet> ExclusionZoneSet::create(
    std::vector<ExclusionZone> zones,
    ProviderMetadata metadata) {
    if (metadata.name.empty() || metadata.source.empty()) {
        return Error{
            ErrorCode::invalid_environment,
            "exclusion metadata must carry a non-empty name and source"};
    }

    // Canonical ordering makes input order irrelevant to routing, diagnostics,
    // and the identifier reported for a rejection.
    std::sort(
        zones.begin(),
        zones.end(),
        [](const ExclusionZone& left, const ExclusionZone& right) {
            if (left.identifier != right.identifier) {
                return left.identifier < right.identifier;
            }
            return left.revision < right.revision;
        });

    auto impl = std::make_shared<Impl>();
    impl->prepared.reserve(zones.size());
    for (std::size_t index = 0U; index < zones.size(); ++index) {
        const ExclusionZone& zone = zones[index];
        if (zone.identifier.empty()) {
            return invalid_topology("exclusion zone identifier must not be empty");
        }
        if (index > 0U && zones[index - 1U].identifier == zone.identifier &&
            zones[index - 1U].revision == zone.revision) {
            return invalid_topology(
                "exclusion zone '" + zone.identifier +
                "' is repeated at revision " + std::to_string(zone.revision));
        }
        if (zone.active_from.has_value() && zone.active_until.has_value() &&
            *zone.active_until <= *zone.active_from) {
            return invalid_topology(
                "exclusion zone '" + zone.identifier +
                "' has an activation window that ends before it begins");
        }
        if (zone.polygons.empty()) {
            return invalid_topology(
                "exclusion zone '" + zone.identifier +
                "' contains no polygon component");
        }

        PreparedZone prepared_zone;
        prepared_zone.polygons.reserve(zone.polygons.size());
        UnitVector sum{};
        for (const ExclusionPolygon& polygon : zone.polygons) {
            auto outer = prepare_ring(polygon.outer);
            if (!outer) {
                return Error{
                    outer.error().code,
                    "exclusion zone '" + zone.identifier + "': " +
                        outer.error().message};
            }
            PreparedPolygon prepared_polygon;
            prepared_polygon.outer = std::move(outer.value());
            prepared_polygon.holes.reserve(polygon.holes.size());
            for (const ExclusionRing& hole : polygon.holes) {
                auto prepared_hole = prepare_ring(hole);
                if (!prepared_hole) {
                    return Error{
                        prepared_hole.error().code,
                        "exclusion zone '" + zone.identifier + "': " +
                            prepared_hole.error().message};
                }
                for (const UnitVector vertex : prepared_hole.value().vertices) {
                    const RingContainment containment =
                        classify_point(prepared_polygon.outer, vertex);
                    if (!containment.interior && !containment.boundary) {
                        return invalid_topology(
                            "exclusion zone '" + zone.identifier +
                            "' has a hole vertex outside its outer ring");
                    }
                }
                prepared_polygon.holes.push_back(std::move(prepared_hole.value()));
            }
            sum = environment_detail::add(
                sum, prepared_polygon.outer.cap_center);
            prepared_zone.polygons.push_back(std::move(prepared_polygon));
        }

        if (environment_detail::norm(sum) > 0.0) {
            prepared_zone.cap_center = environment_detail::normalize(sum);
        } else {
            prepared_zone.cap_center =
                prepared_zone.polygons.front().outer.cap_center;
        }
        for (const PreparedPolygon& polygon : prepared_zone.polygons) {
            prepared_zone.cap_radius_radians = std::max(
                prepared_zone.cap_radius_radians,
                environment_detail::angle_between(
                    prepared_zone.cap_center, polygon.outer.cap_center) +
                    polygon.outer.cap_radius_radians);
        }
        prepared_zone.cap_radius_radians = std::min(
            prepared_zone.cap_radius_radians, std::numbers::pi);
        prepared_zone.cap_cosine = std::cos(prepared_zone.cap_radius_radians);
        prepared_zone.cap_sine = std::sin(prepared_zone.cap_radius_radians);
        impl->maximum_revision =
            std::max(impl->maximum_revision, zone.revision);
        impl->prepared.push_back(std::move(prepared_zone));
    }

    impl->zones = std::move(zones);
    impl->metadata = std::move(metadata);
    return ExclusionZoneSet{std::move(impl)};
}

const ProviderMetadata& ExclusionZoneSet::metadata() const noexcept {
    static const ProviderMetadata empty{};
    return impl_ ? impl_->metadata : empty;
}

std::size_t ExclusionZoneSet::zone_count() const noexcept {
    return impl_ ? impl_->zones.size() : 0U;
}

std::uint64_t ExclusionZoneSet::maximum_revision() const noexcept {
    return impl_ ? impl_->maximum_revision : 0U;
}

const std::vector<ExclusionZone>& ExclusionZoneSet::zones() const noexcept {
    static const std::vector<ExclusionZone> empty;
    return impl_ ? impl_->zones : empty;
}

bool ExclusionZoneSet::contains(
    Coordinate coordinate,
    TimePoint time,
    ExclusionBoundaryPolicy policy) const {
    if (!impl_ || !is_valid(coordinate)) {
        return false;
    }
    const UnitVector point = environment_detail::to_unit_vector(coordinate);
    for (std::size_t index = 0U; index < impl_->zones.size(); ++index) {
        const ExclusionZone& zone = impl_->zones[index];
        if (zone.active_from.has_value() && time < *zone.active_from) {
            continue;
        }
        if (zone.active_until.has_value() && time >= *zone.active_until) {
            continue;
        }
        for (const PreparedPolygon& polygon : impl_->prepared[index].polygons) {
            const PolygonContainment containment =
                classify_point(polygon, point);
            if (containment.interior) {
                return true;
            }
            if (containment.boundary &&
                policy == ExclusionBoundaryPolicy::boundary_excluded) {
                return true;
            }
        }
    }
    return false;
}

ExclusionZoneSet::SegmentResult ExclusionZoneSet::intersects_segment(
    Coordinate from,
    TimePoint from_time,
    Coordinate to,
    TimePoint to_time,
    ExclusionBoundaryPolicy policy) const {
    SegmentResult result;
    if (!impl_ || impl_->zones.empty()) {
        return result;
    }
    if (!is_valid(from) || !is_valid(to)) {
        return result;
    }
    if (to_time < from_time) {
        std::swap(from, to);
        std::swap(from_time, to_time);
    }

    const UnitVector start = environment_detail::to_unit_vector(from);
    const UnitVector end = environment_detail::to_unit_vector(to);
    const double traversal_seconds =
        std::chrono::duration<double>(to_time - from_time).count();

    UnitVector segment_center = environment_detail::add(start, end);
    if (!(environment_detail::norm(segment_center) > 0.0)) {
        segment_center = start;
    }
    segment_center = environment_detail::normalize(segment_center);
    const double segment_radius = std::max(
        environment_detail::angle_between(segment_center, start),
        environment_detail::angle_between(segment_center, end));
    // A zone can only be reached when the two bounding caps overlap. Comparing
    // cosines keeps that rejection free of inverse trigonometry, which matters
    // because it runs once per zone for every transition the solver evaluates.
    const double reach = std::min(
        std::numbers::pi,
        segment_radius + boundary_tolerance_radians);
    const double reach_cosine = std::cos(reach);
    const double reach_sine = std::sin(reach);

    std::vector<double> parameters;
    for (std::size_t index = 0U; index < impl_->zones.size(); ++index) {
        const ExclusionZone& zone = impl_->zones[index];
        const PreparedZone& prepared = impl_->prepared[index];

        // Cheap bounding-cap rejection before anything else. This is the whole
        // spatial index: it runs once per zone for every transition the solver
        // evaluates, so it must stay free of inverse trigonometry.
        ++result.geometry_tests;
        const double overlap_cosine =
            reach_cosine * prepared.cap_cosine - reach_sine * prepared.cap_sine;
        if (environment_detail::dot(segment_center, prepared.cap_center) <
            overlap_cosine) {
            continue;
        }

        // Clip the traversal to the window during which the zone is active, so
        // a zone opening or closing mid-segment constrains only the part of the
        // segment actually sailed while it was open.
        double first_fraction = 0.0;
        double last_fraction = 1.0;
        if (zone.active_from.has_value() || zone.active_until.has_value()) {
            if (traversal_seconds > 0.0) {
                if (zone.active_from.has_value()) {
                    const double offset = std::chrono::duration<double>(
                                              *zone.active_from - from_time)
                                              .count();
                    first_fraction =
                        std::max(first_fraction, offset / traversal_seconds);
                }
                if (zone.active_until.has_value()) {
                    const double offset = std::chrono::duration<double>(
                                              *zone.active_until - from_time)
                                              .count();
                    last_fraction =
                        std::min(last_fraction, offset / traversal_seconds);
                }
                if (first_fraction >= last_fraction) {
                    continue;
                }
            } else {
                if (zone.active_from.has_value() &&
                    from_time < *zone.active_from) {
                    continue;
                }
                if (zone.active_until.has_value() &&
                    from_time >= *zone.active_until) {
                    continue;
                }
            }
        }

        const UnitVector clipped_start =
            first_fraction > 0.0
            ? environment_detail::slerp(start, end, first_fraction)
            : start;
        const UnitVector clipped_end = last_fraction < 1.0
            ? environment_detail::slerp(start, end, last_fraction)
            : end;

        const double clipped_angle =
            environment_detail::angle_between(clipped_start, clipped_end);
        for (const PreparedPolygon& polygon : prepared.polygons) {
            parameters.clear();
            std::size_t tests = 0U;
            collect_ring_crossings(
                polygon.outer,
                clipped_start,
                clipped_end,
                clipped_angle,
                parameters,
                tests);
            for (const PreparedRing& hole : polygon.holes) {
                collect_ring_crossings(
                    hole,
                    clipped_start,
                    clipped_end,
                    clipped_angle,
                    parameters,
                    tests);
            }
            result.geometry_tests += tests;

            const PolygonContainment start_containment =
                classify_point(polygon, clipped_start);
            const PolygonContainment end_containment =
                classify_point(polygon, clipped_end);
            result.geometry_tests +=
                start_containment.geometry_tests + end_containment.geometry_tests;

            bool violated = start_containment.interior ||
                end_containment.interior;
            if (!violated &&
                policy == ExclusionBoundaryPolicy::boundary_excluded) {
                violated = !parameters.empty() || start_containment.boundary ||
                    end_containment.boundary;
            }
            if (!violated && !parameters.empty()) {
                // Boundary contact is permitted, so a crossing only matters
                // when some interval between consecutive contacts is strictly
                // inside the zone.
                parameters.push_back(0.0);
                parameters.push_back(1.0);
                std::sort(parameters.begin(), parameters.end());
                for (std::size_t position = 1U; position < parameters.size();
                     ++position) {
                    const double middle =
                        0.5 * (parameters[position - 1U] + parameters[position]);
                    if (!(parameters[position] - parameters[position - 1U] >
                          0.0)) {
                        continue;
                    }
                    const PolygonContainment inside = classify_point(
                        polygon,
                        environment_detail::slerp(
                            clipped_start, clipped_end, middle));
                    result.geometry_tests += inside.geometry_tests;
                    if (inside.interior) {
                        violated = true;
                        break;
                    }
                }
            }

            if (violated) {
                result.violated = true;
                result.zone_identifier = zone.identifier;
                return result;
            }
        }
    }
    return result;
}

}  // namespace sailroute
