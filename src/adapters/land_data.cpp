#include "sailroute/land_data.hpp"

#include "environment/spherical.hpp"
#include "routing/geodesy.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sailroute {
namespace {

using environment_detail::UnitVector;

constexpr double degrees_to_radians = std::numbers::pi / 180.0;
constexpr double nautical_miles_per_degree =
    detail::earth_radius_nautical_miles * degrees_to_radians;
constexpr double numerical_allowance_nautical_miles = 1.0e-5;
constexpr std::size_t maximum_records = 250000U;

Error invalid_data(std::string message) {
    return {ErrorCode::invalid_environment, "GSHHG: " + std::move(message)};
}

Error resource_error() {
    return {ErrorCode::resource_limit, "GSHHG: source, grid, or geometry work budget exceeded"};
}

struct WorkBudget {
    std::size_t remaining{};

    bool consume(std::size_t amount = 1U) {
        if (amount > remaining) {
            return false;
        }
        remaining -= amount;
        return true;
    }
};

struct Region {
    EnvironmentGridSpec grid;
    double halo_south{};
    double halo_north{};
    double halo_west{};
    double halo_east{};
    double interpolation_error{};
};

Result<Region> make_region(const LandDataOptions& options) {
    const GeographicBounds& bounds = options.bounds;
    const std::array values{
        bounds.south_latitude_degrees, bounds.north_latitude_degrees,
        bounds.west_longitude_degrees, bounds.east_longitude_degrees,
        options.resolution_nautical_miles, options.distance_cap_nautical_miles};
    if (!std::all_of(values.begin(), values.end(), [](double value) {
            return std::isfinite(value);
        })) {
        return Error{ErrorCode::invalid_argument, "GSHHG: bounds and resolution must be finite"};
    }
    if (bounds.south_latitude_degrees < -85.0 ||
        bounds.north_latitude_degrees > 85.0 ||
        bounds.south_latitude_degrees >= bounds.north_latitude_degrees ||
        bounds.west_longitude_degrees < -180.0 ||
        bounds.west_longitude_degrees > 180.0 ||
        bounds.east_longitude_degrees < -180.0 ||
        bounds.east_longitude_degrees > 180.0) {
        return Error{ErrorCode::invalid_argument, "GSHHG: invalid canonical regional bounds (latitude must be within +/-85)"};
    }
    const double latitude_span =
        bounds.north_latitude_degrees - bounds.south_latitude_degrees;
    double longitude_span =
        bounds.east_longitude_degrees - bounds.west_longitude_degrees;
    if (longitude_span < 0.0) {
        longitude_span += 360.0;
    }
    if (latitude_span > 120.0 || longitude_span <= 0.0 ||
        longitude_span > 120.0 ||
        options.resolution_nautical_miles < 0.05 ||
        options.resolution_nautical_miles > 120.0 ||
        options.distance_cap_nautical_miles < 1.0 ||
        options.distance_cap_nautical_miles > 600.0) {
        return Error{ErrorCode::invalid_argument, "GSHHG: regional spans must be in (0,120] degrees, resolution in [0.05,120] nm, and distance cap in [1,600] nm"};
    }
    if (options.maximum_grid_nodes < 4U || options.maximum_grid_nodes > 250000U ||
        options.maximum_source_points < 3U ||
        options.maximum_source_points > 10000000U ||
        options.maximum_geometry_tests == 0U ||
        options.maximum_geometry_tests > 100000000U) {
        return Error{ErrorCode::invalid_argument, "GSHHG: resource budgets must be positive and cannot exceed the documented defaults"};
    }
    const double target_step =
        options.resolution_nautical_miles / nautical_miles_per_degree;
    const double rows = std::ceil(latitude_span / target_step) + 1.0;
    const double columns = std::ceil(longitude_span / target_step) + 1.0;
    if (rows * columns > static_cast<double>(options.maximum_grid_nodes)) {
        return resource_error();
    }
    Region region;
    region.grid.south_latitude_degrees = bounds.south_latitude_degrees;
    region.grid.west_longitude_degrees = bounds.west_longitude_degrees;
    region.grid.latitude_count = static_cast<std::size_t>(rows);
    region.grid.longitude_count = static_cast<std::size_t>(columns);
    region.grid.latitude_step_degrees =
        latitude_span / static_cast<double>(region.grid.latitude_count - 1U);
    region.grid.longitude_step_degrees =
        longitude_span / static_cast<double>(region.grid.longitude_count - 1U);

    // A meridian-then-parallel path bounds the distance between ANY two cell
    // points, including at a dateline crossing. A convex bilinear combination
    // of samples of a 1-Lipschitz field differs by at most this cell diameter.
    region.interpolation_error = nautical_miles_per_degree *
        (region.grid.latitude_step_degrees + region.grid.longitude_step_degrees) +
        numerical_allowance_nautical_miles;
    if (options.distance_cap_nautical_miles <= region.interpolation_error) {
        return Error{ErrorCode::invalid_argument, "GSHHG: distance cap must exceed the grid interpolation error; refine resolution or increase the cap"};
    }
    const double halo_degrees =
        (options.distance_cap_nautical_miles + numerical_allowance_nautical_miles) /
        nautical_miles_per_degree;
    region.halo_south = bounds.south_latitude_degrees - halo_degrees;
    region.halo_north = bounds.north_latitude_degrees + halo_degrees;
    if (region.halo_south <= -90.0 || region.halo_north >= 90.0) {
        return Error{ErrorCode::invalid_argument, "GSHHG: regional distance halo reaches an unsupported pole"};
    }
    const double halo_latitude =
        std::max(std::abs(region.halo_south), std::abs(region.halo_north));
    const double longitude_halo =
        halo_degrees / std::cos(halo_latitude * degrees_to_radians);
    region.halo_west = bounds.west_longitude_degrees - longitude_halo;
    region.halo_east =
        bounds.west_longitude_degrees + longitude_span + longitude_halo;
    return region;
}

std::int32_t signed_word(const unsigned char* bytes) {
    const std::uint32_t value =
        (static_cast<std::uint32_t>(bytes[0]) << 24U) |
        (static_cast<std::uint32_t>(bytes[1]) << 16U) |
        (static_cast<std::uint32_t>(bytes[2]) << 8U) |
        static_cast<std::uint32_t>(bytes[3]);
    return std::bit_cast<std::int32_t>(value);
}

struct Vertex {
    double longitude{};
    double latitude{};
    UnitVector unit;
};

struct Edge {
    Vertex from;
    Vertex to;
    UnitVector normal;
    double south{};
    double north{};
    bool near_region{false};
};

struct Ring {
    std::int32_t id{};
    std::int32_t parent_id{};
    unsigned int level{};
    double west{};
    double east{};
    std::vector<Edge> edges;
    std::size_t parent{};
    std::size_t root{};
};

struct RecordIdentity {
    std::int32_t parent{};
    unsigned int level{};
    std::optional<std::size_t> retained;
};

struct Source {
    std::vector<Ring> rings;
    std::unordered_map<std::int32_t, RecordIdentity> identities;
    unsigned int version{};
};

bool longitude_intersects(double west, double east, const Region& region) {
    if (region.halo_east - region.halo_west >= 360.0) {
        return true;
    }
    const double shift = 360.0 * std::ceil((region.halo_west - east) / 360.0);
    return west + shift <= region.halo_east;
}

bool in_header_longitude(double longitude, double west, double east) {
    const double shift = 360.0 * std::ceil((west - longitude) / 360.0);
    return longitude + shift <= east + 1.0e-8;
}

Result<Source> read_source(
    const std::filesystem::path& path,
    const Region& region,
    const LandDataOptions& options,
    WorkBudget& work) {
    std::error_code file_error;
    if (!std::filesystem::is_regular_file(path, file_error)) {
        return Error{ErrorCode::file_io, "GSHHG: cannot open regular shoreline file: " + path.string()};
    }
    const std::uintmax_t file_size = std::filesystem::file_size(path, file_error);
    if (file_error) {
        return Error{ErrorCode::file_io, "GSHHG: cannot determine shoreline file size: " + path.string()};
    }
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return Error{ErrorCode::file_io, "GSHHG: cannot open shoreline file: " + path.string()};
    }
    if (file_size == 0U) {
        return invalid_data("empty shoreline file");
    }
    if (file_size > static_cast<std::uintmax_t>(options.maximum_source_points) * 8U +
            static_cast<std::uintmax_t>(maximum_records) * 44U) {
        return resource_error();
    }

    Source source;
    std::uintmax_t remaining = file_size;
    std::size_t point_count = 0U;
    while (remaining != 0U) {
        if (source.identities.size() >= maximum_records) {
            return resource_error();
        }
        if (remaining < 44U) {
            return invalid_data("truncated record header at byte " + std::to_string(file_size - remaining));
        }
        std::array<unsigned char, 44U> bytes{};
        if (!input.read(reinterpret_cast<char*>(bytes.data()), 44)) {
            return invalid_data("failed reading record header");
        }
        remaining -= 44U;
        std::array<std::int32_t, 11U> header{};
        for (std::size_t field = 0U; field < header.size(); ++field) {
            header[field] = signed_word(bytes.data() + field * 4U);
        }
        const std::int32_t id = header[0];
        const std::uint32_t flags = std::bit_cast<std::uint32_t>(header[2]);
        const unsigned int level = flags & 255U;
        const unsigned int version = (flags >> 8U) & 255U;
        const std::string context = "record " + std::to_string(id) + ": ";
        if (id < 0 || header[1] < 3 || source.identities.contains(id)) {
            return invalid_data(context + "invalid/duplicate id or point count");
        }
        // Modern native files contain 11 big-endian words. Old 9-word headers,
        // unknown revisions and reserved encodings cannot be decoded safely.
        if (version < 7U || version > 16U || level < 1U || level > 6U ||
            (flags & ~std::uint32_t{0x0303ffffU}) != 0U) {
            return invalid_data(context + "unsupported native version, level, flags, or byte order");
        }
        if (source.version != 0U && source.version != version) {
            return invalid_data(context + "mixed native format versions");
        }
        source.version = version;
        if (header[3] < -180000000 || header[4] > 360000000 ||
            header[3] > header[4] ||
            static_cast<std::int64_t>(header[4]) - header[3] > 360000000 ||
            header[5] < -90000000 || header[6] > 90000000 ||
            header[5] > header[6] || header[7] < 0 || header[8] < 0 ||
            header[9] < -1 || header[10] < -1 ||
            ((level == 1U || level >= 5U) && header[9] != -1) ||
            (level >= 2U && level <= 4U && header[9] < 0)) {
            return invalid_data(context + "invalid bounds, area, or hierarchy fields");
        }
        const std::size_t count = static_cast<std::size_t>(header[1]);
        if (count > options.maximum_source_points - point_count) {
            return resource_error();
        }
        if (static_cast<std::uintmax_t>(count) * 8U > remaining) {
            return invalid_data(context + "truncated coordinate data");
        }
        point_count += count;
        std::vector<Vertex> vertices;
        vertices.reserve(count);
        double minimum_longitude = std::numeric_limits<double>::infinity();
        double maximum_longitude = -std::numeric_limits<double>::infinity();
        double south = 90.0;
        double north = -90.0;
        double longest_edge_degrees = 0.0;
        bool unsupported_edge = false;
        bool has_pole = false;
        for (std::size_t point = 0U; point < count; ++point) {
            std::array<unsigned char, 8U> coordinates{};
            if (!input.read(reinterpret_cast<char*>(coordinates.data()), 8)) {
                return invalid_data(context + "failed reading coordinate data");
            }
            remaining -= 8U;
            const std::int32_t x = signed_word(coordinates.data());
            const std::int32_t y = signed_word(coordinates.data() + 4U);
            if (x < 0 || x > 360000000 || y < header[5] || y > header[6] ||
                !in_header_longitude(
                    static_cast<double>(x) / 1000000.0,
                    static_cast<double>(header[3]) / 1000000.0,
                    static_cast<double>(header[4]) / 1000000.0)) {
                return invalid_data(context + "coordinate outside native/header bounds");
            }
            Vertex vertex;
            vertex.latitude = static_cast<double>(y) / 1000000.0;
            const double raw_longitude = static_cast<double>(x) / 1000000.0;
            vertex.longitude = std::remainder(raw_longitude, 360.0);
            if (!vertices.empty()) {
                const double delta =
                    std::remainder(raw_longitude - vertices.back().longitude, 360.0);
                unsupported_edge = unsupported_edge || std::abs(delta) >= 180.0;
                vertex.longitude = vertices.back().longitude + delta;
            }
            vertex.unit = environment_detail::to_unit_vector(
                {vertex.latitude, std::remainder(vertex.longitude, 360.0)});
            has_pole = has_pole || std::abs(vertex.latitude) == 90.0;
            if (!vertices.empty()) {
                if (!work.consume()) {
                    return resource_error();
                }
                const double edge_angle =
                    environment_detail::angle_between(vertices.back().unit, vertex.unit);
                if (edge_angle < 1.0e-12) {
                    continue;
                }
                unsupported_edge = unsupported_edge ||
                    edge_angle >= std::numbers::pi - 1.0e-9;
                longest_edge_degrees =
                    std::max(longest_edge_degrees, edge_angle / degrees_to_radians);
            }
            minimum_longitude = std::min(minimum_longitude, vertex.longitude);
            maximum_longitude = std::max(maximum_longitude, vertex.longitude);
            south = std::min(south, vertex.latitude);
            north = std::max(north, vertex.latitude);
            vertices.push_back(vertex);
        }
        while (vertices.size() > 1U &&
               environment_detail::angle_between(
                   vertices.front().unit, vertices.back().unit) < 1.0e-12) {
            vertices.pop_back();
        }
        if (vertices.size() < 3U) {
            return invalid_data(context + "ring has fewer than three distinct vertices");
        }
        const double closing_delta = std::remainder(
            vertices.front().longitude - vertices.back().longitude, 360.0);
        const double closing_angle =
            environment_detail::angle_between(vertices.back().unit, vertices.front().unit);
        longest_edge_degrees =
            std::max(longest_edge_degrees, closing_angle / degrees_to_radians);
        unsupported_edge = unsupported_edge || std::abs(closing_delta) >= 180.0 ||
            closing_angle >= std::numbers::pi - 1.0e-9;
        const bool winds_around_pole =
            std::abs(vertices.back().longitude + closing_delta -
                     vertices.front().longitude) > 1.0e-6;
        const bool polar = has_pole || winds_around_pole || level >= 5U;
        if (polar) {
            // A polar ring's interior extends beyond its vertex latitude box.
            if (south + north <= 0.0 || level >= 5U) {
                south = -90.0;
            } else {
                north = 90.0;
            }
        }
        // Every arc is within half its length of an endpoint, including its
        // possible poleward bulge. Header boxes alone would miss that bulge.
        south -= longest_edge_degrees * 0.5;
        north += longest_edge_degrees * 0.5;
        const bool latitude_relevant =
            south <= region.halo_north && north >= region.halo_south;
        if (latitude_relevant &&
            (polar || unsupported_edge ||
             maximum_longitude - minimum_longitude >= 360.0)) {
            return invalid_data(context + "unsupported polar, Antarctica, antipodal, or globe-winding representation affects the region/halo");
        }
        RecordIdentity identity{header[9], level, std::nullopt};
        if (latitude_relevant &&
            longitude_intersects(minimum_longitude, maximum_longitude, region)) {
            Ring ring;
            ring.id = id;
            ring.level = level;
            ring.parent_id = header[9];
            ring.west = minimum_longitude;
            ring.east = maximum_longitude;
            ring.edges.reserve(vertices.size());
            bool non_collinear = false;
            const UnitVector first_normal = environment_detail::normalize(
                environment_detail::cross(vertices[0].unit, vertices[1].unit));
            for (std::size_t edge = 0U; edge < vertices.size(); ++edge) {
                if (!work.consume()) {
                    return resource_error();
                }
                const Vertex& from = vertices[edge];
                const Vertex& to = vertices[(edge + 1U) % vertices.size()];
                const double half_length_degrees =
                    environment_detail::angle_between(from.unit, to.unit) /
                    (2.0 * degrees_to_radians);
                Edge prepared{from, to, environment_detail::cross(from.unit, to.unit),
                    std::min(from.latitude, to.latitude) - half_length_degrees,
                    std::max(from.latitude, to.latitude) + half_length_degrees, false};
                prepared.near_region =
                    prepared.south <= region.halo_north &&
                    prepared.north >= region.halo_south &&
                    longitude_intersects(
                        std::min(from.longitude, to.longitude),
                        std::max(from.longitude, to.longitude), region);
                non_collinear = non_collinear ||
                    std::abs(environment_detail::dot(first_normal, to.unit)) > 1.0e-12;
                ring.edges.push_back(prepared);
            }
            if (!non_collinear) {
                return invalid_data(context + "degenerate zero-area ring");
            }
            identity.retained = source.rings.size();
            source.rings.push_back(std::move(ring));
        }
        source.identities.emplace(id, identity);
    }
    if (input.peek() != std::char_traits<char>::eof() || input.bad()) {
        return invalid_data("file changed or failed during decoding");
    }
    for (const auto& [id, identity] : source.identities) {
        if (identity.level >= 2U && identity.level <= 4U) {
            const auto parent = source.identities.find(identity.parent);
            if (parent == source.identities.end() ||
                parent->second.level + 1U != identity.level) {
                return invalid_data("record " + std::to_string(id) + ": missing or inconsistent hierarchy parent");
            }
        }
    }
    for (unsigned int level = 1U; level <= 4U; ++level) {
        for (std::size_t index = 0U; index < source.rings.size(); ++index) {
            Ring& ring = source.rings[index];
            if (ring.level != level) {
                continue;
            }
            ring.root = index;
            if (level != 1U) {
                const auto& parent = source.identities.at(ring.parent_id);
                if (!parent.retained) {
                    return invalid_data("record " + std::to_string(ring.id) + ": regional ring has no retained enclosing parent");
                }
                ring.parent = *parent.retained;
                ring.root = source.rings[ring.parent].root;
            }
        }
    }
    return source;
}

// Longitude on a non-polar minor arc is monotone. Intersect its great-circle
// plane with the query meridian; parity above the point is independent of
// winding orientation and works on an unwrapped Greenwich/dateline ring.
bool crosses_northward_ray(const Edge& edge, double longitude, double latitude) {
    if ((edge.from.longitude > longitude) == (edge.to.longitude > longitude)) {
        return false;
    }
    const double radians = longitude * degrees_to_radians;
    double crossing = std::atan2(
        -(edge.normal.x * std::cos(radians) +
          edge.normal.y * std::sin(radians)),
        edge.normal.z);
    if (crossing > std::numbers::pi / 2.0) {
        crossing -= std::numbers::pi;
    } else if (crossing < -std::numbers::pi / 2.0) {
        crossing += std::numbers::pi;
    }
    return crossing / degrees_to_radians > latitude;
}

Result<std::vector<double>> sample_source(
    const Source& source,
    const Region& region,
    const LandDataOptions& options,
    WorkBudget& work) {
    const EnvironmentGridSpec& grid = region.grid;
    std::vector<double> samples(grid.latitude_count * grid.longitude_count);
    std::vector<bool> interiors(source.rings.size());
    std::vector<bool> active(source.rings.size());
    std::vector<unsigned int> deepest(source.rings.size());
    for (std::size_t row = 0U; row < grid.latitude_count; ++row) {
        const double latitude = grid.south_latitude_degrees +
            static_cast<double>(row) * grid.latitude_step_degrees;
        for (std::size_t column = 0U; column < grid.longitude_count; ++column) {
            const double longitude = grid.west_longitude_degrees +
                static_cast<double>(column) * grid.longitude_step_degrees;
            const UnitVector point = environment_detail::to_unit_vector(
                {latitude, std::remainder(longitude, 360.0)});
            double distance = options.distance_cap_nautical_miles;
            std::fill(active.begin(), active.end(), false);
            std::fill(deepest.begin(), deepest.end(), 0U);
            for (std::size_t index = 0U; index < source.rings.size(); ++index) {
                const Ring& ring = source.rings[index];
                double ring_longitude = longitude +
                    360.0 * std::ceil((ring.west - longitude) / 360.0);
                bool inside = false;
                for (const Edge& edge : ring.edges) {
                    if (!work.consume()) {
                        return resource_error();
                    }
                    if (ring_longitude <= ring.east &&
                        crosses_northward_ray(edge, ring_longitude, latitude)) {
                        inside = !inside;
                    }
                    if (edge.near_region) {
                        distance = std::min(distance,
                            detail::earth_radius_nautical_miles *
                            environment_detail::distance_to_arc(
                                point, edge.from.unit, edge.to.unit));
                    }
                }
                interiors[index] = inside;
            }
            for (unsigned int level = 1U; level <= 4U; ++level) {
                for (std::size_t index = 0U; index < source.rings.size(); ++index) {
                    const Ring& ring = source.rings[index];
                    if (ring.level != level || !interiors[index]) {
                        continue;
                    }
                    active[index] = level == 1U || active[ring.parent];
                    if (active[index]) {
                        deepest[ring.root] = std::max(deepest[ring.root], level);
                    } else if (distance > numerical_allowance_nautical_miles) {
                        return invalid_data("record " + std::to_string(ring.id) +
                            ": child interior falls outside its hierarchy parent");
                    }
                }
            }
            const bool land = std::any_of(deepest.begin(), deepest.end(),
                [](unsigned int level) { return level % 2U == 1U; });
            samples[row * grid.longitude_count + column] = land ? -distance : distance;
        }
    }
    return samples;
}

}  // namespace

Result<SignedDistanceLandmask> load_gshhg_landmask(
    const std::filesystem::path& path,
    LandDataOptions options) {
    const auto region = make_region(options);
    if (!region) {
        return region.error();
    }
    WorkBudget work{options.maximum_geometry_tests};
    const auto source = read_source(path, region.value(), options, work);
    if (!source) {
        return source.error();
    }
    auto samples = sample_source(source.value(), region.value(), options, work);
    if (!samples) {
        return samples.error();
    }
    LandmaskMetadata metadata;
    metadata.provider.name = "gshhg";
    metadata.provider.revision = "native-" + std::to_string(source.value().version);
    metadata.provider.source =
        path.string() +
        "; GSHHG, Wessel and Smith (https://www.soest.hawaii.edu/pwessel/gshhg/)"
        "; minor great-circle shoreline geometry; distance cap " +
        std::to_string(options.distance_cap_nautical_miles) + " nm";
    metadata.resolution_nautical_miles = options.resolution_nautical_miles;
    metadata.interpolation_error_nautical_miles = region.value().interpolation_error;
    return SignedDistanceLandmask::create(
        region.value().grid, std::move(samples.value()), std::move(metadata));
}

}  // namespace sailroute
