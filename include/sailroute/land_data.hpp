#pragma once

#include "sailroute/environment.hpp"
#include "sailroute/weather.hpp"

#include <cstddef>
#include <filesystem>

namespace sailroute {

/// Bounded, regional conversion of a caller-supplied native GSHHG binary file.
struct LandDataOptions {
    /// Required canonical bounds; west > east crosses the antimeridian.
    /// Each span must be positive and at most 120 degrees, within +/-85 latitude.
    GeographicBounds bounds;
    /// Maximum latitude/equatorial-longitude node spacing, in [0.05, 120] nm.
    double resolution_nautical_miles{2.0};
    /// Distances are saturated at this value, in [1, 600] nm. Also determines
    /// the source-geometry halo; the halo must remain short of either pole.
    double distance_cap_nautical_miles{120.0};
    /// Resource budgets may be reduced, but not increased beyond these defaults.
    std::size_t maximum_grid_nodes{250000U};
    std::size_t maximum_source_points{10000000U};
    std::size_t maximum_geometry_tests{100000000U};
};

/// Reads modern, big-endian, 44-byte-header GSHHG native shoreline records.
///
/// Supports hierarchical levels 1-4, Greenwich/dateline crossings, and minor
/// great-circle edges. Antarctica levels 5/6, pole-enclosing rings, and rings
/// winding around the globe are rejected if they affect the region/halo.
/// Unsupported versions, malformed/truncated files, incomplete hierarchies,
/// and resource exhaustion are errors, never an open-water fallback.
///
/// The caller supplies a complete native shoreline dataset, or a complete
/// hierarchy-preserving regional extraction covering the requested region and
/// halo. Native files carry no coverage manifest: omission of whole records
/// cannot be detected. All records are decoded/validated, but only intersecting
/// or enclosing rings are retained. No data is downloaded.
///
/// Node values use shoreline geometry, not rasterized land/water labels.
/// Saturation preserves the signed distance's 1-Lipschitz bound; metadata adds
/// a full-cell path-diameter allowance to bound interpolation error, including
/// islands between nodes. Use certify_segment(), not a positive interpolated
/// point value alone, to establish clearance.
///
/// Certification is relative to supplied geometry, not chart accuracy. GSHHG
/// is not a nautical chart; its data license/attribution obligations remain the
/// caller's responsibility. Source path, format version and GSHHG attribution
/// are preserved in the returned provider metadata.
[[nodiscard]] Result<SignedDistanceLandmask> load_gshhg_landmask(
    const std::filesystem::path& path,
    LandDataOptions options);

}  // namespace sailroute
