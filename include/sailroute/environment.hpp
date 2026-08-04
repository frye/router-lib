#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace sailroute {

/// Stage 3 optional environmental providers.
///
/// Every provider below is opt-in. A `Router` constructed without a
/// `RoutingEnvironment`, or with an environment that configures no provider,
/// evaluates exactly the arithmetic it evaluated before Stage 3 existed.
///
/// Units and reference frames used throughout this header:
///
///   - Positions are canonical degrees; longitude is in [-180, 180].
///   - Times are UTC whole seconds.
///   - Current is an east/north vector in knots. East is the direction of
///     increasing longitude and north the direction of increasing latitude,
///     so the vector points the way the water flows (the oceanographic "set"
///     convention), which is the opposite of the meteorological convention
///     used for wind direction.
///   - Significant wave height is metres, wave period is seconds, and wave
///     direction is the meteorological direction the waves come *from*,
///     degrees true, matching `RoutePoint::true_wind_direction_degrees`.
///   - Signed land distance is nautical miles, positive over water and
///     negative over land.
///
/// Providers are sampled concurrently by the isochrone solver's worker
/// threads. Every provider must therefore be immutable after construction and
/// its `sample`/query members must be safe to call concurrently from multiple
/// threads. Providers are held by `std::shared_ptr<const T>` so a single
/// instance can be shared across routers and optimizations.

/// Distinguishes a usable sample from each provider-reported failure state.
///
/// An unconfigured provider is represented by an empty provider pointer and is
/// not a sample status or a failure.
enum class EnvironmentSampleStatus {
    available,
    outside_coverage,
    unavailable,
    invalid_data,
};

/// Returns the stable snake_case name of a sample status.
std::string_view to_string(EnvironmentSampleStatus status) noexcept;

/// Identity and attribution for one provider, model, or dataset.
///
/// `ProviderMetadata`, `MissingDataPolicy`, `ExclusionBoundaryPolicy`, and
/// `EnvironmentSampling` are declared in `sailroute/types.hpp` because routing
/// results report them; they are documented there.

/// Optional spatial and temporal extent a provider claims to cover.
///
/// Coverage is advisory: a sample outside it must still return
/// `outside_coverage` rather than an extrapolated value.
struct EnvironmentCoverage {
    std::optional<double> south_latitude_degrees;
    std::optional<double> north_latitude_degrees;
    std::optional<double> west_longitude_degrees;
    std::optional<double> east_longitude_degrees;
    std::optional<TimePoint> first_valid_time;
    std::optional<TimePoint> last_valid_time;
    /// True when the provider covers every longitude, which makes the
    /// longitude bounds above meaningless.
    bool global_longitude_coverage{false};
};

/// Eastward and northward surface-current components in knots.
struct CurrentVector {
    double east_knots{};
    double north_knots{};

    [[nodiscard]] double speed_knots() const noexcept;
    /// Direction the water flows toward, degrees true.
    [[nodiscard]] double set_toward_degrees() const noexcept;
};

/// Significant height, representative period, and origin direction of the sea.
struct WaveState {
    double significant_height_metres{};
    double peak_period_seconds{};
    /// Meteorological direction the waves come from, degrees true.
    double direction_from_degrees{};
};

/// A provider sample: a value, or an explicit reason there is no value.
template <typename T>
struct EnvironmentSample {
    EnvironmentSampleStatus status{EnvironmentSampleStatus::available};
    T value{};

    [[nodiscard]] bool has_value() const noexcept {
        return status == EnvironmentSampleStatus::available;
    }

    static EnvironmentSample available(T value) {
        return EnvironmentSample{EnvironmentSampleStatus::available, std::move(value)};
    }

    static EnvironmentSample without_value(EnvironmentSampleStatus status) {
        return EnvironmentSample{status, T{}};
    }
};

/// Immutable, concurrently sampleable surface-current field.
class CurrentProvider {
public:
    virtual ~CurrentProvider() = default;

    [[nodiscard]] virtual const ProviderMetadata& metadata() const noexcept = 0;
    [[nodiscard]] virtual EnvironmentCoverage coverage() const = 0;
    /// Samples the current at a canonical coordinate and UTC time.
    ///
    /// Implementations must be pure, deterministic, and thread safe, and must
    /// never return a non-finite component with `available` status.
    [[nodiscard]] virtual EnvironmentSample<CurrentVector> sample(
        Coordinate coordinate,
        TimePoint time) const = 0;

protected:
    CurrentProvider() = default;
    CurrentProvider(const CurrentProvider&) = default;
    CurrentProvider& operator=(const CurrentProvider&) = default;
};

/// Immutable, concurrently sampleable sea-state field.
class WaveProvider {
public:
    virtual ~WaveProvider() = default;

    [[nodiscard]] virtual const ProviderMetadata& metadata() const noexcept = 0;
    [[nodiscard]] virtual EnvironmentCoverage coverage() const = 0;
    /// Samples the sea state at a canonical coordinate and UTC time.
    [[nodiscard]] virtual EnvironmentSample<WaveState> sample(
        Coordinate coordinate,
        TimePoint time) const = 0;

protected:
    WaveProvider() = default;
    WaveProvider(const WaveProvider&) = default;
    WaveProvider& operator=(const WaveProvider&) = default;
};

/// Everything a sea-state performance model may use for one evaluation.
///
/// The wind and vessel state are water-relative, matching the frame the polar
/// itself is defined in. `relative_wave_angle_degrees` is the angle between
/// the vessel's water-frame heading and the direction the waves travel toward:
/// 0 degrees is a following sea, 180 degrees is a head sea, and 90 degrees is
/// a beam sea.
struct SeaStateInput {
    double flat_water_speed_knots{};
    double true_wind_speed_knots{};
    double true_wind_angle_degrees{};
    double heading_degrees{};
    double relative_wave_angle_degrees{};
    WaveState wave;
};

/// Replaceable derating model applied after the flat-water polar lookup.
class SeaStatePerformanceModel {
public:
    virtual ~SeaStatePerformanceModel() = default;

    [[nodiscard]] virtual const ProviderMetadata& metadata() const noexcept = 0;
    /// Returns the derated through-water speed in knots.
    ///
    /// The router validates the result: it must be finite, non-negative, and
    /// no greater than `SeaStateInput::flat_water_speed_knots`. A model that
    /// violates that contract fails the optimization rather than being
    /// silently clamped, because a model that accelerates the vessel in waves
    /// is a configuration error rather than a sea state.
    [[nodiscard]] virtual double derated_speed_knots(
        const SeaStateInput& input) const = 0;

protected:
    SeaStatePerformanceModel() = default;
    SeaStatePerformanceModel(const SeaStatePerformanceModel&) = default;
    SeaStatePerformanceModel& operator=(const SeaStatePerformanceModel&) = default;
};

/// Constructs a constant current over all positions and times.
///
/// Deterministic and file-independent, which makes analytic tests and
/// application defaults possible without any external dataset.
[[nodiscard]] Result<std::shared_ptr<const CurrentProvider>>
make_uniform_current_provider(CurrentVector current, ProviderMetadata metadata);

/// Constructs a constant sea state over all positions and times.
[[nodiscard]] Result<std::shared_ptr<const WaveProvider>>
make_uniform_wave_provider(WaveState wave, ProviderMetadata metadata);

/// Regular latitude/longitude sample grid shared by the in-memory fields.
///
/// Values are stored row-major from the south-west corner: index
/// `row * longitude_count + column`, with `row` increasing north and `column`
/// increasing east. Longitudes are canonicalized, so a grid may wrap the
/// antimeridian when `global_longitude_coverage` is set.
struct EnvironmentGridSpec {
    double south_latitude_degrees{};
    double west_longitude_degrees{};
    double latitude_step_degrees{};
    double longitude_step_degrees{};
    std::size_t latitude_count{};
    std::size_t longitude_count{};
    bool global_longitude_coverage{false};
};

/// Constructs a bilinearly interpolated current field from in-memory samples.
///
/// `east_knots` and `north_knots` must both contain
/// `latitude_count * longitude_count` finite values. Positions outside the
/// grid produce `outside_coverage` rather than an extrapolated value.
[[nodiscard]] Result<std::shared_ptr<const CurrentProvider>>
make_grid_current_provider(
    EnvironmentGridSpec spec,
    std::vector<double> east_knots,
    std::vector<double> north_knots,
    ProviderMetadata metadata);

/// Constructs a bilinearly interpolated sea-state field from in-memory samples.
///
/// Heights and periods are interpolated directly; directions are interpolated
/// as unit vectors so the antimeridian of the compass rose is not a
/// discontinuity.
[[nodiscard]] Result<std::shared_ptr<const WaveProvider>>
make_grid_wave_provider(
    EnvironmentGridSpec spec,
    std::vector<double> significant_height_metres,
    std::vector<double> peak_period_seconds,
    std::vector<double> direction_from_degrees,
    ProviderMetadata metadata);

/// Coefficients of the built-in significant-wave-height derating model.
///
/// The model is intentionally simple, documented, and bounded rather than
/// tuned: it is a defensible default and a worked example of the
/// `SeaStatePerformanceModel` contract, not a validated seakeeping prediction.
///
/// The retained speed fraction is
///
///     1 - min(maximum_loss_fraction,
///             height_coefficient * Hs^height_exponent * directional_factor)
///
/// where `Hs` is significant wave height in metres and the directional factor
/// interpolates between `following_sea_factor` at a following sea, one at a
/// beam sea, and `head_sea_factor` at a head sea, using the cosine of the
/// relative wave angle. A steep, short sea hurts more than a long swell of the
/// same height, so the loss is additionally scaled by
/// `reference_period_seconds / max(period, minimum_period_seconds)` when
/// `period_sensitivity` is non-zero.
struct WaveHeightDeratingCoefficients {
    double height_coefficient{0.03};
    double height_exponent{1.5};
    double head_sea_factor{1.6};
    double following_sea_factor{0.35};
    double maximum_loss_fraction{0.6};
    double period_sensitivity{0.0};
    double reference_period_seconds{8.0};
    double minimum_period_seconds{2.0};
};

/// Constructs the built-in significant-wave-height derating model.
[[nodiscard]] Result<std::shared_ptr<const SeaStatePerformanceModel>>
make_wave_height_derating_model(
    WaveHeightDeratingCoefficients coefficients = {},
    std::optional<ProviderMetadata> metadata = std::nullopt);

/// Resolution, error bound, and attribution of a signed-distance landmask.
struct LandmaskMetadata {
    ProviderMetadata provider;
    /// Nominal node spacing of the sampled grid, nautical miles.
    double resolution_nautical_miles{};
    /// Upper bound on the error of an interpolated distance, nautical miles.
    ///
    /// Segment certification adds this to the configured clearance, so a mask
    /// that under-reports its own error can never round a decision toward
    /// accepting land.
    double interpolation_error_nautical_miles{};
};

/// Immutable signed-distance landmask sampled on a regular grid.
///
/// Stored values are the signed distance to the nearest coastline in nautical
/// miles, positive over water and negative over land. Point queries are
/// bilinear, so the field remains 1-Lipschitz in distance to within the
/// declared interpolation error, which is what makes conservative segment
/// certification possible.
class SignedDistanceLandmask {
public:
    SignedDistanceLandmask();
    ~SignedDistanceLandmask();
    SignedDistanceLandmask(const SignedDistanceLandmask&);
    SignedDistanceLandmask(SignedDistanceLandmask&&) noexcept;
    SignedDistanceLandmask& operator=(const SignedDistanceLandmask&);
    SignedDistanceLandmask& operator=(SignedDistanceLandmask&&) noexcept;

    /// Builds a mask from in-memory signed distances in nautical miles.
    ///
    /// `signed_distance_nautical_miles` must contain
    /// `latitude_count * longitude_count` finite values in the row-major order
    /// documented on `EnvironmentGridSpec`.
    [[nodiscard]] static Result<SignedDistanceLandmask> create(
        EnvironmentGridSpec spec,
        std::vector<double> signed_distance_nautical_miles,
        LandmaskMetadata metadata);

    [[nodiscard]] const LandmaskMetadata& metadata() const noexcept;
    [[nodiscard]] const EnvironmentGridSpec& grid() const noexcept;
    /// Bilinearly interpolates signed distance at a canonical coordinate.
    [[nodiscard]] EnvironmentSample<double> signed_distance_nautical_miles(
        Coordinate coordinate) const;

    /// Outcome of certifying that a great-circle segment stays clear of land.
    struct ClearanceResult {
        /// True only when the whole segment is *proved* clear.
        bool clear{false};
        /// `available` when the mask answered everywhere it was asked. Any
        /// other status means the segment left the mask or read invalid data,
        /// which is reported rather than assumed to be open water.
        EnvironmentSampleStatus status{EnvironmentSampleStatus::available};
        /// Point queries performed, for diagnostics.
        std::size_t distance_queries{};
    };

    /// Proves, or fails to prove, that a segment keeps the required clearance.
    ///
    /// A signed distance field is 1-Lipschitz: no point on a segment of length
    /// `L` whose endpoints are at signed distances `a` and `b` can be closer to
    /// land than `(a + b - L) / 2`. When that bound already clears
    /// `clearance_nautical_miles` plus the mask's declared interpolation
    /// error, the whole segment is certified without further sampling.
    /// Otherwise the segment is halved and each half is certified
    /// recursively. Reaching `maximum_subdivision_depth` without a proof
    /// rejects the segment, so two water endpoints can never hide a crossing.
    [[nodiscard]] ClearanceResult certify_segment(
        Coordinate from,
        Coordinate to,
        double clearance_nautical_miles,
        std::size_t maximum_subdivision_depth) const;

private:
    struct Impl;
    explicit SignedDistanceLandmask(std::shared_ptr<const Impl> impl);
    std::shared_ptr<const Impl> impl_;
};

/// Whether touching a zone's boundary counts as entering it is expressed by
/// `ExclusionBoundaryPolicy` in `sailroute/types.hpp`.

/// One closed ring of an exclusion polygon.
///
/// Vertices are canonical coordinates in order. The ring is implicitly closed;
/// a repeated final vertex is accepted and dropped. Winding order is not
/// significant, and edges are great-circle arcs, so a ring may cross the
/// antimeridian without any special encoding.
struct ExclusionRing {
    std::vector<Coordinate> vertices;
};

/// One simple polygon of an exclusion zone, with optional holes.
struct ExclusionPolygon {
    ExclusionRing outer;
    std::vector<ExclusionRing> holes;
};

/// A versioned, optionally time-limited operational exclusion.
///
/// `active_from` and `active_until` bound a half-open UTC interval
/// `[active_from, active_until)`. An unset bound is open-ended, so a zone with
/// neither bound is always active.
struct ExclusionZone {
    /// Stable identity, unique within a set, used for deterministic ordering.
    std::string identifier;
    /// Attribution of the record, for example a notice-to-mariners reference.
    std::string source;
    /// Monotonic revision of this record's geometry and validity.
    std::uint64_t revision{};
    std::optional<TimePoint> active_from;
    std::optional<TimePoint> active_until;
    std::vector<ExclusionPolygon> polygons;
};

/// Immutable, validated, deterministically ordered and indexed exclusion set.
class ExclusionZoneSet {
public:
    ExclusionZoneSet();
    ~ExclusionZoneSet();
    ExclusionZoneSet(const ExclusionZoneSet&);
    ExclusionZoneSet(ExclusionZoneSet&&) noexcept;
    ExclusionZoneSet& operator=(const ExclusionZoneSet&);
    ExclusionZoneSet& operator=(ExclusionZoneSet&&) noexcept;

    /// Validates topology and builds the spatial and temporal index.
    ///
    /// Zones are sorted by `identifier` then `revision`, so input order can
    /// never change routing or diagnostics. Duplicate identifiers at the same
    /// revision, rings with fewer than three distinct vertices, non-finite or
    /// out-of-range coordinates, inverted activation windows, and holes that
    /// escape their outer ring are all rejected.
    [[nodiscard]] static Result<ExclusionZoneSet> create(
        std::vector<ExclusionZone> zones,
        ProviderMetadata metadata);

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept;
    [[nodiscard]] std::size_t zone_count() const noexcept;
    /// Returns the highest revision in the set, or zero when empty.
    [[nodiscard]] std::uint64_t maximum_revision() const noexcept;
    /// Returns the canonically ordered zones.
    [[nodiscard]] const std::vector<ExclusionZone>& zones() const noexcept;
    /// Reports whether a coordinate lies inside a zone active at a time.
    [[nodiscard]] bool contains(
        Coordinate coordinate,
        TimePoint time,
        ExclusionBoundaryPolicy policy) const;

    /// Outcome of testing one timed great-circle segment against the set.
    struct SegmentResult {
        /// True when the segment enters a zone while that zone is active.
        bool violated{false};
        /// Identifier of the first violated zone in canonical order.
        std::string_view zone_identifier;
        /// Containment and arc-intersection tests performed, for diagnostics.
        std::size_t geometry_tests{};
    };

    /// Tests a segment traversed from `from` at `from_time` to `to` at
    /// `to_time` against every zone.
    ///
    /// A zone is considered only for the portion of the traversal during which
    /// it is active: the activation window is clipped to the traversal
    /// interval and converted to a fraction of the great-circle segment, so a
    /// zone that opens or closes mid-segment constrains only the part of the
    /// segment actually sailed while it was open. Position is taken to advance
    /// linearly in time along the great circle, which matches how the solvers
    /// integrate a transition.
    [[nodiscard]] SegmentResult intersects_segment(
        Coordinate from,
        TimePoint from_time,
        Coordinate to,
        TimePoint to_time,
        ExclusionBoundaryPolicy policy) const;

private:
    struct Impl;
    explicit ExclusionZoneSet(std::shared_ptr<const Impl> impl);
    std::shared_ptr<const Impl> impl_;
};

/// Current-provider configuration.
struct CurrentSettings {
    std::shared_ptr<const CurrentProvider> provider;
    MissingDataPolicy missing_data_policy{MissingDataPolicy::fail_route};

    [[nodiscard]] bool configured() const noexcept {
        return static_cast<bool>(provider);
    }
};

/// Wave-provider and sea-state-model configuration.
///
/// Both must be configured together: a wave provider with no model has nothing
/// to apply, and a model with no provider has nothing to apply it to. Either
/// combination is rejected during request validation rather than ignored.
struct WaveSettings {
    std::shared_ptr<const WaveProvider> provider;
    std::shared_ptr<const SeaStatePerformanceModel> model;
    MissingDataPolicy missing_data_policy{MissingDataPolicy::fail_route};

    [[nodiscard]] bool configured() const noexcept {
        return static_cast<bool>(provider) && static_cast<bool>(model);
    }
};

/// Landmask configuration and required clearance.
struct LandmaskSettings {
    std::optional<SignedDistanceLandmask> landmask;
    /// Water that must remain under the vessel, nautical miles. The mask's own
    /// interpolation error is added to this before any segment is certified.
    double clearance_nautical_miles{0.0};
    MissingDataPolicy missing_data_policy{MissingDataPolicy::fail_route};
    /// Bound on the adaptive subdivision used to certify one segment. Each
    /// level halves the segment, so the default resolves a segment to roughly
    /// a four-thousandth of its length before conceding.
    std::size_t maximum_subdivision_depth{12U};

    [[nodiscard]] bool configured() const noexcept {
        return landmask.has_value();
    }
};

/// Exclusion-zone configuration.
struct ExclusionSettings {
    std::optional<ExclusionZoneSet> zones;
    ExclusionBoundaryPolicy boundary_policy{
        ExclusionBoundaryPolicy::boundary_excluded};
    MissingDataPolicy missing_data_policy{MissingDataPolicy::fail_route};

    [[nodiscard]] bool configured() const noexcept {
        return zones.has_value();
    }
};

/// Where along a transition the environment is sampled is expressed by
/// `EnvironmentSampling` in `sailroute/types.hpp`.

/// The complete opt-in Stage 3 environment for one router.
///
/// An environment with no configured provider is exactly equivalent to
/// constructing a `Router` without one.
struct RoutingEnvironment {
    CurrentSettings currents;
    WaveSettings waves;
    LandmaskSettings land;
    ExclusionSettings exclusions;
    EnvironmentSampling sampling{EnvironmentSampling::segment_start};

    [[nodiscard]] bool active() const noexcept {
        return currents.configured() || waves.configured() ||
            land.configured() || exclusions.configured();
    }
};

/// Validates a routing environment's provider and option combinations.
///
/// Returns the first contradiction found, or `std::nullopt` when the
/// environment is coherent. `Router` runs this before any search begins.
[[nodiscard]] std::optional<Error> validate_environment(
    const RoutingEnvironment& environment);

/// Summarizes the sources, models, and policies an environment would apply.
///
/// Returns `std::nullopt` when no provider is configured, which is what makes
/// `RouteResult::environment` absent on the compatibility path.
[[nodiscard]] std::optional<RouteEnvironmentMetadata> describe_environment(
    const RoutingEnvironment& environment);

}  // namespace sailroute
