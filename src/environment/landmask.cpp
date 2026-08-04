#include "sailroute/environment.hpp"

#include "environment/grid.hpp"
#include "routing/geodesy.hpp"

#include <cmath>
#include <utility>
#include <vector>

namespace sailroute {

struct SignedDistanceLandmask::Impl {
    environment_detail::SampleGrid grid;
    std::vector<double> signed_distance_nautical_miles;
    LandmaskMetadata metadata;
};

SignedDistanceLandmask::SignedDistanceLandmask() = default;
SignedDistanceLandmask::~SignedDistanceLandmask() = default;
SignedDistanceLandmask::SignedDistanceLandmask(const SignedDistanceLandmask&) =
    default;
SignedDistanceLandmask::SignedDistanceLandmask(
    SignedDistanceLandmask&&) noexcept = default;
SignedDistanceLandmask& SignedDistanceLandmask::operator=(
    const SignedDistanceLandmask&) = default;
SignedDistanceLandmask& SignedDistanceLandmask::operator=(
    SignedDistanceLandmask&&) noexcept = default;

SignedDistanceLandmask::SignedDistanceLandmask(std::shared_ptr<const Impl> impl)
    : impl_(std::move(impl)) {}

Result<SignedDistanceLandmask> SignedDistanceLandmask::create(
    EnvironmentGridSpec spec,
    std::vector<double> signed_distance_nautical_miles,
    LandmaskMetadata metadata) {
    if (metadata.provider.name.empty() || metadata.provider.source.empty()) {
        return Error{
            ErrorCode::invalid_environment,
            "landmask metadata must carry a non-empty name and source"};
    }
    if (!std::isfinite(metadata.resolution_nautical_miles) ||
        metadata.resolution_nautical_miles <= 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "landmask resolution_nautical_miles must be finite and positive"};
    }
    if (!std::isfinite(metadata.interpolation_error_nautical_miles) ||
        metadata.interpolation_error_nautical_miles < 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "landmask interpolation_error_nautical_miles must be finite and "
            "non-negative"};
    }
    auto grid = environment_detail::SampleGrid::create(spec);
    if (!grid) {
        return grid.error();
    }
    const std::size_t expected = spec.latitude_count * spec.longitude_count;
    if (signed_distance_nautical_miles.size() != expected) {
        return Error{
            ErrorCode::invalid_environment,
            "landmask must contain latitude_count * longitude_count samples"};
    }
    for (const double value : signed_distance_nautical_miles) {
        if (!std::isfinite(value)) {
            return Error{
                ErrorCode::invalid_environment,
                "landmask contains a non-finite signed distance"};
        }
    }

    auto impl = std::make_shared<Impl>();
    impl->grid = std::move(grid.value());
    impl->signed_distance_nautical_miles =
        std::move(signed_distance_nautical_miles);
    impl->metadata = std::move(metadata);
    return SignedDistanceLandmask{std::move(impl)};
}

const LandmaskMetadata& SignedDistanceLandmask::metadata() const noexcept {
    static const LandmaskMetadata empty{};
    return impl_ ? impl_->metadata : empty;
}

const EnvironmentGridSpec& SignedDistanceLandmask::grid() const noexcept {
    static const EnvironmentGridSpec empty{};
    return impl_ ? impl_->grid.spec() : empty;
}

EnvironmentSample<double> SignedDistanceLandmask::signed_distance_nautical_miles(
    Coordinate coordinate) const {
    if (!impl_) {
        return EnvironmentSample<double>::without_value(
            EnvironmentSampleStatus::unavailable);
    }
    const auto weights = impl_->grid.locate(coordinate);
    if (!weights.has_value()) {
        return EnvironmentSample<double>::without_value(
            EnvironmentSampleStatus::outside_coverage);
    }
    const double distance =
        impl_->grid.blend(*weights, impl_->signed_distance_nautical_miles);
    if (!std::isfinite(distance)) {
        return EnvironmentSample<double>::without_value(
            EnvironmentSampleStatus::invalid_data);
    }
    return EnvironmentSample<double>::available(distance);
}

SignedDistanceLandmask::ClearanceResult SignedDistanceLandmask::certify_segment(
    Coordinate from,
    Coordinate to,
    double clearance_nautical_miles,
    std::size_t maximum_subdivision_depth) const {
    ClearanceResult result;
    if (!impl_) {
        result.status = EnvironmentSampleStatus::unavailable;
        return result;
    }
    if (!std::isfinite(clearance_nautical_miles) ||
        clearance_nautical_miles < 0.0) {
        result.status = EnvironmentSampleStatus::invalid_data;
        return result;
    }

    const double required =
        clearance_nautical_miles +
        impl_->metadata.interpolation_error_nautical_miles;

    const auto query = [&](Coordinate coordinate) {
        ++result.distance_queries;
        return signed_distance_nautical_miles(coordinate);
    };

    const auto start = query(from);
    if (!start.has_value()) {
        result.status = start.status;
        return result;
    }
    const auto end = query(to);
    if (!end.has_value()) {
        result.status = end.status;
        return result;
    }
    if (start.value < required || end.value < required) {
        return result;
    }

    // Depth-first certification over an explicit stack. Each entry holds a
    // sub-segment and the signed distances already measured at its endpoints,
    // so halving a segment costs exactly one new query.
    struct Interval {
        Coordinate from;
        Coordinate to;
        double from_distance{};
        double to_distance{};
        std::size_t depth{};
    };
    std::vector<Interval> pending;
    pending.push_back(Interval{from, to, start.value, end.value, 0U});

    while (!pending.empty()) {
        const Interval interval = pending.back();
        pending.pop_back();

        const double length = detail::great_circle_distance_nautical_miles(
            interval.from, interval.to);
        // A signed distance field is 1-Lipschitz along a path, so the lower
        // bound of the field over the sub-segment is (a + b - length) / 2.
        const double lower_bound =
            0.5 * (interval.from_distance + interval.to_distance - length);
        if (lower_bound >= required) {
            continue;
        }
        if (interval.depth >= maximum_subdivision_depth) {
            // Undecidable within the configured budget. Conceding here would
            // let a crossing hide between two water endpoints.
            return result;
        }

        const double bearing =
            detail::initial_bearing_degrees(interval.from, interval.to);
        const Coordinate middle =
            detail::destination_point(interval.from, bearing, length * 0.5);
        const auto midpoint = query(middle);
        if (!midpoint.has_value()) {
            result.status = midpoint.status;
            return result;
        }
        if (midpoint.value < required) {
            return result;
        }
        pending.push_back(Interval{
            middle,
            interval.to,
            midpoint.value,
            interval.to_distance,
            interval.depth + 1U});
        pending.push_back(Interval{
            interval.from,
            middle,
            interval.from_distance,
            midpoint.value,
            interval.depth + 1U});
    }

    result.clear = true;
    return result;
}

}  // namespace sailroute
