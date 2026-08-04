#include "sailroute/environment.hpp"

#include <cmath>
#include <string_view>

namespace sailroute {

std::string_view to_string(MissingDataPolicy policy) noexcept {
    switch (policy) {
        case MissingDataPolicy::fail_route: return "fail_route";
        case MissingDataPolicy::reject_transition: return "reject_transition";
    }
    return "unknown";
}

std::string_view to_string(ExclusionBoundaryPolicy policy) noexcept {
    switch (policy) {
        case ExclusionBoundaryPolicy::boundary_excluded:
            return "boundary_excluded";
        case ExclusionBoundaryPolicy::boundary_allowed:
            return "boundary_allowed";
    }
    return "unknown";
}

std::string_view to_string(EnvironmentSampling sampling) noexcept {
    switch (sampling) {
        case EnvironmentSampling::segment_start: return "segment_start";
        case EnvironmentSampling::midpoint: return "midpoint";
    }
    return "unknown";
}

std::optional<Error> validate_environment(const RoutingEnvironment& environment) {
    if (static_cast<bool>(environment.waves.provider) !=
        static_cast<bool>(environment.waves.model)) {
        return Error{
            ErrorCode::invalid_environment,
            environment.waves.provider
                ? "a wave provider requires a sea-state performance model"
                : "a sea-state performance model requires a wave provider"};
    }
    if (environment.land.configured()) {
        if (!std::isfinite(environment.land.clearance_nautical_miles) ||
            environment.land.clearance_nautical_miles < 0.0) {
            return Error{
                ErrorCode::invalid_environment,
                "landmask clearance_nautical_miles must be finite and "
                "non-negative"};
        }
        if (environment.land.maximum_subdivision_depth == 0U) {
            return Error{
                ErrorCode::invalid_environment,
                "landmask maximum_subdivision_depth must be positive"};
        }
        if (environment.land.maximum_subdivision_depth > 32U) {
            return Error{
                ErrorCode::invalid_environment,
                "landmask maximum_subdivision_depth must not exceed 32"};
        }
        if (environment.land.landmask->metadata().provider.name.empty()) {
            return Error{
                ErrorCode::invalid_environment,
                "landmask is configured but carries no metadata, so it was "
                "not built by SignedDistanceLandmask::create"};
        }
    } else if (environment.land.clearance_nautical_miles != 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "a land clearance is configured without a landmask to enforce it"};
    }
    if (environment.exclusions.configured() &&
        environment.exclusions.zones->metadata().name.empty()) {
        return Error{
            ErrorCode::invalid_environment,
            "exclusion zones are configured but carry no metadata, so they "
            "were not built by ExclusionZoneSet::create"};
    }
    if (environment.sampling == EnvironmentSampling::midpoint &&
        !environment.active()) {
        return Error{
            ErrorCode::invalid_environment,
            "midpoint environment sampling requires at least one provider"};
    }
    return std::nullopt;
}

std::optional<RouteEnvironmentMetadata> describe_environment(
    const RoutingEnvironment& environment) {
    if (!environment.active()) {
        return std::nullopt;
    }

    RouteEnvironmentMetadata metadata;
    metadata.sampling = environment.sampling;
    metadata.current_policy = environment.currents.missing_data_policy;
    metadata.wave_policy = environment.waves.missing_data_policy;
    metadata.land_policy = environment.land.missing_data_policy;
    metadata.exclusion_policy = environment.exclusions.missing_data_policy;
    if (environment.currents.configured()) {
        metadata.current_provider = environment.currents.provider->metadata();
    }
    if (environment.waves.configured()) {
        metadata.wave_provider = environment.waves.provider->metadata();
        metadata.sea_state_model = environment.waves.model->metadata();
    }
    if (environment.land.configured()) {
        const LandmaskMetadata& land = environment.land.landmask->metadata();
        metadata.landmask = land.provider;
        metadata.land_resolution_nautical_miles = land.resolution_nautical_miles;
        metadata.land_interpolation_error_nautical_miles =
            land.interpolation_error_nautical_miles;
        metadata.land_clearance_nautical_miles =
            environment.land.clearance_nautical_miles;
    }
    if (environment.exclusions.configured()) {
        metadata.exclusions = environment.exclusions.zones->metadata();
        metadata.exclusion_boundary_policy =
            environment.exclusions.boundary_policy;
        metadata.exclusion_zone_count =
            environment.exclusions.zones->zone_count();
        metadata.exclusion_revision =
            environment.exclusions.zones->maximum_revision();
    }
    return metadata;
}

}  // namespace sailroute
