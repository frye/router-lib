#include "dataset.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace sailroute {
namespace {

[[nodiscard]] EnsembleVariableCategories categories_for(
    const RoutingEnvironment& environment) noexcept {
    return EnsembleVariableCategories{
        true,
        environment.currents.configured(),
        environment.waves.configured(),
        environment.land.configured(),
        environment.exclusions.configured()};
}

[[nodiscard]] Error member_error(
    ErrorCode code,
    std::string_view identifier,
    std::string message) {
    return Error{
        code,
        "ensemble member '" + std::string{identifier} + "': " + std::move(message)};
}

[[nodiscard]] bool same_grid_geometry(
    const ForecastGridIdentity& lhs,
    const ForecastGridIdentity& rhs) noexcept {
    return lhs.latitude_count == rhs.latitude_count &&
        lhs.longitude_count == rhs.longitude_count &&
        lhs.south_latitude_degrees == rhs.south_latitude_degrees &&
        lhs.west_longitude_degrees == rhs.west_longitude_degrees &&
        lhs.latitude_step_degrees == rhs.latitude_step_degrees &&
        lhs.longitude_step_degrees == rhs.longitude_step_degrees &&
        lhs.global_longitude_coverage == rhs.global_longitude_coverage &&
        lhs.duplicate_longitude_endpoint == rhs.duplicate_longitude_endpoint;
}

[[nodiscard]] double canonical_longitude(double longitude_degrees) noexcept {
    double canonical = std::fmod(longitude_degrees, 360.0);
    if (canonical < 0.0) {
        canonical += 360.0;
    }
    if (canonical >= 360.0) {
        canonical -= 360.0;
    }
    return canonical;
}

[[nodiscard]] double longitude_span(const GeographicBounds& bounds) noexcept {
    if (bounds.west_longitude_degrees == -180.0 &&
        bounds.east_longitude_degrees == 180.0) {
        return 360.0;
    }
    double span =
        bounds.east_longitude_degrees - bounds.west_longitude_degrees;
    if (span < 0.0) {
        span += 360.0;
    }
    return span;
}

[[nodiscard]] bool same_bounds(
    const GeographicBounds& lhs,
    const GeographicBounds& rhs) noexcept {
    return lhs.south_latitude_degrees == rhs.south_latitude_degrees &&
        lhs.north_latitude_degrees == rhs.north_latitude_degrees &&
        canonical_longitude(lhs.west_longitude_degrees) ==
            canonical_longitude(rhs.west_longitude_degrees) &&
        longitude_span(lhs) == longitude_span(rhs);
}

[[nodiscard]] bool same_grid_coverage(
    const ForecastGridIdentity& lhs,
    const ForecastGridIdentity& rhs) noexcept {
    if (lhs.interpolation_bounds || rhs.interpolation_bounds) {
        return lhs.interpolation_bounds && rhs.interpolation_bounds &&
            same_bounds(*lhs.interpolation_bounds, *rhs.interpolation_bounds);
    }
    if (lhs.global_longitude_coverage != rhs.global_longitude_coverage ||
        lhs.south_latitude_degrees != rhs.south_latitude_degrees) {
        return false;
    }
    const double lhs_north =
        lhs.south_latitude_degrees +
        lhs.latitude_step_degrees *
            static_cast<double>(lhs.latitude_count - 1U);
    const double rhs_north =
        rhs.south_latitude_degrees +
        rhs.latitude_step_degrees *
            static_cast<double>(rhs.latitude_count - 1U);
    if (lhs_north != rhs_north || lhs.global_longitude_coverage) {
        return lhs_north == rhs_north;
    }
    const double lhs_east =
        lhs.west_longitude_degrees +
        lhs.longitude_step_degrees *
            static_cast<double>(lhs.longitude_count - 1U);
    const double rhs_east =
        rhs.west_longitude_degrees +
        rhs.longitude_step_degrees *
            static_cast<double>(rhs.longitude_count - 1U);
    return lhs.west_longitude_degrees == rhs.west_longitude_degrees &&
        lhs_east == rhs_east;
}

}  // namespace

std::string_view to_string(EnsembleAlignmentMode mode) noexcept {
    switch (mode) {
        case EnsembleAlignmentMode::strict_shared_search:
            return "strict_shared_search";
        case EnsembleAlignmentMode::permissive_member_local:
            return "permissive_member_local";
    }
    return "unknown";
}

EnsembleDataset::EnsembleDataset() : impl_(std::make_shared<Impl>()) {}
EnsembleDataset::~EnsembleDataset() = default;
EnsembleDataset::EnsembleDataset(const EnsembleDataset&) = default;
EnsembleDataset::EnsembleDataset(EnsembleDataset&&) noexcept = default;
EnsembleDataset& EnsembleDataset::operator=(const EnsembleDataset&) = default;
EnsembleDataset& EnsembleDataset::operator=(EnsembleDataset&&) noexcept = default;

EnsembleDataset::EnsembleDataset(std::shared_ptr<const Impl> impl)
    : impl_(std::move(impl)) {}

Result<EnsembleDataset> EnsembleDataset::load(
    EnsembleRunMetadata metadata,
    std::vector<EnsembleMemberInput> members,
    EnsembleLoadOptions options) {
    if (metadata.run_identifier.empty()) {
        return Error{
            ErrorCode::invalid_argument,
            "ensemble run_identifier must not be empty"};
    }
    if (metadata.model_identifier.empty()) {
        return Error{
            ErrorCode::invalid_argument,
            "ensemble model_identifier must not be empty"};
    }
    if (metadata.schema_revision == 0U) {
        return Error{
            ErrorCode::invalid_argument,
            "ensemble schema_revision must be positive"};
    }
    if (members.empty()) {
        return Error{ErrorCode::invalid_argument, "ensemble must contain at least one member"};
    }

    std::sort(
        members.begin(),
        members.end(),
        [](const EnsembleMemberInput& lhs, const EnsembleMemberInput& rhs) {
            return lhs.identifier < rhs.identifier;
        });

    double maximum_weight = 0.0;
    for (std::size_t index = 0; index < members.size(); ++index) {
        const EnsembleMemberInput& member = members[index];
        if (member.identifier.empty()) {
            return Error{
                ErrorCode::invalid_argument,
                "ensemble member identifier must not be empty"};
        }
        if (index != 0U && members[index - 1U].identifier == member.identifier) {
            return Error{
                ErrorCode::invalid_argument,
                "duplicate ensemble member identifier '" + member.identifier + "'"};
        }
        if (!std::isfinite(member.weight) || member.weight < 0.0) {
            return member_error(
                ErrorCode::invalid_argument,
                member.identifier,
                "weight must be finite and non-negative");
        }
        maximum_weight = std::max(maximum_weight, member.weight);

        if (const auto environment_error = validate_environment(member.environment)) {
            return member_error(
                environment_error->code,
                member.identifier,
                environment_error->message);
        }
    }
    if (maximum_weight == 0.0) {
        return Error{
            ErrorCode::invalid_argument,
            "ensemble total weight must be positive"};
    }

    long double scaled_total_weight = 0.0L;
    for (const EnsembleMemberInput& member : members) {
        scaled_total_weight +=
            static_cast<long double>(member.weight) /
            static_cast<long double>(maximum_weight);
    }

    auto impl = std::make_shared<Impl>();
    impl->metadata = std::move(metadata);
    impl->members.reserve(members.size());
    impl->weather.reserve(members.size());
    impl->environments.reserve(members.size());

    for (EnsembleMemberInput& member : members) {
        const long double normalized =
            (static_cast<long double>(member.weight) /
             static_cast<long double>(maximum_weight)) /
            scaled_total_weight;
        const double normalized_weight = static_cast<double>(normalized);
        if (!std::isfinite(normalized_weight) ||
            (member.weight > 0.0 && normalized_weight == 0.0)) {
            return member_error(
                ErrorCode::invalid_argument,
                member.identifier,
                "weight normalization erased or overflowed a positive weight");
        }

        Result<WeatherDataset> weather_result =
            member.bounds
                ? WeatherDataset::load(member.grib_path, *member.bounds)
                : WeatherDataset::load(member.grib_path);
        if (!weather_result) {
            return member_error(
                weather_result.error().code,
                member.identifier,
                weather_result.error().message);
        }

        WeatherDataset weather = std::move(weather_result.value());
        EnsembleMemberMetadata member_metadata;
        member_metadata.identifier = member.identifier;
        member_metadata.original_weight = member.weight;
        member_metadata.normalized_weight = normalized_weight;
        member_metadata.weather = weather.metadata();
        member_metadata.wind_valid_times = weather.valid_times();
        member_metadata.wind_grid = weather.grid_identity();
        member_metadata.configured_variables = categories_for(member.environment);
        member_metadata.environment = describe_environment(member.environment);
        if (member.environment.currents.configured()) {
            member_metadata.current_coverage =
                member.environment.currents.provider->coverage();
        }
        if (member.environment.waves.configured()) {
            member_metadata.wave_coverage =
                member.environment.waves.provider->coverage();
        }

        impl->members.push_back(std::move(member_metadata));
        impl->weather.push_back(std::move(weather));
        impl->environments.push_back(std::move(member.environment));
    }

    const EnsembleMemberMetadata& reference = impl->members.front();
    for (std::size_t index = 1U; index < impl->members.size(); ++index) {
        const EnsembleMemberMetadata& member = impl->members[index];
        if (member.weather.initialization_time !=
            reference.weather.initialization_time) {
            impl->alignment.exact_initialization_time = false;
        }
        if (member.wind_valid_times != reference.wind_valid_times) {
            impl->alignment.exact_wind_valid_times = false;
        }
        if (member.weather.first_valid_time != reference.weather.first_valid_time ||
            member.weather.last_valid_time != reference.weather.last_valid_time) {
            impl->alignment.equal_wind_horizon = false;
        }
        if (!same_grid_geometry(member.wind_grid, reference.wind_grid)) {
            impl->alignment.exact_wind_grid_geometry = false;
        }
        if (!same_grid_coverage(member.wind_grid, reference.wind_grid)) {
            impl->alignment.exact_wind_coverage = false;
            impl->alignment.partial_wind_coverage = true;
        }
        if (member.configured_variables != reference.configured_variables) {
            impl->alignment.aligned_environment_categories = false;
        }
    }

    if (options.alignment == EnsembleAlignmentMode::strict_shared_search) {
        if (!impl->alignment.exact_initialization_time) {
            return Error{
                ErrorCode::incomplete_forecast,
                "strict ensemble members must have an identical "
                "initialization time"};
        }
        if (!impl->alignment.exact_wind_valid_times) {
            return Error{
                ErrorCode::incomplete_forecast,
                "strict ensemble members must have an identical wind valid-time axis"};
        }
        if (!impl->alignment.exact_wind_grid_geometry ||
            !impl->alignment.exact_wind_coverage) {
            return Error{
                ErrorCode::incomplete_forecast,
                "strict ensemble members must have identical wind grid geometry and coverage"};
        }
        if (!impl->alignment.aligned_environment_categories) {
            return Error{
                ErrorCode::invalid_environment,
                "strict ensemble members must configure identical variable categories"};
        }
    }

    return EnsembleDataset{std::move(impl)};
}

const EnsembleRunMetadata& EnsembleDataset::metadata() const noexcept {
    if (!impl_) {
        static const EnsembleRunMetadata empty_metadata;
        return empty_metadata;
    }
    return impl_->metadata;
}

const std::vector<EnsembleMemberMetadata>& EnsembleDataset::members() const noexcept {
    if (!impl_) {
        static const std::vector<EnsembleMemberMetadata> empty_members;
        return empty_members;
    }
    return impl_->members;
}

const EnsembleAlignmentStatus& EnsembleDataset::alignment() const noexcept {
    if (!impl_) {
        static const EnsembleAlignmentStatus empty_alignment;
        return empty_alignment;
    }
    return impl_->alignment;
}

std::size_t EnsembleDataset::member_count() const noexcept {
    return impl_ ? impl_->members.size() : 0U;
}

const WeatherDataset* EnsembleDataset::member_weather(std::size_t index) const noexcept {
    return impl_ && index < impl_->weather.size() ? &impl_->weather[index] : nullptr;
}

const RoutingEnvironment* EnsembleDataset::member_environment(
    std::size_t index) const noexcept {
    return impl_ && index < impl_->environments.size()
        ? &impl_->environments[index]
        : nullptr;
}

}  // namespace sailroute
