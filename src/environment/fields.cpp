#include "sailroute/environment.hpp"

#include "environment/grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <utility>

namespace sailroute {
namespace {

class UniformCurrentProvider final : public CurrentProvider {
public:
    UniformCurrentProvider(CurrentVector current, ProviderMetadata metadata)
        : current_(current), metadata_(std::move(metadata)) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }

    [[nodiscard]] EnvironmentCoverage coverage() const override {
        EnvironmentCoverage coverage;
        coverage.south_latitude_degrees = -90.0;
        coverage.north_latitude_degrees = 90.0;
        coverage.global_longitude_coverage = true;
        return coverage;
    }

    [[nodiscard]] EnvironmentSample<CurrentVector> sample(
        Coordinate coordinate,
        TimePoint) const override {
        if (!is_valid(coordinate)) {
            return EnvironmentSample<CurrentVector>::without_value(
                EnvironmentSampleStatus::outside_coverage);
        }
        return EnvironmentSample<CurrentVector>::available(current_);
    }

private:
    CurrentVector current_;
    ProviderMetadata metadata_;
};

class UniformWaveProvider final : public WaveProvider {
public:
    UniformWaveProvider(WaveState wave, ProviderMetadata metadata)
        : wave_(wave), metadata_(std::move(metadata)) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }

    [[nodiscard]] EnvironmentCoverage coverage() const override {
        EnvironmentCoverage coverage;
        coverage.south_latitude_degrees = -90.0;
        coverage.north_latitude_degrees = 90.0;
        coverage.global_longitude_coverage = true;
        return coverage;
    }

    [[nodiscard]] EnvironmentSample<WaveState> sample(
        Coordinate coordinate,
        TimePoint) const override {
        if (!is_valid(coordinate)) {
            return EnvironmentSample<WaveState>::without_value(
                EnvironmentSampleStatus::outside_coverage);
        }
        return EnvironmentSample<WaveState>::available(wave_);
    }

private:
    WaveState wave_;
    ProviderMetadata metadata_;
};

class GridCurrentProvider final : public CurrentProvider {
public:
    GridCurrentProvider(
        environment_detail::SampleGrid grid,
        std::vector<double> east,
        std::vector<double> north,
        ProviderMetadata metadata)
        : grid_(grid),
          east_(std::move(east)),
          north_(std::move(north)),
          metadata_(std::move(metadata)) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }

    [[nodiscard]] EnvironmentCoverage coverage() const override {
        return grid_.coverage();
    }

    [[nodiscard]] EnvironmentSample<CurrentVector> sample(
        Coordinate coordinate,
        TimePoint) const override {
        const auto weights = grid_.locate(coordinate);
        if (!weights.has_value()) {
            return EnvironmentSample<CurrentVector>::without_value(
                EnvironmentSampleStatus::outside_coverage);
        }
        const CurrentVector current{
            grid_.blend(*weights, east_),
            grid_.blend(*weights, north_)};
        if (!std::isfinite(current.east_knots) ||
            !std::isfinite(current.north_knots)) {
            return EnvironmentSample<CurrentVector>::without_value(
                EnvironmentSampleStatus::invalid_data);
        }
        return EnvironmentSample<CurrentVector>::available(current);
    }

private:
    environment_detail::SampleGrid grid_;
    std::vector<double> east_;
    std::vector<double> north_;
    ProviderMetadata metadata_;
};

class GridWaveProvider final : public WaveProvider {
public:
    GridWaveProvider(
        environment_detail::SampleGrid grid,
        std::vector<double> height,
        std::vector<double> period,
        std::vector<double> direction_east,
        std::vector<double> direction_north,
        ProviderMetadata metadata)
        : grid_(grid),
          height_(std::move(height)),
          period_(std::move(period)),
          direction_east_(std::move(direction_east)),
          direction_north_(std::move(direction_north)),
          metadata_(std::move(metadata)) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }

    [[nodiscard]] EnvironmentCoverage coverage() const override {
        return grid_.coverage();
    }

    [[nodiscard]] EnvironmentSample<WaveState> sample(
        Coordinate coordinate,
        TimePoint) const override {
        const auto weights = grid_.locate(coordinate);
        if (!weights.has_value()) {
            return EnvironmentSample<WaveState>::without_value(
                EnvironmentSampleStatus::outside_coverage);
        }
        WaveState wave;
        wave.significant_height_metres = grid_.blend(*weights, height_);
        wave.peak_period_seconds = grid_.blend(*weights, period_);
        wave.direction_from_degrees = environment_detail::direction_from_unit(
            grid_.blend(*weights, direction_east_),
            grid_.blend(*weights, direction_north_));
        if (!std::isfinite(wave.significant_height_metres) ||
            !std::isfinite(wave.peak_period_seconds) ||
            !std::isfinite(wave.direction_from_degrees) ||
            wave.significant_height_metres < 0.0 ||
            wave.peak_period_seconds < 0.0) {
            return EnvironmentSample<WaveState>::without_value(
                EnvironmentSampleStatus::invalid_data);
        }
        return EnvironmentSample<WaveState>::available(wave);
    }

private:
    environment_detail::SampleGrid grid_;
    std::vector<double> height_;
    std::vector<double> period_;
    std::vector<double> direction_east_;
    std::vector<double> direction_north_;
    ProviderMetadata metadata_;
};

class WaveHeightDeratingModel final : public SeaStatePerformanceModel {
public:
    WaveHeightDeratingModel(
        WaveHeightDeratingCoefficients coefficients,
        ProviderMetadata metadata)
        : coefficients_(coefficients), metadata_(std::move(metadata)) {}

    [[nodiscard]] const ProviderMetadata& metadata() const noexcept override {
        return metadata_;
    }

    [[nodiscard]] double derated_speed_knots(
        const SeaStateInput& input) const override {
        const double height = input.wave.significant_height_metres;
        if (!(height > 0.0)) {
            return input.flat_water_speed_knots;
        }

        // The relative wave angle is measured from a following sea, so its
        // cosine is +1 running with the waves and -1 punching into them.
        const double alignment = std::cos(
            input.relative_wave_angle_degrees * std::numbers::pi / 180.0);
        const double directional = alignment >= 0.0
            ? 1.0 + alignment * (coefficients_.following_sea_factor - 1.0)
            : 1.0 - alignment * (coefficients_.head_sea_factor - 1.0);

        double loss = coefficients_.height_coefficient *
            std::pow(height, coefficients_.height_exponent) * directional;
        if (coefficients_.period_sensitivity > 0.0) {
            const double period = std::max(
                input.wave.peak_period_seconds,
                coefficients_.minimum_period_seconds);
            const double steepness =
                coefficients_.reference_period_seconds / period;
            loss *= 1.0 + coefficients_.period_sensitivity * (steepness - 1.0);
        }
        loss = std::clamp(loss, 0.0, coefficients_.maximum_loss_fraction);
        return input.flat_water_speed_knots * (1.0 - loss);
    }

private:
    WaveHeightDeratingCoefficients coefficients_;
    ProviderMetadata metadata_;
};

std::optional<Error> validate_metadata(const ProviderMetadata& metadata) {
    if (metadata.name.empty()) {
        return Error{
            ErrorCode::invalid_environment,
            "provider metadata must carry a non-empty name"};
    }
    if (metadata.source.empty()) {
        return Error{
            ErrorCode::invalid_environment,
            "provider metadata must carry a non-empty source attribution"};
    }
    return std::nullopt;
}

std::optional<Error> validate_samples(
    const std::vector<double>& values,
    std::size_t expected,
    std::string_view field) {
    if (values.size() != expected) {
        return Error{
            ErrorCode::invalid_environment,
            "environment field " + std::string{field} +
                " must contain latitude_count * longitude_count samples"};
    }
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return Error{
                ErrorCode::invalid_environment,
                "environment field " + std::string{field} +
                    " contains a non-finite sample"};
        }
    }
    return std::nullopt;
}

}  // namespace

double CurrentVector::speed_knots() const noexcept {
    return std::hypot(east_knots, north_knots);
}

double CurrentVector::set_toward_degrees() const noexcept {
    if (east_knots == 0.0 && north_knots == 0.0) {
        return 0.0;
    }
    return detail_normalize_degrees(
        std::atan2(east_knots, north_knots) * 180.0 / std::numbers::pi);
}

std::string_view to_string(EnvironmentSampleStatus status) noexcept {
    switch (status) {
        case EnvironmentSampleStatus::available: return "available";
        case EnvironmentSampleStatus::outside_coverage: return "outside_coverage";
        case EnvironmentSampleStatus::unavailable: return "unavailable";
        case EnvironmentSampleStatus::invalid_data: return "invalid_data";
    }
    return "unknown";
}

Result<std::shared_ptr<const CurrentProvider>> make_uniform_current_provider(
    CurrentVector current,
    ProviderMetadata metadata) {
    if (const auto error = validate_metadata(metadata); error.has_value()) {
        return *error;
    }
    if (!std::isfinite(current.east_knots) ||
        !std::isfinite(current.north_knots)) {
        return Error{
            ErrorCode::invalid_environment,
            "uniform current components must be finite"};
    }
    return std::shared_ptr<const CurrentProvider>{
        std::make_shared<const UniformCurrentProvider>(
            current, std::move(metadata))};
}

Result<std::shared_ptr<const WaveProvider>> make_uniform_wave_provider(
    WaveState wave,
    ProviderMetadata metadata) {
    if (const auto error = validate_metadata(metadata); error.has_value()) {
        return *error;
    }
    if (!std::isfinite(wave.significant_height_metres) ||
        wave.significant_height_metres < 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "significant wave height must be finite and non-negative"};
    }
    if (!std::isfinite(wave.peak_period_seconds) ||
        wave.peak_period_seconds < 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "wave period must be finite and non-negative"};
    }
    if (!std::isfinite(wave.direction_from_degrees)) {
        return Error{
            ErrorCode::invalid_environment, "wave direction must be finite"};
    }
    wave.direction_from_degrees =
        detail_normalize_degrees(wave.direction_from_degrees);
    return std::shared_ptr<const WaveProvider>{
        std::make_shared<const UniformWaveProvider>(wave, std::move(metadata))};
}

Result<std::shared_ptr<const CurrentProvider>> make_grid_current_provider(
    EnvironmentGridSpec spec,
    std::vector<double> east_knots,
    std::vector<double> north_knots,
    ProviderMetadata metadata) {
    if (const auto error = validate_metadata(metadata); error.has_value()) {
        return *error;
    }
    auto grid = environment_detail::SampleGrid::create(spec);
    if (!grid) {
        return grid.error();
    }
    const std::size_t expected = spec.latitude_count * spec.longitude_count;
    if (const auto error = validate_samples(east_knots, expected, "east_knots");
        error.has_value()) {
        return *error;
    }
    if (const auto error = validate_samples(north_knots, expected, "north_knots");
        error.has_value()) {
        return *error;
    }
    return std::shared_ptr<const CurrentProvider>{
        std::make_shared<const GridCurrentProvider>(
            std::move(grid.value()),
            std::move(east_knots),
            std::move(north_knots),
            std::move(metadata))};
}

Result<std::shared_ptr<const WaveProvider>> make_grid_wave_provider(
    EnvironmentGridSpec spec,
    std::vector<double> significant_height_metres,
    std::vector<double> peak_period_seconds,
    std::vector<double> direction_from_degrees,
    ProviderMetadata metadata) {
    if (const auto error = validate_metadata(metadata); error.has_value()) {
        return *error;
    }
    auto grid = environment_detail::SampleGrid::create(spec);
    if (!grid) {
        return grid.error();
    }
    const std::size_t expected = spec.latitude_count * spec.longitude_count;
    if (const auto error = validate_samples(
            significant_height_metres, expected, "significant_height_metres");
        error.has_value()) {
        return *error;
    }
    if (const auto error = validate_samples(
            peak_period_seconds, expected, "peak_period_seconds");
        error.has_value()) {
        return *error;
    }
    if (const auto error = validate_samples(
            direction_from_degrees, expected, "direction_from_degrees");
        error.has_value()) {
        return *error;
    }
    for (const double height : significant_height_metres) {
        if (height < 0.0) {
            return Error{
                ErrorCode::invalid_environment,
                "significant wave height samples must be non-negative"};
        }
    }
    for (const double period : peak_period_seconds) {
        if (period < 0.0) {
            return Error{
                ErrorCode::invalid_environment,
                "wave period samples must be non-negative"};
        }
    }

    // Directions blend as unit vectors, so 359 and 1 degrees average to zero
    // rather than to 180.
    std::vector<double> east(expected, 0.0);
    std::vector<double> north(expected, 0.0);
    for (std::size_t index = 0U; index < expected; ++index) {
        environment_detail::unit_from_direction(
            direction_from_degrees[index], east[index], north[index]);
    }
    return std::shared_ptr<const WaveProvider>{
        std::make_shared<const GridWaveProvider>(
            std::move(grid.value()),
            std::move(significant_height_metres),
            std::move(peak_period_seconds),
            std::move(east),
            std::move(north),
            std::move(metadata))};
}

Result<std::shared_ptr<const SeaStatePerformanceModel>>
make_wave_height_derating_model(
    WaveHeightDeratingCoefficients coefficients,
    std::optional<ProviderMetadata> metadata) {
    if (!std::isfinite(coefficients.height_coefficient) ||
        coefficients.height_coefficient < 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "wave derating height_coefficient must be finite and non-negative"};
    }
    if (!std::isfinite(coefficients.height_exponent) ||
        coefficients.height_exponent <= 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "wave derating height_exponent must be finite and positive"};
    }
    if (!std::isfinite(coefficients.head_sea_factor) ||
        coefficients.head_sea_factor < 0.0 ||
        !std::isfinite(coefficients.following_sea_factor) ||
        coefficients.following_sea_factor < 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "wave derating directional factors must be finite and non-negative"};
    }
    if (!std::isfinite(coefficients.maximum_loss_fraction) ||
        coefficients.maximum_loss_fraction < 0.0 ||
        coefficients.maximum_loss_fraction > 1.0) {
        return Error{
            ErrorCode::invalid_environment,
            "wave derating maximum_loss_fraction must be finite and in [0, 1]"};
    }
    if (!std::isfinite(coefficients.period_sensitivity) ||
        coefficients.period_sensitivity < 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "wave derating period_sensitivity must be finite and non-negative"};
    }
    if (!std::isfinite(coefficients.reference_period_seconds) ||
        coefficients.reference_period_seconds <= 0.0 ||
        !std::isfinite(coefficients.minimum_period_seconds) ||
        coefficients.minimum_period_seconds <= 0.0) {
        return Error{
            ErrorCode::invalid_environment,
            "wave derating periods must be finite and positive"};
    }

    ProviderMetadata resolved = metadata.has_value()
        ? std::move(*metadata)
        : ProviderMetadata{
              "wave_height_derating",
              "sailroute built-in significant-wave-height derating model; "
              "approximate demonstration coefficients, not a validated "
              "seakeeping prediction",
              "1"};
    if (const auto error = validate_metadata(resolved); error.has_value()) {
        return *error;
    }
    return std::shared_ptr<const SeaStatePerformanceModel>{
        std::make_shared<const WaveHeightDeratingModel>(
            coefficients, std::move(resolved))};
}

}  // namespace sailroute
