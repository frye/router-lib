#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace sailroute {

/// Time range, retained grid shape, coverage, and source of loaded weather.
struct ForecastMetadata {
    TimePoint first_valid_time;
    TimePoint last_valid_time;
    std::size_t latitude_count{};
    std::size_t longitude_count{};
    bool global_longitude_coverage{};
    std::string source;
};

/// Canonical load bounds; west greater than east crosses the antimeridian.
struct GeographicBounds {
    double south_latitude_degrees{};
    double west_longitude_degrees{};
    double north_latitude_degrees{};
    double east_longitude_degrees{};
};

class WeatherDataset;

/// Wind lookup for a fixed forecast time.
///
/// Locating the surrounding forecast steps costs a binary search and is
/// identical for every node evaluated at a given routing step. A sampler
/// resolves the time bracket once so per-node lookups only do the spatial
/// bracket and the bilinear blend.
///
/// A sampler shares ownership of the dataset, so it stays valid independently.
class WeatherSampler {
public:
    WeatherSampler() noexcept;
    ~WeatherSampler();
    WeatherSampler(const WeatherSampler&);
    WeatherSampler(WeatherSampler&&) noexcept;
    WeatherSampler& operator=(const WeatherSampler&);
    WeatherSampler& operator=(WeatherSampler&&) noexcept;

    /// Reports whether the sampler resolved a forecast time.
    [[nodiscard]] bool valid() const noexcept;
    /// Interpolates wind at the sampler's time, to the bit as `interpolate`.
    [[nodiscard]] Result<Wind> sample(Coordinate coordinate) const;

private:
    friend class WeatherDataset;

    struct Impl;
    std::shared_ptr<const Impl> impl_;
};

class WeatherDataset {
public:
    WeatherDataset();
    ~WeatherDataset();
    WeatherDataset(const WeatherDataset&);
    WeatherDataset(WeatherDataset&&) noexcept;
    WeatherDataset& operator=(const WeatherDataset&);
    WeatherDataset& operator=(WeatherDataset&&) noexcept;

    /// Loads every supported wind field in a GRIB1 or GRIB2 file.
    static Result<WeatherDataset> load(const std::filesystem::path& path);
    /// Loads the interpolation subgrid required by canonical geographic bounds.
    static Result<WeatherDataset> load(
        const std::filesystem::path& path,
        GeographicBounds bounds);

    /// Returns valid times, retained grid dimensions, coverage, and source.
    [[nodiscard]] const ForecastMetadata& metadata() const;
    /// Interpolates eastward/northward wind in m/s at a coordinate and UTC time.
    [[nodiscard]] Result<Wind> interpolate(Coordinate coordinate, TimePoint time) const;
    /// Resolves the forecast time bracket once for repeated per-coordinate lookups.
    ///
    /// Fails only when the dataset is empty. A time outside forecast coverage is
    /// reported by `WeatherSampler::sample`, after the coordinate is validated,
    /// so the error matches what `interpolate` would return for the same request.
    [[nodiscard]] Result<WeatherSampler> sampler_at(TimePoint time) const;

private:
    friend class WeatherSampler;

    struct Impl;
    explicit WeatherDataset(std::shared_ptr<const Impl> impl);
    static Result<WeatherDataset> load_impl(
        const std::filesystem::path& path,
        std::optional<GeographicBounds> bounds);
    std::shared_ptr<const Impl> impl_;
};

}  // namespace sailroute
