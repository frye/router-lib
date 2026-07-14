#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace sailroute {

struct ForecastMetadata {
    TimePoint first_valid_time;
    TimePoint last_valid_time;
    std::size_t latitude_count{};
    std::size_t longitude_count{};
    bool global_longitude_coverage{};
    std::string source;
};

struct GeographicBounds {
    double south_latitude_degrees{};
    double west_longitude_degrees{};
    double north_latitude_degrees{};
    double east_longitude_degrees{};
};

class WeatherDataset {
public:
    WeatherDataset();
    ~WeatherDataset();
    WeatherDataset(const WeatherDataset&);
    WeatherDataset(WeatherDataset&&) noexcept;
    WeatherDataset& operator=(const WeatherDataset&);
    WeatherDataset& operator=(WeatherDataset&&) noexcept;

    static Result<WeatherDataset> load(const std::filesystem::path& path);
    static Result<WeatherDataset> load(
        const std::filesystem::path& path,
        GeographicBounds bounds);

    [[nodiscard]] const ForecastMetadata& metadata() const;
    [[nodiscard]] Result<Wind> interpolate(Coordinate coordinate, TimePoint time) const;

private:
    struct Impl;
    explicit WeatherDataset(std::shared_ptr<const Impl> impl);
    static Result<WeatherDataset> load_impl(
        const std::filesystem::path& path,
        std::optional<GeographicBounds> bounds);
    std::shared_ptr<const Impl> impl_;
};

}  // namespace sailroute
