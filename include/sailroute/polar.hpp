#pragma once

#include "sailroute/error.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace sailroute {

class VesselPolar {
public:
    VesselPolar();
    ~VesselPolar();
    VesselPolar(const VesselPolar&);
    VesselPolar(VesselPolar&&) noexcept;
    VesselPolar& operator=(const VesselPolar&);
    VesselPolar& operator=(VesselPolar&&) noexcept;

    /// Loads a CSV matrix or Expedition-style vessel polar.
    static Result<VesselPolar> load(const std::filesystem::path& path);
    /// Returns approximate demonstration data, not navigation-certified data.
    static VesselPolar default_racer_cruiser_45ft();

    /// Bilinearly interpolates boat speed, folding TWA and clamping both axes.
    [[nodiscard]] double boat_speed_knots(
        double true_wind_speed_knots,
        double true_wind_angle_degrees) const noexcept;
    /// Identifies the loaded file or built-in polar.
    [[nodiscard]] const std::string& source() const noexcept;

private:
    struct Impl;
    explicit VesselPolar(std::shared_ptr<const Impl> impl);
    std::shared_ptr<const Impl> impl_;
};

}  // namespace sailroute
