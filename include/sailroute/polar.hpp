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

    static Result<VesselPolar> load(const std::filesystem::path& path);
    static VesselPolar default_racer_cruiser_45ft();

    [[nodiscard]] double boat_speed_knots(
        double true_wind_speed_knots,
        double true_wind_angle_degrees) const noexcept;
    [[nodiscard]] const std::string& source() const noexcept;

private:
    struct Impl;
    explicit VesselPolar(std::shared_ptr<const Impl> impl);
    std::shared_ptr<const Impl> impl_;
};

}  // namespace sailroute
