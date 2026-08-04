#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

namespace sailroute {

class VesselPolar;

/// True wind angles that maximise progress toward and away from the wind.
struct VelocityMadeGoodAngles {
    double upwind_degrees{0.0};
    double downwind_degrees{180.0};
    bool valid{false};
};

/// Boat-speed lookup for a fixed true wind speed.
///
/// Resolving the true-wind-speed bracket is the more expensive half of a polar
/// lookup, and it is identical for every heading evaluated at a given routing
/// node. A slice resolves it once so per-heading lookups only interpolate in
/// true wind angle.
///
/// A slice borrows the polar's data and does not extend its lifetime; it must
/// not outlive the `VesselPolar` that produced it.
class PolarSlice {
public:
    PolarSlice() noexcept = default;

    /// Reports whether the slice came from a valid polar and wind speed.
    [[nodiscard]] bool valid() const noexcept { return boat_speeds_ != nullptr; }
    /// Interpolates boat speed at the slice's wind speed, folding the angle.
    [[nodiscard]] double speed_knots(double true_wind_angle_degrees) const noexcept;
    /// Returns the slice's best upwind and downwind true wind angles.
    [[nodiscard]] VelocityMadeGoodAngles velocity_made_good_angles() const noexcept;
    /// Reports whether the wind speed exceeded the polar's last tabulated column,
    /// so `speed_knots` is extrapolating by holding that column.
    [[nodiscard]] bool above_tabulated_wind_speed() const noexcept {
        return above_tabulated_wind_speed_;
    }

private:
    friend class VesselPolar;

    const double* wind_angles_{nullptr};
    const double* boat_speeds_{nullptr};
    const double* upwind_vmg_{nullptr};
    const double* downwind_vmg_{nullptr};
    std::size_t wind_angle_count_{0};
    std::size_t wind_speed_count_{0};
    std::size_t wind_lower_{0};
    double wind_fraction_{0.0};
    PolarAngleInterpolation interpolation_{PolarAngleInterpolation::linear};
    bool above_tabulated_wind_speed_{false};
};

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
    /// Resolves the wind-speed bracket once for repeated per-angle lookups.
    ///
    /// With linear angle interpolation this is equivalent to `boat_speed_knots`
    /// at the same wind speed, to the bit.
    [[nodiscard]] PolarSlice slice_at(
        double true_wind_speed_knots,
        PolarAngleInterpolation interpolation =
            PolarAngleInterpolation::linear) const noexcept;
    /// Returns the polar's highest tabulated true wind speed, or zero if empty.
    [[nodiscard]] double maximum_tabulated_wind_speed_knots() const noexcept;
    /// Identifies the loaded file or built-in polar.
    [[nodiscard]] const std::string& source() const noexcept;

private:
    struct Impl;
    explicit VesselPolar(std::shared_ptr<const Impl> impl);
    std::shared_ptr<const Impl> impl_;
};

}  // namespace sailroute
