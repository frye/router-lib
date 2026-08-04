#pragma once

#include "sailroute/types.hpp"

#include <cstddef>
#include <cstdint>
#include <tuple>

namespace sailroute::detail {

using SailConfigurationIdentity = std::uint32_t;
using ReefConfigurationIdentity = std::uint32_t;

inline constexpr SailConfigurationIdentity reserved_sail_configuration = 0U;
inline constexpr ReefConfigurationIdentity reserved_reef_configuration = 0U;

struct OperationalConfiguration {
    std::int8_t board{};
    SailConfigurationIdentity sail{reserved_sail_configuration};
    ReefConfigurationIdentity reef{reserved_reef_configuration};

    friend bool operator==(
        const OperationalConfiguration&,
        const OperationalConfiguration&) noexcept = default;

    friend bool operator<(
        const OperationalConfiguration& left,
        const OperationalConfiguration& right) noexcept {
        return std::tie(left.board, left.sail, left.reef) <
            std::tie(right.board, right.sail, right.reef);
    }
};

struct SolverStateKey {
    std::size_t spatial{};
    std::int64_t time_bucket{};
    TimePoint arrival;
    OperationalConfiguration configuration;

    friend bool operator==(
        const SolverStateKey&,
        const SolverStateKey&) noexcept = default;

    friend bool operator<(
        const SolverStateKey& left,
        const SolverStateKey& right) noexcept {
        return std::tie(
                   left.spatial,
                   left.time_bucket,
                   left.arrival,
                   left.configuration) <
            std::tie(
                   right.spatial,
                   right.time_bucket,
                   right.arrival,
                   right.configuration);
    }
};

struct SolverLabelIdentity {
    SolverStateKey state;
    TimePoint arrival;
    std::size_t predecessor{};
    std::size_t action{};
    std::size_t ordinal{};
};

[[nodiscard]] inline bool dominates(
    const SolverLabelIdentity& left,
    const SolverLabelIdentity& right) noexcept {
    return left.state == right.state && left.arrival <= right.arrival;
}

}  // namespace sailroute::detail
