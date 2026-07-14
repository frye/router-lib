#pragma once

#include "sailroute/error.hpp"
#include "sailroute/types.hpp"

#include <chrono>
#include <optional>

namespace sailroute::detail {

inline constexpr std::chrono::minutes minimum_routing_interval{5};

[[nodiscard]] std::optional<Error> validate_routing_intervals(
    const RoutingOptions& options);

[[nodiscard]] std::chrono::seconds routing_step(
    const RoutingOptions& options,
    std::chrono::seconds elapsed,
    std::chrono::seconds remaining) noexcept;

}  // namespace sailroute::detail
