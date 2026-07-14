#include "intervals.hpp"

#include <algorithm>
#include <string>

namespace sailroute::detail {

std::optional<Error> validate_routing_intervals(
    const RoutingOptions& options) {
    if (options.time_step.has_value()) {
        if (*options.time_step < minimum_routing_interval) {
            return Error{
                ErrorCode::invalid_argument,
                "routing time_step must be at least 5 minutes"};
        }
        return std::nullopt;
    }

    if (options.routing_intervals.empty()) {
        return Error{
            ErrorCode::invalid_argument,
            "routing_intervals must contain at least one interval"};
    }

    std::chrono::minutes previous_cutoff = std::chrono::minutes::zero();
    for (std::size_t index = 0U;
         index < options.routing_intervals.size();
         ++index) {
        const RoutingInterval& tier = options.routing_intervals[index];
        if (tier.interval < minimum_routing_interval) {
            return Error{
                ErrorCode::invalid_argument,
                "routing interval " + std::to_string(index + 1U) +
                    " must be at least 5 minutes"};
        }

        const bool final_tier =
            index + 1U == options.routing_intervals.size();
        if (!tier.until_elapsed.has_value()) {
            if (!final_tier) {
                return Error{
                    ErrorCode::invalid_argument,
                    "only the final routing interval may be open-ended"};
            }
            continue;
        }
        if (final_tier) {
            return Error{
                ErrorCode::invalid_argument,
                "the final routing interval must be open-ended"};
        }
        if (*tier.until_elapsed <= previous_cutoff) {
            return Error{
                ErrorCode::invalid_argument,
                "routing interval cutoffs must be positive and strictly increasing"};
        }
        previous_cutoff = *tier.until_elapsed;
    }
    return std::nullopt;
}

std::chrono::seconds routing_step(
    const RoutingOptions& options,
    std::chrono::seconds elapsed,
    std::chrono::seconds remaining) noexcept {
    if (options.time_step.has_value()) {
        return std::min(
            std::chrono::duration_cast<std::chrono::seconds>(
                *options.time_step),
            remaining);
    }

    for (const RoutingInterval& tier : options.routing_intervals) {
        if (tier.until_elapsed.has_value()) {
            const auto cutoff =
                std::chrono::duration_cast<std::chrono::seconds>(
                    *tier.until_elapsed);
            if (elapsed >= cutoff) {
                continue;
            }
            const auto interval =
                std::chrono::duration_cast<std::chrono::seconds>(
                    tier.interval);
            return std::min({interval, cutoff - elapsed, remaining});
        }
        return std::min(
            std::chrono::duration_cast<std::chrono::seconds>(tier.interval),
            remaining);
    }
    return std::chrono::seconds::zero();
}

}  // namespace sailroute::detail
