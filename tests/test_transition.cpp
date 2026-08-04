#include "../src/routing/state.hpp"
#include "../src/routing/transition.hpp"
#include "test_support.hpp"

#include <chrono>
#include <algorithm>
#include <limits>
#include <map>
#include <random>
#include <vector>

using namespace std::chrono_literals;

TEST_CASE("solver state dominance preserves operational configurations") {
    const sailroute::detail::SolverStateKey base{
        42U,
        3,
        sailroute::TimePoint{100s},
        sailroute::detail::OperationalConfiguration{1, 0U, 0U}};
    const sailroute::detail::SolverLabelIdentity earlier{
        base,
        sailroute::TimePoint{100s},
        1U,
        2U,
        3U};
    auto later = earlier;
    later.arrival += 1s;
    later.state.arrival = later.arrival;

    REQUIRE(!sailroute::detail::dominates(earlier, later));
    REQUIRE(!sailroute::detail::dominates(later, earlier));

    auto duplicate = earlier;
    duplicate.ordinal += 1U;
    REQUIRE(sailroute::detail::dominates(earlier, duplicate));

    auto other_board = later;
    other_board.state.configuration.board = -1;
    REQUIRE(!sailroute::detail::dominates(earlier, other_board));

    auto other_sail = later;
    other_sail.state.configuration.sail = 1U;
    REQUIRE(!sailroute::detail::dominates(earlier, other_sail));

    auto other_reef = later;
    other_reef.state.configuration.reef = 1U;
    REQUIRE(!sailroute::detail::dominates(earlier, other_reef));

    auto other_time_bucket = later;
    other_time_bucket.state.time_bucket += 1;
    REQUIRE(!sailroute::detail::dominates(earlier, other_time_bucket));
}

TEST_CASE("transition primitives use knots and degrees true") {
    sailroute::RoutingOptions options;
    const auto wind = sailroute::detail::evaluate_wind(
        sailroute::Wind{0.0, -10.0},
        options);
    REQUIRE(wind.has_value());
    REQUIRE(wind.value().has_value());
    REQUIRE_NEAR(wind.value()->speed_knots, 19.438444924406049, 1.0e-12);
    REQUIRE_NEAR(wind.value()->direction_from_degrees, 0.0, 1.0e-12);

    REQUIRE(sailroute::detail::board_for_heading(90.0, 0.0) == 1);
    REQUIRE(sailroute::detail::board_for_heading(270.0, 0.0) == -1);
    REQUIRE(sailroute::detail::board_for_heading(180.0, 0.0) == 0);
}

TEST_CASE("transition primitives distinguish errors from infeasible actions") {
    sailroute::RoutingOptions options;
    const auto invalid = sailroute::detail::evaluate_wind(
        sailroute::Wind{std::numeric_limits<double>::quiet_NaN(), 0.0},
        options);
    REQUIRE(!invalid.has_value());
    REQUIRE(
        invalid.error().code ==
        sailroute::ErrorCode::incomplete_forecast);

    options.maximum_true_wind_speed_knots = 10.0;
    const auto limited = sailroute::detail::evaluate_wind(
        sailroute::Wind{0.0, -10.0},
        options);
    REQUIRE(limited.has_value());
    REQUIRE(!limited.value().has_value());
}

TEST_CASE("maneuver delay retains reserved sail and reef identities") {
    sailroute::ManeuverPenalties penalties;
    penalties.tack_penalty = 30s;
    penalties.gybe_penalty = 20s;
    const sailroute::detail::OperationalConfiguration port{-1, 0U, 0U};
    const sailroute::detail::OperationalConfiguration starboard{1, 0U, 0U};

    REQUIRE(
        sailroute::detail::maneuver_delay(
            penalties,
            port,
            45.0,
            starboard,
            45.0) == 30s);
    REQUIRE(
        sailroute::detail::maneuver_delay(
            penalties,
            port,
            170.0,
            starboard,
            170.0) == 20s);
    REQUIRE(
        sailroute::detail::maneuver_delay(
            penalties,
            port,
            45.0,
            port,
            45.0) == 0s);
}

TEST_CASE("state ordering is independent of insertion order") {
    std::vector<sailroute::detail::SolverStateKey> states;
    for (std::size_t index = 0U; index < 32U; ++index) {
        states.push_back(sailroute::detail::SolverStateKey{
            index % 7U,
            static_cast<std::int64_t>(index % 5U),
            sailroute::TimePoint{
                std::chrono::seconds{static_cast<std::int64_t>(index * 11U)}},
            sailroute::detail::OperationalConfiguration{
                static_cast<std::int8_t>(
                    static_cast<int>(index % 3U) - 1),
                static_cast<std::uint32_t>(index % 2U),
                static_cast<std::uint32_t>(index % 4U)}});
    }

    std::map<sailroute::detail::SolverStateKey, std::size_t> reference;
    for (std::size_t index = 0U; index < states.size(); ++index) {
        reference.emplace(states[index], index);
    }
    const std::vector<sailroute::detail::SolverStateKey> expected = [&] {
        std::vector<sailroute::detail::SolverStateKey> ordered;
        for (const auto& [state, index] : reference) {
            static_cast<void>(index);
            ordered.push_back(state);
        }
        return ordered;
    }();

    std::mt19937 generator{42U};
    for (std::size_t repetition = 0U; repetition < 16U; ++repetition) {
        std::shuffle(states.begin(), states.end(), generator);
        std::map<sailroute::detail::SolverStateKey, std::size_t> shuffled;
        for (std::size_t index = 0U; index < states.size(); ++index) {
            shuffled.emplace(states[index], index);
        }
        std::vector<sailroute::detail::SolverStateKey> actual;
        for (const auto& [state, index] : shuffled) {
            static_cast<void>(index);
            actual.push_back(state);
        }
        REQUIRE(actual == expected);
    }
}
