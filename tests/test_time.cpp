#include "sailroute/time.hpp"

#include "test_support.hpp"

using namespace std::chrono_literals;

TEST_CASE("UTC timestamps round trip") {
    const auto parsed = sailroute::parse_utc_time("2026-07-14T16:19:01Z");
    REQUIRE(parsed.has_value());
    REQUIRE(sailroute::format_utc_time(parsed.value()) == "2026-07-14T16:19:01Z");
}

TEST_CASE("invalid UTC timestamps are rejected") {
    REQUIRE(!sailroute::parse_utc_time("2026-02-30T12:00:00Z").has_value());
    REQUIRE(!sailroute::parse_utc_time("2026-07-14 12:00:00").has_value());
    REQUIRE(!sailroute::parse_utc_time("2026-07-14T25:00:00Z").has_value());
    REQUIRE(!sailroute::parse_utc_time("2026-07-14T-1:00:00Z").has_value());
    REQUIRE(!sailroute::parse_utc_time("2026-07-14T12:-1:00Z").has_value());
    REQUIRE(!sailroute::parse_utc_time("2026-07-14T12:00:-1Z").has_value());
}
