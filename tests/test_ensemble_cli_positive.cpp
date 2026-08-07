/// Positive end-to-end CLI integration test for ensemble mode.
///
/// Writes two constant-wind GRIB fixtures, invokes the sailroute CLI binary
/// with --ensemble-member for both, and asserts exit code 0 and JSON output
/// containing the ensemble_route_result_v1 schema version.
///
/// The CLI binary path is injected at compile time via SAILROUTE_CLI_PATH.
/// The test requires ECCODES environment (ECCODES_SAMPLES_PATH) to be
/// available, matching the same requirement as the compatibility corpus.

#include "grib_fixture.hpp"
#include "sailroute/serialization.hpp"
#include "sailroute/time.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#ifndef SAILROUTE_CLI_PATH
#error "SAILROUTE_CLI_PATH must be defined by CMake"
#endif

namespace {

/// Run a shell command, capture combined stdout+stderr, return exit code.
int run_command(const std::string& cmd, std::string& output) {
    output.clear();
#ifdef _WIN32
    FILE* pipe = _popen(cmd.c_str(), "r");
#else
    FILE* pipe = popen(cmd.c_str(), "r");
#endif
    if (pipe == nullptr)
        throw std::runtime_error("popen failed: " + cmd);

    std::array<char, 4096> buf{};
    while (std::fgets(buf.data(), static_cast<int>(buf.size()), pipe) != nullptr)
        output.append(buf.data());

#ifdef _WIN32
    return _pclose(pipe);
#else
    int raw = pclose(pipe);
#if defined(WIFEXITED) && defined(WEXITSTATUS)
    if (WIFEXITED(raw))
        return WEXITSTATUS(raw);
#endif
    return raw;
#endif
}

/// Escape a path for shell inclusion (single-quote wrapping for Unix).
std::string shell_quote(const std::filesystem::path& p) {
    std::string s;
    s.push_back('\'');
    for (char c : p.string()) {
        if (c == '\'') s.append("'\\''");
        else           s.push_back(c);
    }
    s.push_back('\'');
    return s;
}

}  // namespace

int main() {
    // Write two GRIB fixtures: identical constant southward wind so that the
    // ensemble route is deterministic and both members should reach the
    // destination within the short routing window.
    const sailroute::test::ConstantWindGribFixture::Options fixture_options{
        .initial_forecast_hour = 6,
        .final_forecast_hour = 24};
    sailroute::test::ConstantWindGribFixture member_a{fixture_options};
    sailroute::test::ConstantWindGribFixture member_b{fixture_options};

    const std::string cli_path = SAILROUTE_CLI_PATH;

    // Build the command:
    //   sailroute \
    //     --ensemble-member gfs:1.0:/path/to/member_a.grib \
    //     --ensemble-member ecmwf:1.0:/path/to/member_b.grib \
    //     --start 2.0,1.0 \
    //     --destination 0.0,1.0 \
    //     --lattice-level 0
    //
    // Start is at the northern edge, destination at the southern edge.  The
    // fixture's wind blows south (-10 m/s north component), so this is a
    // pure downwind run — quick to solve at any subdivision level.
    // --lattice-level 0 uses the coarsest lattice to keep the test fast.
    const std::string cmd =
        shell_quote(cli_path) +
        " --ensemble-member gfs:1.0:" + shell_quote(member_a.path()) +
        " --ensemble-member ecmwf:1.0:" + shell_quote(member_b.path()) +
        " --start 2.0,1.0"
        " --destination 0.0,1.0"
        " --lattice-level 0"
        " --lattice-time-bucket-minutes 30"
        " --lattice-search a-star"
        " 2>&1";

    std::string output;
    const int exit_code = run_command(cmd, output);

    if (exit_code != 0) {
        std::cerr
            << "sailroute ensemble CLI returned non-zero exit code "
            << exit_code << "\nOutput:\n"
            << output << '\n';
        return 1;
    }

    const bool has_schema =
        output.find("ensemble_route_result_v1") != std::string::npos;
    if (!has_schema) {
        std::cerr
            << "sailroute ensemble CLI output missing schema_version "
               "\"ensemble_route_result_v1\"\nOutput:\n"
            << output << '\n';
        return 1;
    }

    const bool has_members =
        output.find("\"members\":[") != std::string::npos ||
        output.find("\"members\": [") != std::string::npos;
    if (!has_members) {
        std::cerr
            << "sailroute ensemble CLI output missing \"members\" array\n"
               "Output:\n"
            << output << '\n';
        return 1;
    }
    const auto parsed = sailroute::ensemble_route_from_json(output);
    const auto initialization =
        sailroute::parse_utc_time("2026-07-14T12:00:00Z");
    const auto first_valid =
        sailroute::parse_utc_time("2026-07-14T18:00:00Z");
    if (!parsed || !initialization || !first_valid ||
        parsed.value().metadata.initialization_time !=
            initialization.value() ||
        parsed.value().member_metadata.front().weather.first_valid_time !=
            first_valid.value()) {
        std::cerr
            << "sailroute ensemble CLI output has incorrect forecast-cycle "
               "metadata\nOutput:\n"
            << output << '\n';
        return 1;
    }

    const std::string single_member_cmd =
        shell_quote(cli_path) +
        " --ensemble-member gfs:1.0:" + shell_quote(member_a.path()) +
        " --start 2.0,1.0"
        " --destination 0.0,1.0"
        " --lattice-level 0"
        " 2>&1";
    output.clear();
    const int single_member_exit =
        run_command(single_member_cmd, output);
    if (single_member_exit != 0 ||
        output.find("ensemble_route_result_v1") == std::string::npos) {
        std::cerr
            << "sailroute single-member ensemble CLI failed with exit code "
            << single_member_exit << "\nOutput:\n"
            << output << '\n';
        return 1;
    }

    std::cout << "OK: ensemble CLI positive test passed\n";
    return 0;
}
