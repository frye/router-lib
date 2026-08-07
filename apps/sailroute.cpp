#include "sailroute/sailroute.hpp"
#include "sailroute/serialization.hpp"

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr int exit_usage = 2;
constexpr int exit_input = 3;
constexpr int exit_routing = 4;
constexpr int exit_output = 5;

struct CliOptions {
    std::filesystem::path grib_path;
    std::optional<std::filesystem::path> polar_path;
    std::string json_path{"-"};
    std::optional<std::filesystem::path> gpx_path;
    std::optional<std::filesystem::path> isochrones_json_path;
    std::optional<std::filesystem::path> isochrones_gpx_path;
    sailroute::Coordinate start;
    sailroute::Coordinate destination;
    std::optional<sailroute::TimePoint> departure;
    sailroute::RoutingOptions routing;
    bool help{};
};

/// One named, weighted GRIB member specified by --ensemble-member ID:WEIGHT:PATH.
struct EnsembleMemberSpec {
    std::string identifier;
    double weight{1.0};
    std::filesystem::path grib_path;
};

struct EnsembleCliOptions {
    std::vector<EnsembleMemberSpec> members;
    std::string run_id;
    sailroute::EnsembleObjectiveKind objective{
        sailroute::EnsembleObjectiveKind::weighted_mean_elapsed_arrival};
    std::optional<double> target_seconds;
    std::optional<std::filesystem::path> rival_path;
    bool experimental_beam{false};
    std::string json_path{"-"};
    std::optional<std::filesystem::path> polar_path;
    sailroute::Coordinate start;
    sailroute::Coordinate destination;
    std::optional<sailroute::TimePoint> departure;
    sailroute::RoutingOptions routing;
    sailroute::EnsembleLatticeRoutingOptions lattice;
    sailroute::EnsembleBeamRoutingOptions beam;
    bool help{};
};

void print_help(std::ostream& output) {
    output <<
        "Usage: sailroute --grib PATH --start LAT,LON --destination LAT,LON [options]\n"
        "\n"
        "Required:\n"
        "  --grib PATH                         GRIB forecast file\n"
        "  --start LAT,LON                     Departure coordinate\n"
        "  --destination LAT,LON               Destination coordinate\n"
        "\n"
        "Input and output:\n"
        "  --departure YYYY-MM-DDTHH:MM:SSZ    UTC departure time\n"
        "  --polar PATH                        Vessel polar file (default: built-in)\n"
        "  --json PATH|-                       JSON output (default: stdout)\n"
        "  --gpx PATH                          Also write a GPX 1.1 track\n"
        "  --isochrones-json PATH              Write retained frontiers as GeoJSON\n"
        "  --isochrones-gpx PATH               Write retained frontiers as GPX 1.1\n"
        "\n"
        "Routing controls:\n"
        "  --solver MODE                       isochrone|lattice (default: isochrone)\n"
        "  --lattice-level N                  Icosphere subdivision level (0-8)\n"
        "  --lattice-time-bucket-minutes N    Time-state bucket width (> 0)\n"
        "  --lattice-refinement-levels N      Coarse-to-fine levels (default: 1)\n"
        "  --lattice-corridor-nm N            Refinement corridor width (> 0)\n"
        "  --lattice-corridor-retries N       Bounded widening retries (default: 2)\n"
        "  --lattice-progress-expansions N    Search callback cadence (> 0)\n"
        "  --lattice-search MODE              a-star|dijkstra (default: a-star)\n"
        "  --routing-intervals SPEC            Interval schedule (default: 30m@4h,1h@24h,3h)\n"
        "  --time-step-minutes N               Constant interval in minutes (>= 5)\n"
        "  --heading-step-degrees N            Heading increment (0 < N <= 180)\n"
        "  --arrival-radius-nm N               Arrival radius (> 0)\n"
        "  --spatial-bucket-nm N               Spatial pruning bucket size (> 0)\n"
        "  --max-nodes-per-bucket N            Nodes retained per bucket (> 0)\n"
        "  --tack-penalty-seconds N            Time lost to a tack (default 0)\n"
        "  --gybe-penalty-seconds N            Time lost to a gybe (default 0)\n"
        "  --downwind-twa-degrees N            Downwind TWA threshold (default 150)\n"
        "  --heading-augmentation MODE         none|destination-bearing|vmg|both\n"
        "  --wind-sampling MODE                segment-start|midpoint\n"
        "  --midpoint-wind-threshold-minutes N Minimum step for midpoint sampling\n"
        "  --polar-angle-interpolation MODE    linear|monotone-cubic\n"
        "  --maximum-wind-speed-knots N        Wind speed the vessel will not sail in\n"
        "  --above-polar-range MODE            clamp|no-speed\n"
        "  --pruning-strategy MODE             distance-grid|bearing-sectors\n"
        "  --pruning-sector-degrees N          Sector width for bearing-sectors\n"
        "  --worker-count N                    Worker threads (0 selects automatic)\n"
        "  --maximum-route-duration-hours N    Maximum route duration (> 0)\n"
        "  --minimum-boat-speed-knots N        Minimum usable speed (>= 0)\n"
        "\n"
        "  -h, --help                          Show this help\n"
        "\n"
        "Ensemble mode (mutually exclusive with --grib):\n"
        "  --ensemble-member ID:WEIGHT:PATH    Add a named weighted GRIB member\n"
        "                                      (repeatable; at least one required)\n"
        "                                      PATH may contain colons after the\n"
        "                                      second delimiter\n"
        "  --ensemble-objective KIND           Risk objective:\n"
        "                                        weighted_mean_elapsed_arrival (default)\n"
        "                                        weighted_p75_elapsed_arrival\n"
        "                                        weighted_p90_elapsed_arrival\n"
        "                                        probability_before_target\n"
        "                                        probability_beating_rival\n"
        "  --ensemble-target-seconds N         Elapsed-arrival threshold (seconds,\n"
        "                                      required for probability_before_target)\n"
        "  --ensemble-rival PATH               Rival outcomes JSON file in\n"
        "                                      ensemble_rival_outcomes_v1 format\n"
        "                                      (required for probability_beating_rival)\n"
        "  --ensemble-run-id ID                Run identifier embedded in output\n"
        "  --experimental-ensemble-beam        Enable experimental isochrone beam\n"
        "                                      solver (default: time_dependent_lattice)\n"
        "  --json PATH|-                       Ensemble JSON output (default: stdout)\n"
        "\n"
        "Ensemble mode rejects: --gpx, --isochrones-json, --isochrones-gpx\n";
}

sailroute::Error usage_error(std::string message) {
    return sailroute::Error{sailroute::ErrorCode::invalid_argument, std::move(message)};
}

bool parse_double(std::string_view text, double& value) {
    if (text.empty()) {
        return false;
    }
    const char* const begin = text.data();
    const char* const end = begin + text.size();
    const auto parsed = std::from_chars(begin, end, value, std::chars_format::general);
    return parsed.ec == std::errc{} && parsed.ptr == end && std::isfinite(value);
}

bool parse_unsigned(std::string_view text, unsigned long long& value) {
    if (text.empty()) {
        return false;
    }
    const char* const begin = text.data();
    const char* const end = begin + text.size();
    const auto parsed = std::from_chars(begin, end, value);
    return parsed.ec == std::errc{} && parsed.ptr == end;
}

sailroute::Result<std::chrono::minutes> parse_duration_minutes(
    std::string_view text,
    std::string_view description) {
    if (text.size() < 2U) {
        return usage_error(
            std::string{description} + " must be an integer followed by m or h");
    }

    unsigned long long multiplier = 0U;
    switch (text.back()) {
        case 'm':
            multiplier = 1U;
            break;
        case 'h':
            multiplier = 60U;
            break;
        default:
            return usage_error(
                std::string{description} + " must use an m or h suffix");
    }

    unsigned long long value = 0U;
    using Rep = std::chrono::minutes::rep;
    const auto maximum =
        static_cast<unsigned long long>(std::numeric_limits<Rep>::max());
    if (!parse_unsigned(text.substr(0U, text.size() - 1U), value) ||
        value == 0U ||
        value > maximum / multiplier) {
        return usage_error(
            std::string{description} + " must be a positive duration");
    }
    return std::chrono::minutes{
        static_cast<Rep>(value * multiplier)};
}

sailroute::Result<std::vector<sailroute::RoutingInterval>>
parse_routing_intervals(std::string_view text) {
    if (text.empty()) {
        return usage_error("--routing-intervals must not be empty");
    }
    if (text.back() == ',') {
        return usage_error(
            "--routing-intervals must not contain empty tiers");
    }

    std::vector<sailroute::RoutingInterval> intervals;
    std::chrono::minutes previous_cutoff = std::chrono::minutes::zero();
    std::size_t offset = 0U;
    while (offset < text.size()) {
        const std::size_t comma = text.find(',', offset);
        const bool final_tier = comma == std::string_view::npos;
        const std::string_view tier =
            final_tier ? text.substr(offset) : text.substr(offset, comma - offset);
        if (tier.empty()) {
            return usage_error(
                "--routing-intervals must not contain empty tiers");
        }

        const std::size_t at = tier.find('@');
        if (at != std::string_view::npos &&
            tier.find('@', at + 1U) != std::string_view::npos) {
            return usage_error(
                "--routing-intervals tiers may contain at most one @");
        }
        auto interval = parse_duration_minutes(
            tier.substr(0U, at),
            "routing interval");
        if (!interval) {
            return interval.error();
        }
        if (interval.value() < std::chrono::minutes{5}) {
            return usage_error(
                "routing intervals must be at least 5 minutes");
        }

        std::optional<std::chrono::minutes> cutoff;
        if (at != std::string_view::npos) {
            if (final_tier) {
                return usage_error(
                    "--routing-intervals must end with an open-ended interval");
            }
            auto parsed_cutoff = parse_duration_minutes(
                tier.substr(at + 1U),
                "routing interval cutoff");
            if (!parsed_cutoff) {
                return parsed_cutoff.error();
            }
            if (parsed_cutoff.value() <= previous_cutoff) {
                return usage_error(
                    "routing interval cutoffs must be strictly increasing");
            }
            cutoff = parsed_cutoff.value();
            previous_cutoff = *cutoff;
        } else if (!final_tier) {
            return usage_error(
                "only the final routing interval may omit a cutoff");
        }

        intervals.push_back(
            sailroute::RoutingInterval{interval.value(), cutoff});
        if (final_tier) {
            break;
        }
        offset = comma + 1U;
    }
    return intervals;
}

sailroute::Result<sailroute::Coordinate> parse_coordinate(
    std::string_view text,
    std::string_view option) {
    const std::size_t comma = text.find(',');
    if (comma == std::string_view::npos || comma == 0 || comma + 1 >= text.size() ||
        text.find(',', comma + 1) != std::string_view::npos) {
        return usage_error(std::string{option} + " must use LAT,LON");
    }

    sailroute::Coordinate coordinate;
    if (!parse_double(text.substr(0, comma), coordinate.latitude_degrees) ||
        !parse_double(text.substr(comma + 1), coordinate.longitude_degrees)) {
        return usage_error(std::string{option} + " must contain finite numeric coordinates");
    }
    if (!sailroute::is_valid(coordinate)) {
        return usage_error(
            std::string{option} + " latitude must be in [-90,90] and longitude in [-180,180]");
    }
    return coordinate;
}

bool is_option(std::string_view argument, std::string_view canonical, std::string_view alias = {}) {
    return argument == canonical || (!alias.empty() && argument == alias);
}

sailroute::Result<CliOptions> parse_arguments(int argc, char* argv[]) {
    CliOptions options;
    bool grib_seen = false;
    bool start_seen = false;
    bool destination_seen = false;
    bool departure_seen = false;
    bool polar_seen = false;
    bool json_seen = false;
    bool gpx_seen = false;
    bool isochrones_json_seen = false;
    bool isochrones_gpx_seen = false;
    bool routing_intervals_seen = false;
    bool time_step_seen = false;
    bool heading_step_seen = false;
    bool arrival_radius_seen = false;
    bool spatial_bucket_seen = false;
    bool max_nodes_seen = false;
    bool workers_seen = false;
    bool max_duration_seen = false;
    bool minimum_speed_seen = false;
    bool tack_penalty_seen = false;
    bool gybe_penalty_seen = false;
    bool downwind_threshold_seen = false;
    bool heading_augmentation_seen = false;
    bool wind_sampling_seen = false;
    bool midpoint_threshold_seen = false;
    bool polar_interpolation_seen = false;
    bool maximum_wind_speed_seen = false;
    bool above_polar_range_seen = false;
    bool pruning_strategy_seen = false;
    bool pruning_sector_seen = false;
    bool solver_seen = false;
    bool lattice_level_seen = false;
    bool lattice_bucket_seen = false;
    bool lattice_refinement_seen = false;
    bool lattice_corridor_seen = false;
    bool lattice_corridor_retries_seen = false;
    bool lattice_progress_seen = false;
    bool lattice_search_seen = false;

    for (int index = 1; index < argc; ++index) {
        const std::string_view argument{argv[index]};
        if (argument == "--help" || argument == "-h") {
            options.help = true;
            continue;
        }

        const auto value_after = [&](std::string_view name)
            -> sailroute::Result<std::string_view> {
            if (index + 1 >= argc) {
                return usage_error(std::string{name} + " requires a value");
            }
            ++index;
            return std::string_view{argv[index]};
        };
        const auto reject_duplicate = [](bool& seen, std::string_view name)
            -> std::optional<sailroute::Error> {
            if (seen) {
                return usage_error(std::string{name} + " may only be specified once");
            }
            seen = true;
            return std::nullopt;
        };

        if (argument == "--grib") {
            if (const auto duplicate = reject_duplicate(grib_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value().empty()) {
                return usage_error("--grib path must not be empty");
            }
            options.grib_path = value.value();
        } else if (argument == "--start") {
            if (const auto duplicate = reject_duplicate(start_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            auto coordinate = parse_coordinate(value.value(), argument);
            if (!coordinate) {
                return coordinate.error();
            }
            options.start = coordinate.value();
        } else if (argument == "--destination") {
            if (const auto duplicate = reject_duplicate(destination_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            auto coordinate = parse_coordinate(value.value(), argument);
            if (!coordinate) {
                return coordinate.error();
            }
            options.destination = coordinate.value();
        } else if (argument == "--departure") {
            if (const auto duplicate = reject_duplicate(departure_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            auto departure = sailroute::parse_utc_time(value.value());
            if (!departure) {
                return usage_error(departure.error().message);
            }
            options.departure = departure.value();
        } else if (argument == "--polar") {
            if (const auto duplicate = reject_duplicate(polar_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value().empty()) {
                return usage_error("--polar path must not be empty");
            }
            options.polar_path = std::filesystem::path{value.value()};
        } else if (argument == "--json") {
            if (const auto duplicate = reject_duplicate(json_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value().empty()) {
                return usage_error("--json path must not be empty");
            }
            options.json_path = value.value();
        } else if (argument == "--gpx") {
            if (const auto duplicate = reject_duplicate(gpx_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value().empty() || value.value() == "-") {
                return usage_error("--gpx requires a file path");
            }
            options.gpx_path = std::filesystem::path{value.value()};
        } else if (argument == "--isochrones-json") {
            if (const auto duplicate =
                    reject_duplicate(isochrones_json_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value().empty() || value.value() == "-") {
                return usage_error("--isochrones-json requires a file path");
            }
            options.isochrones_json_path =
                std::filesystem::path{value.value()};
        } else if (argument == "--isochrones-gpx") {
            if (const auto duplicate =
                    reject_duplicate(isochrones_gpx_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value().empty() || value.value() == "-") {
                return usage_error("--isochrones-gpx requires a file path");
            }
            options.isochrones_gpx_path =
                std::filesystem::path{value.value()};
        } else if (argument == "--solver") {
            if (const auto duplicate = reject_duplicate(solver_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value() == "isochrone") {
                options.routing.solver = sailroute::RoutingSolver::isochrone_beam;
            } else if (value.value() == "lattice") {
                options.routing.solver =
                    sailroute::RoutingSolver::time_dependent_lattice;
            } else {
                return usage_error("--solver must be isochrone or lattice");
            }
        } else if (
            argument == "--lattice-level" ||
            argument == "--lattice-refinement-levels" ||
            argument == "--lattice-corridor-retries" ||
            argument == "--lattice-time-bucket-minutes" ||
            argument == "--lattice-progress-expansions") {
            bool* seen = &lattice_level_seen;
            if (argument == "--lattice-refinement-levels") {
                seen = &lattice_refinement_seen;
            } else if (argument == "--lattice-corridor-retries") {
                seen = &lattice_corridor_retries_seen;
            } else if (argument == "--lattice-time-bucket-minutes") {
                seen = &lattice_bucket_seen;
            } else if (argument == "--lattice-progress-expansions") {
                seen = &lattice_progress_seen;
            }
            if (const auto duplicate = reject_duplicate(*seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0U;
            if (!parse_unsigned(value.value(), parsed) ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<std::size_t>::max())) {
                return usage_error(std::string{argument} + " must be a practical integer");
            }
            if (argument == "--lattice-level") {
                if (parsed > 8U) {
                    return usage_error("--lattice-level must be in [0,8]");
                }
                options.routing.lattice.subdivision_level =
                    static_cast<std::size_t>(parsed);
            } else if (argument == "--lattice-refinement-levels") {
                options.routing.lattice.refinement_levels =
                    static_cast<std::size_t>(parsed);
            } else if (argument == "--lattice-corridor-retries") {
                options.routing.lattice.corridor_widening_retries =
                    static_cast<std::size_t>(parsed);
            } else if (parsed == 0U) {
                return usage_error(std::string{argument} + " must be positive");
            } else if (argument == "--lattice-time-bucket-minutes") {
                using Rep = std::chrono::minutes::rep;
                if (parsed > static_cast<unsigned long long>(
                                 std::numeric_limits<Rep>::max())) {
                    return usage_error(
                        "--lattice-time-bucket-minutes is too large");
                }
                options.routing.lattice.time_bucket =
                    std::chrono::minutes{static_cast<Rep>(parsed)};
            } else {
                options.routing.lattice.progress_every_n_expansions =
                    static_cast<std::size_t>(parsed);
            }
        } else if (argument == "--lattice-search") {
            if (const auto duplicate =
                    reject_duplicate(lattice_search_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            if (value.value() == "a-star") {
                options.routing.lattice.search_algorithm =
                    sailroute::LatticeSearchAlgorithm::a_star;
            } else if (value.value() == "dijkstra") {
                options.routing.lattice.search_algorithm =
                    sailroute::LatticeSearchAlgorithm::dijkstra;
            } else {
                return usage_error(
                    "--lattice-search must be a-star or dijkstra");
            }
        } else if (argument == "--lattice-corridor-nm") {
            if (const auto duplicate =
                    reject_duplicate(lattice_corridor_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0) {
                return usage_error("--lattice-corridor-nm must be positive");
            }
            options.routing.lattice.corridor_width_nautical_miles = parsed;
        } else if (argument == "--routing-intervals") {
            if (const auto duplicate =
                    reject_duplicate(routing_intervals_seen, argument)) {
                return *duplicate;
            }
            if (time_step_seen) {
                return usage_error(
                    "--routing-intervals and --time-step-minutes are mutually exclusive");
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            auto intervals = parse_routing_intervals(value.value());
            if (!intervals) {
                return intervals.error();
            }
            options.routing.routing_intervals =
                std::move(intervals.value());
            options.routing.use_routing_intervals = true;
        } else if (is_option(argument, "--time-step-minutes", "--time-step")) {
            if (const auto duplicate = reject_duplicate(time_step_seen, "--time-step-minutes")) {
                return *duplicate;
            }
            if (routing_intervals_seen) {
                return usage_error(
                    "--routing-intervals and --time-step-minutes are mutually exclusive");
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0;
            using Rep = std::chrono::minutes::rep;
            if (!parse_unsigned(value.value(), parsed) || parsed < 5U ||
                parsed > static_cast<unsigned long long>(std::numeric_limits<Rep>::max())) {
                return usage_error(
                    std::string{argument} +
                    " must be an integer of at least 5");
            }
            options.routing.time_step = std::chrono::minutes{static_cast<Rep>(parsed)};
            options.routing.use_routing_intervals = false;
        } else if (is_option(argument, "--heading-step-degrees", "--heading-step")) {
            if (const auto duplicate =
                    reject_duplicate(heading_step_seen, "--heading-step-degrees")) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0 || parsed > 180.0) {
                return usage_error(std::string{argument} + " must be in (0,180]");
            }
            options.routing.heading_step_degrees = parsed;
        } else if (is_option(argument, "--arrival-radius-nm", "--arrival-radius")) {
            if (const auto duplicate =
                    reject_duplicate(arrival_radius_seen, "--arrival-radius-nm")) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0) {
                return usage_error(std::string{argument} + " must be greater than zero");
            }
            options.routing.arrival_radius_nautical_miles = parsed;
        } else if (is_option(argument, "--spatial-bucket-nm", "--spatial-bucket")) {
            if (const auto duplicate =
                    reject_duplicate(spatial_bucket_seen, "--spatial-bucket-nm")) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0) {
                return usage_error(std::string{argument} + " must be greater than zero");
            }
            options.routing.spatial_bucket_nautical_miles = parsed;
        } else if (argument == "--max-nodes-per-bucket") {
            if (const auto duplicate = reject_duplicate(max_nodes_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0;
            if (!parse_unsigned(value.value(), parsed) || parsed == 0 ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<std::size_t>::max())) {
                return usage_error("--max-nodes-per-bucket must be a positive integer");
            }
            options.routing.max_nodes_per_bucket = static_cast<std::size_t>(parsed);
        } else if (
            argument == "--tack-penalty-seconds" ||
            argument == "--gybe-penalty-seconds") {
            const bool is_tack = argument == "--tack-penalty-seconds";
            if (const auto duplicate = reject_duplicate(
                    is_tack ? tack_penalty_seen : gybe_penalty_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0;
            using Rep = std::chrono::seconds::rep;
            if (!parse_unsigned(value.value(), parsed) ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<Rep>::max())) {
                return usage_error(
                    std::string{argument} + " must be a non-negative integer");
            }
            const std::chrono::seconds penalty{static_cast<Rep>(parsed)};
            if (is_tack) {
                options.routing.maneuver.tack_penalty = penalty;
            } else {
                options.routing.maneuver.gybe_penalty = penalty;
            }
        } else if (argument == "--downwind-twa-degrees") {
            if (const auto duplicate =
                    reject_duplicate(downwind_threshold_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed < 0.0 ||
                parsed > 180.0) {
                return usage_error(std::string{argument} + " must be in [0,180]");
            }
            options.routing.maneuver.downwind_true_wind_angle_degrees = parsed;
        } else if (argument == "--heading-augmentation") {
            if (const auto duplicate =
                    reject_duplicate(heading_augmentation_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            const std::string_view text = value.value();
            if (text == "none") {
                options.routing.heading_augmentation =
                    sailroute::HeadingAugmentation::none;
            } else if (text == "destination-bearing") {
                options.routing.heading_augmentation =
                    sailroute::HeadingAugmentation::destination_bearing;
            } else if (text == "vmg") {
                options.routing.heading_augmentation =
                    sailroute::HeadingAugmentation::velocity_made_good;
            } else if (text == "both") {
                options.routing.heading_augmentation = sailroute::
                    HeadingAugmentation::destination_bearing_and_velocity_made_good;
            } else {
                return usage_error(
                    "--heading-augmentation must be none, destination-bearing, "
                    "vmg, or both");
            }
        } else if (argument == "--wind-sampling") {
            if (const auto duplicate = reject_duplicate(wind_sampling_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            const std::string_view text = value.value();
            if (text == "segment-start") {
                options.routing.wind_sampling = sailroute::WindSampling::segment_start;
            } else if (text == "midpoint") {
                options.routing.wind_sampling = sailroute::WindSampling::midpoint;
            } else {
                return usage_error(
                    "--wind-sampling must be segment-start or midpoint");
            }
        } else if (argument == "--midpoint-wind-threshold-minutes") {
            if (const auto duplicate =
                    reject_duplicate(midpoint_threshold_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0;
            using Rep = std::chrono::minutes::rep;
            if (!parse_unsigned(value.value(), parsed) ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<Rep>::max())) {
                return usage_error(
                    std::string{argument} + " must be a non-negative integer");
            }
            options.routing.midpoint_wind_sampling_threshold =
                std::chrono::minutes{static_cast<Rep>(parsed)};
        } else if (argument == "--polar-angle-interpolation") {
            if (const auto duplicate =
                    reject_duplicate(polar_interpolation_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            const std::string_view text = value.value();
            if (text == "linear") {
                options.routing.polar_angle_interpolation =
                    sailroute::PolarAngleInterpolation::linear;
            } else if (text == "monotone-cubic") {
                options.routing.polar_angle_interpolation =
                    sailroute::PolarAngleInterpolation::monotone_cubic;
            } else {
                return usage_error(
                    "--polar-angle-interpolation must be linear or monotone-cubic");
            }
        } else if (argument == "--maximum-wind-speed-knots") {
            if (const auto duplicate =
                    reject_duplicate(maximum_wind_speed_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0) {
                return usage_error(
                    std::string{argument} + " must be greater than zero");
            }
            options.routing.maximum_true_wind_speed_knots = parsed;
        } else if (argument == "--above-polar-range") {
            if (const auto duplicate =
                    reject_duplicate(above_polar_range_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            const std::string_view text = value.value();
            if (text == "clamp") {
                options.routing.above_polar_range =
                    sailroute::AbovePolarRangePolicy::clamp;
            } else if (text == "no-speed") {
                options.routing.above_polar_range =
                    sailroute::AbovePolarRangePolicy::no_speed;
            } else {
                return usage_error("--above-polar-range must be clamp or no-speed");
            }
        } else if (argument == "--pruning-strategy") {
            if (const auto duplicate =
                    reject_duplicate(pruning_strategy_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            const std::string_view text = value.value();
            if (text == "distance-grid") {
                options.routing.pruning_strategy =
                    sailroute::PruningStrategy::destination_distance_grid;
            } else if (text == "bearing-sectors") {
                options.routing.pruning_strategy =
                    sailroute::PruningStrategy::bearing_sectors;
            } else {
                return usage_error(
                    "--pruning-strategy must be distance-grid or bearing-sectors");
            }
        } else if (argument == "--pruning-sector-degrees") {
            if (const auto duplicate =
                    reject_duplicate(pruning_sector_seen, argument)) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0 ||
                parsed > 180.0) {
                return usage_error(std::string{argument} + " must be in (0,180]");
            }
            options.routing.pruning_sector_degrees = parsed;
        } else if (is_option(argument, "--worker-count", "--workers")) {
            if (const auto duplicate = reject_duplicate(workers_seen, "--worker-count")) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0;
            if (!parse_unsigned(value.value(), parsed) ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<std::size_t>::max())) {
                return usage_error(std::string{argument} + " must be a non-negative integer");
            }
            options.routing.worker_count = static_cast<std::size_t>(parsed);
        } else if (
            is_option(
                argument,
                "--maximum-route-duration-hours",
                "--max-duration-hours") ||
            argument == "--max-duration") {
            if (const auto duplicate =
                    reject_duplicate(max_duration_seen, "--maximum-route-duration-hours")) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0;
            using Rep = std::chrono::hours::rep;
            if (!parse_unsigned(value.value(), parsed) || parsed == 0 ||
                parsed > static_cast<unsigned long long>(std::numeric_limits<Rep>::max())) {
                return usage_error(std::string{argument} + " must be a positive integer");
            }
            options.routing.maximum_route_duration =
                std::chrono::hours{static_cast<Rep>(parsed)};
        } else if (
            is_option(
                argument,
                "--minimum-boat-speed-knots",
                "--min-boat-speed")) {
            if (const auto duplicate =
                    reject_duplicate(minimum_speed_seen, "--minimum-boat-speed-knots")) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed < 0.0) {
                return usage_error(std::string{argument} + " must be non-negative");
            }
            options.routing.minimum_boat_speed_knots = parsed;
        } else {
            return usage_error("unknown option: " + std::string{argument});
        }
    }

    if (options.help) {
        return options;
    }
    const bool lattice_option_seen =
        lattice_level_seen ||
        lattice_bucket_seen ||
        lattice_refinement_seen ||
        lattice_corridor_seen ||
        lattice_corridor_retries_seen ||
        lattice_progress_seen ||
        lattice_search_seen;
    if (lattice_option_seen &&
        options.routing.solver !=
            sailroute::RoutingSolver::time_dependent_lattice) {
        return usage_error(
            "lattice options require explicit --solver lattice selection");
    }
    if (!grib_seen || !start_seen || !destination_seen) {
        std::string missing;
        if (!grib_seen) {
            missing.append(" --grib");
        }
        if (!start_seen) {
            missing.append(" --start");
        }
        if (!destination_seen) {
            missing.append(" --destination");
        }
        return usage_error("missing required option(s):" + missing);
    }
    std::vector<std::pair<std::string_view, std::filesystem::path>> outputs;
    if (options.json_path != "-") {
        outputs.emplace_back("--json", options.json_path);
    }
    if (options.gpx_path) {
        outputs.emplace_back("--gpx", *options.gpx_path);
    }
    if (options.isochrones_json_path) {
        outputs.emplace_back(
            "--isochrones-json",
            *options.isochrones_json_path);
    }
    if (options.isochrones_gpx_path) {
        outputs.emplace_back(
            "--isochrones-gpx",
            *options.isochrones_gpx_path);
    }
    for (std::size_t left = 0U; left < outputs.size(); ++left) {
        std::error_code left_error;
        const auto left_path =
            std::filesystem::absolute(outputs[left].second, left_error)
                .lexically_normal();
        if (left_error) {
            continue;
        }
        for (std::size_t right = left + 1U; right < outputs.size(); ++right) {
            std::error_code right_error;
            const auto right_path =
                std::filesystem::absolute(outputs[right].second, right_error)
                    .lexically_normal();
            if (!right_error && left_path == right_path) {
                return usage_error(
                    std::string{outputs[left].first} + " and " +
                    std::string{outputs[right].first} +
                    " must use different output paths");
            }
        }
    }
    return options;
}

int report_error(std::string_view category, const sailroute::Error& error, int exit_code) {
    std::cerr << "sailroute: " << category << ": " << sailroute::to_string(error.code)
              << ": " << error.message << '\n';
    return exit_code;
}

sailroute::Result<std::string> write_file(
    const std::filesystem::path& path,
    std::string_view contents) {
    if (contents.size() >
        static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max())) {
        return sailroute::Error{
            sailroute::ErrorCode::output_error,
            "output is too large to write: " + path.string()};
    }
    std::ofstream output{path, std::ios::binary | std::ios::trunc};
    if (!output) {
        return sailroute::Error{
            sailroute::ErrorCode::output_error,
            "cannot open output file: " + path.string()};
    }
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.close();
    if (!output) {
        return sailroute::Error{
            sailroute::ErrorCode::output_error,
            "failed writing output file: " + path.string()};
    }
    return std::string{};
}

int run(const CliOptions& options) {
    auto weather = sailroute::WeatherDataset::load(options.grib_path);
    if (!weather) {
        return report_error("input", weather.error(), exit_input);
    }

    sailroute::VesselPolar polar = sailroute::VesselPolar::default_racer_cruiser_45ft();
    if (options.polar_path) {
        auto loaded_polar = sailroute::VesselPolar::load(*options.polar_path);
        if (!loaded_polar) {
            return report_error("input", loaded_polar.error(), exit_input);
        }
        polar = std::move(loaded_polar.value());
    }

    sailroute::RoutingOptions routing = options.routing;
    routing.capture_isochrones =
        options.isochrones_json_path.has_value() ||
        options.isochrones_gpx_path.has_value();
    sailroute::RouteRequest request{
        options.start,
        options.destination,
        options.departure,
        routing};
    sailroute::Router router{std::move(weather.value()), std::move(polar)};
    auto route = router.optimize(request);
    if (!route) {
        return report_error("routing", route.error(), exit_routing);
    }
    if (route.value().completion ==
        sailroute::RouteCompletion::forecast_exhausted) {
        std::cerr
            << "sailroute: warning: forecast coverage ended before the "
               "destination was reached; writing the best supported partial "
               "route\n";
    } else if (
        route.value().completion ==
        sailroute::RouteCompletion::duration_exhausted) {
        std::cerr
            << "sailroute: warning: maximum route duration ended before the "
               "destination was reached; writing the best partial route\n";
    }

    auto json = sailroute::route_to_json(route.value());
    if (!json) {
        return report_error("output", json.error(), exit_output);
    }
    if (options.json_path == "-") {
        std::cout << json.value();
        std::cout.flush();
        if (!std::cout) {
            return report_error(
                "output",
                sailroute::Error{
                    sailroute::ErrorCode::output_error,
                    "failed writing JSON to standard output"},
                exit_output);
        }
    } else {
        auto written = write_file(std::filesystem::path{options.json_path}, json.value());
        if (!written) {
            return report_error("output", written.error(), exit_output);
        }
    }

    if (options.gpx_path) {
        auto gpx = sailroute::route_to_gpx(route.value());
        if (!gpx) {
            return report_error("output", gpx.error(), exit_output);
        }
        auto written = write_file(*options.gpx_path, gpx.value());
        if (!written) {
            return report_error("output", written.error(), exit_output);
        }
    }
    if (options.isochrones_json_path) {
        auto json = sailroute::isochrones_to_json(route.value());
        if (!json) {
            return report_error("output", json.error(), exit_output);
        }
        auto written = write_file(
            *options.isochrones_json_path,
            json.value());
        if (!written) {
            return report_error("output", written.error(), exit_output);
        }
    }
    if (options.isochrones_gpx_path) {
        auto gpx = sailroute::isochrones_to_gpx(route.value());
        if (!gpx) {
            return report_error("output", gpx.error(), exit_output);
        }
        auto written = write_file(*options.isochrones_gpx_path, gpx.value());
        if (!written) {
            return report_error("output", written.error(), exit_output);
        }
    }
    return 0;
}

/// Parses --ensemble-member ID:WEIGHT:PATH.
/// Splits on the first two ':' characters only; PATH may itself contain ':'.
sailroute::Result<EnsembleMemberSpec> parse_ensemble_member(
    std::string_view text) {
    const std::size_t first_colon = text.find(':');
    if (first_colon == std::string_view::npos || first_colon == 0U)
        return usage_error(
            "--ensemble-member requires ID:WEIGHT:PATH format");
    const std::size_t second_colon = text.find(':', first_colon + 1U);
    if (second_colon == std::string_view::npos ||
        second_colon == first_colon + 1U)
        return usage_error(
            "--ensemble-member requires ID:WEIGHT:PATH format");
    if (second_colon + 1U >= text.size())
        return usage_error("--ensemble-member PATH must not be empty");

    EnsembleMemberSpec spec;
    spec.identifier = std::string{text.substr(0U, first_colon)};
    if (spec.identifier.empty())
        return usage_error("--ensemble-member identifier must not be empty");

    const auto weight_sv =
        text.substr(first_colon + 1U, second_colon - first_colon - 1U);
    if (!parse_double(weight_sv, spec.weight) || spec.weight < 0.0)
        return usage_error(
            "--ensemble-member weight must be a non-negative finite number");

    spec.grib_path =
        std::filesystem::path{text.substr(second_colon + 1U)};
    return spec;
}

sailroute::Result<EnsembleCliOptions> parse_ensemble_arguments(
    int argc, char* argv[]) {
    EnsembleCliOptions opts;
    bool start_seen = false;
    bool destination_seen = false;
    bool departure_seen = false;
    bool polar_seen = false;
    bool json_seen = false;
    bool objective_seen = false;
    bool target_seen = false;
    bool rival_seen = false;
    bool run_id_seen = false;
    bool experimental_beam_seen = false;
    bool gpx_rejected = false;
    bool isochrones_json_rejected = false;
    bool isochrones_gpx_rejected = false;
    // Shared routing options flags
    bool routing_intervals_seen = false;
    bool time_step_seen = false;
    bool heading_step_seen = false;
    bool arrival_radius_seen = false;
    bool spatial_bucket_seen = false;
    bool max_nodes_seen = false;
    bool workers_seen = false;
    bool max_duration_seen = false;
    bool minimum_speed_seen = false;
    bool tack_penalty_seen = false;
    bool gybe_penalty_seen = false;
    bool downwind_threshold_seen = false;
    bool heading_augmentation_seen = false;
    bool wind_sampling_seen = false;
    bool midpoint_threshold_seen = false;
    bool polar_interpolation_seen = false;
    bool maximum_wind_speed_seen = false;
    bool above_polar_range_seen = false;
    bool pruning_strategy_seen = false;
    bool pruning_sector_seen = false;
    bool lattice_level_seen = false;
    bool lattice_time_bucket_seen = false;
    bool lattice_search_seen = false;

    for (int index = 1; index < argc; ++index) {
        const std::string_view argument{argv[index]};
        if (argument == "--help" || argument == "-h") {
            opts.help = true;
            continue;
        }

        const auto value_after = [&](std::string_view name)
            -> sailroute::Result<std::string_view> {
            if (index + 1 >= argc)
                return usage_error(std::string{name} + " requires a value");
            ++index;
            return std::string_view{argv[index]};
        };
        const auto reject_duplicate = [](bool& seen, std::string_view name)
            -> std::optional<sailroute::Error> {
            if (seen)
                return usage_error(
                    std::string{name} + " may only be specified once");
            seen = true;
            return std::nullopt;
        };

        if (argument == "--grib") {
            return usage_error(
                "--grib is not supported in ensemble mode; use "
                "--ensemble-member instead");
        } else if (argument == "--gpx") {
            return usage_error(
                "--gpx is not supported in ensemble mode");
        } else if (argument == "--isochrones-json") {
            return usage_error(
                "--isochrones-json is not supported in ensemble mode");
        } else if (argument == "--isochrones-gpx") {
            return usage_error(
                "--isochrones-gpx is not supported in ensemble mode");
        } else if (argument == "--ensemble-member") {
            auto value = value_after(argument);
            if (!value) return value.error();
            auto spec = parse_ensemble_member(value.value());
            if (!spec) return spec.error();
            opts.members.push_back(std::move(spec.value()));
        } else if (argument == "--ensemble-objective") {
            if (const auto dup = reject_duplicate(objective_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            const auto sv = value.value();
            if (sv == "weighted_mean_elapsed_arrival")
                opts.objective =
                    sailroute::EnsembleObjectiveKind::weighted_mean_elapsed_arrival;
            else if (sv == "weighted_p75_elapsed_arrival")
                opts.objective =
                    sailroute::EnsembleObjectiveKind::weighted_p75_elapsed_arrival;
            else if (sv == "weighted_p90_elapsed_arrival")
                opts.objective =
                    sailroute::EnsembleObjectiveKind::weighted_p90_elapsed_arrival;
            else if (sv == "probability_before_target")
                opts.objective =
                    sailroute::EnsembleObjectiveKind::probability_before_target;
            else if (sv == "probability_beating_rival")
                opts.objective =
                    sailroute::EnsembleObjectiveKind::probability_beating_rival;
            else
                return usage_error(
                    "--ensemble-objective must be one of: "
                    "weighted_mean_elapsed_arrival, "
                    "weighted_p75_elapsed_arrival, "
                    "weighted_p90_elapsed_arrival, "
                    "probability_before_target, "
                    "probability_beating_rival");
        } else if (argument == "--ensemble-target-seconds") {
            if (const auto dup = reject_duplicate(target_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0)
                return usage_error(
                    "--ensemble-target-seconds must be a positive number");
            opts.target_seconds = parsed;
        } else if (argument == "--ensemble-rival") {
            if (const auto dup = reject_duplicate(rival_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            if (value.value().empty())
                return usage_error("--ensemble-rival path must not be empty");
            opts.rival_path = std::filesystem::path{value.value()};
        } else if (argument == "--ensemble-run-id") {
            if (const auto dup = reject_duplicate(run_id_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            opts.run_id = std::string{value.value()};
        } else if (argument == "--experimental-ensemble-beam") {
            if (const auto dup = reject_duplicate(
                    experimental_beam_seen, argument))
                return *dup;
            opts.experimental_beam = true;
        } else if (argument == "--start") {
            if (const auto dup = reject_duplicate(start_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            auto coord = parse_coordinate(value.value(), argument);
            if (!coord) return coord.error();
            opts.start = coord.value();
        } else if (argument == "--destination") {
            if (const auto dup = reject_duplicate(destination_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            auto coord = parse_coordinate(value.value(), argument);
            if (!coord) return coord.error();
            opts.destination = coord.value();
        } else if (argument == "--departure") {
            if (const auto dup = reject_duplicate(departure_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            auto dep = sailroute::parse_utc_time(value.value());
            if (!dep)
                return usage_error(dep.error().message);
            opts.departure = dep.value();
        } else if (argument == "--polar") {
            if (const auto dup = reject_duplicate(polar_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            if (value.value().empty())
                return usage_error("--polar path must not be empty");
            opts.polar_path = std::filesystem::path{value.value()};
        } else if (argument == "--json") {
            if (const auto dup = reject_duplicate(json_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            if (value.value().empty())
                return usage_error("--json path must not be empty");
            opts.json_path = std::string{value.value()};
        } else if (argument == "--routing-intervals") {
            if (const auto dup = reject_duplicate(
                    routing_intervals_seen, argument))
                return *dup;
            if (time_step_seen)
                return usage_error(
                    "--routing-intervals and --time-step-minutes are mutually exclusive");
            auto value = value_after(argument);
            if (!value) return value.error();
            auto intervals = parse_routing_intervals(value.value());
            if (!intervals) return intervals.error();
            opts.routing.routing_intervals = std::move(intervals.value());
            opts.routing.use_routing_intervals = true;
        } else if (is_option(argument, "--time-step-minutes", "--time-step")) {
            if (const auto dup = reject_duplicate(time_step_seen, "--time-step-minutes"))
                return *dup;
            if (routing_intervals_seen)
                return usage_error(
                    "--routing-intervals and --time-step-minutes are mutually exclusive");
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            using Rep = std::chrono::minutes::rep;
            if (!parse_unsigned(value.value(), parsed) || parsed < 5U ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<Rep>::max()))
                return usage_error(
                    std::string{argument} + " must be an integer of at least 5");
            opts.routing.time_step = std::chrono::minutes{static_cast<Rep>(parsed)};
            opts.routing.use_routing_intervals = false;
            opts.beam.time_step =
                std::chrono::minutes{static_cast<Rep>(parsed)};
        } else if (is_option(argument, "--heading-step-degrees", "--heading-step")) {
            if (const auto dup = reject_duplicate(heading_step_seen, "--heading-step-degrees"))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0 || parsed > 180.0)
                return usage_error(std::string{argument} + " must be in (0,180]");
            opts.routing.heading_step_degrees = parsed;
            opts.beam.heading_step_degrees = parsed;
        } else if (is_option(argument, "--arrival-radius-nm", "--arrival-radius")) {
            if (const auto dup = reject_duplicate(arrival_radius_seen, "--arrival-radius-nm"))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0)
                return usage_error(std::string{argument} + " must be greater than zero");
            opts.routing.arrival_radius_nautical_miles = parsed;
        } else if (is_option(argument, "--spatial-bucket-nm", "--spatial-bucket")) {
            if (const auto dup = reject_duplicate(spatial_bucket_seen, "--spatial-bucket-nm"))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0)
                return usage_error(std::string{argument} + " must be greater than zero");
            opts.routing.spatial_bucket_nautical_miles = parsed;
            opts.beam.centroid_bucket_nautical_miles = parsed;
        } else if (argument == "--max-nodes-per-bucket") {
            if (const auto dup = reject_duplicate(max_nodes_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            if (!parse_unsigned(value.value(), parsed) || parsed == 0 ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<std::size_t>::max()))
                return usage_error("--max-nodes-per-bucket must be a positive integer");
            opts.routing.max_nodes_per_bucket = static_cast<std::size_t>(parsed);
            opts.beam.max_nodes_per_bucket = static_cast<std::size_t>(parsed);
        } else if (
            argument == "--tack-penalty-seconds" ||
            argument == "--gybe-penalty-seconds") {
            const bool is_tack = argument == "--tack-penalty-seconds";
            if (const auto dup = reject_duplicate(
                    is_tack ? tack_penalty_seen : gybe_penalty_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            using Rep = std::chrono::seconds::rep;
            if (!parse_unsigned(value.value(), parsed) ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<Rep>::max()))
                return usage_error(
                    std::string{argument} + " must be a non-negative integer");
            const std::chrono::seconds penalty{static_cast<Rep>(parsed)};
            if (is_tack) opts.routing.maneuver.tack_penalty = penalty;
            else         opts.routing.maneuver.gybe_penalty = penalty;
        } else if (argument == "--downwind-twa-degrees") {
            if (const auto dup = reject_duplicate(downwind_threshold_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed < 0.0 || parsed > 180.0)
                return usage_error(std::string{argument} + " must be in [0,180]");
            opts.routing.maneuver.downwind_true_wind_angle_degrees = parsed;
        } else if (argument == "--heading-augmentation") {
            if (const auto dup = reject_duplicate(heading_augmentation_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            const auto text = value.value();
            if (text == "none")
                opts.routing.heading_augmentation = sailroute::HeadingAugmentation::none;
            else if (text == "destination-bearing")
                opts.routing.heading_augmentation =
                    sailroute::HeadingAugmentation::destination_bearing;
            else if (text == "vmg")
                opts.routing.heading_augmentation =
                    sailroute::HeadingAugmentation::velocity_made_good;
            else if (text == "both")
                opts.routing.heading_augmentation =
                    sailroute::HeadingAugmentation::
                        destination_bearing_and_velocity_made_good;
            else
                return usage_error(
                    "--heading-augmentation must be none, destination-bearing, vmg, or both");
        } else if (argument == "--wind-sampling") {
            if (const auto dup = reject_duplicate(wind_sampling_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            const auto text = value.value();
            if (text == "segment-start")
                opts.routing.wind_sampling = sailroute::WindSampling::segment_start;
            else if (text == "midpoint")
                opts.routing.wind_sampling = sailroute::WindSampling::midpoint;
            else
                return usage_error(
                    "--wind-sampling must be segment-start or midpoint");
        } else if (argument == "--midpoint-wind-threshold-minutes") {
            if (const auto dup = reject_duplicate(midpoint_threshold_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            using Rep = std::chrono::minutes::rep;
            if (!parse_unsigned(value.value(), parsed) ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<Rep>::max()))
                return usage_error(
                    std::string{argument} + " must be a non-negative integer");
            opts.routing.midpoint_wind_sampling_threshold =
                std::chrono::minutes{static_cast<Rep>(parsed)};
        } else if (argument == "--polar-angle-interpolation") {
            if (const auto dup = reject_duplicate(polar_interpolation_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            const auto text = value.value();
            if (text == "linear")
                opts.routing.polar_angle_interpolation =
                    sailroute::PolarAngleInterpolation::linear;
            else if (text == "monotone-cubic")
                opts.routing.polar_angle_interpolation =
                    sailroute::PolarAngleInterpolation::monotone_cubic;
            else
                return usage_error(
                    "--polar-angle-interpolation must be linear or monotone-cubic");
        } else if (argument == "--maximum-wind-speed-knots") {
            if (const auto dup = reject_duplicate(maximum_wind_speed_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0)
                return usage_error(
                    std::string{argument} + " must be greater than zero");
            opts.routing.maximum_true_wind_speed_knots = parsed;
        } else if (argument == "--above-polar-range") {
            if (const auto dup = reject_duplicate(above_polar_range_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            const auto text = value.value();
            if (text == "clamp")
                opts.routing.above_polar_range = sailroute::AbovePolarRangePolicy::clamp;
            else if (text == "no-speed")
                opts.routing.above_polar_range =
                    sailroute::AbovePolarRangePolicy::no_speed;
            else
                return usage_error("--above-polar-range must be clamp or no-speed");
        } else if (argument == "--pruning-strategy") {
            if (const auto dup = reject_duplicate(pruning_strategy_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            const auto text = value.value();
            if (text == "distance-grid")
                opts.routing.pruning_strategy =
                    sailroute::PruningStrategy::destination_distance_grid;
            else if (text == "bearing-sectors")
                opts.routing.pruning_strategy =
                    sailroute::PruningStrategy::bearing_sectors;
            else
                return usage_error(
                    "--pruning-strategy must be distance-grid or bearing-sectors");
        } else if (argument == "--pruning-sector-degrees") {
            if (const auto dup = reject_duplicate(pruning_sector_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed <= 0.0 || parsed > 180.0)
                return usage_error(std::string{argument} + " must be in (0,180]");
            opts.routing.pruning_sector_degrees = parsed;
        } else if (is_option(argument, "--worker-count", "--workers")) {
            if (const auto dup = reject_duplicate(workers_seen, "--worker-count"))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            if (!parse_unsigned(value.value(), parsed) ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<std::size_t>::max()))
                return usage_error(std::string{argument} + " must be a non-negative integer");
            opts.routing.worker_count = static_cast<std::size_t>(parsed);
        } else if (
            is_option(argument, "--maximum-route-duration-hours",
                      "--max-duration-hours") ||
            argument == "--max-duration") {
            if (const auto dup = reject_duplicate(
                    max_duration_seen, "--maximum-route-duration-hours"))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            using Rep = std::chrono::hours::rep;
            if (!parse_unsigned(value.value(), parsed) || parsed == 0 ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<Rep>::max()))
                return usage_error(std::string{argument} + " must be a positive integer");
            opts.routing.maximum_route_duration =
                std::chrono::hours{static_cast<Rep>(parsed)};
        } else if (
            is_option(argument, "--minimum-boat-speed-knots", "--min-boat-speed")) {
            if (const auto dup = reject_duplicate(minimum_speed_seen,
                                                   "--minimum-boat-speed-knots"))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            double parsed = 0.0;
            if (!parse_double(value.value(), parsed) || parsed < 0.0)
                return usage_error(std::string{argument} + " must be non-negative");
            opts.routing.minimum_boat_speed_knots = parsed;
        } else if (argument == "--lattice-level") {
            if (const auto dup = reject_duplicate(lattice_level_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            if (!parse_unsigned(value.value(), parsed))
                return usage_error("--lattice-level must be a practical integer");
            if (parsed > 8U)
                return usage_error("--lattice-level must be in [0,8]");
            opts.lattice.subdivision_level = static_cast<std::size_t>(parsed);
        } else if (argument == "--lattice-time-bucket-minutes") {
            if (const auto dup =
                    reject_duplicate(lattice_time_bucket_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            unsigned long long parsed = 0;
            using Rep = std::chrono::minutes::rep;
            if (!parse_unsigned(value.value(), parsed) || parsed == 0U ||
                parsed > static_cast<unsigned long long>(
                             std::numeric_limits<Rep>::max())) {
                return usage_error(
                    "--lattice-time-bucket-minutes must be a positive integer");
            }
            opts.lattice.time_bucket =
                std::chrono::minutes{static_cast<Rep>(parsed)};
        } else if (argument == "--lattice-search") {
            if (const auto dup = reject_duplicate(lattice_search_seen, argument))
                return *dup;
            auto value = value_after(argument);
            if (!value) return value.error();
            if (value.value() == "a-star") {
                opts.lattice.search_algorithm =
                    sailroute::LatticeSearchAlgorithm::a_star;
            } else if (value.value() == "dijkstra") {
                opts.lattice.search_algorithm =
                    sailroute::LatticeSearchAlgorithm::dijkstra;
            } else {
                return usage_error(
                    "--lattice-search must be a-star or dijkstra");
            }
        } else {
            return usage_error("unknown option: " + std::string{argument});
        }
    }

    if (opts.help) return opts;

    if (opts.members.empty())
        return usage_error(
            "ensemble mode requires at least one --ensemble-member input");
    if (!start_seen || !destination_seen) {
        std::string missing;
        if (!start_seen) missing.append(" --start");
        if (!destination_seen) missing.append(" --destination");
        return usage_error("missing required option(s):" + missing);
    }
    if (opts.objective ==
            sailroute::EnsembleObjectiveKind::probability_before_target &&
        !opts.target_seconds) {
        return usage_error(
            "probability_before_target objective requires "
            "--ensemble-target-seconds");
    }
    if (opts.objective !=
            sailroute::EnsembleObjectiveKind::probability_before_target &&
        opts.target_seconds) {
        return usage_error(
            "--ensemble-target-seconds is only used with "
            "probability_before_target objective");
    }
    if (opts.objective ==
            sailroute::EnsembleObjectiveKind::probability_beating_rival &&
        !opts.rival_path) {
        return usage_error(
            "probability_beating_rival objective requires --ensemble-rival");
    }
    if (opts.objective !=
            sailroute::EnsembleObjectiveKind::probability_beating_rival &&
        opts.rival_path) {
        return usage_error(
            "--ensemble-rival is only used with "
            "probability_beating_rival objective");
    }
    if (routing_intervals_seen) {
        return usage_error(
            "--routing-intervals is not supported in ensemble mode");
    }
    if (workers_seen || heading_augmentation_seen ||
        pruning_strategy_seen || pruning_sector_seen) {
        return usage_error(
            "--worker-count, --heading-augmentation, and deterministic pruning "
            "controls are not supported in ensemble mode");
    }
    if (opts.experimental_beam) {
        if (lattice_level_seen || lattice_time_bucket_seen ||
            lattice_search_seen) {
            return usage_error(
                "lattice controls are not used by "
                "--experimental-ensemble-beam");
        }
    } else if (time_step_seen || heading_step_seen ||
               spatial_bucket_seen || max_nodes_seen) {
        return usage_error(
            "--time-step-minutes, --heading-step-degrees, "
            "--spatial-bucket-nm, and --max-nodes-per-bucket require "
            "--experimental-ensemble-beam in ensemble mode");
    }
    return opts;
}

int run_ensemble(const EnsembleCliOptions& opts) {
    sailroute::VesselPolar polar =
        sailroute::VesselPolar::default_racer_cruiser_45ft();
    if (opts.polar_path) {
        auto loaded = sailroute::VesselPolar::load(*opts.polar_path);
        if (!loaded)
            return report_error("input", loaded.error(), exit_input);
        polar = std::move(loaded.value());
    }

    // Build member input list.
    std::vector<sailroute::EnsembleMemberInput> members;
    members.reserve(opts.members.size());
    for (const auto& spec : opts.members) {
        sailroute::EnsembleMemberInput m;
        m.identifier = spec.identifier;
        m.weight = spec.weight;
        m.grib_path = spec.grib_path;
        members.push_back(std::move(m));
    }

    sailroute::EnsembleRunMetadata meta;
    meta.run_identifier = opts.run_id.empty()
        ? "sailroute-ensemble-" +
              std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                 std::chrono::system_clock::now()
                                     .time_since_epoch())
                                 .count())
        : opts.run_id;
    meta.model_identifier = "sailroute-cli";
    meta.schema_revision = 1U;

    auto dataset = sailroute::EnsembleDataset::load(
        std::move(meta), std::move(members));
    if (!dataset)
        return report_error("input", dataset.error(), exit_input);

    // Build objective.
    sailroute::EnsembleObjective objective;
    objective.kind = opts.objective;
    if (opts.target_seconds)
        objective.target =
            sailroute::EnsembleArrivalTarget{*opts.target_seconds};

    if (opts.rival_path) {
        // Read and parse the rival outcomes JSON.
        std::ifstream rival_file{*opts.rival_path, std::ios::binary};
        if (!rival_file) {
            std::cerr << "sailroute: input: cannot open rival file: "
                      << opts.rival_path->string() << '\n';
            return exit_input;
        }
        std::string rival_json;
        std::array<char, 8'192U> chunk{};
        while (rival_file) {
            rival_file.read(
                chunk.data(), static_cast<std::streamsize>(chunk.size()));
            const std::size_t count =
                static_cast<std::size_t>(rival_file.gcount());
            if (count >
                sailroute::ensemble_json_max_input_bytes -
                    rival_json.size()) {
                return report_error(
                    "input",
                    sailroute::Error{
                        sailroute::ErrorCode::invalid_argument,
                        "ensemble rival JSON input size limit exceeded"},
                    exit_input);
            }
            rival_json.append(chunk.data(), count);
        }
        if (!rival_file.eof()) {
            return report_error(
                "input",
                sailroute::Error{
                    sailroute::ErrorCode::file_io,
                    "failed reading rival file: " +
                        opts.rival_path->string()},
                exit_input);
        }
        auto rival_doc =
            sailroute::ensemble_rival_outcomes_from_json(rival_json);
        if (!rival_doc) {
            return report_error("input", rival_doc.error(), exit_input);
        }
        objective.rival_outcomes =
            std::move(rival_doc.value().member_outcomes);
    }

    sailroute::EnsembleRouteRequest request;
    request.start = opts.start;
    request.destination = opts.destination;
    request.departure_time = opts.departure;
    request.options = opts.routing;
    request.objective = std::move(objective);
    request.solver = opts.experimental_beam
        ? sailroute::EnsembleSolver::experimental_isochrone_beam
        : sailroute::EnsembleSolver::time_dependent_lattice;
    request.enable_experimental_beam = opts.experimental_beam;
    request.lattice = opts.lattice;
    request.beam = opts.beam;

    sailroute::EnsembleRouter router{std::move(dataset.value()),
                                     std::move(polar)};
    auto result = router.optimize(request);
    if (!result)
        return report_error("routing", result.error(), exit_routing);

    sailroute::EnsembleRunMetadata output_metadata =
        router.dataset().metadata();
    const auto& member_metadata = router.dataset().members();
    if (!member_metadata.empty()) {
        output_metadata.initialization_time =
            member_metadata.front().weather.initialization_time;
        std::string attribution;
        for (const auto& member : member_metadata) {
            if (!attribution.empty()) attribution.append("; ");
            attribution.append(member.identifier);
            attribution.append(": ");
            attribution.append(member.weather.source);
        }
        output_metadata.attribution = std::move(attribution);
    }
    sailroute::EnsembleRouteDocument doc{
        std::move(output_metadata),
        member_metadata,
        std::move(result.value()),
    };
    auto json = sailroute::ensemble_route_to_json(doc);
    if (!json)
        return report_error("output", json.error(), exit_output);

    if (opts.json_path == "-") {
        std::cout << json.value();
        std::cout.flush();
        if (!std::cout)
            return report_error(
                "output",
                sailroute::Error{sailroute::ErrorCode::output_error,
                                  "failed writing JSON to standard output"},
                exit_output);
    } else {
        auto written =
            write_file(std::filesystem::path{opts.json_path}, json.value());
        if (!written)
            return report_error("output", written.error(), exit_output);
    }
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    // Detect ensemble mode: any --ensemble-member (or related) arg present.
    bool ensemble_mode = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (arg == "--ensemble-member" || arg == "--ensemble-objective" ||
            arg == "--ensemble-target-seconds" || arg == "--ensemble-rival" ||
            arg == "--ensemble-run-id" ||
            arg == "--experimental-ensemble-beam") {
            ensemble_mode = true;
            break;
        }
    }

    if (ensemble_mode) {
        auto options = parse_ensemble_arguments(argc, argv);
        if (!options) {
            std::cerr << "sailroute: " << options.error().message << "\n"
                      << "Try 'sailroute --help' for usage.\n";
            return exit_usage;
        }
        if (options.value().help) {
            print_help(std::cout);
            return 0;
        }
        return run_ensemble(options.value());
    }

    auto options = parse_arguments(argc, argv);
    if (!options) {
        std::cerr << "sailroute: " << options.error().message << "\n"
                  << "Try 'sailroute --help' for usage.\n";
        return exit_usage;
    }
    if (options.value().help) {
        print_help(std::cout);
        return 0;
    }
    return run(options.value());
}
