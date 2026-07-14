#include "sailroute/sailroute.hpp"

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
        "  --time-step-minutes N               Routing time step (> 0)\n"
        "  --heading-step-degrees N            Heading increment (0 < N <= 180)\n"
        "  --arrival-radius-nm N               Arrival radius (> 0)\n"
        "  --spatial-bucket-nm N               Spatial pruning bucket size (> 0)\n"
        "  --max-nodes-per-bucket N            Nodes retained per bucket (> 0)\n"
        "  --worker-count N                    Worker threads (0 selects automatic)\n"
        "  --maximum-route-duration-hours N    Maximum route duration (> 0)\n"
        "  --minimum-boat-speed-knots N        Minimum usable speed (>= 0)\n"
        "\n"
        "  -h, --help                          Show this help\n";
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
    bool time_step_seen = false;
    bool heading_step_seen = false;
    bool arrival_radius_seen = false;
    bool spatial_bucket_seen = false;
    bool max_nodes_seen = false;
    bool workers_seen = false;
    bool max_duration_seen = false;
    bool minimum_speed_seen = false;

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
        } else if (is_option(argument, "--time-step-minutes", "--time-step")) {
            if (const auto duplicate = reject_duplicate(time_step_seen, "--time-step-minutes")) {
                return *duplicate;
            }
            auto value = value_after(argument);
            if (!value) {
                return value.error();
            }
            unsigned long long parsed = 0;
            using Rep = std::chrono::minutes::rep;
            if (!parse_unsigned(value.value(), parsed) || parsed == 0 ||
                parsed > static_cast<unsigned long long>(std::numeric_limits<Rep>::max())) {
                return usage_error(std::string{argument} + " must be a positive integer");
            }
            options.routing.time_step = std::chrono::minutes{static_cast<Rep>(parsed)};
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

}  // namespace

int main(int argc, char* argv[]) {
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
