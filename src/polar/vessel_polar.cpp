#include "sailroute/polar.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace sailroute {

struct VesselPolar::Impl {
    std::vector<double> wind_speeds;
    std::vector<double> wind_angles;
    std::vector<double> boat_speeds;
    std::string source_description;
};

namespace {

constexpr std::size_t max_axis_size = 512;

struct SourceLine {
    std::size_t number;
    std::string text;
};

struct ParsedPolar {
    std::vector<double> wind_speeds;
    std::vector<double> wind_angles;
    std::vector<double> boat_speeds;
};

struct NumericToken {
    bool valid;
    bool finite;
    double value;
};

std::string_view trim(std::string_view value) noexcept {
    while (!value.empty()) {
        const char c = value.front();
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
            break;
        }
        value.remove_prefix(1);
    }
    while (!value.empty()) {
        const char c = value.back();
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
            break;
        }
        value.remove_suffix(1);
    }
    return value;
}

std::string_view unquote(std::string_view value) noexcept {
    value = trim(value);
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') ||
         (value.front() == '\'' && value.back() == '\''))) {
        value.remove_prefix(1);
        value.remove_suffix(1);
        value = trim(value);
    }
    return value;
}

NumericToken parse_number(std::string_view token) noexcept {
    token = unquote(token);
    if (token.empty()) {
        return {false, false, 0.0};
    }
    if (token.front() == '+') {
        token.remove_prefix(1);
        if (token.empty()) {
            return {false, false, 0.0};
        }
    }

    double value = 0.0;
    const char* const begin = token.data();
    const char* const end = begin + token.size();
    const auto parsed = std::from_chars(begin, end, value, std::chars_format::general);
    if (parsed.ec != std::errc{} || parsed.ptr != end) {
        return {false, false, 0.0};
    }
    return {true, std::isfinite(value), value};
}

std::string_view remove_inline_comment(std::string_view line) noexcept {
    std::size_t comment = line.size();
    const auto shorten_at = [&comment](std::size_t position) {
        if (position != std::string_view::npos && position < comment) {
            comment = position;
        }
    };
    shorten_at(line.find('#'));
    shorten_at(line.find('!'));
    shorten_at(line.find("//"));
    return trim(line.substr(0, comment));
}

bool is_full_line_comment(std::string_view line) noexcept {
    line = trim(line);
    return line.empty() || line.front() == '#' || line.front() == '!' ||
           line.front() == '%' || line.front() == ';' || line.starts_with("//");
}

std::vector<std::string_view> split_tokens(std::string_view line) {
    std::vector<std::string_view> tokens;

    char delimiter = '\0';
    if (line.find(';') != std::string_view::npos) {
        delimiter = ';';
    } else if (line.find(',') != std::string_view::npos) {
        delimiter = ',';
    } else if (line.find('\t') != std::string_view::npos) {
        delimiter = '\t';
    }

    if (delimiter != '\0') {
        std::size_t start = 0;
        while (start <= line.size()) {
            const std::size_t end = line.find(delimiter, start);
            tokens.push_back(trim(line.substr(
                start, end == std::string_view::npos ? line.size() - start : end - start)));
            if (end == std::string_view::npos) {
                break;
            }
            start = end + 1;
        }
    } else {
        std::size_t start = 0;
        while (start < line.size()) {
            while (start < line.size() &&
                   (line[start] == ' ' || line[start] == '\t' || line[start] == '\r')) {
                ++start;
            }
            if (start == line.size()) {
                break;
            }
            std::size_t end = start;
            while (end < line.size() && line[end] != ' ' && line[end] != '\t' &&
                   line[end] != '\r') {
                ++end;
            }
            tokens.push_back(line.substr(start, end - start));
            start = end;
        }
    }

    while (!tokens.empty() && trim(tokens.back()).empty()) {
        tokens.pop_back();
    }
    return tokens;
}

std::string lowercase(std::string_view value) {
    std::string result;
    result.reserve(value.size());
    for (const char c : value) {
        if (c >= 'A' && c <= 'Z') {
            result.push_back(static_cast<char>(c - 'A' + 'a'));
        } else {
            result.push_back(c);
        }
    }
    return result;
}

bool looks_like_header(const std::vector<std::string_view>& tokens) {
    std::string labels;
    for (const auto token : tokens) {
        if (!parse_number(token).valid) {
            labels.append(token);
            labels.push_back(' ');
        }
    }
    labels = lowercase(labels);
    return labels.find("twa") != std::string::npos ||
           labels.find("tws") != std::string::npos ||
           labels.find("wind") != std::string::npos ||
           labels.find("angle") != std::string::npos ||
           labels.find("speed") != std::string::npos;
}

Error invalid_polar(std::size_t line, std::string message) {
    if (line != 0) {
        message = "polar line " + std::to_string(line) + ": " + std::move(message);
    }
    return Error{ErrorCode::invalid_polar, std::move(message)};
}

bool strictly_increasing(const std::vector<double>& values) noexcept {
    for (std::size_t i = 1; i < values.size(); ++i) {
        if (!(values[i] > values[i - 1])) {
            return false;
        }
    }
    return true;
}

Result<ParsedPolar> parse_polar_lines(const std::vector<SourceLine>& lines) {
    ParsedPolar result;
    std::size_t header_index = lines.size();

    for (std::size_t line_index = 0; line_index < lines.size(); ++line_index) {
        std::string_view text = lines[line_index].text;
        if (line_index == 0 && text.starts_with("\xEF\xBB\xBF")) {
            text.remove_prefix(3);
        }
        if (is_full_line_comment(text)) {
            continue;
        }
        text = remove_inline_comment(text);
        if (text.empty()) {
            continue;
        }

        const auto tokens = split_tokens(text);
        if (tokens.empty()) {
            continue;
        }

        std::size_t first_number = tokens.size();
        for (std::size_t i = 0; i < tokens.size(); ++i) {
            if (parse_number(tokens[i]).valid) {
                first_number = i;
                break;
            }
        }

        const bool numeric_only = first_number == 0;
        const bool blank_corner =
            first_number != tokens.size() && first_number > 0 && trim(tokens[0]).empty();
        if (!numeric_only && !blank_corner && !looks_like_header(tokens)) {
            continue;
        }
        if (first_number == tokens.size()) {
            continue;
        }

        std::size_t wind_start = first_number;
        if (numeric_only && tokens.size() >= 3) {
            const NumericToken corner = parse_number(tokens[0]);
            if (corner.valid && corner.finite && corner.value == 0.0) {
                std::size_t next_value_count = 0;
                for (std::size_t next = line_index + 1; next < lines.size(); ++next) {
                    std::string_view next_text = lines[next].text;
                    if (is_full_line_comment(next_text)) {
                        continue;
                    }
                    next_text = remove_inline_comment(next_text);
                    const auto next_tokens = split_tokens(next_text);
                    std::size_t next_first_number = next_tokens.size();
                    for (std::size_t i = 0; i < next_tokens.size(); ++i) {
                        if (parse_number(next_tokens[i]).valid) {
                            next_first_number = i;
                            break;
                        }
                    }
                    if (next_first_number != next_tokens.size()) {
                        next_value_count = next_tokens.size() - next_first_number;
                        break;
                    }
                }
                if (next_value_count == tokens.size()) {
                    wind_start = 1;
                }
            }
        }

        std::vector<double> candidate;
        candidate.reserve(tokens.size() - wind_start);
        for (std::size_t i = wind_start; i < tokens.size(); ++i) {
            const NumericToken number = parse_number(tokens[i]);
            if (!number.valid) {
                return invalid_polar(
                    lines[line_index].number,
                    "invalid TWS value '" + std::string(trim(tokens[i])) + "'");
            }
            if (!number.finite) {
                return invalid_polar(
                    lines[line_index].number, "TWS values must be finite");
            }
            candidate.push_back(number.value);
        }

        if (candidate.size() < 2) {
            continue;
        }
        if (!strictly_increasing(candidate) || candidate.front() < 0.0) {
            if (looks_like_header(tokens) || blank_corner) {
                return invalid_polar(
                    lines[line_index].number,
                    "TWS values must be nonnegative and strictly increasing");
            }
            continue;
        }

        result.wind_speeds = std::move(candidate);
        header_index = line_index;
        break;
    }

    if (header_index == lines.size()) {
        return invalid_polar(
            0,
            "no TWS header found; expected a matrix headed by TWA/TWS and wind speeds");
    }
    if (result.wind_speeds.size() > max_axis_size) {
        return invalid_polar(
            lines[header_index].number, "too many TWS columns (maximum is 512)");
    }

    for (std::size_t line_index = header_index + 1; line_index < lines.size(); ++line_index) {
        std::string_view text = lines[line_index].text;
        if (is_full_line_comment(text)) {
            continue;
        }
        text = remove_inline_comment(text);
        if (text.empty()) {
            continue;
        }

        const auto tokens = split_tokens(text);
        std::size_t first_number = tokens.size();
        for (std::size_t i = 0; i < tokens.size(); ++i) {
            if (parse_number(tokens[i]).valid) {
                first_number = i;
                break;
            }
        }
        if (first_number == tokens.size()) {
            continue;
        }

        const std::size_t expected_values = result.wind_speeds.size() + 1;
        if (tokens.size() - first_number != expected_values) {
            return invalid_polar(
                lines[line_index].number,
                "expected one TWA and " + std::to_string(result.wind_speeds.size()) +
                    " boat-speed values, found " +
                    std::to_string(tokens.size() - first_number));
        }

        std::vector<double> row;
        row.reserve(expected_values);
        for (std::size_t i = first_number; i < tokens.size(); ++i) {
            const NumericToken number = parse_number(tokens[i]);
            if (!number.valid) {
                return invalid_polar(
                    lines[line_index].number,
                    "invalid numeric value '" + std::string(trim(tokens[i])) + "'");
            }
            if (!number.finite) {
                return invalid_polar(
                    lines[line_index].number, "TWA and boat speeds must be finite");
            }
            row.push_back(number.value);
        }

        const double angle = row.front();
        if (angle < 0.0 || angle > 180.0) {
            return invalid_polar(
                lines[line_index].number, "TWA must be within 0 to 180 degrees");
        }
        if (!result.wind_angles.empty() && !(angle > result.wind_angles.back())) {
            return invalid_polar(
                lines[line_index].number, "TWA rows must be strictly increasing");
        }
        for (std::size_t i = 1; i < row.size(); ++i) {
            if (row[i] < 0.0) {
                return invalid_polar(
                    lines[line_index].number, "boat speeds must be nonnegative");
            }
        }

        result.wind_angles.push_back(angle);
        result.boat_speeds.insert(result.boat_speeds.end(), row.begin() + 1, row.end());
        if (result.wind_angles.size() > max_axis_size) {
            return invalid_polar(
                lines[line_index].number, "too many TWA rows (maximum is 512)");
        }
    }

    if (result.wind_angles.size() < 2) {
        return invalid_polar(0, "polar must contain at least two TWA rows");
    }
    if (result.wind_angles.back() - result.wind_angles.front() < 30.0) {
        return invalid_polar(0, "TWA axis must span at least 30 degrees");
    }
    if (result.wind_speeds.back() - result.wind_speeds.front() < 1.0) {
        return invalid_polar(0, "TWS axis must span at least 1 knot");
    }
    if (result.wind_speeds.front() > 0.0) {
        const std::size_t old_columns = result.wind_speeds.size();
        std::vector<double> speeds_with_calm;
        speeds_with_calm.reserve(
            result.wind_angles.size() * (old_columns + 1U));
        for (std::size_t row = 0; row < result.wind_angles.size(); ++row) {
            speeds_with_calm.push_back(0.0);
            const auto begin =
                result.boat_speeds.begin() +
                static_cast<std::ptrdiff_t>(row * old_columns);
            speeds_with_calm.insert(
                speeds_with_calm.end(),
                begin,
                begin + static_cast<std::ptrdiff_t>(old_columns));
        }
        result.wind_speeds.insert(result.wind_speeds.begin(), 0.0);
        result.boat_speeds = std::move(speeds_with_calm);
    }

    return result;
}

double fold_angle(double angle) noexcept {
    angle = std::fabs(std::fmod(angle, 360.0));
    return angle > 180.0 ? 360.0 - angle : angle;
}

struct Interval {
    std::size_t lower;
    double fraction;
};

Interval find_interval(const std::vector<double>& axis, double value) noexcept {
    if (value <= axis.front()) {
        return {0, 0.0};
    }
    if (value >= axis.back()) {
        return {axis.size() - 2, 1.0};
    }
    const auto upper = std::upper_bound(axis.begin(), axis.end(), value);
    const std::size_t high = static_cast<std::size_t>(upper - axis.begin());
    const std::size_t low = high - 1;
    const double fraction = (value - axis[low]) / (axis[high] - axis[low]);
    return {low, fraction};
}

}  // namespace

VesselPolar::VesselPolar()
    : impl_(std::make_shared<const Impl>(Impl{
          {0.0, 6.0, 10.0, 14.0, 20.0, 30.0, 40.0},
          {0.0, 30.0, 45.0, 60.0, 75.0, 90.0, 110.0, 120.0, 135.0, 150.0, 165.0, 180.0},
          {
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 4.1, 5.6, 6.5, 7.1, 7.3, 7.3,
              0.0, 4.7, 6.2, 7.1, 7.8, 8.1, 8.1,
              0.0, 5.1, 6.7, 7.6, 8.5, 9.0, 9.0,
              0.0, 5.3, 6.9, 7.9, 8.9, 9.7, 9.7,
              0.0, 5.2, 6.9, 8.0, 9.2, 10.2, 10.4,
              0.0, 5.1, 6.8, 7.9, 9.3, 10.5, 10.8,
              0.0, 4.8, 6.5, 7.7, 9.2, 10.6, 11.0,
              0.0, 4.5, 6.2, 7.4, 9.0, 10.5, 11.0,
              0.0, 4.1, 5.8, 7.0, 8.7, 10.2, 10.8,
              0.0, 3.8, 5.4, 6.6, 8.3, 9.8, 10.4,
          },
          "Built-in approximate conservative 45-foot racer-cruiser polar "
          "(generic planning data; not manufacturer or measured performance)"})) {}

VesselPolar::~VesselPolar() = default;
VesselPolar::VesselPolar(const VesselPolar&) = default;
VesselPolar::VesselPolar(VesselPolar&&) noexcept = default;
VesselPolar& VesselPolar::operator=(const VesselPolar&) = default;
VesselPolar& VesselPolar::operator=(VesselPolar&&) noexcept = default;

VesselPolar::VesselPolar(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}

Result<VesselPolar> VesselPolar::load(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        return Error{
            ErrorCode::file_io, "unable to open polar file '" + path.string() + "'"};
    }

    std::vector<SourceLine> lines;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        if (line.size() > 1024U * 1024U) {
            return invalid_polar(line_number, "line exceeds 1 MiB");
        }
        lines.push_back({line_number, std::move(line)});
    }
    if (input.bad()) {
        return Error{
            ErrorCode::file_io, "error while reading polar file '" + path.string() + "'"};
    }
    if (lines.empty()) {
        return invalid_polar(0, "polar file is empty");
    }

    auto parsed = parse_polar_lines(lines);
    if (!parsed) {
        return parsed.error();
    }

    ParsedPolar data = std::move(parsed.value());
    auto impl = std::make_shared<const Impl>(Impl{
        std::move(data.wind_speeds),
        std::move(data.wind_angles),
        std::move(data.boat_speeds),
        "Loaded vessel polar from '" + path.string() + "'"});
    return VesselPolar{std::move(impl)};
}

VesselPolar VesselPolar::default_racer_cruiser_45ft() {
    return VesselPolar{};
}

double VesselPolar::boat_speed_knots(
    double true_wind_speed_knots, double true_wind_angle_degrees) const noexcept {
    if (!impl_ || !std::isfinite(true_wind_speed_knots) ||
        !std::isfinite(true_wind_angle_degrees) || true_wind_speed_knots <= 0.0) {
        return 0.0;
    }

    const double angle = fold_angle(true_wind_angle_degrees);
    const Interval wind = find_interval(impl_->wind_speeds, true_wind_speed_knots);
    const Interval twa = find_interval(impl_->wind_angles, angle);
    const std::size_t columns = impl_->wind_speeds.size();
    const std::size_t row0 = twa.lower * columns;
    const std::size_t row1 = row0 + columns;

    const double low_angle_speed =
        impl_->boat_speeds[row0 + wind.lower] +
        wind.fraction *
            (impl_->boat_speeds[row0 + wind.lower + 1] -
             impl_->boat_speeds[row0 + wind.lower]);
    const double high_angle_speed =
        impl_->boat_speeds[row1 + wind.lower] +
        wind.fraction *
            (impl_->boat_speeds[row1 + wind.lower + 1] -
             impl_->boat_speeds[row1 + wind.lower]);
    return low_angle_speed + twa.fraction * (high_angle_speed - low_angle_speed);
}

const std::string& VesselPolar::source() const noexcept {
    static const std::string empty_source;
    return impl_ ? impl_->source_description : empty_source;
}

}  // namespace sailroute
