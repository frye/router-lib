#include "sailroute/polar.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <numbers>
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
    // Best upwind and downwind true wind angle for each tabulated wind speed,
    // under each angle-interpolation mode. Derived at construction so a routing
    // node can look them up without scanning the polar.
    std::vector<double> upwind_vmg_linear;
    std::vector<double> downwind_vmg_linear;
    std::vector<double> upwind_vmg_cubic;
    std::vector<double> downwind_vmg_cubic;
    std::vector<std::uint8_t> sailing_points;
    std::vector<std::uint8_t> sailing_intervals;
    double minimum_sailing_angle_degrees{0.0};
    std::vector<double> column_minimum_angles;
    std::vector<double> column_maximum_angles;
    std::vector<double> between_minimum_angles;
    std::vector<double> between_maximum_angles;
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
    std::vector<std::uint8_t> sailing_points;
    std::vector<std::uint8_t> sailing_intervals;
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
    } else if (!line.empty() && line.front() == '\t') {
        // Preserve a blank tab-separated matrix corner; ordinary tabs are
        // whitespace so native rows can mix tabs and spaces.
        tokens.push_back({});
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

Result<ParsedPolar> parse_matrix_lines(const std::vector<SourceLine>& lines) {
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
            const auto numeric_count = std::count_if(
                tokens.begin(), tokens.end(),
                [](std::string_view token) { return parse_number(token).valid; });
            if (numeric_count >= 2) {
                return invalid_polar(
                    lines[line_index].number, "invalid matrix header before TWS values");
            }
            continue;
        }
        if (first_number == tokens.size()) {
            return invalid_polar(
                lines[line_index].number, "matrix header has no numeric TWS values");
        }
        for (std::size_t i = 0; i < first_number; ++i) {
            if (i == 0 && blank_corner) {
                continue;
            }
            if (!looks_like_header({tokens[i]}) && tokens[i] != "/" && tokens[i] != "\\") {
                return invalid_polar(
                    lines[line_index].number, "invalid or missing TWS header value");
            }
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
            return invalid_polar(
                lines[line_index].number, "matrix requires at least two TWS columns");
        }
        if (!strictly_increasing(candidate) || candidate.front() < 0.0) {
            return invalid_polar(
                lines[line_index].number,
                "TWS values must be nonnegative and strictly increasing");
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
        if (first_number != 0) {
            return invalid_polar(
                lines[line_index].number, "expected a numeric TWA at start of matrix row");
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

Interval find_interval(const double* axis, std::size_t count, double value) noexcept {
    if (value <= axis[0]) {
        return {0, 0.0};
    }
    if (value >= axis[count - 1]) {
        return {count - 2, 1.0};
    }
    const auto upper = std::upper_bound(axis, axis + count, value);
    const std::size_t high = static_cast<std::size_t>(upper - axis);
    const std::size_t low = high - 1;
    const double fraction = (value - axis[low]) / (axis[high] - axis[low]);
    return {low, fraction};
}

Interval find_interval(const std::vector<double>& axis, double value) noexcept {
    return find_interval(axis.data(), axis.size(), value);
}

Result<ParsedPolar> parse_expedition_lines(const std::vector<SourceLine>& lines) {
    struct Curve {
        double wind;
        std::vector<double> angles;
        std::vector<double> speeds;
    };
    std::vector<Curve> curves;
    std::vector<double> angles;
    for (const auto& line : lines) {
        std::string_view text = line.text;
        if (line.number == 1 && text.starts_with("\xEF\xBB\xBF")) {
            text.remove_prefix(3);
        }
        if (is_full_line_comment(text)) {
            continue;
        }
        text = remove_inline_comment(text);
        auto tokens = split_tokens(trim(text));
        if (tokens.empty()) {
            continue;
        }
        if (!parse_number(tokens.front()).valid) {
            const auto numeric_count = std::count_if(
                tokens.begin(), tokens.end(),
                [](std::string_view token) { return parse_number(token).valid; });
            if (curves.empty() && !tokens.front().empty() &&
                !looks_like_header(tokens) && numeric_count < 2) {
                // Expedition exports may include a boat name or version preamble.
                continue;
            }
            return invalid_polar(line.number, "expected TWS at start of Expedition row");
        }
        if (tokens.size() < 5U || tokens.size() % 2U != 1U) {
            return invalid_polar(
                line.number, "expected TWS followed by at least two complete TWA/BSP pairs");
        }
        if ((tokens.size() - 1U) / 2U > max_axis_size) {
            return invalid_polar(line.number, "too many TWA/BSP pairs (maximum is 512)");
        }
        std::vector<double> numbers;
        for (const auto token : tokens) {
            const NumericToken number = parse_number(token);
            if (!number.valid || !number.finite) {
                return invalid_polar(
                    line.number, "Expedition TWS, TWA and BSP values must be finite numbers");
            }
            numbers.push_back(number.value);
        }
        Curve curve{numbers.front(), {}, {}};
        if (curve.wind < 0.0 ||
            (!curves.empty() && curve.wind <= curves.back().wind)) {
            return invalid_polar(
                line.number, "Expedition TWS rows must be nonnegative and strictly increasing");
        }
        for (std::size_t i = 1; i < numbers.size(); i += 2U) {
            const double angle = numbers[i];
            if (angle < 0.0 || angle > 180.0 ||
                (!curve.angles.empty() && angle <= curve.angles.back())) {
                return invalid_polar(
                    line.number, "Expedition TWA pairs must be within 0 to 180 degrees "
                        "and strictly increasing");
            }
            if (numbers[i + 1U] < 0.0) {
                return invalid_polar(line.number, "Expedition BSP values must be nonnegative");
            }
            curve.angles.push_back(angle);
            curve.speeds.push_back(numbers[i + 1U]);
        }
        if (curve.angles.back() - curve.angles.front() < 30.0) {
            return invalid_polar(line.number, "Expedition TWA curve must span at least 30 degrees");
        }
        angles.insert(angles.end(), curve.angles.begin(), curve.angles.end());
        curves.push_back(std::move(curve));
        if (curves.size() > max_axis_size) {
            return invalid_polar(line.number, "too many Expedition TWS rows (maximum is 512)");
        }
    }
    if (curves.size() < 2U || curves.back().wind - curves.front().wind < 1.0) {
        return invalid_polar(
            lines.front().number, "Expedition polar requires at least two TWS rows spanning 1 knot");
    }
    std::sort(angles.begin(), angles.end());
    angles.erase(std::unique(angles.begin(), angles.end()), angles.end());
    if (angles.size() > max_axis_size) {
        return invalid_polar(
            lines.back().number, "too many distinct Expedition TWAs (maximum is 512)");
    }
    if (curves.front().wind > 0.0) {
        Curve calm{0.0, curves.front().angles,
            std::vector<double>(curves.front().speeds.size(), 0.0)};
        curves.insert(curves.begin(), std::move(calm));
    }
    ParsedPolar result;
    result.wind_angles = std::move(angles);
    for (const auto& curve : curves) {
        result.wind_speeds.push_back(curve.wind);
    }
    for (std::size_t row = 0; row < result.wind_angles.size(); ++row) {
        const double angle = result.wind_angles[row];
        for (const auto& curve : curves) {
            const Interval interval = find_interval(curve.angles, angle);
            const double low = curve.speeds[interval.lower];
            const double high = curve.speeds[interval.lower + 1U];
            result.boat_speeds.push_back(low + interval.fraction * (high - low));
            const bool in_domain =
                angle > 0.0 && angle >= curve.angles.front() && angle <= curve.angles.back();
            const bool positive_interval =
                curve.angles[interval.lower] > 0.0 && low > 0.0 && high > 0.0;
            const bool positive_point = interval.fraction == 0.0 ? low > 0.0
                : interval.fraction == 1.0 ? high > 0.0 : positive_interval;
            result.sailing_points.push_back(in_domain && positive_point);
            const bool supports_next = row + 1U < result.wind_angles.size() &&
                result.wind_angles[row + 1U] <= curve.angles.back();
            result.sailing_intervals.push_back(
                in_domain && supports_next && positive_interval &&
                angle < curve.angles.back());
        }
    }
    return result;
}

Result<ParsedPolar> parse_polar_lines(
    const std::vector<SourceLine>& lines, PolarFormat format) {
    if (format == PolarFormat::matrix) {
        return parse_matrix_lines(lines);
    }
    if (format == PolarFormat::expedition) {
        return parse_expedition_lines(lines);
    }
    if (format != PolarFormat::automatic) {
        return invalid_polar(0, "unrecognized polar format");
    }
    // A successful numeric parse is not sufficient format detection: the
    // zero-corner matrix variant can also be a valid native Expedition file.
    auto matrix = parse_matrix_lines(lines);
    auto expedition = parse_expedition_lines(lines);
    if (matrix && expedition) {
        const auto first_data = std::find_if(lines.begin(), lines.end(), [](const auto& line) {
            if (is_full_line_comment(line.text)) {
                return false;
            }
            const auto tokens = split_tokens(remove_inline_comment(line.text));
            return !tokens.empty() && parse_number(tokens.front()).valid;
        });
        return invalid_polar(
            first_data == lines.end() ? lines.front().number : first_data->number,
            "ambiguous numeric polar; select PolarFormat::matrix or PolarFormat::expedition");
    }
    if (matrix) {
        return matrix;
    }
    if (expedition) {
        return expedition;
    }
    return invalid_polar(
        0, "unrecognized or malformed polar; matrix: " + matrix.error().message +
            "; Expedition: " + expedition.error().message);
}

// Fritsch-Carlson slope for a monotone cubic (PCHIP) through `points`.
//
// A plain cubic spline through polar rows overshoots between widely spaced
// angles and can invent boat speeds the vessel cannot reach. Constraining the
// slopes keeps the curve monotone on every interval the data is monotone on,
// so it resolves the polar's peak without inventing a taller one.
template <typename ValueAt>
double pchip_slope_for(
    const double* x,
    std::size_t count,
    std::size_t index,
    const ValueAt& value_at) noexcept {
    if (count < 2U) {
        return 0.0;
    }
    const auto secant = [&](std::size_t i) {
        return (value_at(i + 1U) - value_at(i)) / (x[i + 1U] - x[i]);
    };
    if (count == 2U) {
        return secant(0U);
    }

    if (index == 0U || index + 1U == count) {
        // One-sided three-point slope, limited so an end interval that is
        // monotone in the data stays monotone in the curve.
        const std::size_t base = index == 0U ? 0U : count - 3U;
        const double h0 = x[base + 1U] - x[base];
        const double h1 = x[base + 2U] - x[base + 1U];
        const double delta0 = secant(base);
        const double delta1 = secant(base + 1U);
        double slope = 0.0;
        double adjacent = 0.0;
        double other = 0.0;
        if (index == 0U) {
            slope = ((2.0 * h0 + h1) * delta0 - h0 * delta1) / (h0 + h1);
            adjacent = delta0;
            other = delta1;
        } else {
            slope = ((2.0 * h1 + h0) * delta1 - h1 * delta0) / (h0 + h1);
            adjacent = delta1;
            other = delta0;
        }
        if (slope * adjacent <= 0.0) {
            return 0.0;
        }
        if (adjacent * other <= 0.0 && std::fabs(slope) > std::fabs(3.0 * adjacent)) {
            return 3.0 * adjacent;
        }
        return slope;
    }

    const double h0 = x[index] - x[index - 1U];
    const double h1 = x[index + 1U] - x[index];
    const double delta0 = secant(index - 1U);
    const double delta1 = secant(index);
    if (delta0 * delta1 <= 0.0) {
        return 0.0;
    }
    const double w0 = 2.0 * h1 + h0;
    const double w1 = h1 + 2.0 * h0;
    return (w0 + w1) / (w0 / delta0 + w1 / delta1);
}

double pchip_slope(
    const double* x,
    const double* y,
    std::size_t count,
    std::size_t index) noexcept {
    return pchip_slope_for(
        x,
        count,
        index,
        [y](std::size_t value_index) noexcept { return y[value_index]; });
}

double hermite(
    double x0,
    double x1,
    double y0,
    double y1,
    double slope0,
    double slope1,
    double x) noexcept {
    const double h = x1 - x0;
    const double t = (x - x0) / h;
    const double t2 = t * t;
    const double t3 = t2 * t;
    return y0 * (2.0 * t3 - 3.0 * t2 + 1.0) + h * slope0 * (t3 - 2.0 * t2 + t) +
        y1 * (-2.0 * t3 + 3.0 * t2) + h * slope1 * (t3 - t2);
}

// Locates the true wind angle maximising `speed(angle) * cos(angle - target)`,
// which is velocity made good toward the wind for target 0 and away from it for
// target 180. A coarse sweep finds the basin and golden-section refines it,
// which avoids assuming the polar is unimodal at the sweep's resolution.
template <typename SpeedAtAngle>
double velocity_made_good_angle(
    const SpeedAtAngle& speed,
    double lower_degrees,
    double upper_degrees,
    bool upwind) {
    const double sign = upwind ? 1.0 : -1.0;
    const auto objective = [&](double angle) {
        return speed(angle) * sign * std::cos(angle * std::numbers::pi / 180.0);
    };

    constexpr int sweep_steps = 180;
    double best_angle = lower_degrees;
    double best_value = -std::numeric_limits<double>::infinity();
    for (int step = 0; step <= sweep_steps; ++step) {
        const double angle = lower_degrees +
            (upper_degrees - lower_degrees) * static_cast<double>(step) /
                static_cast<double>(sweep_steps);
        const double value = objective(angle);
        if (value > best_value) {
            best_value = value;
            best_angle = angle;
        }
    }

    const double window = (upper_degrees - lower_degrees) /
        static_cast<double>(sweep_steps);
    double low = std::max(lower_degrees, best_angle - window);
    double high = std::min(upper_degrees, best_angle + window);
    constexpr double inverse_golden_ratio = 0.618033988749894848;
    double probe_low = high - inverse_golden_ratio * (high - low);
    double probe_high = low + inverse_golden_ratio * (high - low);
    double value_low = objective(probe_low);
    double value_high = objective(probe_high);
    for (int iteration = 0; iteration < 40; ++iteration) {
        if (value_low < value_high) {
            low = probe_low;
            probe_low = probe_high;
            value_low = value_high;
            probe_high = low + inverse_golden_ratio * (high - low);
            value_high = objective(probe_high);
        } else {
            high = probe_high;
            probe_high = probe_low;
            value_high = value_low;
            probe_low = high - inverse_golden_ratio * (high - low);
            value_low = objective(probe_low);
        }
    }

    const double refined = 0.5 * (low + high);
    return objective(refined) >= best_value ? refined : best_angle;
}

// Interpolates one tabulated wind-speed column across true wind angle.
double column_speed(
    const std::vector<double>& wind_angles,
    const std::vector<double>& boat_speeds,
    std::size_t column_count,
    std::size_t column,
    double angle,
    bool monotone_cubic) {
    const std::size_t rows = wind_angles.size();
    const auto value_at = [&](std::size_t row) {
        return boat_speeds[row * column_count + column];
    };
    const Interval row = find_interval(wind_angles, angle);
    const double y0 = value_at(row.lower);
    const double y1 = value_at(row.lower + 1U);
    if (!monotone_cubic) {
        return y0 + row.fraction * (y1 - y0);
    }

    const double slope0 =
        pchip_slope_for(wind_angles.data(), rows, row.lower, value_at);
    const double slope1 = pchip_slope_for(
        wind_angles.data(), rows, row.lower + 1U, value_at);
    return hermite(
        wind_angles[row.lower],
        wind_angles[row.lower + 1U],
        y0,
        y1,
        slope0,
        slope1,
        wind_angles[row.lower] +
            row.fraction * (wind_angles[row.lower + 1U] - wind_angles[row.lower]));
}

template <typename PolarImpl>
void fill_sailing_support(PolarImpl& impl) {
    const std::size_t columns = impl.wind_speeds.size();
    const std::size_t rows = impl.wind_angles.size();
    if (impl.sailing_points.empty()) {
        impl.sailing_points.resize(rows * columns);
        impl.sailing_intervals.resize(rows * columns);
        for (std::size_t row = 0; row < rows; ++row) {
            for (std::size_t column = 0; column < columns; ++column) {
                const std::size_t index = row * columns + column;
                const bool point =
                    impl.wind_angles[row] > 0.0 && impl.boat_speeds[index] > 0.0;
                impl.sailing_points[index] = point;
                impl.sailing_intervals[index] =
                    point && row + 1U < rows && impl.boat_speeds[index + columns] > 0.0;
            }
        }
    }
    // Zero TWS is a scaling anchor, not an independently sailable curve.
    // Borrow its neighbor's domain only for positive winds interpolated above it.
    if (impl.wind_speeds.front() == 0.0) {
        for (std::size_t row = 0; row < rows; ++row) {
            impl.sailing_points[row * columns] = impl.sailing_points[row * columns + 1U];
            impl.sailing_intervals[row * columns] =
                impl.sailing_intervals[row * columns + 1U];
        }
    }
    const double none = std::numeric_limits<double>::quiet_NaN();
    impl.column_minimum_angles.assign(columns, none);
    impl.column_maximum_angles.assign(columns, none);
    impl.between_minimum_angles.assign(columns - 1U, none);
    impl.between_maximum_angles.assign(columns - 1U, none);
    const auto bounds = [&](std::size_t low_column, std::size_t high_column) {
        double minimum = none;
        double maximum = none;
        const auto include = [&](double low, double high) {
            low = std::max(low, impl.minimum_sailing_angle_degrees);
            if (low > high) {
                return;
            }
            if (!std::isfinite(minimum)) {
                minimum = low;
            }
            maximum = high;
        };
        for (std::size_t row = 0; row < rows; ++row) {
            const std::size_t base = row * columns;
            if (impl.sailing_points[base + low_column] &&
                impl.sailing_points[base + high_column]) {
                include(impl.wind_angles[row], impl.wind_angles[row]);
            }
            if (row + 1U < rows && impl.sailing_intervals[base + low_column] &&
                impl.sailing_intervals[base + high_column]) {
                include(impl.wind_angles[row], impl.wind_angles[row + 1U]);
            }
        }
        return std::pair{minimum, maximum};
    };
    for (std::size_t column = 0; column < columns; ++column) {
        const auto [minimum, maximum] = bounds(column, column);
        impl.column_minimum_angles[column] = minimum;
        impl.column_maximum_angles[column] = maximum;
        if (column + 1U < columns) {
            const auto [between_minimum, between_maximum] = bounds(column, column + 1U);
            impl.between_minimum_angles[column] = between_minimum;
            impl.between_maximum_angles[column] = between_maximum;
        }
    }
}

// Derives the per-column velocity-made-good optima cached on the polar.
template <typename PolarImpl>
void fill_velocity_made_good_angles(PolarImpl& impl) {
    const std::size_t columns = impl.wind_speeds.size();
    if (columns == 0U || impl.wind_angles.size() < 2U) {
        return;
    }
    impl.upwind_vmg_linear.assign(columns, 0.0);
    impl.downwind_vmg_linear.assign(columns, 180.0);
    impl.upwind_vmg_cubic.assign(columns, 0.0);
    impl.downwind_vmg_cubic.assign(columns, 180.0);

    for (std::size_t column = 0U; column < columns; ++column) {
        const double lowest_angle = impl.column_minimum_angles[column];
        const double highest_angle = impl.column_maximum_angles[column];
        if (!std::isfinite(lowest_angle)) {
            continue;
        }
        for (const bool cubic : {false, true}) {
            const auto speed = [&](double angle) {
                const Interval interval = find_interval(impl.wind_angles, angle);
                const std::size_t row = interval.lower +
                    static_cast<std::size_t>(interval.fraction == 1.0);
                const auto& support = interval.fraction == 0.0 || interval.fraction == 1.0
                    ? impl.sailing_points : impl.sailing_intervals;
                if (!support[row * columns + column]) {
                    return 0.0;
                }
                return column_speed(
                    impl.wind_angles, impl.boat_speeds, columns, column, angle, cubic);
            };
            const double upwind = velocity_made_good_angle(
                speed, lowest_angle, std::clamp(90.0, lowest_angle, highest_angle), true);
            const double downwind = velocity_made_good_angle(
                speed, std::clamp(90.0, lowest_angle, highest_angle), highest_angle, false);
            if (cubic) {
                impl.upwind_vmg_cubic[column] = upwind;
                impl.downwind_vmg_cubic[column] = downwind;
            } else {
                impl.upwind_vmg_linear[column] = upwind;
                impl.downwind_vmg_linear[column] = downwind;
            }
        }
        if (impl.wind_speeds.front() == 0.0) {
            impl.upwind_vmg_linear[0] = impl.upwind_vmg_linear[1];
            impl.downwind_vmg_linear[0] = impl.downwind_vmg_linear[1];
            impl.upwind_vmg_cubic[0] = impl.upwind_vmg_cubic[1];
            impl.downwind_vmg_cubic[0] = impl.downwind_vmg_cubic[1];
        }
    }
}

template <typename PolarImpl>
std::shared_ptr<const PolarImpl> make_impl(PolarImpl impl) {
    fill_sailing_support(impl);
    fill_velocity_made_good_angles(impl);
    return std::make_shared<const PolarImpl>(std::move(impl));
}

}  // namespace

VesselPolar::VesselPolar()
    : impl_(make_impl(Impl{
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
          "(generic planning data; not manufacturer or measured performance)",
          {},
          {},
          {},
          {},
          {},
          {},
          0.0,
          {},
          {},
          {},
          {}})) {}

VesselPolar::~VesselPolar() = default;
VesselPolar::VesselPolar(const VesselPolar&) = default;
VesselPolar::VesselPolar(VesselPolar&&) noexcept = default;
VesselPolar& VesselPolar::operator=(const VesselPolar&) = default;
VesselPolar& VesselPolar::operator=(VesselPolar&&) noexcept = default;

VesselPolar::VesselPolar(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}

Result<VesselPolar> VesselPolar::load(const std::filesystem::path& path) {
    return load(path, PolarLoadOptions{});
}

Result<VesselPolar> VesselPolar::load(
    const std::filesystem::path& path, const PolarLoadOptions& options) {
    if (options.minimum_sailing_angle_degrees &&
        (!std::isfinite(*options.minimum_sailing_angle_degrees) ||
         *options.minimum_sailing_angle_degrees <= 0.0 ||
         *options.minimum_sailing_angle_degrees > 180.0)) {
        return invalid_polar(0, "minimum sailing angle must be finite and within (0, 180] degrees");
    }
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

    auto parsed = parse_polar_lines(lines, options.format);
    if (!parsed) {
        return parsed.error();
    }

    ParsedPolar data = std::move(parsed.value());
    auto impl = make_impl(Impl{
        std::move(data.wind_speeds),
        std::move(data.wind_angles),
        std::move(data.boat_speeds),
        "Loaded vessel polar from '" + path.string() + "'",
        {},
        {},
        {},
        {},
        std::move(data.sailing_points),
        std::move(data.sailing_intervals),
        options.minimum_sailing_angle_degrees.value_or(0.0),
        {},
        {},
        {},
        {}});
    return VesselPolar{std::move(impl)};
}

VesselPolar VesselPolar::default_racer_cruiser_45ft() {
    return VesselPolar{};
}

double VesselPolar::boat_speed_knots(
    double true_wind_speed_knots, double true_wind_angle_degrees) const noexcept {
    return slice_at(true_wind_speed_knots).speed_knots(true_wind_angle_degrees);
}

PolarSlice VesselPolar::slice_at(
    double true_wind_speed_knots,
    PolarAngleInterpolation interpolation) const noexcept {
    PolarSlice slice;
    if (!impl_ || !std::isfinite(true_wind_speed_knots) ||
        true_wind_speed_knots <= 0.0) {
        return slice;
    }

    const Interval wind = find_interval(impl_->wind_speeds, true_wind_speed_knots);
    slice.wind_angles_ = impl_->wind_angles.data();
    slice.boat_speeds_ = impl_->boat_speeds.data();
    slice.sailing_points_ = impl_->sailing_points.data();
    slice.sailing_intervals_ = impl_->sailing_intervals.data();
    slice.wind_angle_count_ = impl_->wind_angles.size();
    slice.wind_speed_count_ = impl_->wind_speeds.size();
    slice.wind_lower_ = wind.lower;
    slice.wind_fraction_ = wind.fraction;
    if (wind.fraction == 0.0 || wind.fraction == 1.0) {
        const std::size_t column =
            wind.lower + static_cast<std::size_t>(wind.fraction == 1.0);
        slice.minimum_sailing_angle_degrees_ = impl_->column_minimum_angles[column];
        slice.maximum_sailing_angle_degrees_ = impl_->column_maximum_angles[column];
    } else {
        slice.minimum_sailing_angle_degrees_ = impl_->between_minimum_angles[wind.lower];
        slice.maximum_sailing_angle_degrees_ = impl_->between_maximum_angles[wind.lower];
    }
    slice.interpolation_ = interpolation;
    slice.above_tabulated_wind_speed_ =
        !impl_->wind_speeds.empty() && true_wind_speed_knots > impl_->wind_speeds.back();
    const bool cubic = interpolation == PolarAngleInterpolation::monotone_cubic;
    const std::vector<double>& upwind =
        cubic ? impl_->upwind_vmg_cubic : impl_->upwind_vmg_linear;
    const std::vector<double>& downwind =
        cubic ? impl_->downwind_vmg_cubic : impl_->downwind_vmg_linear;
    if (!upwind.empty()) {
        slice.upwind_vmg_ = upwind.data();
        slice.downwind_vmg_ = downwind.data();
    }
    return slice;
}

double VesselPolar::maximum_tabulated_wind_speed_knots() const noexcept {
    if (!impl_ || impl_->wind_speeds.empty()) {
        return 0.0;
    }
    return impl_->wind_speeds.back();
}

double VesselPolar::maximum_boat_speed_knots() const noexcept {
    if (!impl_ || impl_->boat_speeds.empty()) {
        return 0.0;
    }
    return *std::max_element(
        impl_->boat_speeds.begin(), impl_->boat_speeds.end());
}

VelocityMadeGoodAngles PolarSlice::velocity_made_good_angles() const noexcept {
    VelocityMadeGoodAngles angles;
    if (upwind_vmg_ == nullptr || !std::isfinite(minimum_sailing_angle_degrees_)) {
        return angles;
    }
    // The optimum moves smoothly with wind speed, so blending the two bracketing
    // columns is enough to place the extra headings the search evaluates exactly.
    angles.upwind_degrees = upwind_vmg_[wind_lower_] +
        wind_fraction_ * (upwind_vmg_[wind_lower_ + 1U] - upwind_vmg_[wind_lower_]);
    angles.downwind_degrees = downwind_vmg_[wind_lower_] +
        wind_fraction_ * (downwind_vmg_[wind_lower_ + 1U] - downwind_vmg_[wind_lower_]);
    const auto supported_proposal = [&](double proposal, bool upwind) {
        proposal = std::clamp(
            proposal, minimum_sailing_angle_degrees_, maximum_sailing_angle_degrees_);
        if (supports_sailing_angle(proposal)) {
            return proposal;
        }
        // Blending VMG targets can land inside a forbidden interior sector.
        double best = minimum_sailing_angle_degrees_;
        double best_vmg = -std::numeric_limits<double>::infinity();
        for (std::size_t row = 0; row < wind_angle_count_; ++row) {
            const double angle = std::clamp(
                wind_angles_[row],
                minimum_sailing_angle_degrees_, maximum_sailing_angle_degrees_);
            if (!supports_sailing_angle(angle)) {
                continue;
            }
            const double vmg = speed_knots(angle) *
                std::cos(angle * std::numbers::pi / 180.0) * (upwind ? 1.0 : -1.0);
            if (vmg > best_vmg) {
                best_vmg = vmg;
                best = angle;
            }
        }
        return best;
    };
    angles.upwind_degrees = supported_proposal(angles.upwind_degrees, true);
    angles.downwind_degrees = supported_proposal(angles.downwind_degrees, false);
    angles.valid = true;
    return angles;
}

bool PolarSlice::supports_sailing_angle(double true_wind_angle_degrees) const noexcept {
    if (!valid() || !std::isfinite(true_wind_angle_degrees) ||
        !std::isfinite(minimum_sailing_angle_degrees_)) {
        return false;
    }
    double angle = fold_angle(true_wind_angle_degrees);
    // Inverting a geodesic can perturb an exactly tabulated heading by a few
    // ulps. This tolerance must not turn a real zero-speed sector into support.
    constexpr double angle_roundoff_degrees = 1.0e-8;
    const auto upper = std::lower_bound(wind_angles_, wind_angles_ + wind_angle_count_, angle);
    if (upper != wind_angles_ + wind_angle_count_ &&
        std::abs(*upper - angle) <= angle_roundoff_degrees) {
        angle = *upper;
    } else if (upper != wind_angles_ &&
               std::abs(*(upper - 1) - angle) <= angle_roundoff_degrees) {
        angle = *(upper - 1);
    }
    if (angle < minimum_sailing_angle_degrees_ || angle > maximum_sailing_angle_degrees_) {
        return false;
    }
    const Interval interval = find_interval(wind_angles_, wind_angle_count_, angle);
    const bool point = interval.fraction == 0.0 || interval.fraction == 1.0;
    const std::size_t row = interval.lower +
        static_cast<std::size_t>(interval.fraction == 1.0);
    const auto* support = point ? sailing_points_ : sailing_intervals_;
    const std::size_t base = row * wind_speed_count_ + wind_lower_;
    return (wind_fraction_ == 1.0 || support[base]) &&
        (wind_fraction_ == 0.0 || support[base + 1U]);
}

double PolarSlice::speed_knots(double true_wind_angle_degrees) const noexcept {
    if (boat_speeds_ == nullptr || !std::isfinite(true_wind_angle_degrees)) {
        return 0.0;
    }

    const double angle = fold_angle(true_wind_angle_degrees);
    const Interval twa = find_interval(wind_angles_, wind_angle_count_, angle);
    const std::size_t columns = wind_speed_count_;
    const std::size_t row0 = twa.lower * columns;
    const std::size_t row1 = row0 + columns;

    const double low_angle_speed =
        boat_speeds_[row0 + wind_lower_] +
        wind_fraction_ *
            (boat_speeds_[row0 + wind_lower_ + 1] - boat_speeds_[row0 + wind_lower_]);
    const double high_angle_speed =
        boat_speeds_[row1 + wind_lower_] +
        wind_fraction_ *
            (boat_speeds_[row1 + wind_lower_ + 1] - boat_speeds_[row1 + wind_lower_]);
    if (interpolation_ != PolarAngleInterpolation::monotone_cubic ||
        wind_angle_count_ < 3U) {
        return low_angle_speed + twa.fraction * (high_angle_speed - low_angle_speed);
    }

    // Blend the wind-speed columns first, then run the monotone cubic across the
    // angle rows. Only the four rows bracketing the interval affect the result.
    const auto blended = [&](std::size_t row) {
        const std::size_t base = row * columns + wind_lower_;
        return boat_speeds_[base] +
            wind_fraction_ * (boat_speeds_[base + 1U] - boat_speeds_[base]);
    };
    const std::size_t first = twa.lower > 0U ? twa.lower - 1U : 0U;
    const std::size_t last =
        std::min(twa.lower + 2U, wind_angle_count_ - 1U);
    const std::size_t local_count = last - first + 1U;
    std::array<double, 4> local_angles{};
    std::array<double, 4> local_speeds{};
    for (std::size_t index = 0U; index < local_count; ++index) {
        local_angles[index] = wind_angles_[first + index];
        local_speeds[index] = blended(first + index);
    }
    const std::size_t local_lower = twa.lower - first;
    const double slope0 = pchip_slope(
        local_angles.data(), local_speeds.data(), local_count, local_lower);
    const double slope1 = pchip_slope(
        local_angles.data(), local_speeds.data(), local_count, local_lower + 1U);
    return hermite(
        wind_angles_[twa.lower],
        wind_angles_[twa.lower + 1U],
        low_angle_speed,
        high_angle_speed,
        slope0,
        slope1,
        wind_angles_[twa.lower] +
            twa.fraction *
                (wind_angles_[twa.lower + 1U] - wind_angles_[twa.lower]));
}

const std::string& VesselPolar::source() const noexcept {
    static const std::string empty_source;
    return impl_ ? impl_->source_description : empty_source;
}

}  // namespace sailroute
