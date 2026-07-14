#include "sailroute/router.hpp"

#include "routing/geodesy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <limits>
#include <map>
#include <mutex>
#include <numbers>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace sailroute {
namespace {

using NodeIndex = std::size_t;
constexpr NodeIndex no_parent = std::numeric_limits<NodeIndex>::max();

struct SearchNode {
    RoutePoint point;
    NodeIndex parent{no_parent};
};

struct ExpansionOrdinal {
    std::size_t parent{};
    std::size_t heading{};

    friend bool operator<(
        const ExpansionOrdinal& left,
        const ExpansionOrdinal& right) noexcept {
        return std::tie(left.parent, left.heading) <
               std::tie(right.parent, right.heading);
    }
};

struct Candidate {
    RoutePoint point;
    NodeIndex parent{};
    double distance_to_destination{};
    ExpansionOrdinal ordinal;
};

struct BucketKey {
    std::int64_t east{};
    std::int64_t north{};

    friend bool operator<(const BucketKey& left, const BucketKey& right) noexcept {
        return std::tie(left.east, left.north) < std::tie(right.east, right.north);
    }
};

struct Arrival {
    Candidate candidate;
    double fraction{};
};

struct ExpansionBuffer {
    std::vector<Candidate> candidates;
    std::optional<Arrival> best_arrival;
    std::optional<Error> interpolation_error;
    std::exception_ptr exception;
    std::size_t expanded_nodes{};
    std::size_t generated_candidates{};
    bool non_finite_wind{};

    void clear() {
        candidates.clear();
        best_arrival.reset();
        interpolation_error.reset();
        exception = nullptr;
        expanded_nodes = 0U;
        generated_candidates = 0U;
        non_finite_wind = false;
    }
};

std::optional<Error> validate_request(const RouteRequest& request) {
    if (!is_valid(request.start)) {
        return Error{
            ErrorCode::invalid_argument,
            "start coordinate must contain finite latitude [-90, 90] and longitude [-180, 180]"};
    }
    if (!is_valid(request.destination)) {
        return Error{
            ErrorCode::invalid_argument,
            "destination coordinate must contain finite latitude [-90, 90] and longitude [-180, 180]"};
    }

    const RoutingOptions& options = request.options;
    if (options.time_step <= std::chrono::minutes::zero()) {
        return Error{ErrorCode::invalid_argument, "routing time_step must be positive"};
    }
    if (!std::isfinite(options.heading_step_degrees) ||
        options.heading_step_degrees <= 0.0 ||
        options.heading_step_degrees > 180.0 ||
        360.0 / options.heading_step_degrees > 1'000'000.0) {
        return Error{
            ErrorCode::invalid_argument,
            "heading_step_degrees must be finite, practical, and in the range (0, 180]"};
    }
    if (!std::isfinite(options.arrival_radius_nautical_miles) ||
        options.arrival_radius_nautical_miles <= 0.0) {
        return Error{
            ErrorCode::invalid_argument,
            "arrival_radius_nautical_miles must be finite and positive"};
    }
    if (!std::isfinite(options.spatial_bucket_nautical_miles) ||
        options.spatial_bucket_nautical_miles <= 0.0) {
        return Error{
            ErrorCode::invalid_argument,
            "spatial_bucket_nautical_miles must be finite and positive"};
    }
    if (options.max_nodes_per_bucket == 0U) {
        return Error{ErrorCode::invalid_argument, "max_nodes_per_bucket must be positive"};
    }
    if (options.maximum_route_duration <= std::chrono::hours::zero()) {
        return Error{ErrorCode::invalid_argument, "maximum_route_duration must be positive"};
    }
    if (!std::isfinite(options.minimum_boat_speed_knots) ||
        options.minimum_boat_speed_knots < 0.0) {
        return Error{
            ErrorCode::invalid_argument,
            "minimum_boat_speed_knots must be finite and non-negative"};
    }
    return std::nullopt;
}

Result<std::pair<TimePoint, DepartureSource>> select_departure(
    const RouteRequest& request,
    const ForecastMetadata& metadata) {
    if (metadata.first_valid_time > metadata.last_valid_time) {
        return Error{
            ErrorCode::incomplete_forecast,
            "forecast metadata has an invalid time range"};
    }

    if (request.departure_time.has_value()) {
        const TimePoint departure = *request.departure_time;
        if (departure < metadata.first_valid_time || departure > metadata.last_valid_time) {
            return Error{
                ErrorCode::departure_outside_forecast,
                "explicit departure is outside forecast coverage"};
        }
        return std::pair{departure, DepartureSource::explicit_time};
    }

    const TimePoint now =
        std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now());
    if (now >= metadata.first_valid_time && now <= metadata.last_valid_time) {
        return std::pair{now, DepartureSource::current_time};
    }
    return std::pair{metadata.first_valid_time, DepartureSource::forecast_start_fallback};
}

double true_wind_angle(double heading_degrees, double wind_from_degrees) noexcept {
    return detail::angular_difference_degrees(heading_degrees, wind_from_degrees);
}

std::optional<double> arrival_fraction(
    Coordinate segment_start,
    double heading_degrees,
    double segment_distance_nautical_miles,
    Coordinate destination,
    double arrival_radius_nautical_miles) {
    const double start_distance =
        detail::great_circle_distance_nautical_miles(segment_start, destination);
    if (start_distance <= arrival_radius_nautical_miles) {
        return 0.0;
    }
    if (segment_distance_nautical_miles <= 0.0) {
        return std::nullopt;
    }

    const double start_to_destination =
        start_distance / detail::earth_radius_nautical_miles;
    const double bearing_delta =
        (detail::initial_bearing_degrees(segment_start, destination) - heading_degrees) *
        std::numbers::pi / 180.0;
    const double along_track_angle = std::atan2(
        std::sin(start_to_destination) * std::cos(bearing_delta),
        std::cos(start_to_destination));
    const double segment_angle =
        segment_distance_nautical_miles / detail::earth_radius_nautical_miles;
    const double closest_angle = std::clamp(along_track_angle, 0.0, segment_angle);
    const double closest_fraction = closest_angle / segment_angle;
    const Coordinate closest = detail::destination_point(
        segment_start,
        heading_degrees,
        segment_distance_nautical_miles * closest_fraction);
    if (detail::great_circle_distance_nautical_miles(closest, destination) >
        arrival_radius_nautical_miles) {
        return std::nullopt;
    }

    double outside = 0.0;
    double inside = closest_fraction;
    for (int iteration = 0; iteration < 60; ++iteration) {
        const double middle = (outside + inside) / 2.0;
        const Coordinate point = detail::destination_point(
            segment_start,
            heading_degrees,
            segment_distance_nautical_miles * middle);
        if (detail::great_circle_distance_nautical_miles(point, destination) <=
            arrival_radius_nautical_miles) {
            inside = middle;
        } else {
            outside = middle;
        }
    }
    return inside;
}

bool better_arrival(const Arrival& left, const Arrival& right) noexcept {
    if (left.candidate.point.time != right.candidate.point.time) {
        return left.candidate.point.time < right.candidate.point.time;
    }
    if (left.fraction != right.fraction) {
        return left.fraction < right.fraction;
    }
    return left.candidate.ordinal < right.candidate.ordinal;
}

void expand_candidate_range(
    ExpansionBuffer& buffer,
    const WeatherDataset& weather,
    const VesselPolar& polar,
    const RouteRequest& request,
    const std::vector<SearchNode>& nodes,
    const std::vector<NodeIndex>& frontier,
    std::size_t begin,
    std::size_t end,
    TimePoint current_time,
    std::chrono::seconds step,
    double step_hours,
    std::size_t heading_count) {
    buffer.clear();
    const std::size_t parent_count = end - begin;
    if (parent_count <=
        std::numeric_limits<std::size_t>::max() / heading_count) {
        const std::size_t maximum_candidates = parent_count * heading_count;
        if (maximum_candidates > buffer.candidates.capacity()) {
            buffer.candidates.reserve(maximum_candidates);
        }
    }

    for (std::size_t frontier_index = begin; frontier_index < end;
         ++frontier_index) {
        const NodeIndex parent_index = frontier[frontier_index];
        const SearchNode& parent = nodes[parent_index];
        ++buffer.expanded_nodes;

        const auto wind_result =
            weather.interpolate(parent.point.position, current_time);
        if (!wind_result) {
            if (!buffer.interpolation_error.has_value()) {
                buffer.interpolation_error = wind_result.error();
            }
            continue;
        }
        const Wind wind = wind_result.value();
        const double wind_speed = wind.speed_knots();
        const double wind_direction = wind.direction_from_degrees();
        if (!std::isfinite(wind_speed) || !std::isfinite(wind_direction)) {
            buffer.non_finite_wind = true;
            return;
        }

        for (std::size_t heading_index = 0U; heading_index < heading_count;
             ++heading_index) {
            const double heading =
                static_cast<double>(heading_index) *
                request.options.heading_step_degrees;
            const double boat_speed = polar.boat_speed_knots(
                wind_speed,
                true_wind_angle(heading, wind_direction));
            if (!std::isfinite(boat_speed) || boat_speed <= 0.0 ||
                boat_speed < request.options.minimum_boat_speed_knots) {
                continue;
            }

            const double segment_distance = boat_speed * step_hours;
            const Coordinate position = detail::destination_point(
                parent.point.position,
                heading,
                segment_distance);
            Candidate candidate{
                RoutePoint{
                    position,
                    current_time + step,
                    heading,
                    boat_speed,
                    wind_speed,
                    wind_direction,
                    parent.point.cumulative_distance_nautical_miles +
                        segment_distance},
                parent_index,
                detail::great_circle_distance_nautical_miles(
                    position,
                    request.destination),
                ExpansionOrdinal{frontier_index, heading_index}};
            ++buffer.generated_candidates;

            const std::optional<double> fraction = arrival_fraction(
                parent.point.position,
                heading,
                segment_distance,
                request.destination,
                request.options.arrival_radius_nautical_miles);
            if (fraction.has_value()) {
                const double travelled = segment_distance * *fraction;
                const auto elapsed_seconds = std::chrono::seconds{
                    static_cast<std::chrono::seconds::rep>(
                        std::llround(
                            static_cast<double>(step.count()) * *fraction))};
                candidate.point.position = detail::destination_point(
                    parent.point.position,
                    heading,
                    travelled);
                candidate.point.time = current_time + elapsed_seconds;
                candidate.point.cumulative_distance_nautical_miles =
                    parent.point.cumulative_distance_nautical_miles + travelled;
                candidate.distance_to_destination =
                    detail::great_circle_distance_nautical_miles(
                        candidate.point.position,
                        request.destination);

                Arrival arrival{std::move(candidate), *fraction};
                if (!buffer.best_arrival.has_value() ||
                    better_arrival(arrival, *buffer.best_arrival)) {
                    buffer.best_arrival = std::move(arrival);
                }
            } else {
                buffer.candidates.push_back(std::move(candidate));
            }
        }
    }
}

class CandidateExpansionWorkers {
public:
    CandidateExpansionWorkers(
        const WeatherDataset& weather,
        const VesselPolar& polar,
        const RouteRequest& request)
        : weather_(weather), polar_(polar), request_(request) {}

    CandidateExpansionWorkers(const CandidateExpansionWorkers&) = delete;
    CandidateExpansionWorkers& operator=(const CandidateExpansionWorkers&) = delete;

    ~CandidateExpansionWorkers() {
        {
            std::lock_guard lock(mutex_);
            stopping_ = true;
        }
        start_condition_.notify_all();
        for (std::thread& worker : threads_) {
            worker.join();
        }
    }

    [[nodiscard]] std::size_t worker_count(
        std::size_t frontier_size,
        std::size_t heading_count) const noexcept {
        if (frontier_size < 2U || request_.options.worker_count == 1U) {
            return 1U;
        }
        if (request_.options.worker_count > 1U) {
            return std::min(request_.options.worker_count, frontier_size);
        }

        const unsigned hardware_threads = std::thread::hardware_concurrency();
        const std::size_t hardware_limit =
            hardware_threads == 0U
                ? 1U
                : static_cast<std::size_t>(hardware_threads);
        constexpr std::size_t heading_attempts_per_worker = 256U;
        const std::size_t heading_attempts =
            frontier_size >
                    std::numeric_limits<std::size_t>::max() / heading_count
                ? std::numeric_limits<std::size_t>::max()
                : frontier_size * heading_count;
        const std::size_t useful_workers =
            std::max<std::size_t>(
                1U,
                heading_attempts / heading_attempts_per_worker);
        return std::min({hardware_limit, frontier_size, useful_workers});
    }

    void expand(
        std::size_t active_workers,
        const std::vector<SearchNode>& nodes,
        const std::vector<NodeIndex>& frontier,
        TimePoint current_time,
        std::chrono::seconds step,
        double step_hours,
        std::size_t heading_count) {
        ensure_worker_count(active_workers);
        {
            std::lock_guard lock(mutex_);
            nodes_ = &nodes;
            frontier_ = &frontier;
            current_time_ = current_time;
            step_ = step;
            step_hours_ = step_hours;
            heading_count_ = heading_count;
            active_workers_ = active_workers;
            completed_workers_ = 0U;
            ++generation_;
        }
        start_condition_.notify_all();
        run_worker(0U);

        {
            std::unique_lock lock(mutex_);
            done_condition_.wait(
                lock,
                [this] { return completed_workers_ == threads_.size(); });
        }
        for (std::size_t index = 0U; index < active_workers; ++index) {
            if (buffers_[index].exception) {
                std::rethrow_exception(buffers_[index].exception);
            }
        }
    }

    [[nodiscard]] ExpansionBuffer& buffer(std::size_t index) {
        return buffers_[index];
    }

private:
    void ensure_worker_count(std::size_t worker_count) {
        if (buffers_.size() >= worker_count) {
            return;
        }
        buffers_.resize(worker_count);
        while (threads_.size() + 1U < worker_count) {
            const std::size_t index = threads_.size() + 1U;
            const std::size_t initial_generation = generation_;
            threads_.emplace_back(
                [this, index, initial_generation] {
                    worker_loop(index, initial_generation);
                });
        }
    }

    [[nodiscard]] std::pair<std::size_t, std::size_t> range_for(
        std::size_t worker_index) const noexcept {
        const std::size_t quotient = frontier_->size() / active_workers_;
        const std::size_t remainder = frontier_->size() % active_workers_;
        const std::size_t begin =
            worker_index * quotient + std::min(worker_index, remainder);
        const std::size_t count =
            quotient + (worker_index < remainder ? 1U : 0U);
        return {begin, begin + count};
    }

    void run_worker(std::size_t worker_index) noexcept {
        ExpansionBuffer& buffer = buffers_[worker_index];
        try {
            const auto [begin, end] = range_for(worker_index);
            expand_candidate_range(
                buffer,
                weather_,
                polar_,
                request_,
                *nodes_,
                *frontier_,
                begin,
                end,
                current_time_,
                step_,
                step_hours_,
                heading_count_);
        } catch (...) {
            buffer.clear();
            buffer.exception = std::current_exception();
        }
    }

    void worker_loop(
        std::size_t worker_index,
        std::size_t observed_generation) noexcept {
        while (true) {
            {
                std::unique_lock lock(mutex_);
                start_condition_.wait(
                    lock,
                    [this, observed_generation] {
                        return stopping_ || generation_ != observed_generation;
                    });
                if (stopping_) {
                    return;
                }
                observed_generation = generation_;
            }

            if (worker_index < active_workers_) {
                run_worker(worker_index);
            }
            {
                std::lock_guard lock(mutex_);
                ++completed_workers_;
            }
            done_condition_.notify_one();
        }
    }

    const WeatherDataset& weather_;
    const VesselPolar& polar_;
    const RouteRequest& request_;
    std::vector<ExpansionBuffer> buffers_;
    std::vector<std::thread> threads_;
    std::mutex mutex_;
    std::condition_variable start_condition_;
    std::condition_variable done_condition_;
    const std::vector<SearchNode>* nodes_{};
    const std::vector<NodeIndex>* frontier_{};
    TimePoint current_time_{};
    std::chrono::seconds step_{};
    double step_hours_{};
    std::size_t heading_count_{};
    std::size_t active_workers_{};
    std::size_t completed_workers_{};
    std::size_t generation_{};
    bool stopping_{};
};

BucketKey bucket_for(
    Coordinate coordinate,
    Coordinate destination,
    double bucket_size_nautical_miles) noexcept {
    double longitude_delta = coordinate.longitude_degrees - destination.longitude_degrees;
    longitude_delta = std::fmod(longitude_delta + 540.0, 360.0) - 180.0;
    const double mean_latitude =
        (coordinate.latitude_degrees + destination.latitude_degrees) *
        std::numbers::pi / 360.0;
    const double east_nautical_miles =
        longitude_delta * 60.0 * std::cos(mean_latitude);
    const double north_nautical_miles =
        (coordinate.latitude_degrees - destination.latitude_degrees) * 60.0;
    return BucketKey{
        static_cast<std::int64_t>(
            std::floor(east_nautical_miles / bucket_size_nautical_miles)),
        static_cast<std::int64_t>(
            std::floor(north_nautical_miles / bucket_size_nautical_miles))};
}

bool dominates(const Candidate& left, const Candidate& right) noexcept {
    if (left.distance_to_destination != right.distance_to_destination) {
        return left.distance_to_destination < right.distance_to_destination;
    }
    if (left.point.boat_speed_knots != right.point.boat_speed_knots) {
        return left.point.boat_speed_knots > right.point.boat_speed_knots;
    }
    return left.ordinal < right.ordinal;
}

std::vector<std::size_t> prune_candidates(
    const std::vector<Candidate>& candidates,
    Coordinate destination,
    const RoutingOptions& options) {
    std::map<BucketKey, std::vector<std::size_t>> buckets;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        buckets[bucket_for(
                    candidates[index].point.position,
                    destination,
                    options.spatial_bucket_nautical_miles)]
            .push_back(index);
    }

    std::vector<std::size_t> retained;
    retained.reserve(candidates.size());
    for (auto& [key, bucket] : buckets) {
        static_cast<void>(key);
        std::stable_sort(
            bucket.begin(),
            bucket.end(),
            [&candidates](std::size_t left, std::size_t right) {
                return dominates(candidates[left], candidates[right]);
            });

        std::vector<std::size_t> selected;
        selected.reserve(std::min(options.max_nodes_per_bucket, bucket.size()));
        selected.push_back(bucket.front());
        while (selected.size() < options.max_nodes_per_bucket &&
               selected.size() < bucket.size()) {
            std::size_t best_index = bucket.front();
            double best_separation = -1.0;
            bool found = false;
            for (const std::size_t candidate_index : bucket) {
                if (std::find(selected.begin(), selected.end(), candidate_index) !=
                    selected.end()) {
                    continue;
                }
                double separation = 180.0;
                for (const std::size_t selected_index : selected) {
                    separation = std::min(
                        separation,
                        detail::angular_difference_degrees(
                            candidates[candidate_index].point.heading_degrees,
                            candidates[selected_index].point.heading_degrees));
                }
                if (!found || separation > best_separation ||
                    (separation == best_separation &&
                     dominates(candidates[candidate_index], candidates[best_index]))) {
                    best_index = candidate_index;
                    best_separation = separation;
                    found = true;
                }
            }
            if (!found) {
                break;
            }
            selected.push_back(best_index);
        }
        retained.insert(retained.end(), selected.begin(), selected.end());
    }

    std::sort(
        retained.begin(),
        retained.end(),
        [&candidates](std::size_t left, std::size_t right) {
            return candidates[left].ordinal < candidates[right].ordinal;
        });
    return retained;
}

std::vector<RoutePoint> reconstruct_route(
    const std::vector<SearchNode>& nodes,
    NodeIndex arrival_index) {
    std::vector<RoutePoint> reversed;
    for (NodeIndex index = arrival_index; index != no_parent; index = nodes[index].parent) {
        reversed.push_back(nodes[index].point);
    }
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

Isochrone capture_isochrone(
    const std::vector<SearchNode>& nodes,
    const std::vector<NodeIndex>& frontier) {
    Isochrone isochrone;
    isochrone.time = nodes[frontier.front()].point.time;
    isochrone.points.reserve(frontier.size());
    for (const NodeIndex index : frontier) {
        isochrone.points.push_back(nodes[index].point.position);
    }
    return isochrone;
}

Error exhausted_error(
    bool forecast_limited,
    const RouteDiagnostics& diagnostics) {
    const std::string suffix =
        " after " + std::to_string(diagnostics.time_steps) + " time steps and " +
        std::to_string(diagnostics.expanded_nodes) + " expanded nodes";
    if (forecast_limited) {
        return Error{
            ErrorCode::forecast_exhausted,
            "forecast coverage ended before the destination was reached" + suffix};
    }
    return Error{
        ErrorCode::no_route,
        "maximum route duration ended before the destination was reached" + suffix};
}

}  // namespace

Router::Router(WeatherDataset weather, VesselPolar polar)
    : weather_(std::move(weather)), polar_(std::move(polar)) {}

Result<RouteResult> Router::optimize(const RouteRequest& request) const {
    if (const std::optional<Error> validation = validate_request(request);
        validation.has_value()) {
        return *validation;
    }

    const ForecastMetadata& metadata = weather_.metadata();
    const auto selected_departure = select_departure(request, metadata);
    if (!selected_departure) {
        return selected_departure.error();
    }
    auto [departure, departure_source] = selected_departure.value();

    auto start_wind = weather_.interpolate(request.start, departure);
    auto destination_wind = weather_.interpolate(request.destination, departure);
    if (!request.departure_time.has_value() &&
        departure_source == DepartureSource::current_time &&
        (!start_wind.has_value() || !destination_wind.has_value())) {
        departure = metadata.first_valid_time;
        departure_source = DepartureSource::forecast_start_fallback;
        start_wind = weather_.interpolate(request.start, departure);
        destination_wind = weather_.interpolate(request.destination, departure);
    }
    if (!start_wind) {
        return start_wind.error();
    }
    if (!destination_wind) {
        return destination_wind.error();
    }

    RouteDiagnostics diagnostics;
    std::vector<SearchNode> nodes;
    nodes.reserve(1024);
    nodes.push_back(SearchNode{
        RoutePoint{
            request.start,
            departure,
            0.0,
            0.0,
            start_wind.value().speed_knots(),
            start_wind.value().direction_from_degrees(),
            0.0},
        no_parent});

    if (detail::great_circle_distance_nautical_miles(
            request.start,
            request.destination) <= request.options.arrival_radius_nautical_miles) {
        RouteResult result;
        result.departure_time = departure;
        result.arrival_time = departure;
        result.departure_source = departure_source;
        result.forecast_source = metadata.source;
        result.polar_source = polar_.source();
        result.points.push_back(nodes.front().point);
        result.diagnostics = diagnostics;
        return result;
    }

    std::vector<NodeIndex> frontier{0U};
    std::vector<Candidate> candidates;
    std::vector<NodeIndex> next_frontier;
    std::vector<Isochrone> isochrones;

    const TimePoint horizon_end = departure + request.options.maximum_route_duration;
    const TimePoint route_end = std::min(horizon_end, metadata.last_valid_time);
    const bool forecast_limited = metadata.last_valid_time <= horizon_end;
    const auto configured_step =
        std::chrono::duration_cast<std::chrono::seconds>(request.options.time_step);
    const std::size_t heading_count = static_cast<std::size_t>(
        std::ceil(360.0 / request.options.heading_step_degrees));
    ExpansionBuffer single_worker_buffer;
    CandidateExpansionWorkers expansion_workers{weather_, polar_, request};

    while (!frontier.empty()) {
        const TimePoint current_time = nodes[frontier.front()].point.time;
        if (current_time >= route_end) {
            return exhausted_error(forecast_limited, diagnostics);
        }
        const auto remaining =
            std::chrono::duration_cast<std::chrono::seconds>(route_end - current_time);
        const auto step = std::min(configured_step, remaining);
        if (step <= std::chrono::seconds::zero()) {
            return exhausted_error(forecast_limited, diagnostics);
        }

        ++diagnostics.time_steps;
        candidates.clear();
        std::optional<Arrival> best_arrival;
        std::optional<Error> interpolation_error;

        const double step_hours =
            std::chrono::duration<double, std::ratio<3600>>(step).count();
        const std::size_t active_workers =
            expansion_workers.worker_count(frontier.size(), heading_count);
        const auto merge_buffer = [&](const ExpansionBuffer& buffer) {
            diagnostics.expanded_nodes += buffer.expanded_nodes;
            diagnostics.generated_candidates += buffer.generated_candidates;
            if (!interpolation_error.has_value() &&
                buffer.interpolation_error.has_value()) {
                interpolation_error = buffer.interpolation_error;
            }
            if (buffer.best_arrival.has_value() &&
                (!best_arrival.has_value() ||
                 better_arrival(*buffer.best_arrival, *best_arrival))) {
                best_arrival = buffer.best_arrival;
            }
        };

        if (active_workers == 1U) {
            expand_candidate_range(
                single_worker_buffer,
                weather_,
                polar_,
                request,
                nodes,
                frontier,
                0U,
                frontier.size(),
                current_time,
                step,
                step_hours,
                heading_count);
            merge_buffer(single_worker_buffer);
            if (single_worker_buffer.non_finite_wind) {
                return Error{
                    ErrorCode::incomplete_forecast,
                    "forecast interpolation produced non-finite wind"};
            }
            if (!best_arrival.has_value()) {
                candidates.insert(
                    candidates.end(),
                    std::make_move_iterator(
                        single_worker_buffer.candidates.begin()),
                    std::make_move_iterator(
                        single_worker_buffer.candidates.end()));
            }
        } else {
            expansion_workers.expand(
                active_workers,
                nodes,
                frontier,
                current_time,
                step,
                step_hours,
                heading_count);
            bool non_finite_wind = false;
            for (std::size_t index = 0U; index < active_workers; ++index) {
                const ExpansionBuffer& buffer = expansion_workers.buffer(index);
                merge_buffer(buffer);
                non_finite_wind =
                    non_finite_wind || buffer.non_finite_wind;
            }
            if (non_finite_wind) {
                return Error{
                    ErrorCode::incomplete_forecast,
                    "forecast interpolation produced non-finite wind"};
            }
            if (!best_arrival.has_value()) {
                std::size_t candidate_count = 0U;
                for (std::size_t index = 0U; index < active_workers; ++index) {
                    candidate_count +=
                        expansion_workers.buffer(index).candidates.size();
                }
                candidates.reserve(candidate_count);
                for (std::size_t index = 0U; index < active_workers; ++index) {
                    ExpansionBuffer& buffer =
                        expansion_workers.buffer(index);
                    candidates.insert(
                        candidates.end(),
                        std::make_move_iterator(buffer.candidates.begin()),
                        std::make_move_iterator(buffer.candidates.end()));
                }
            }
        }

        if (best_arrival.has_value()) {
            nodes.push_back(SearchNode{
                std::move(best_arrival->candidate.point),
                best_arrival->candidate.parent});
            ++diagnostics.retained_candidates;
            RouteResult result;
            result.departure_time = departure;
            result.arrival_time = nodes.back().point.time;
            result.departure_source = departure_source;
            result.forecast_source = metadata.source;
            result.polar_source = polar_.source();
            result.points = reconstruct_route(nodes, nodes.size() - 1U);
            result.isochrones = std::move(isochrones);
            result.diagnostics = diagnostics;
            return result;
        }

        if (candidates.empty()) {
            if (interpolation_error.has_value()) {
                return *interpolation_error;
            }
            return Error{
                ErrorCode::no_route,
                "no heading met the minimum boat speed at routing step " +
                    std::to_string(diagnostics.time_steps)};
        }

        const std::vector<std::size_t> retained =
            prune_candidates(candidates, request.destination, request.options);
        next_frontier.clear();
        next_frontier.reserve(retained.size());
        nodes.reserve(nodes.size() + retained.size());
        for (const std::size_t candidate_index : retained) {
            Candidate& candidate = candidates[candidate_index];
            nodes.push_back(SearchNode{std::move(candidate.point), candidate.parent});
            next_frontier.push_back(nodes.size() - 1U);
        }
        diagnostics.retained_candidates += retained.size();
        frontier.swap(next_frontier);
        if (request.options.capture_isochrones) {
            isochrones.push_back(capture_isochrone(nodes, frontier));
        }
    }

    return Error{ErrorCode::no_route, "isochrone pruning removed every candidate"};
}

}  // namespace sailroute
