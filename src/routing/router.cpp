#include "sailroute/router.hpp"

#include "routing/contours.hpp"
#include "routing/environment_context.hpp"
#include "routing/front.hpp"
#include "routing/geodesy.hpp"
#include "routing/intervals.hpp"
#include "routing/lattice.hpp"
#include "routing/lattice_solver.hpp"

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
    // Zero unless maneuver penalties are active, in which case candidates on
    // opposite boards are kept apart so a marginally closer wrong-tack
    // candidate cannot displace a genuinely faster one.
    std::int64_t board{};

    friend bool operator<(const BucketKey& left, const BucketKey& right) noexcept {
        return std::tie(left.east, left.north, left.board) <
            std::tie(right.east, right.north, right.board);
    }
};

struct Arrival {
    Candidate candidate;
    double fraction{};
};

struct CandidateEvaluation {
    Candidate candidate;
    std::optional<double> arrival_fraction;
};

struct ExpansionBuffer {
    std::vector<Candidate> candidates;
    std::vector<Arrival> arrivals;
    // Headings evaluated for the parent being expanded, rebuilt per parent so
    // augmentation does not allocate.
    std::vector<double> headings;
    std::optional<Arrival> best_arrival;
    std::optional<Error> interpolation_error;
    // A provider or model failure under a fail_route policy. Recorded rather
    // than thrown so every worker finishes and the reported error is the one
    // from the lowest worker index, independent of scheduling.
    std::optional<Error> environment_error;
    EnvironmentDiagnostics environment;
    std::exception_ptr exception;
    std::size_t expanded_nodes{};
    std::size_t generated_candidates{};
    bool non_finite_wind{};

    void clear() {
        candidates.clear();
        arrivals.clear();
        best_arrival.reset();
        interpolation_error.reset();
        environment_error.reset();
        environment = EnvironmentDiagnostics{};
        exception = nullptr;
        expanded_nodes = 0U;
        generated_candidates = 0U;
        non_finite_wind = false;
    }
};

struct MidpointWeatherSamplers {
    WeatherSampler unpenalized;
    WeatherSampler tack;
    WeatherSampler gybe;

    [[nodiscard]] const WeatherSampler& for_delay(
        std::chrono::seconds delay,
        const ManeuverPenalties& penalties) const noexcept {
        if (delay > std::chrono::seconds::zero() &&
            delay == penalties.tack_penalty) {
            return tack;
        }
        if (delay > std::chrono::seconds::zero() &&
            delay == penalties.gybe_penalty) {
            return gybe;
        }
        return unpenalized;
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
    if (const auto interval_error = detail::validate_routing_intervals(options);
        interval_error.has_value()) {
        return interval_error;
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
    if (options.maneuver.tack_penalty < std::chrono::seconds::zero() ||
        options.maneuver.gybe_penalty < std::chrono::seconds::zero()) {
        return Error{
            ErrorCode::invalid_argument,
            "maneuver penalties must be non-negative"};
    }
    if (!std::isfinite(
            options.maneuver.downwind_true_wind_angle_degrees) ||
        options.maneuver.downwind_true_wind_angle_degrees < 0.0 ||
        options.maneuver.downwind_true_wind_angle_degrees > 180.0) {
        return Error{
            ErrorCode::invalid_argument,
            "downwind_true_wind_angle_degrees must be finite and in [0, 180]"};
    }
    if (options.midpoint_wind_sampling_threshold <
        std::chrono::minutes::zero()) {
        return Error{
            ErrorCode::invalid_argument,
            "midpoint_wind_sampling_threshold must be non-negative"};
    }
    if (options.maximum_true_wind_speed_knots.has_value() &&
        (!std::isfinite(*options.maximum_true_wind_speed_knots) ||
         *options.maximum_true_wind_speed_knots <= 0.0)) {
        return Error{
            ErrorCode::invalid_argument,
            "maximum_true_wind_speed_knots must be finite and positive"};
    }
    if (options.progress.every_n_steps == 0U) {
        return Error{
            ErrorCode::invalid_argument,
            "progress every_n_steps must be positive"};
    }
    constexpr std::uint8_t known_payloads =
        static_cast<std::uint8_t>(RoutingProgressPayload::retained_points) |
        static_cast<std::uint8_t>(RoutingProgressPayload::provisional_route) |
        static_cast<std::uint8_t>(RoutingProgressPayload::display_contours) |
        static_cast<std::uint8_t>(RoutingProgressPayload::destination_front) |
        static_cast<std::uint8_t>(RoutingProgressPayload::search_points);
    if ((static_cast<std::uint8_t>(options.progress.payload) &
         static_cast<std::uint8_t>(~known_payloads)) != 0U) {
        return Error{
            ErrorCode::invalid_argument,
            "progress payload contains unsupported flags"};
    }
    if (options.progress.display_contours.alpha_nautical_miles.has_value() &&
        (!std::isfinite(
             *options.progress.display_contours.alpha_nautical_miles) ||
         *options.progress.display_contours.alpha_nautical_miles <= 0.0)) {
        return Error{
            ErrorCode::invalid_argument,
            "display contour alpha_nautical_miles must be finite and positive"};
    }
    if (!std::isfinite(
            options.progress.destination_front.half_angle_degrees) ||
        options.progress.destination_front.half_angle_degrees <= 0.0 ||
        options.progress.destination_front.half_angle_degrees > 180.0) {
        return Error{
            ErrorCode::invalid_argument,
            "destination front half_angle_degrees must be finite and in (0, 180]"};
    }
    if (options.progress.destination_front.mode ==
            DestinationFrontMode::eligible_pre_prune &&
        options.progress.destination_front.minimum_secondary_segment_points == 0U) {
        return Error{
            ErrorCode::invalid_argument,
            "destination front minimum_secondary_segment_points must be positive"};
    }
    if (options.solver == RoutingSolver::time_dependent_lattice) {
        if (options.lattice.time_bucket <= std::chrono::minutes::zero()) {
            return Error{
                ErrorCode::invalid_argument,
                "lattice time_bucket must be positive"};
        }
        if (options.lattice.progress_every_n_expansions == 0U) {
            return Error{
                ErrorCode::invalid_argument,
                "lattice progress_every_n_expansions must be positive"};
        }
        if (!std::isfinite(options.lattice.corridor_width_nautical_miles) ||
            options.lattice.corridor_width_nautical_miles <= 0.0) {
            return Error{
                ErrorCode::invalid_argument,
                "lattice corridor_width_nautical_miles must be finite and positive"};
        }
        if (options.lattice.subdivision_level +
                options.lattice.refinement_levels >
            detail::GeodesicLattice::maximum_subdivision_level) {
            return Error{
                ErrorCode::invalid_argument,
                "lattice subdivision and refinement levels exceed the supported maximum"};
        }
        if (options.capture_isochrones ||
            has_payload(
                options.progress.payload,
                RoutingProgressPayload::display_contours) ||
            has_payload(
                options.progress.payload,
                RoutingProgressPayload::destination_front)) {
            return Error{
                ErrorCode::invalid_argument,
                "lattice routing does not produce isochrones, display contours, or destination fronts"};
        }
    }
    if (options.pruning_strategy == PruningStrategy::bearing_sectors &&
        (!std::isfinite(options.pruning_sector_degrees) ||
         options.pruning_sector_degrees <= 0.0 ||
         options.pruning_sector_degrees > 180.0)) {
        return Error{
            ErrorCode::invalid_argument,
            "pruning_sector_degrees must be finite and in (0, 180]"};
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

double normalize_heading(double heading_degrees) noexcept {
    double heading = std::fmod(heading_degrees, 360.0);
    if (heading < 0.0) {
        heading += 360.0;
    }
    return heading;
}

// Which side the wind crosses the boat: 1 and -1 are opposite boards, 0 is head
// to wind or dead downwind, where no board is defined and turning costs nothing.
std::int8_t board_for(double heading_degrees, double wind_from_degrees) noexcept {
    const double delta = normalize_heading(heading_degrees - wind_from_degrees);
    if (delta == 0.0 || delta == 180.0) {
        return 0;
    }
    return delta < 180.0 ? std::int8_t{1} : std::int8_t{-1};
}

// Time lost turning from one board to the other. A board change close to the
// wind is a tack and one away from it is a gybe; the mean of the two true wind
// angles decides which, so a turn is classified by where it actually passes
// through the wind rather than by its endpoints alone.
std::chrono::seconds maneuver_penalty(
    const ManeuverPenalties& penalties,
    std::int8_t parent_board,
    double parent_true_wind_angle,
    std::int8_t board,
    double candidate_true_wind_angle) noexcept {
    if (parent_board == 0 || board == 0 || parent_board == board) {
        return std::chrono::seconds::zero();
    }
    const double mean_angle =
        0.5 * (parent_true_wind_angle + candidate_true_wind_angle);
    return mean_angle >= penalties.downwind_true_wind_angle_degrees
        ? penalties.gybe_penalty
        : penalties.tack_penalty;
}

// Geometry that depends only on the expanding parent, hoisted out of the
// per-heading loop where it was previously recomputed for every candidate.
struct ParentGeometry {
    detail::PreparedOrigin origin;
    double distance_to_destination{};
    double bearing_to_destination_degrees{};
};

[[nodiscard]] ParentGeometry prepare_parent(
    Coordinate position,
    Coordinate destination) noexcept {
    return ParentGeometry{
        detail::prepare_origin(position),
        detail::great_circle_distance_nautical_miles(position, destination),
        detail::initial_bearing_degrees(position, destination)};
}

// Slack applied to the exact triangle-inequality rejection below, large enough
// to absorb rounding in the distance computation and far smaller than any
// meaningful arrival radius, so a genuine arrival is never rejected.
constexpr double arrival_bound_slack_nautical_miles = 1.0e-6;

// Halvings used to locate the arrival radius crossing along a segment. Kept at
// 60 so results stay bit-identical; the loop only runs for candidates that
// actually reach the destination, so it is not on the hot path.
constexpr int arrival_bisection_iterations = 60;

std::optional<double> arrival_fraction(
    const ParentGeometry& parent,
    double heading_degrees,
    double segment_distance_nautical_miles,
    Coordinate destination,
    double arrival_radius_nautical_miles) {
    const double start_distance = parent.distance_to_destination;
    if (start_distance <= arrival_radius_nautical_miles) {
        return 0.0;
    }
    if (segment_distance_nautical_miles <= 0.0) {
        return std::nullopt;
    }
    // Great-circle distance is a metric, so no point on a segment of length d
    // can be closer to the destination than start_distance - d. When even that
    // bound stays outside the arrival radius the segment cannot arrive, and the
    // projection, endpoint construction, and bisection below are all skipped.
    if (start_distance - segment_distance_nautical_miles >
        arrival_radius_nautical_miles + arrival_bound_slack_nautical_miles) {
        return std::nullopt;
    }

    const double start_to_destination =
        start_distance / detail::earth_radius_nautical_miles;
    const double bearing_delta =
        (parent.bearing_to_destination_degrees - heading_degrees) *
        std::numbers::pi / 180.0;
    const double along_track_angle = std::atan2(
        std::sin(start_to_destination) * std::cos(bearing_delta),
        std::cos(start_to_destination));
    const double segment_angle =
        segment_distance_nautical_miles / detail::earth_radius_nautical_miles;
    const double closest_angle = std::clamp(along_track_angle, 0.0, segment_angle);
    const double closest_fraction = closest_angle / segment_angle;
    const Coordinate closest = detail::destination_point_from(
        parent.origin,
        heading_degrees,
        segment_distance_nautical_miles * closest_fraction);
    if (detail::great_circle_distance_nautical_miles(closest, destination) >
        arrival_radius_nautical_miles) {
        return std::nullopt;
    }

    double outside = 0.0;
    double inside = closest_fraction;
    for (int iteration = 0; iteration < arrival_bisection_iterations; ++iteration) {
        const double middle = (outside + inside) / 2.0;
        const Coordinate point = detail::destination_point_from(
            parent.origin,
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
    const WeatherSampler& weather,
    const MidpointWeatherSamplers* midpoint_weather,
    const VesselPolar& polar,
    const RoutingEnvironment& environment,
    const RouteRequest& request,
    const std::vector<SearchNode>& nodes,
    const std::vector<NodeIndex>& frontier,
    std::size_t begin,
    std::size_t end,
    TimePoint current_time,
    std::chrono::seconds step,
    double step_hours,
    std::size_t heading_count,
    bool preserve_all_arrivals) {
    buffer.clear();
    const RoutingOptions& options = request.options;
    const bool augment_bearing =
        options.heading_augmentation == HeadingAugmentation::destination_bearing ||
        options.heading_augmentation ==
            HeadingAugmentation::destination_bearing_and_velocity_made_good;
    const bool augment_velocity_made_good =
        options.heading_augmentation == HeadingAugmentation::velocity_made_good ||
        options.heading_augmentation ==
            HeadingAugmentation::destination_bearing_and_velocity_made_good;
    const std::size_t augmented_heading_count =
        (augment_bearing ? 1U : 0U) +
        (augment_velocity_made_good ? 4U : 0U);
    const std::size_t headings_per_parent =
        heading_count + augmented_heading_count;
    const std::size_t parent_count = end - begin;
    if (parent_count <=
        std::numeric_limits<std::size_t>::max() / headings_per_parent) {
        const std::size_t maximum_candidates = parent_count * headings_per_parent;
        if (maximum_candidates > buffer.candidates.capacity()) {
            buffer.candidates.reserve(maximum_candidates);
        }
    }

    const bool penalise_maneuvers = options.maneuver.active();
    const bool environment_active = environment.active();
    // A degenerate environment sample, reused so the no-provider path keeps
    // reading zero current and no waves without any branch in the inner loop.
    const detail::EnvironmentSamples no_environment{};

    for (std::size_t frontier_index = begin; frontier_index < end;
         ++frontier_index) {
        const NodeIndex parent_index = frontier[frontier_index];
        const SearchNode& parent = nodes[parent_index];
        ++buffer.expanded_nodes;

        const auto wind_result = weather.sample(parent.point.position);
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

        // A wind speed the vessel is not expected to sail in removes the node
        // rather than silently reading the polar's top-end row.
        if (options.maximum_true_wind_speed_knots.has_value() &&
            wind_speed > *options.maximum_true_wind_speed_knots) {
            continue;
        }

        // The environment is a function of position and time, not heading, so
        // it is sampled once per parent alongside the wind.
        const detail::EnvironmentSamples* parent_samples = &no_environment;
        detail::EnvironmentSamples sampled_environment;
        if (environment_active) {
            detail::EnvironmentSampleResult sampled = detail::sample_environment(
                environment,
                parent.point.position,
                parent.point.time,
                buffer.environment);
            if (sampled.outcome == detail::EnvironmentOutcome::failed) {
                if (!buffer.environment_error.has_value()) {
                    buffer.environment_error = *sampled.error;
                }
                return;
            }
            if (sampled.outcome == detail::EnvironmentOutcome::rejected) {
                continue;
            }
            sampled_environment = sampled.samples;
            parent_samples = &sampled_environment;
        }
        // Hoisted so the per-heading loop below reads locals rather than
        // reloading through a pointer the compiler cannot prove is unaliased.
        const bool parent_has_current = parent_samples->has_current;
        const bool parent_has_wave = parent_samples->has_wave;
        const CurrentVector parent_current = parent_samples->current;
        const WaveState parent_wave = parent_samples->wave;
        // Declared outside the heading loop so the no-provider path never
        // touches them; they are only refreshed when an environment exists.
        bool has_current = false;
        bool has_wave = false;
        CurrentVector current{};
        WaveState wave{};

        const ParentGeometry geometry =
            prepare_parent(parent.point.position, request.destination);
        // The wind speed is fixed for this parent, so resolve the polar's wind
        // bracket once instead of per heading.
        const PolarSlice polar_slice =
            polar.slice_at(wind_speed, options.polar_angle_interpolation);
        if (options.above_polar_range == AbovePolarRangePolicy::no_speed &&
            polar_slice.above_tabulated_wind_speed()) {
            continue;
        }

        const std::int8_t parent_board = parent.parent == no_parent
            ? std::int8_t{0}
            : board_for(
                  parent.point.heading_degrees,
                  parent.point.true_wind_direction_degrees);
        const double parent_angle = parent.parent == no_parent
            ? 0.0
            : true_wind_angle(
                  parent.point.heading_degrees,
                  parent.point.true_wind_direction_degrees);

        buffer.headings.clear();
        for (std::size_t index = 0U; index < heading_count; ++index) {
            buffer.headings.push_back(
                static_cast<double>(index) * options.heading_step_degrees);
        }
        if (augment_bearing) {
            buffer.headings.push_back(
                normalize_heading(geometry.bearing_to_destination_degrees));
        }
        if (augment_velocity_made_good) {
            const VelocityMadeGoodAngles optima =
                polar_slice.velocity_made_good_angles();
            if (optima.valid) {
                buffer.headings.push_back(
                    normalize_heading(wind_direction + optima.upwind_degrees));
                buffer.headings.push_back(
                    normalize_heading(wind_direction - optima.upwind_degrees));
                buffer.headings.push_back(
                    normalize_heading(wind_direction + optima.downwind_degrees));
                buffer.headings.push_back(
                    normalize_heading(wind_direction - optima.downwind_degrees));
            }
        }

        for (std::size_t heading_index = 0U;
             heading_index < buffer.headings.size();
             ++heading_index) {
            const double heading = buffer.headings[heading_index];
            const double candidate_angle = true_wind_angle(heading, wind_direction);
            double performance_wind_speed = wind_speed;
            double performance_wind_angle = candidate_angle;
            double boat_speed = polar_slice.speed_knots(candidate_angle);
            if (!std::isfinite(boat_speed) || boat_speed <= 0.0 ||
                boat_speed < options.minimum_boat_speed_knots) {
                continue;
            }

            // Everything from here to the ground translation below stays in the
            // water frame, which is the frame the polar and the apparent wind
            // are defined in.
            double flat_water_speed = boat_speed;
            double relative_wave_angle = 0.0;
            if (environment_active) {
                has_current = parent_has_current;
                has_wave = parent_has_wave;
                current = parent_current;
                wave = parent_wave;
            }
            if (has_wave) {
                relative_wave_angle = detail::relative_wave_angle_degrees(
                    heading, wave.direction_from_degrees);
                Result<double> derated = detail::apply_sea_state(
                    environment,
                    flat_water_speed,
                    wind_speed,
                    candidate_angle,
                    heading,
                    wave,
                    buffer.environment);
                if (!derated) {
                    if (!buffer.environment_error.has_value()) {
                        buffer.environment_error = derated.error();
                    }
                    return;
                }
                boat_speed = derated.value();
                if (!std::isfinite(boat_speed) || boat_speed <= 0.0 ||
                    boat_speed < options.minimum_boat_speed_knots) {
                    continue;
                }
            }

            // A tack or gybe eats into the step before any distance is made.
            double penalty_seconds = 0.0;
            double usable_hours = step_hours;
            std::chrono::seconds maneuver_delay{};
            if (penalise_maneuvers) {
                maneuver_delay = maneuver_penalty(
                    options.maneuver,
                    parent_board,
                    parent_angle,
                    board_for(heading, wind_direction),
                    candidate_angle);
                if (maneuver_delay > std::chrono::seconds::zero()) {
                    if (maneuver_delay >= step) {
                        continue;
                    }
                    penalty_seconds =
                        static_cast<double>(maneuver_delay.count());
                    usable_hours = std::chrono::duration<double, std::ratio<3600>>(
                                       step - maneuver_delay)
                                       .count();
                }
            }

            // The vessel moves through the water; only here is that velocity
            // translated into the ground frame the search advances positions in.
            detail::GroundVelocity ground{heading, boat_speed};
            if (has_current) {
                ground = detail::ground_velocity(heading, boat_speed, current);
            }

            // Second-order integration can refine wind, environment, or both at
            // the provisional segment midpoint.
            const bool midpoint_environment = environment_active &&
                environment.sampling == EnvironmentSampling::midpoint;
            if (midpoint_weather != nullptr || midpoint_environment) {
                const Coordinate midpoint = detail::destination_point_from(
                    geometry.origin,
                    ground.course_degrees,
                    ground.speed_knots * usable_hours * 0.5);
                if (midpoint_environment) {
                    detail::EnvironmentSampleResult sampled =
                        detail::sample_environment(
                            environment,
                            midpoint,
                            current_time +
                                detail::sailing_midpoint_offset(
                                    step, maneuver_delay),
                            buffer.environment);
                    if (sampled.outcome == detail::EnvironmentOutcome::failed) {
                        if (!buffer.environment_error.has_value()) {
                            buffer.environment_error = *sampled.error;
                        }
                        return;
                    }
                    if (sampled.outcome ==
                        detail::EnvironmentOutcome::rejected) {
                        continue;
                    }
                    has_current = sampled.samples.has_current;
                    has_wave = sampled.samples.has_wave;
                    current = sampled.samples.current;
                    wave = sampled.samples.wave;
                }
                if (midpoint_weather != nullptr) {
                    const auto midpoint_wind =
                        midpoint_weather
                            ->for_delay(maneuver_delay, options.maneuver)
                            .sample(midpoint);
                    if (midpoint_wind) {
                        const double midpoint_speed =
                            midpoint_wind.value().speed_knots();
                        const double midpoint_direction =
                            midpoint_wind.value().direction_from_degrees();
                        if (std::isfinite(midpoint_speed) &&
                            std::isfinite(midpoint_direction) &&
                            (!options.maximum_true_wind_speed_knots.has_value() ||
                             midpoint_speed <=
                                 *options.maximum_true_wind_speed_knots)) {
                            const PolarSlice midpoint_slice = polar.slice_at(
                                midpoint_speed,
                                options.polar_angle_interpolation);
                            const double midpoint_angle =
                                true_wind_angle(heading, midpoint_direction);
                            const double refined = midpoint_slice.speed_knots(
                                midpoint_angle);
                            if (std::isfinite(refined)) {
                                flat_water_speed = refined;
                                boat_speed = refined;
                                performance_wind_speed = midpoint_speed;
                                performance_wind_angle = midpoint_angle;
                            }
                        }
                    }
                }
                if (has_wave) {
                    relative_wave_angle = detail::relative_wave_angle_degrees(
                        heading, wave.direction_from_degrees);
                    Result<double> derated = detail::apply_sea_state(
                        environment,
                        flat_water_speed,
                        performance_wind_speed,
                        performance_wind_angle,
                        heading,
                        wave,
                        buffer.environment);
                    if (!derated) {
                        if (!buffer.environment_error.has_value()) {
                            buffer.environment_error = derated.error();
                        }
                        return;
                    }
                    boat_speed = derated.value();
                }
                if (!std::isfinite(boat_speed) || boat_speed <= 0.0 ||
                    boat_speed < options.minimum_boat_speed_knots) {
                    continue;
                }
                ground = has_current
                    ? detail::ground_velocity(heading, boat_speed, current)
                    : detail::GroundVelocity{heading, boat_speed};
            }

            if (!(ground.speed_knots > 0.0)) {
                continue;
            }
            const double segment_distance = ground.speed_knots * usable_hours;
            const Coordinate position = detail::destination_point_from(
                geometry.origin,
                ground.course_degrees,
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
                        segment_distance,
                    std::nullopt},
                parent_index,
                detail::great_circle_distance_nautical_miles(
                    position,
                    request.destination),
                ExpansionOrdinal{frontier_index, heading_index}};
            ++buffer.generated_candidates;

            const std::optional<double> fraction = arrival_fraction(
                geometry,
                ground.course_degrees,
                segment_distance,
                request.destination,
                options.arrival_radius_nautical_miles);
            if (fraction.has_value()) {
                const double travelled = segment_distance * *fraction;
                const auto elapsed_seconds = std::chrono::seconds{
                    static_cast<std::chrono::seconds::rep>(
                        std::llround(
                            penalty_seconds +
                            static_cast<double>(step.count() -
                                                static_cast<std::chrono::seconds::rep>(
                                                    penalty_seconds)) *
                                *fraction))};
                candidate.point.position = detail::destination_point_from(
                    geometry.origin,
                    ground.course_degrees,
                    travelled);
                candidate.point.time = current_time + elapsed_seconds;
                candidate.point.cumulative_distance_nautical_miles =
                    parent.point.cumulative_distance_nautical_miles + travelled;
                candidate.distance_to_destination =
                    detail::great_circle_distance_nautical_miles(
                        candidate.point.position,
                        request.destination);
            }

            if (has_current || has_wave) {
                RoutePointEnvironment audit;
                audit.speed_over_ground_knots = ground.speed_knots;
                audit.course_over_ground_degrees = ground.course_degrees;
                audit.current_east_knots = current.east_knots;
                audit.current_north_knots = current.north_knots;
                audit.flat_water_speed_knots = flat_water_speed;
                audit.significant_wave_height_metres =
                    wave.significant_height_metres;
                audit.wave_period_seconds = wave.peak_period_seconds;
                audit.relative_wave_angle_degrees = relative_wave_angle;
                audit.current_applied = has_current;
                audit.wave_applied = has_wave;
                candidate.point.environment = audit;
            }

            if (environment_active) {
                // Land and exclusion rejection happens before retention, so a
                // rejected transition never reaches progress output either.
                const detail::SegmentCheckResult check =
                    detail::check_segment_geometry(
                        environment,
                        parent.point.position,
                        parent.point.time,
                        candidate.point.position,
                        candidate.point.time,
                        buffer.environment);
                if (check.outcome == detail::EnvironmentOutcome::failed) {
                    if (!buffer.environment_error.has_value()) {
                        buffer.environment_error = *check.error;
                    }
                    return;
                }
                if (check.outcome == detail::EnvironmentOutcome::rejected) {
                    continue;
                }
            }

            if (fraction.has_value()) {
                Arrival arrival{std::move(candidate), *fraction};
                if (preserve_all_arrivals) {
                    buffer.arrivals.push_back(std::move(arrival));
                } else if (!buffer.best_arrival.has_value() ||
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
        const VesselPolar& polar,
        const RoutingEnvironment& environment,
        const RouteRequest& request,
        bool preserve_all_arrivals)
        : polar_(polar),
          environment_(environment),
          request_(request),
          preserve_all_arrivals_(preserve_all_arrivals) {}

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
        const WeatherSampler& sampler,
        const MidpointWeatherSamplers* midpoint_sampler,
        const std::vector<SearchNode>& nodes,
        const std::vector<NodeIndex>& frontier,
        TimePoint current_time,
        std::chrono::seconds step,
        double step_hours,
        std::size_t heading_count) {
        ensure_worker_count(active_workers);
        {
            std::lock_guard lock(mutex_);
            sampler_ = sampler;
            midpoint_sampler_ = midpoint_sampler;
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
                sampler_,
                midpoint_sampler_,
                polar_,
                environment_,
                request_,
                *nodes_,
                *frontier_,
                begin,
                end,
                current_time_,
                step_,
                step_hours_,
                heading_count_,
                preserve_all_arrivals_);
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

    WeatherSampler sampler_;
    const MidpointWeatherSamplers* midpoint_sampler_{nullptr};
    const VesselPolar& polar_;
    const RoutingEnvironment& environment_;
    const RouteRequest& request_;
    bool preserve_all_arrivals_{};
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
            std::floor(north_nautical_miles / bucket_size_nautical_miles)),
        0};
}

// Buckets by range from the destination and bearing seen from it. Sector width
// grows with range, so far from the destination this keeps candidates that a
// fixed grid would merge, at the cost of over-merging as the fan converges.
BucketKey sector_bucket_for(
    Coordinate coordinate,
    Coordinate destination,
    double bucket_size_nautical_miles,
    double sector_degrees) noexcept {
    const double distance =
        detail::great_circle_distance_nautical_miles(coordinate, destination);
    const double bearing =
        normalize_heading(detail::initial_bearing_degrees(destination, coordinate));
    return BucketKey{
        static_cast<std::int64_t>(std::floor(bearing / sector_degrees)),
        static_cast<std::int64_t>(
            std::floor(distance / bucket_size_nautical_miles)),
        0};
}

BucketKey pruning_key_for(const Candidate& candidate, Coordinate destination,
                          const RoutingOptions& options) noexcept {
    BucketKey key =
        options.pruning_strategy == PruningStrategy::bearing_sectors
        ? sector_bucket_for(
              candidate.point.position,
              destination,
              options.spatial_bucket_nautical_miles,
              options.pruning_sector_degrees)
        : bucket_for(
              candidate.point.position,
              destination,
              options.spatial_bucket_nautical_miles);
    if (options.maneuver.active()) {
        key.board = board_for(
            candidate.point.heading_degrees,
            candidate.point.true_wind_direction_degrees);
    }
    return key;
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

// Buffers reused across every routing step so pruning does not reallocate.
struct PruneScratch {
    std::vector<BucketKey> keys;
    std::vector<std::size_t> order;
    std::vector<double> min_separation;
    std::vector<char> selected;
    std::vector<std::size_t> retained;
};

// Retains a heading-diverse subset of each spatial bucket.
//
// Candidates are grouped by bucket and ordered by dominance in a single sort
// rather than through a node-allocating std::map. Within a bucket the most
// dominant candidate is taken first, then repeatedly the candidate whose
// heading is furthest from everything already selected. That farthest-point
// choice previously rescanned the selected list for every candidate on every
// pick, which is quadratic in the per-bucket cap; caching each candidate's
// distance to the nearest selected heading makes it linear.
//
// Selection order within a bucket, and therefore the retained set, is identical
// to the previous implementation. Buckets are independent and the result is
// sorted by expansion ordinal, so grouping order does not affect the output.
void prune_candidates_into(
    const std::vector<Candidate>& candidates,
    Coordinate destination,
    const RoutingOptions& options,
    PruneScratch& scratch) {
    const std::size_t count = candidates.size();
    scratch.retained.clear();
    if (count == 0U) {
        return;
    }

    scratch.keys.resize(count);
    scratch.order.resize(count);
    scratch.min_separation.assign(count, 0.0);
    scratch.selected.assign(count, 0);
    for (std::size_t index = 0U; index < count; ++index) {
        scratch.keys[index] = pruning_key_for(candidates[index], destination, options);
        scratch.order[index] = index;
    }

    std::sort(
        scratch.order.begin(),
        scratch.order.end(),
        [&candidates, &scratch](std::size_t left, std::size_t right) {
            if (scratch.keys[left] < scratch.keys[right]) {
                return true;
            }
            if (scratch.keys[right] < scratch.keys[left]) {
                return false;
            }
            return dominates(candidates[left], candidates[right]);
        });

    scratch.retained.reserve(count);
    const auto heading_of = [&](std::size_t position) {
        return candidates[scratch.order[position]].point.heading_degrees;
    };

    std::size_t run_begin = 0U;
    while (run_begin < count) {
        std::size_t run_end = run_begin + 1U;
        while (run_end < count &&
               !(scratch.keys[scratch.order[run_begin]] <
                 scratch.keys[scratch.order[run_end]]) &&
               !(scratch.keys[scratch.order[run_end]] <
                 scratch.keys[scratch.order[run_begin]])) {
            ++run_end;
        }

        const std::size_t run_size = run_end - run_begin;
        const std::size_t limit = std::min(options.max_nodes_per_bucket, run_size);

        scratch.selected[run_begin] = 1;
        scratch.retained.push_back(scratch.order[run_begin]);
        for (std::size_t position = run_begin + 1U; position < run_end; ++position) {
            scratch.selected[position] = 0;
            scratch.min_separation[position] = std::min(
                180.0,
                detail::angular_difference_degrees(
                    heading_of(position),
                    heading_of(run_begin)));
        }

        for (std::size_t taken = 1U; taken < limit; ++taken) {
            std::size_t best_position = count;
            double best_separation = -1.0;
            // The run is in dominance order and the comparison is strict, so
            // equal separations keep the more dominant candidate, matching the
            // previous explicit tie-break.
            for (std::size_t position = run_begin; position < run_end; ++position) {
                if (scratch.selected[position] != 0) {
                    continue;
                }
                if (scratch.min_separation[position] > best_separation) {
                    best_position = position;
                    best_separation = scratch.min_separation[position];
                }
            }
            if (best_position == count) {
                break;
            }

            scratch.selected[best_position] = 1;
            scratch.retained.push_back(scratch.order[best_position]);
            const double chosen_heading = heading_of(best_position);
            for (std::size_t position = run_begin; position < run_end; ++position) {
                if (scratch.selected[position] != 0) {
                    continue;
                }
                scratch.min_separation[position] = std::min(
                    scratch.min_separation[position],
                    detail::angular_difference_degrees(
                        heading_of(position),
                        chosen_heading));
            }
        }

        run_begin = run_end;
    }

    std::sort(
        scratch.retained.begin(),
        scratch.retained.end(),
        [&candidates](std::size_t left, std::size_t right) {
            return candidates[left].ordinal < candidates[right].ordinal;
        });
}

void reconstruct_route_into(
    const std::vector<SearchNode>& nodes,
    NodeIndex arrival_index,
    std::vector<RoutePoint>& route) {
    route.clear();
    for (NodeIndex index = arrival_index; index != no_parent; index = nodes[index].parent) {
        route.push_back(nodes[index].point);
    }
    std::reverse(route.begin(), route.end());
}

std::vector<RoutePoint> reconstruct_route(
    const std::vector<SearchNode>& nodes,
    NodeIndex arrival_index) {
    std::vector<RoutePoint> route;
    reconstruct_route_into(nodes, arrival_index, route);
    return route;
}

RouteResult make_route_result(
    TimePoint departure,
    DepartureSource departure_source,
    RouteCompletion completion,
    const std::string& forecast_source,
    const std::string& polar_source,
    std::vector<RoutePoint> points,
    std::vector<Isochrone> isochrones,
    const RouteDiagnostics& diagnostics,
    const RoutingEnvironment& environment,
    const EnvironmentDiagnostics& environment_diagnostics) {
    RouteResult result;
    result.departure_time = departure;
    result.arrival_time = points.back().time;
    result.departure_source = departure_source;
    result.completion = completion;
    result.forecast_source = forecast_source;
    result.polar_source = polar_source;
    result.points = std::move(points);
    result.isochrones = std::move(isochrones);
    result.diagnostics = diagnostics;
    if (environment.active()) {
        result.environment_diagnostics = environment_diagnostics;
        result.environment = describe_environment(environment);
    }
    return result;
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

struct ProgressScratch {
    std::vector<Coordinate> retained_points;
    std::vector<RoutePoint> provisional_route;
    std::vector<Coordinate> contour_points;
    std::vector<DisplayContourSegment> contour_segments;
    std::vector<Coordinate> front_source_points;
    std::vector<Coordinate> front_points;
    std::vector<IsochroneFrontSegment> front_segments;
};

void capture_retained_points(
    const std::vector<SearchNode>& nodes,
    const std::vector<NodeIndex>& frontier,
    std::vector<Coordinate>& points) {
    points.clear();
    points.reserve(frontier.size());
    for (const NodeIndex index : frontier) {
        points.push_back(nodes[index].point.position);
    }
}

Error route_duration_exhausted_error(const RouteDiagnostics& diagnostics) {
    const std::string suffix =
        " after " + std::to_string(diagnostics.time_steps) + " time steps and " +
        std::to_string(diagnostics.expanded_nodes) + " expanded nodes";
    return Error{
        ErrorCode::no_route,
        "maximum route duration ended before the destination was reached" + suffix};
}

Error cancelled_error(const RouteDiagnostics& diagnostics) {
    return Error{
        ErrorCode::cancelled,
        "routing cancelled after " + std::to_string(diagnostics.time_steps) +
            (diagnostics.time_steps == 1U ? " time step and " : " time steps and ") +
            std::to_string(diagnostics.expanded_nodes) +
            (diagnostics.expanded_nodes == 1U
                 ? " expanded node"
                 : " expanded nodes")};
}

}  // namespace

Router::Router(WeatherDataset weather, VesselPolar polar)
    : weather_(std::move(weather)), polar_(std::move(polar)) {}

Router::Router(
    WeatherDataset weather,
    VesselPolar polar,
    RoutingEnvironment environment)
    : weather_(std::move(weather)),
      polar_(std::move(polar)),
      environment_(std::move(environment)) {}

const RoutingEnvironment& Router::environment() const noexcept {
    return environment_;
}

Result<RouteResult> Router::optimize(const RouteRequest& request) const {
    return optimize_view_controlled(request, RoutingViewControlCallback{});
}

Result<RouteResult> Router::optimize(
    const RouteRequest& request,
    const RoutingProgressCallback& on_progress) const {
    if (!on_progress) {
        return optimize_view_controlled(request, RoutingViewControlCallback{});
    }
    return optimize_controlled(request, [&on_progress](const RoutingProgress& progress) {
        on_progress(progress);
        return RoutingProgressDecision::continue_routing;
    });
}

Result<RouteResult> Router::optimize_controlled(
    const RouteRequest& request,
    const RoutingControlCallback& on_progress) const {
    if (!on_progress) {
        return optimize_view_controlled(request, RoutingViewControlCallback{});
    }
    RouteRequest owning_request = request;
    owning_request.options.progress.payload =
        RoutingProgressPayload::retained_points |
        RoutingProgressPayload::provisional_route;
    return optimize_view_controlled(
        owning_request,
        [&on_progress](const RoutingProgressView& view) {
            RoutingProgress progress{
                Isochrone{
                    view.time,
                    std::vector<Coordinate>{
                        view.retained_points.begin(),
                        view.retained_points.end()}},
                std::vector<RoutePoint>{
                    view.provisional_route.begin(),
                    view.provisional_route.end()},
                IsochroneFront{
                    std::vector<Coordinate>{
                        view.destination_front.points.begin(),
                        view.destination_front.points.end()},
                    std::vector<IsochroneFrontSegment>{
                        view.destination_front.segments.begin(),
                        view.destination_front.segments.end()}},
                view.diagnostics,
                view.solver,
                std::vector<Coordinate>{
                    view.search_points.begin(),
                    view.search_points.end()},
                view.search};
            return on_progress(progress);
        });
}

Result<RouteResult> Router::optimize_view(
    const RouteRequest& request,
    const RoutingProgressViewCallback& on_progress) const {
    if (!on_progress) {
        return optimize_view_controlled(request, RoutingViewControlCallback{});
    }
    return optimize_view_controlled(
        request,
        [&on_progress](const RoutingProgressView& progress) {
            on_progress(progress);
            return RoutingProgressDecision::continue_routing;
        });
}

Result<RouteResult> Router::optimize_view_controlled(
    const RouteRequest& request,
    const RoutingViewControlCallback& on_progress) const {
    if (const std::optional<Error> validation = validate_request(request);
        validation.has_value()) {
        return *validation;
    }
    if (const std::optional<Error> validation =
            validate_environment(environment_);
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
    if (request.options.solver == RoutingSolver::time_dependent_lattice) {
        return detail::optimize_lattice_route(
            weather_,
            polar_,
            environment_,
            request,
            departure,
            departure_source,
            on_progress);
    }

    RouteDiagnostics diagnostics;
    EnvironmentDiagnostics environment_diagnostics;
    std::vector<SearchNode> nodes;
    nodes.reserve(1024);
    PruneScratch prune_scratch;
    nodes.push_back(SearchNode{
        RoutePoint{
            request.start,
            departure,
            0.0,
            0.0,
            start_wind.value().speed_knots(),
            start_wind.value().direction_from_degrees(),
            0.0,
            std::nullopt},
        no_parent});

    if (detail::great_circle_distance_nautical_miles(
            request.start,
            request.destination) <= request.options.arrival_radius_nautical_miles) {
        return make_route_result(
            departure,
            departure_source,
            RouteCompletion::destination_reached,
            metadata.source,
            polar_.source(),
            std::vector<RoutePoint>{nodes.front().point},
            {},
            diagnostics,
            environment_,
            environment_diagnostics);
    }

    std::vector<NodeIndex> frontier{0U};
    std::vector<Candidate> candidates;
    std::vector<CandidateEvaluation> candidate_evaluations;
    std::vector<NodeIndex> next_frontier;
    std::vector<Isochrone> isochrones;

    const TimePoint horizon_end = departure + request.options.maximum_route_duration;
    const TimePoint route_end = std::min(horizon_end, metadata.last_valid_time);
    const bool forecast_limited = metadata.last_valid_time < horizon_end;
    const std::size_t heading_count = static_cast<std::size_t>(
        std::ceil(360.0 / request.options.heading_step_degrees));
    ExpansionBuffer single_worker_buffer;
    const bool has_segment_eligibility =
        static_cast<bool>(request.options.segment_eligibility);
    CandidateExpansionWorkers expansion_workers{
        polar_,
        environment_,
        request,
        has_segment_eligibility};
    ProgressScratch progress_scratch;
    NodeIndex best_route_end = 0U;
    auto finish_without_arrival = [&]() -> Result<RouteResult> {
        if (!forecast_limited) {
            return route_duration_exhausted_error(diagnostics);
        }
        return make_route_result(
            departure,
            departure_source,
            RouteCompletion::forecast_exhausted,
            metadata.source,
            polar_.source(),
            reconstruct_route(nodes, best_route_end),
            std::move(isochrones),
            diagnostics,
            environment_,
            environment_diagnostics);
    };

    while (!frontier.empty()) {
        const TimePoint current_time = nodes[frontier.front()].point.time;
        if (current_time >= route_end) {
            return finish_without_arrival();
        }
        const auto remaining =
            std::chrono::duration_cast<std::chrono::seconds>(route_end - current_time);
        const auto elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(
                current_time - departure);
        const auto step =
            detail::routing_step(request.options, elapsed, remaining);
        if (step <= std::chrono::seconds::zero()) {
            return finish_without_arrival();
        }

        ++diagnostics.time_steps;
        candidates.clear();
        candidate_evaluations.clear();
        std::optional<Arrival> best_arrival;
        std::optional<Error> interpolation_error;
        std::optional<Error> environment_error;
        std::size_t generated_this_step = 0U;
        std::size_t eligible_this_step = 0U;

        const double step_hours =
            std::chrono::duration<double, std::ratio<3600>>(step).count();
        // The forecast time is fixed for the whole step, so locate the
        // surrounding forecast steps once rather than per node.
        auto sampler_result = weather_.sampler_at(current_time);
        if (!sampler_result) {
            return sampler_result.error();
        }
        const WeatherSampler& sampler = sampler_result.value();

        // A maneuver consumes time before sailing starts, so its sailed midpoint
        // occurs later than the unpenalized midpoint. Resolve the three possible
        // time brackets once per step rather than once per candidate.
        std::optional<MidpointWeatherSamplers> midpoint_sampler_storage;
        const MidpointWeatherSamplers* midpoint_sampler = nullptr;
        if (request.options.wind_sampling == WindSampling::midpoint &&
            step >= request.options.midpoint_wind_sampling_threshold) {
            const auto sampler_for_delay =
                [&](std::chrono::seconds delay) -> Result<WeatherSampler> {
                return weather_.sampler_at(
                    current_time +
                    detail::sailing_midpoint_offset(step, delay));
            };
            auto unpenalized = sampler_for_delay(std::chrono::seconds::zero());
            if (!unpenalized) {
                return unpenalized.error();
            }
            midpoint_sampler_storage = MidpointWeatherSamplers{
                unpenalized.value(),
                unpenalized.value(),
                unpenalized.value()};
            const auto resolve_penalized =
                [&](std::chrono::seconds delay,
                    WeatherSampler& destination) -> std::optional<Error> {
                if (delay <= std::chrono::seconds::zero() || delay >= step) {
                    return std::nullopt;
                }
                auto result = sampler_for_delay(delay);
                if (!result) {
                    return result.error();
                }
                destination = std::move(result.value());
                return std::nullopt;
            };
            if (auto error = resolve_penalized(
                    request.options.maneuver.tack_penalty,
                    midpoint_sampler_storage->tack);
                error.has_value()) {
                return *error;
            }
            if (request.options.maneuver.gybe_penalty ==
                request.options.maneuver.tack_penalty) {
                midpoint_sampler_storage->gybe =
                    midpoint_sampler_storage->tack;
            } else {
                if (auto error = resolve_penalized(
                        request.options.maneuver.gybe_penalty,
                        midpoint_sampler_storage->gybe);
                    error.has_value()) {
                    return *error;
                }
            }
            midpoint_sampler = &midpoint_sampler_storage.value();
        }

        const std::size_t active_workers =
            expansion_workers.worker_count(frontier.size(), heading_count);
        const auto merge_buffer = [&](const ExpansionBuffer& buffer) {
            diagnostics.expanded_nodes += buffer.expanded_nodes;
            diagnostics.generated_candidates += buffer.generated_candidates;
            generated_this_step += buffer.generated_candidates;
            detail::merge(environment_diagnostics, buffer.environment);
            // Buffers merge in worker-index order, so the surfaced error is the
            // same whichever worker happened to finish first.
            if (!environment_error.has_value() &&
                buffer.environment_error.has_value()) {
                environment_error = buffer.environment_error;
            }
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
        const auto append_for_eligibility = [&candidate_evaluations](
                                                ExpansionBuffer& buffer) {
            candidate_evaluations.reserve(
                candidate_evaluations.size() +
                buffer.candidates.size() +
                buffer.arrivals.size());
            for (Candidate& candidate : buffer.candidates) {
                candidate_evaluations.push_back(
                    CandidateEvaluation{std::move(candidate), std::nullopt});
            }
            for (Arrival& arrival : buffer.arrivals) {
                candidate_evaluations.push_back(CandidateEvaluation{
                    std::move(arrival.candidate),
                    arrival.fraction});
            }
        };

        if (active_workers == 1U) {
            expand_candidate_range(
                single_worker_buffer,
                sampler,
                midpoint_sampler,
                polar_,
                environment_,
                request,
                nodes,
                frontier,
                0U,
                frontier.size(),
                current_time,
                step,
                step_hours,
                heading_count,
                has_segment_eligibility);
            merge_buffer(single_worker_buffer);
            if (environment_error.has_value()) {
                return *environment_error;
            }
            if (single_worker_buffer.non_finite_wind) {
                return Error{
                    ErrorCode::incomplete_forecast,
                    "forecast interpolation produced non-finite wind"};
            }
            if (has_segment_eligibility) {
                append_for_eligibility(single_worker_buffer);
            } else if (!best_arrival.has_value()) {
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
                sampler,
                midpoint_sampler,
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
            if (environment_error.has_value()) {
                return *environment_error;
            }
            if (non_finite_wind) {
                return Error{
                    ErrorCode::incomplete_forecast,
                    "forecast interpolation produced non-finite wind"};
            }
            if (has_segment_eligibility) {
                for (std::size_t index = 0U; index < active_workers; ++index) {
                    append_for_eligibility(expansion_workers.buffer(index));
                }
            } else if (!best_arrival.has_value()) {
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

        if (has_segment_eligibility) {
            std::sort(
                candidate_evaluations.begin(),
                candidate_evaluations.end(),
                [](const CandidateEvaluation& left,
                   const CandidateEvaluation& right) {
                    return left.candidate.ordinal < right.candidate.ordinal;
                });
            for (CandidateEvaluation& evaluation : candidate_evaluations) {
                const RouteSegmentView segment{
                    nodes[evaluation.candidate.parent].point,
                    evaluation.candidate.point};
                if (!request.options.segment_eligibility(segment)) {
                    continue;
                }
                ++eligible_this_step;
                if (evaluation.arrival_fraction.has_value()) {
                    Arrival arrival{
                        std::move(evaluation.candidate),
                        *evaluation.arrival_fraction};
                    if (!best_arrival.has_value() ||
                        better_arrival(arrival, *best_arrival)) {
                        best_arrival = std::move(arrival);
                    }
                } else {
                    candidates.push_back(std::move(evaluation.candidate));
                }
            }
        }

        if (best_arrival.has_value()) {
            nodes.push_back(SearchNode{
                std::move(best_arrival->candidate.point),
                best_arrival->candidate.parent});
            ++diagnostics.retained_candidates;
            return make_route_result(
                departure,
                departure_source,
                RouteCompletion::destination_reached,
                metadata.source,
                polar_.source(),
                reconstruct_route(nodes, nodes.size() - 1U),
                std::move(isochrones),
                diagnostics,
                environment_,
                environment_diagnostics);
        }

        if (candidates.empty()) {
            if (has_segment_eligibility &&
                generated_this_step > 0U &&
                eligible_this_step == 0U) {
                return Error{
                    ErrorCode::no_route,
                    "segment eligibility rejected every candidate at routing step " +
                        std::to_string(diagnostics.time_steps)};
            }
            if (interpolation_error.has_value()) {
                return *interpolation_error;
            }
            return Error{
                ErrorCode::no_route,
                "no heading met the minimum boat speed at routing step " +
                    std::to_string(diagnostics.time_steps)};
        }

        prune_candidates_into(
            candidates, request.destination, request.options, prune_scratch);
        const std::vector<std::size_t>& retained = prune_scratch.retained;
        const bool deliver_progress =
            on_progress &&
            diagnostics.time_steps %
                    request.options.progress.every_n_steps ==
                0U;
        const RoutingProgressPayload progress_payload =
            request.options.progress.payload;
        const bool build_pre_prune_front =
            deliver_progress &&
            has_payload(
                progress_payload,
                RoutingProgressPayload::destination_front) &&
            request.options.progress.destination_front.mode ==
                DestinationFrontMode::eligible_pre_prune;
        std::size_t provisional_candidate_index =
            std::numeric_limits<std::size_t>::max();
        if (build_pre_prune_front) {
            double provisional_candidate_distance =
                std::numeric_limits<double>::infinity();
            for (const std::size_t candidate_index : retained) {
                if (candidates[candidate_index].distance_to_destination <
                    provisional_candidate_distance) {
                    provisional_candidate_distance =
                        candidates[candidate_index].distance_to_destination;
                    provisional_candidate_index = candidate_index;
                }
            }
            progress_scratch.front_source_points.clear();
            progress_scratch.front_source_points.reserve(candidates.size());
            for (const Candidate& candidate : candidates) {
                progress_scratch.front_source_points.push_back(
                    candidate.point.position);
            }
            if (const auto error = detail::build_destination_front_into(
                    progress_scratch.front_source_points,
                    request.destination,
                    candidates[provisional_candidate_index].point.position,
                    request.options.spatial_bucket_nautical_miles,
                    request.options.progress.destination_front,
                    progress_scratch.front_points,
                    progress_scratch.front_segments);
                error.has_value()) {
                return *error;
            }
        }
        next_frontier.clear();
        next_frontier.reserve(retained.size());
        // Reserving exactly nodes.size() + retained.size() would size the buffer
        // to the current requirement on every step, so the search reallocated
        // and copied the whole node array each time and grew in quadratic time.
        // Grow geometrically instead.
        if (const std::size_t required = nodes.size() + retained.size();
            required > nodes.capacity()) {
            nodes.reserve(std::max(required, nodes.capacity() * 2U));
        }
        NodeIndex provisional_route_end = no_parent;
        double provisional_distance = std::numeric_limits<double>::infinity();
        for (const std::size_t candidate_index : retained) {
            Candidate& candidate = candidates[candidate_index];
            nodes.push_back(SearchNode{std::move(candidate.point), candidate.parent});
            next_frontier.push_back(nodes.size() - 1U);
            if (candidate.distance_to_destination < provisional_distance) {
                provisional_distance = candidate.distance_to_destination;
                provisional_route_end = nodes.size() - 1U;
            }
        }
        diagnostics.retained_candidates += retained.size();
        frontier.swap(next_frontier);
        best_route_end = provisional_route_end;
        if (request.options.capture_isochrones) {
            isochrones.push_back(capture_isochrone(nodes, frontier));
        }
        if (deliver_progress) {
            const RoutingProgressPayload payload = progress_payload;
            const bool needs_retained_points =
                has_payload(
                    payload,
                    RoutingProgressPayload::retained_points) ||
                has_payload(
                    payload,
                    RoutingProgressPayload::display_contours) ||
                has_payload(
                    payload,
                    RoutingProgressPayload::destination_front);
            if (needs_retained_points) {
                capture_retained_points(
                    nodes,
                    frontier,
                    progress_scratch.retained_points);
            } else {
                progress_scratch.retained_points.clear();
            }
            if (has_payload(
                    payload,
                    RoutingProgressPayload::provisional_route)) {
                reconstruct_route_into(
                    nodes,
                    provisional_route_end,
                    progress_scratch.provisional_route);
            } else {
                progress_scratch.provisional_route.clear();
            }
            if (has_payload(
                    payload,
                    RoutingProgressPayload::display_contours)) {
                if (const auto error = detail::build_display_contours_into(
                        progress_scratch.retained_points,
                        request.options.progress.display_contours,
                        progress_scratch.contour_points,
                        progress_scratch.contour_segments);
                    error.has_value()) {
                    return *error;
                }
            } else {
                progress_scratch.contour_points.clear();
                progress_scratch.contour_segments.clear();
            }
            if (has_payload(
                    payload,
                    RoutingProgressPayload::destination_front)) {
                if (request.options.progress.destination_front.mode ==
                    DestinationFrontMode::retained_frontier) {
                    if (const auto error = detail::build_destination_front_into(
                            progress_scratch.retained_points,
                            request.destination,
                            request.options.spatial_bucket_nautical_miles,
                            request.options.progress.destination_front,
                            progress_scratch.front_points,
                            progress_scratch.front_segments);
                        error.has_value()) {
                        return *error;
                    }
                }
            } else {
                progress_scratch.front_points.clear();
                progress_scratch.front_segments.clear();
            }

            const RoutingProgressView progress{
                nodes[frontier.front()].point.time,
                has_payload(
                    payload,
                    RoutingProgressPayload::retained_points)
                    ? std::span<const Coordinate>{
                          progress_scratch.retained_points}
                    : std::span<const Coordinate>{},
                progress_scratch.provisional_route,
                DisplayContourView{
                    progress_scratch.contour_points,
                    progress_scratch.contour_segments},
                IsochroneFrontView{
                    progress_scratch.front_points,
                    progress_scratch.front_segments},
                diagnostics,
                RoutingSolver::isochrone_beam,
                {},
                {}};
            const RoutingProgressDecision decision = on_progress(progress);
            if (decision == RoutingProgressDecision::cancel) {
                return cancelled_error(diagnostics);
            }
        }
    }

    return Error{ErrorCode::no_route, "isochrone pruning removed every candidate"};
}

}  // namespace sailroute
