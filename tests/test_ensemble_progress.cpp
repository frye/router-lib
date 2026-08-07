#include "sailroute/ensemble.hpp"

#include "ensemble/lattice_solver.hpp"
#include "grib_fixture.hpp"
#include "test_support.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Test fixtures and helpers shared across this file
// ---------------------------------------------------------------------------

namespace {

using namespace std::chrono_literals;
using sailroute::EnsembleDataset;
using sailroute::EnsembleMemberInput;
using sailroute::EnsembleObjectiveValue;
using sailroute::EnsembleObjectiveValueClass;
using sailroute::EnsembleProgress;
using sailroute::EnsembleProgressView;
using sailroute::EnsembleRouter;
using sailroute::EnsembleRouteRequest;
using sailroute::EnsembleRunMetadata;
using sailroute::EnsembleSolver;
using sailroute::EnsembleSolverPhase;
using sailroute::ErrorCode;
using sailroute::RoutingProgressDecision;
using sailroute::TimePoint;
using sailroute::test::ConstantWindGribFixture;

constexpr std::chrono::seconds kEpoch{1'784'035'200};

EnsembleRunMetadata progress_metadata() {
    return EnsembleRunMetadata{
        "progress-cycle",
        "progress-model",
        TimePoint{kEpoch},
        "ensemble progress test",
        1U};
}

EnsembleMemberInput progress_input(
    std::string id,
    const ConstantWindGribFixture& fixture,
    double weight = 1.0) {
    EnsembleMemberInput m;
    m.identifier = std::move(id);
    m.weight = weight;
    m.grib_path = fixture.path();
    return m;
}

/// Compact ensemble request suitable for quick progress tests.
EnsembleRouteRequest progress_request(
    sailroute::Coordinate start = {1.0, 0.5},
    sailroute::Coordinate dest  = {1.0, 0.6}) {
    EnsembleRouteRequest req;
    req.start = start;
    req.destination = dest;
    req.departure_time = TimePoint{kEpoch};
    req.options.maximum_route_duration = 2h;
    req.lattice.subdivision_level = 0U;
    req.lattice.time_bucket = 30min;
    req.lattice.max_labels_per_state = 512U;
    req.lattice.max_total_labels = 5'000U;
    return req;
}

/// Load a two-member dataset from one fixture shared by both members.
sailroute::Result<EnsembleDataset> two_member_dataset(
    const ConstantWindGribFixture& fixture) {
    return EnsembleDataset::load(
        progress_metadata(),
        {progress_input("alpha", fixture), progress_input("beta", fixture)});
}

}  // namespace

// ---------------------------------------------------------------------------
// to_string(EnsembleSolverPhase)
// ---------------------------------------------------------------------------

TEST_CASE("to_string EnsembleSolverPhase returns stable snake_case names") {
    REQUIRE(
        sailroute::to_string(EnsembleSolverPhase::initializing) ==
        "initializing");
    REQUIRE(
        sailroute::to_string(EnsembleSolverPhase::searching) == "searching");
    REQUIRE(
        sailroute::to_string(EnsembleSolverPhase::finalizing) == "finalizing");
}

// ---------------------------------------------------------------------------
// No behavioral change without callbacks
// ---------------------------------------------------------------------------

TEST_CASE("ensemble optimize without callback produces same result as with null callback") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    const auto without_cb = router.optimize(progress_request());
    const auto with_null  = router.optimize(progress_request(),
        sailroute::EnsembleProgressCallback{});

    REQUIRE(without_cb.has_value());
    REQUIRE(with_null.has_value());
    REQUIRE(
        without_cb.value().members.size() ==
        with_null.value().members.size());
    REQUIRE(
        without_cb.value().members.at(0).outcome.outcome_class ==
        with_null.value().members.at(0).outcome.outcome_class);
    REQUIRE(
        without_cb.value().members.at(0).outcome.elapsed_arrival_seconds ==
        with_null.value().members.at(0).outcome.elapsed_arrival_seconds);
}

// ---------------------------------------------------------------------------
// Owning progress callback: phase sequence
// ---------------------------------------------------------------------------

TEST_CASE("ensemble owning callback receives initializing first and finalizing last") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    std::vector<EnsembleSolverPhase> phases;
    const auto result = router.optimize(
        progress_request(),
        [&phases](const EnsembleProgress& p) {
            phases.push_back(p.phase);
        });

    REQUIRE(result.has_value());
    // At minimum: initializing and finalizing must always fire.
    REQUIRE(phases.size() >= 2U);
    REQUIRE(phases.front() == EnsembleSolverPhase::initializing);
    REQUIRE(phases.back()  == EnsembleSolverPhase::finalizing);
    // Any searching phases must not appear before initializing or after finalizing.
    for (std::size_t i = 1U; i + 1U < phases.size(); ++i) {
        REQUIRE(phases[i] == EnsembleSolverPhase::searching);
    }
}

// ---------------------------------------------------------------------------
// View callback: same phase sequence as owning
// ---------------------------------------------------------------------------

TEST_CASE("ensemble view callback delivers same phase sequence as owning callback") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    auto ds2 = two_member_dataset(fixture);
    REQUIRE(ds2.has_value());

    EnsembleRouter owning_router{std::move(ds.value())};
    EnsembleRouter view_router{std::move(ds2.value())};

    std::vector<EnsembleSolverPhase> owning_phases;
    std::vector<EnsembleSolverPhase> view_phases;

    static_cast<void>(owning_router.optimize(
        progress_request(),
        [&owning_phases](const EnsembleProgress& p) {
            owning_phases.push_back(p.phase);
        }));

    static_cast<void>(view_router.optimize_view(
        progress_request(),
        [&view_phases](const EnsembleProgressView& p) {
            view_phases.push_back(p.phase);
        }));

    REQUIRE(owning_phases == view_phases);
}

// ---------------------------------------------------------------------------
// Owning/view field equivalence
// ---------------------------------------------------------------------------

TEST_CASE("ensemble owning and view callbacks carry identical field values") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    auto ds2 = two_member_dataset(fixture);
    REQUIRE(ds2.has_value());

    EnsembleRouter owning_router{std::move(ds.value())};
    EnsembleRouter view_router{std::move(ds2.value())};

    // Capture the finalizing snapshot from each — that always fires.
    std::optional<EnsembleProgress> owning_snap;
    std::optional<EnsembleProgressView> view_snap;

    static_cast<void>(owning_router.optimize(
        progress_request(),
        [&owning_snap](const EnsembleProgress& p) {
            if (p.phase == EnsembleSolverPhase::finalizing)
                owning_snap = p;
        }));

    static_cast<void>(view_router.optimize_view(
        progress_request(),
        [&view_snap](const EnsembleProgressView& p) {
            if (p.phase == EnsembleSolverPhase::finalizing)
                view_snap = p;
        }));

    REQUIRE(owning_snap.has_value());
    REQUIRE(view_snap.has_value());

    REQUIRE(owning_snap->active_member_count == 0U);
    REQUIRE(owning_snap->active_label_count == 0U);
    REQUIRE(owning_snap->active_member_count == view_snap->active_member_count);
    REQUIRE(owning_snap->active_label_count  == view_snap->active_label_count);
    REQUIRE(owning_snap->retained_label_count == view_snap->retained_label_count);
    REQUIRE(owning_snap->generated_states    == view_snap->generated_states);
    REQUIRE(owning_snap->settled_states      == view_snap->settled_states);
    REQUIRE(
        owning_snap->current_objective_bound.value_class ==
        view_snap->current_objective_bound.value_class);
    REQUIRE(owning_snap->policy_alternative_count == view_snap->policy_alternative_count);
}

// ---------------------------------------------------------------------------
// Callback-scoped view: copying the view inside the callback is safe
// ---------------------------------------------------------------------------

TEST_CASE("EnsembleProgressView can be copied inside callback without aliasing") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    // Collect all view snapshots by value copy — no dangling references.
    std::vector<EnsembleProgressView> all_views;
    const auto result = router.optimize_view(
        progress_request(),
        [&all_views](const EnsembleProgressView& p) {
            all_views.push_back(p);  // copy
        });

    REQUIRE(result.has_value());
    REQUIRE(!all_views.empty());
    // All copied views are still accessible after optimize_view returns.
    REQUIRE(all_views.front().phase == EnsembleSolverPhase::initializing);
    REQUIRE(all_views.back().phase  == EnsembleSolverPhase::finalizing);
}

// ---------------------------------------------------------------------------
// active_member_count is bounded by dataset member count
// ---------------------------------------------------------------------------

TEST_CASE("ensemble progress active_member_count never exceeds dataset member count") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    const std::size_t member_count = ds.value().member_count();
    EnsembleRouter router{std::move(ds.value())};

    const auto result = router.optimize_view(
        progress_request(),
        [&member_count](const EnsembleProgressView& p) {
            REQUIRE(p.active_member_count <= member_count);
        });
    REQUIRE(result.has_value());
}

// ---------------------------------------------------------------------------
// retained_label_count > 0 after search
// ---------------------------------------------------------------------------

TEST_CASE("ensemble progress retained_label_count is positive after searching phase") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    std::size_t max_retained = 0U;
    const auto result = router.optimize_view(
        progress_request(),
        [&max_retained](const EnsembleProgressView& p) {
            max_retained = std::max(max_retained, p.retained_label_count);
        });
    REQUIRE(result.has_value());
    REQUIRE(max_retained > 0U);
}

// ---------------------------------------------------------------------------
// current_objective_bound starts as positive_infinity
// ---------------------------------------------------------------------------

TEST_CASE("ensemble progress current_objective_bound begins as positive_infinity") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    bool first_seen = false;
    static_cast<void>(router.optimize_view(
        progress_request(),
        [&first_seen](const EnsembleProgressView& p) {
            if (!first_seen) {
                first_seen = true;
                REQUIRE(
                    p.current_objective_bound.value_class ==
                    EnsembleObjectiveValueClass::positive_infinity);
            }
        }));
    REQUIRE(first_seen);
}

// ---------------------------------------------------------------------------
// Cancellation from owning callback at initializing
// ---------------------------------------------------------------------------

TEST_CASE("ensemble owning callback can cancel at initializing phase") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    std::size_t call_count = 0U;
    const auto result = router.optimize(
        progress_request(),
        [&call_count](const EnsembleProgress& p) -> RoutingProgressDecision {
            ++call_count;
            if (p.phase == EnsembleSolverPhase::initializing)
                return RoutingProgressDecision::cancel;
            return RoutingProgressDecision::continue_routing;
        });

    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == ErrorCode::cancelled);
    REQUIRE(call_count == 1U);
}

// ---------------------------------------------------------------------------
// Cancellation from view callback at initializing
// ---------------------------------------------------------------------------

TEST_CASE("ensemble view callback can cancel at initializing phase") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    const auto result = router.optimize_view(
        progress_request(),
        [](const EnsembleProgressView& p) -> RoutingProgressDecision {
            if (p.phase == EnsembleSolverPhase::initializing)
                return RoutingProgressDecision::cancel;
            return RoutingProgressDecision::continue_routing;
        });

    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == ErrorCode::cancelled);
}

// ---------------------------------------------------------------------------
// Cancellation after initializing (second callback)
// ---------------------------------------------------------------------------

TEST_CASE("ensemble owning callback can cancel at the second progress delivery") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    std::size_t call_count = 0U;
    const auto result = router.optimize(
        progress_request(),
        [&call_count](const EnsembleProgress&) -> RoutingProgressDecision {
            ++call_count;
            // Let initializing pass; cancel on the second callback (searching
            // or finalizing depending on fixture size).
            if (call_count >= 2U)
                return RoutingProgressDecision::cancel;
            return RoutingProgressDecision::continue_routing;
        });

    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == ErrorCode::cancelled);
    REQUIRE(call_count >= 2U);
}

// ---------------------------------------------------------------------------
// Exception propagation from owning callback
// ---------------------------------------------------------------------------

TEST_CASE("ensemble owning callback exceptions propagate synchronously") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    bool propagated = false;
    try {
        static_cast<void>(router.optimize(
            progress_request(),
            [](const EnsembleProgress&) -> RoutingProgressDecision {
                throw std::runtime_error{"ensemble progress fault"};
            }));
    } catch (const std::runtime_error& ex) {
        propagated =
            std::string_view{ex.what()} == "ensemble progress fault";
    }
    REQUIRE(propagated);
}

// ---------------------------------------------------------------------------
// Exception propagation from view callback
// ---------------------------------------------------------------------------

TEST_CASE("ensemble view callback exceptions propagate synchronously") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    bool propagated = false;
    try {
        static_cast<void>(router.optimize_view(
            progress_request(),
            [](const EnsembleProgressView&) -> RoutingProgressDecision {
                throw std::runtime_error{"ensemble view progress fault"};
            }));
    } catch (const std::runtime_error& ex) {
        propagated =
            std::string_view{ex.what()} == "ensemble view progress fault";
    }
    REQUIRE(propagated);
}

// ---------------------------------------------------------------------------
// settle_states monotonically increases across searching callbacks
// ---------------------------------------------------------------------------

TEST_CASE("ensemble progress settled_states is non-decreasing across callbacks") {
    const ConstantWindGribFixture fixture;
    auto ds = two_member_dataset(fixture);
    REQUIRE(ds.has_value());
    EnsembleRouter router{std::move(ds.value())};

    std::size_t last_settled = 0U;
    bool ordering_violated = false;
    static_cast<void>(router.optimize_view(
        progress_request(),
        [&last_settled, &ordering_violated](const EnsembleProgressView& p) {
            if (p.phase == EnsembleSolverPhase::searching) {
                if (p.settled_states < last_settled)
                    ordering_violated = true;
                last_settled = p.settled_states;
            }
        }));
    REQUIRE(!ordering_violated);
}

// ---------------------------------------------------------------------------
// Internal lattice cancellation callback (direct detail call, no progress API)
// ---------------------------------------------------------------------------

TEST_CASE("internal ensemble lattice cancellation callback produces cancelled error") {
    const ConstantWindGribFixture fixture;
    auto ds = EnsembleDataset::load(
        progress_metadata(),
        {progress_input("member", fixture)});
    REQUIRE(ds.has_value());

    const auto result = sailroute::detail::optimize_ensemble_lattice_route(
        ds.value(),
        sailroute::VesselPolar::default_racer_cruiser_45ft(),
        progress_request(),
        [] { return true; });

    REQUIRE(!result.has_value());
    REQUIRE(result.error().code == ErrorCode::cancelled);
}
