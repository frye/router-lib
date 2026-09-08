# Migrating to 0.6

Version 0.6 prioritizes physically consistent, cruising-oriented routing over
byte-identical 0.3.2 behavior. Rebuild all C++ consumers: public aggregate layouts
have grown.

| Previous behavior | 0.6 behavior |
| --- | --- |
| Omitted departure could rewind to forecast start | Current UTC must be covered; historical departures must be explicit |
| CLI silently selected the built-in polar | Supply `--polar` or explicitly select `--demo-polar` |
| Two-mile arrival circle for beam, exact destination for lattice | Shared arrival-region semantics; default radius is 0.1 nm |
| Default ten-mile pruning buckets | Default two-mile buckets with spatial/heading/board diversity |
| Default fixed heading grid and segment-start wind | Destination/VMG proposals and bounded midpoint integration |
| Above-polar wind silently clamped for routing | Default `no_speed`; explicit `clamp` remains an advanced choice |
| Raw positive polar interpolation implied feasibility | Sailing support is checked separately; native Expedition rows are supported |
| Midpoint failure could restore old speed | Missing or infeasible physical samples reject/fail explicitly |
| Maneuver cost hidden inside a sailing leg | Explicit maneuver vertices, including current drift |
| Stationary waits generated anywhere | Holding needs `holding_eligibility` authorization |
| Coarse-node eligibility callback | Beam eligibility visits accepted physical segment candidates in caller-thread order |
| Current added only after polar evaluation | Wind is converted to the water frame for the polar before ground motion |
| Polar-only A* heuristic with currents | Current-configured deterministic searches use Dijkstra ordering |
| Duration partial serialized as forecast exhaustion | `route_result_v2` preserves the precise completion value |
| Ordinary serialization included ensemble types | Include `ensemble_serialization.hpp` explicitly |
| Ensemble results required policy-shaped output | Compact selected-route/member-risk results can omit alternatives |

Raw `PolarSlice::speed_knots` remains an interpolation utility, including endpoint
clamping. Call `supports_sailing_angle` before treating it as sailable; the router
does so automatically. Between wind curves, support is conservative rather than
invented outside either curve. An explicit minimum sailing angle can narrow,
but not extend, imported support.

`boat_speed_factor` scales boat performance, not weather. Forecast-wind limits
apply to meteorological ground-relative wind; polar support applies to
water-relative wind. Environmental output distinguishes those quantities.

`maximum_integration_step` defaults to fifteen minutes and is distinct from
search intervals/lattice time boundaries. Route points can therefore be more
frequent than search frontiers. Consumers should use actual timestamps rather
than assume every route leg equals a configured search interval.

Search results remain approximations. Progress routes are provisional, and
parent-chain reconstruction does not recover pruned alternatives. Future-weather
ranking assists retention without becoming a proof of optimality. A partial
route is not necessarily the best position for weather beyond forecast coverage.

The local GSHHG adapter is optional and bounded to documented regional support.
It does not download data, replace nautical charts, infer missing polygons, or
claim depth clearance. Keep dataset completeness, source accuracy and license
obligations separate from its numerical interpolation-error bound.

Keep existing racing/ensemble features behind their optional headers and build
option. Policy metadata describes retained alternatives; applications still own
new-forecast replanning from the observed vessel state. The experimental beam
remains gated rather than being silently substituted for another solver.

The old compatibility corpus and golden output remain unchanged as historical
reference. They are no longer a v0.6 acceptance test. Use current behavior,
geometry, physics, strategic-route and serialization contracts for upgrades;
do not regenerate an old golden file merely to approve a new algorithm.
