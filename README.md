# Indoor Cleaning Coverage Planner

This branch provides a ROS 2 Humble coverage-planning service for cleaning
robots operating in warehouses, factories, and offices. Given a required
region of interest (ROI), optional exclusion polygons, and cleaning
parameters, it returns deterministic edge-cleaning and complete-coverage
segments.

The planner is intentionally not an executor. It returns sparse segment
endpoints with headings. Nav2 or another downstream navigation system is
responsible for collision-free motion between those endpoints and for
handling dynamic obstacles.

## Packages

- `opennav_cleaning_coverage_msgs`: `ComputeCleaningCoverage` action and
  structured component/segment messages.
- `opennav_cleaning_coverage`: lifecycle action server, strict geometry
  processing, Fields2Cover v2 swath generation, validation, visualization,
  and bounded filesystem artifacts.

The old HTTP/demo package is not part of this branch.

## Requirements and build

- ROS 2 Humble
- Nav2 messages and lifecycle manager
- Fields2Cover v2.0.0 with Python bindings
- Shapely, NumPy, and OpenCV Python packages

Build the two packages after Fields2Cover:

```bash
colcon build --packages-select fields2cover
source install/setup.bash
colcon build --packages-select \
  opennav_cleaning_coverage_msgs \
  opennav_cleaning_coverage
source install/setup.bash
```

Launch the lifecycle server:

```bash
ros2 launch opennav_cleaning_coverage cleaning_coverage.launch.py
```

The lifecycle manager configures and activates
`cleaning_coverage_server`. The action name is
`/compute_cleaning_coverage`.

## Geometry modes

Every request requires an ROI. Empty polygon frame IDs use the configured
planning frame, which defaults to `map`. Other frames are transformed through
TF before planning.

| Mode | Effective planning geometry |
| --- | --- |
| `USER` | ROI minus caller-supplied exclusions. No costmap is queried. |
| `COSTMAP` | ROI intersected with traversable cells from Nav2's global costmap. Explicit exclusions are rejected. |
| `FUSED` | ROI intersected with traversable costmap cells, then caller-supplied exclusions are subtracted. |

The default costmap service is `/global_costmap/get_costmap`. Unknown cells are
blocked. Known cells are traversable when their cost is at or below
`max_traversable_cost`, whose default is zero. Small known obstacle components
are filtered using a threshold derived from `op_width` and map resolution
unless `min_obstacle_area` is configured explicitly. Tiny traversable islands
are filtered independently by a derived or explicit `min_cleanable_area`.
Topology-preserving simplification is limited to half a costmap cell.

The global costmap is expected to be static and already inflated for the
robot. Final validation checks every cleaning centerline against the resulting
safe geometry and sweeps the current footprint from
`/global_costmap/published_footprint` along every cleaning segment. The sweep
checks lethal and unknown cells while leaving the existing inflation gradient
to the centerline threshold, so inflation is not applied twice. Adjust Nav2's
inflation radius when more or less wall clearance is required.

Dynamic obstacles are deliberately outside this planner's scope and must be
handled during execution.

## Planning controls

`op_width` is the canonical Fields2Cover operation width and determines swath
spacing. Overlap is disabled and is not exposed as a separate ratio.

Angle values use degrees:

- `FIXED` normalizes `fixed_angle_deg` into `[0, 180)`.
- `AUTO` evaluates angles using `angle_step_deg`, defaulting to 5 degrees.
- Objectives are `MIN_TURNS`, `MIN_LENGTH`, and `BALANCED`. `BALANCED`
  minimizes turns first and uses length as its tie-breaker.

Decomposition can be `NONE`, `BOUSTROPHEDON`, or `TRAPEZOIDAL`. `AUTO` skips
decomposition for simple convex polygons and uses Boustrophedon decomposition
for concave polygons or polygons with holes.

`headland_width` defaults to zero. Only one coverage pass is generated.
Turning radius, curvature, speed, and connector paths are intentionally left
to downstream Nav2 execution.

## Result contract

Disconnected effective regions are returned as separate components, ordered
by descending area. For each component:

1. The outer edge is first.
2. Internal exclusion edges follow in centroid order.
3. Fields2Cover-ordered coverage swaths follow.

Each cleaning segment contains only a start pose and end pose. Transition
segments also contain endpoint pairs only; they are requests for downstream
navigation, not straight-line connector paths. Separated collinear swaths are
never merged.

The edge and coverage strokes are buffered by `op_width / 2`, unioned, and
clipped to the effective area. The default minimum coverage ratio is `0.98`.
The result includes the measured ratio and uncovered polygons. Planning aborts
atomically when the threshold is not met or when any generated cleaning
segment leaves the safe area.

One action goal is processed at a time. A concurrent goal is rejected as busy.
Feedback reports geometry extraction, swath generation, ordering, validation,
and artifact-writing stages. Cancellation and the default 30-second planning
deadline are checked between expensive stages. Canceled or failed goals never
return a partial executable plan.

## Filesystem and marker output

Every successful plan creates one composite PNG in
`/tmp/opennav_cleaning_coverage` by default. Images are no larger than
1600 × 1600. The server retains at most 20 images and 100 MiB, and deletes only
files with its own `cleaning_plan_*.png` prefix.

Set `publish_markers: true` to publish transient-local edge, coverage, and
transition markers on `cleaning_coverage_markers`.

Infrastructure settings are in
`opennav_cleaning_coverage/config/cleaning_coverage.yaml`. Geometry and
planning choices belong to each action goal; callers cannot choose filesystem
paths.

## Tests

```bash
colcon test --packages-select \
  opennav_cleaning_coverage_msgs \
  opennav_cleaning_coverage
colcon test-result --verbose
```

The test suite covers strict geometry, costmap extraction, deterministic edge
and swath order, coverage validation, bounded artifacts, lifecycle action
execution, and a mocked Nav2 `GetCostmap` service.
