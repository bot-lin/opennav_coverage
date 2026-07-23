# CLAUDE.md

## Project

This branch contains an indoor cleaning coverage planner for ROS 2 Humble:

- `opennav_cleaning_coverage_msgs` defines `ComputeCleaningCoverage`.
- `opennav_cleaning_coverage` implements the lifecycle action server and the
  ROS-independent planning core.

The planner uses Fields2Cover v2.0.0. It produces edge-cleaning and coverage
segment endpoints; Nav2 is responsible for execution and dynamic obstacles.

## Build and test

```bash
colcon build --packages-select \
  opennav_cleaning_coverage_msgs \
  opennav_cleaning_coverage

colcon test --packages-select \
  opennav_cleaning_coverage_msgs \
  opennav_cleaning_coverage

colcon test-result --verbose
```

The repository development container is `ros2-humble-dev`, and this workspace
is mounted at `/data/ws_full_path`.

## Architecture

Keep the planning core independent from ROS:

- `geometry.py`: strict ROI/exclusion validation and USER/COSTMAP/FUSED
  effective geometry.
- `edges.py`: deterministic endpoint-only boundary segments.
- `f2c_adapter.py`: narrow Fields2Cover v2 adapter.
- `planner.py`: atomic orchestration, transition endpoint pairs, coverage
  ratio, cancellation, deadlines, and final segment validation.
- `artifacts.py`: bounded composite PNG output.
- `ros_conversions.py` and `server.py`: ROS messages, TF, lifecycle, action,
  Nav2 costmap, footprint, and markers.

The Fields2Cover v2 Python `Route.length()` call crashes in the current
environment. The adapter safely uses
`RP_RoutePlannerBase.genRoute(...).getVectorSwaths()` for global F2C order and
must not call `Route.length()`.

## Behavioral constraints

- Default planning frame: `map`.
- Exactly one action goal at a time.
- Costmap is static, inflated, and treats unknown as blocked.
- `op_width` is the canonical swath spacing; overlap remains disabled.
- Angles are degrees at the ROS interface and radians only inside the F2C
  adapter.
- Return sparse start/end poses, never interpolated paths.
- Edge cleaning precedes coverage cleaning.
- Do not merge separated collinear swaths.
- Never silently repair invalid polygons.
- Failed or canceled goals do not return partial executable components.
- Every successful goal writes one bounded planner-owned PNG.
