# Copyright 2026 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Conversions at the ROS action boundary."""

from .models import (
    AngleMode,
    DecompositionMode,
    GeometryMode,
    Objective,
    PlanRequest,
    PlanningError,
)


def goal_to_request(
    goal,
    transform_polygon,
    default_angle_step_deg: float,
    default_min_coverage_ratio: float,
) -> PlanRequest:
    """Convert a ROS goal after transforming every polygon to the planning frame."""
    try:
        geometry_mode = GeometryMode(goal.geometry_mode)
        angle_mode = AngleMode(goal.angle_mode)
        objective = Objective(goal.optimization_objective)
        decomposition = DecompositionMode(goal.decomposition_mode)
    except ValueError as error:
        raise PlanningError(
            "INVALID_PARAMETERS", f"invalid enum value in action goal: {error}") from error

    return PlanRequest(
        roi=transform_polygon(goal.roi),
        exclusions=tuple(
            transform_polygon(exclusion) for exclusion in goal.exclusions),
        geometry_mode=geometry_mode,
        op_width=float(goal.op_width),
        angle_mode=angle_mode,
        fixed_angle_deg=float(goal.fixed_angle_deg),
        angle_step_deg=(
            float(goal.angle_step_deg)
            if goal.angle_step_deg > 0.0
            else default_angle_step_deg
        ),
        objective=objective,
        decomposition=decomposition,
        headland_width=float(goal.headland_width),
        max_traversable_cost=int(goal.max_traversable_cost),
        min_coverage_ratio=(
            float(goal.min_coverage_ratio)
            if goal.min_coverage_ratio > 0.0
            else default_min_coverage_ratio
        ),
    )
