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

from geometry_msgs.msg import Point32
from opennav_cleaning_coverage_msgs.action import ComputeCleaningCoverage

from opennav_cleaning_coverage.models import (
    AngleMode,
    GeometryMode,
    Objective,
    Polygon2D,
)
from opennav_cleaning_coverage.ros_conversions import goal_to_request


def test_goal_conversion_applies_infrastructure_defaults_and_polygon_transform():
    goal = ComputeCleaningCoverage.Goal()
    goal.geometry_mode = goal.USER
    goal.roi.header.frame_id = "odom"
    goal.roi.polygon.points = [
        Point32(x=0.0, y=0.0),
        Point32(x=4.0, y=0.0),
        Point32(x=4.0, y=2.0),
        Point32(x=0.0, y=2.0),
    ]
    goal.op_width = 0.8
    goal.angle_mode = goal.AUTO
    goal.angle_step_deg = 0.0
    goal.optimization_objective = goal.MIN_TURNS
    goal.decomposition_mode = goal.DECOMPOSITION_AUTO
    goal.min_coverage_ratio = 0.0
    transformed_frames = []

    def transform(value):
        transformed_frames.append(value.header.frame_id)
        return Polygon2D.from_tuples(
            [(point.x + 1.0, point.y) for point in value.polygon.points])

    request = goal_to_request(
        goal,
        transform,
        default_angle_step_deg=5.0,
        default_min_coverage_ratio=0.98,
    )

    assert transformed_frames == ["odom"]
    assert request.geometry_mode == GeometryMode.USER
    assert request.angle_mode == AngleMode.AUTO
    assert request.objective == Objective.MIN_TURNS
    assert request.angle_step_deg == 5.0
    assert request.min_coverage_ratio == 0.98
    assert request.roi.vertices[0].x == 1.0
