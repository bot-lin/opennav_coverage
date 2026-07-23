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

import pytest

from opennav_cleaning_coverage.models import (
    AngleMode,
    GeometryMode,
    GridMap,
    PlanRequest,
    Polygon2D,
)
from opennav_cleaning_coverage.planner import CleaningCoveragePlanner


def polygon(points):
    return Polygon2D.from_tuples(points)


def signature(result):
    return tuple(
        (
            component.index,
            tuple(
                (
                    round(segment.start.x, 6),
                    round(segment.start.y, 6),
                    round(segment.end.x, 6),
                    round(segment.end.y, 6),
                )
                for segment in component.edge_segments
                + component.coverage_segments
            ),
        )
        for component in result.components
    )


@pytest.mark.parametrize(
    "plan_request,costmap,expected_components",
    [
        (
            PlanRequest(
                roi=polygon([(0, 0), (12, 0), (12, 8), (0, 8)]),
                exclusions=(polygon([(5, 3), (7, 3), (7, 5), (5, 5)]),),
                op_width=1.0,
                angle_mode=AngleMode.AUTO,
                angle_step_deg=30.0,
            ),
            None,
            1,
        ),
        (
            PlanRequest(
                roi=polygon([(0, 0), (8, 0), (8, 3), (3, 3), (3, 7), (0, 7)]),
                op_width=1.0,
                angle_mode=AngleMode.AUTO,
                angle_step_deg=30.0,
            ),
            None,
            1,
        ),
        (
            PlanRequest(
                roi=polygon([(0, 0), (8, 0), (8, 4), (0, 4)]),
                geometry_mode=GeometryMode.COSTMAP,
                op_width=1.0,
                angle_mode=AngleMode.FIXED,
                fixed_angle_deg=0.0,
            ),
            GridMap(
                width=8,
                height=4,
                resolution=1.0,
                origin_x=0.0,
                origin_y=0.0,
                origin_yaw=0.0,
                data=tuple(
                    100 if column == 3 else 0
                    for _row in range(4)
                    for column in range(8)
                ),
            ),
            2,
        ),
    ],
    ids=["warehouse_with_pillar", "factory_l_shape", "office_components"],
)
def test_golden_indoor_scenarios_are_complete_and_deterministic(
    plan_request, costmap, expected_components
):
    planner = CleaningCoveragePlanner()

    first = planner.plan(plan_request, costmap=costmap)
    second = planner.plan(plan_request, costmap=costmap)

    assert len(first.components) == expected_components
    assert first.coverage_ratio >= plan_request.min_coverage_ratio
    assert signature(first) == signature(second)
