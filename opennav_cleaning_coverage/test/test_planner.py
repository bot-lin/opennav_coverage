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
    CleaningSegment,
    GeometryMode,
    GridMap,
    PlanRequest,
    PlanningError,
    Polygon2D,
    Pose2D,
    SegmentType,
)
from opennav_cleaning_coverage.planner import CleaningCoveragePlanner


def test_planner_returns_edge_first_atomic_component_and_coverage_ratio():
    request = PlanRequest(
        roi=Polygon2D.from_tuples(
            [(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)]),
        op_width=1.0,
        angle_mode=AngleMode.FIXED,
        fixed_angle_deg=0.0,
    )

    result = CleaningCoveragePlanner().plan(request)

    assert len(result.components) == 1
    component = result.components[0]
    assert len(component.edge_segments) == 4
    assert len(component.coverage_segments) == 2
    assert len(component.transition_segments) == 2
    assert component.ordered_poses[0] == component.edge_segments[0].start
    assert component.coverage_ratio == pytest.approx(1.0)
    assert result.coverage_ratio == pytest.approx(1.0)
    assert result.uncovered_regions == ()


def test_planner_aborts_atomically_when_a_generated_segment_leaves_safe_area():
    class UnsafeSwathGenerator:
        def generate(
            self, _polygon, _request, component_index, checkpoint=None
        ):
            del checkpoint
            return [
                CleaningSegment(
                    segment_type=SegmentType.COVERAGE,
                    component_index=component_index,
                    boundary_index=-1,
                    swath_index=0,
                    start=Pose2D(1.0, 1.0, 0.0),
                    end=Pose2D(5.0, 1.0, 0.0),
                )
            ]

    request = PlanRequest(
        roi=Polygon2D.from_tuples(
            [(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)]),
        op_width=1.0,
        angle_mode=AngleMode.FIXED,
        min_coverage_ratio=0.1,
    )
    footprint = Polygon2D.from_tuples(
        [(-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)])

    with pytest.raises(PlanningError) as error:
        CleaningCoveragePlanner(
            swath_generator=UnsafeSwathGenerator()).plan(
                request, footprint=footprint)

    assert error.value.code == "PLANNING_FAILED"


def test_planner_honors_cancellation_before_expensive_generation():
    request = PlanRequest(
        roi=Polygon2D.from_tuples(
            [(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)]),
        op_width=1.0,
        angle_mode=AngleMode.FIXED,
    )

    with pytest.raises(PlanningError) as error:
        CleaningCoveragePlanner().plan(request, cancel_check=lambda: True)

    assert error.value.code == "CANCELED"


def test_planner_honors_cancellation_during_costmap_preprocessing():
    request = PlanRequest(
        roi=Polygon2D.from_tuples([]),
        geometry_mode=GeometryMode.COSTMAP,
        op_width=1.0,
    )
    costmap = GridMap(
        width=100,
        height=100,
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(255,) * 10000,
    )
    footprint = Polygon2D.from_tuples(
        [(-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)])
    checks = 0

    def cancel_after_preprocessing_starts():
        nonlocal checks
        checks += 1
        return checks >= 3

    with pytest.raises(PlanningError) as error:
        CleaningCoveragePlanner().plan(
            request,
            costmap=costmap,
            footprint=footprint,
            cancel_check=cancel_after_preprocessing_starts,
        )

    assert error.value.code == "CANCELED"
    assert checks == 3


def test_planner_returns_transition_pair_between_disconnected_components():
    request = PlanRequest(
        roi=Polygon2D.from_tuples(
            [(0.0, 0.0), (5.0, 0.0), (5.0, 2.0), (0.0, 2.0)]),
        geometry_mode=GeometryMode.COSTMAP,
        op_width=1.0,
        angle_mode=AngleMode.FIXED,
    )
    costmap = GridMap(
        width=5,
        height=2,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(0, 0, 100, 0, 0, 0, 0, 100, 0, 0),
    )

    result = CleaningCoveragePlanner().plan(request, costmap=costmap)

    assert len(result.components) == 2
    previous = result.components[0]
    current = result.components[1]
    cross_component = current.transition_segments[0]
    assert cross_component.start == previous.coverage_segments[-1].end
    assert cross_component.end == current.edge_segments[0].start
