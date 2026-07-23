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
    CleaningSegment,
    GridMap,
    PlanningError,
    Polygon2D,
    Pose2D,
    SegmentType,
)
from opennav_cleaning_coverage.safety import FootprintCollisionChecker


def segment(end_x):
    return CleaningSegment(
        segment_type=SegmentType.COVERAGE,
        component_index=0,
        boundary_index=-1,
        swath_index=0,
        start=Pose2D(1.0, 1.5, 0.0),
        end=Pose2D(end_x, 1.5, 0.0),
    )


def test_swept_footprint_rejects_lethal_cell_before_centerline_reaches_it():
    costmap = GridMap(
        width=6,
        height=4,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=tuple(
            254 if (column, row) == (4, 1) else 0
            for row in range(4)
            for column in range(6)
        ),
    )
    footprint = Polygon2D.from_tuples(
        [(-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)])
    checker = FootprintCollisionChecker(footprint, costmap)

    checker.validate([segment(3.6)])
    with pytest.raises(PlanningError) as error:
        checker.validate([segment(3.9)])

    assert error.value.code == "PLANNING_FAILED"
