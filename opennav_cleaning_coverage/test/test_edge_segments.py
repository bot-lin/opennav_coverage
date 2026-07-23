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

import math

import pytest
from shapely.geometry import Polygon

from opennav_cleaning_coverage.edges import EdgeSegmentGenerator
from opennav_cleaning_coverage.models import SegmentType


def test_edges_are_sparse_deterministic_segments_with_outer_boundary_first():
    polygon = Polygon(
        [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        holes=[[(4.0, 4.0), (4.0, 6.0), (6.0, 6.0), (6.0, 4.0)]],
    )

    segments = EdgeSegmentGenerator().generate(polygon, component_index=3)

    assert len(segments) == 8
    assert [segment.boundary_index for segment in segments] == [0] * 4 + [1] * 4
    assert all(segment.segment_type == SegmentType.EDGE for segment in segments)
    assert all(segment.component_index == 3 for segment in segments)
    assert (segments[0].start.x, segments[0].start.y) == pytest.approx((0.0, 0.0))
    assert (segments[0].end.x, segments[0].end.y) == pytest.approx((10.0, 0.0))
    assert segments[0].start.yaw == pytest.approx(0.0)
    assert segments[4].start.yaw == pytest.approx(math.pi / 2.0)
