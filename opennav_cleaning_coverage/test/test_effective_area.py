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

from opennav_cleaning_coverage.geometry import EffectiveAreaBuilder
from opennav_cleaning_coverage.models import (
    GeometryMode,
    GridMap,
    PlanRequest,
    PlanningError,
    Polygon2D,
)


def square(min_x, min_y, max_x, max_y):
    return Polygon2D.from_tuples(
        [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)])


def test_user_geometry_subtracts_explicit_exclusions():
    request = PlanRequest(
        roi=square(0.0, 0.0, 10.0, 10.0),
        exclusions=(square(4.0, 4.0, 6.0, 6.0),),
        geometry_mode=GeometryMode.USER,
        op_width=1.0,
    )

    components = EffectiveAreaBuilder().build(request)

    assert len(components) == 1
    assert components[0].area == pytest.approx(96.0)
    assert len(components[0].interiors) == 1


def test_costmap_geometry_splits_components_at_blocked_cells():
    request = PlanRequest(
        roi=square(0.0, 0.0, 4.0, 2.0),
        geometry_mode=GeometryMode.COSTMAP,
        op_width=1.0,
    )
    costmap = GridMap(
        width=4,
        height=2,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(0, 100, 0, 0, 0, 100, 0, 0),
    )

    components = EffectiveAreaBuilder().build(request, costmap)

    assert [component.area for component in components] == pytest.approx([4.0, 2.0])


def test_costmap_geometry_derives_roi_from_map_bounds_when_goal_roi_is_empty():
    request = PlanRequest(
        roi=Polygon2D.from_tuples([]),
        geometry_mode=GeometryMode.COSTMAP,
        op_width=1.0,
    )
    costmap = GridMap(
        width=4,
        height=2,
        resolution=1.0,
        origin_x=2.0,
        origin_y=-3.0,
        origin_yaw=0.0,
        data=(0,) * 8,
    )

    components = EffectiveAreaBuilder().build(request, costmap)

    assert len(components) == 1
    assert components[0].area == pytest.approx(8.0)
    assert components[0].bounds == pytest.approx((2.0, -3.0, 6.0, -1.0))


def test_costmap_filters_known_obstacle_components_below_configured_area():
    request = PlanRequest(
        roi=square(0.0, 0.0, 3.0, 3.0),
        geometry_mode=GeometryMode.COSTMAP,
        op_width=1.0,
    )
    costmap = GridMap(
        width=3,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(0, 0, 0, 0, 100, 0, 0, 0, 0),
    )

    components = EffectiveAreaBuilder(min_obstacle_area=2.0).build(
        request, costmap)

    assert len(components) == 1
    assert components[0].area == pytest.approx(9.0)
    assert len(components[0].interiors) == 0


def test_unknown_cells_remain_blocked_even_when_known_noise_is_filtered():
    request = PlanRequest(
        roi=square(0.0, 0.0, 3.0, 3.0),
        geometry_mode=GeometryMode.COSTMAP,
        op_width=1.0,
    )
    costmap = GridMap(
        width=3,
        height=3,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(0, 0, 0, 0, 255, 0, 0, 0, 0),
    )

    components = EffectiveAreaBuilder(min_obstacle_area=100.0).build(
        request, costmap)

    assert components[0].area == pytest.approx(8.0)
    assert len(components[0].interiors) == 1


def test_fused_geometry_applies_costmap_and_explicit_exclusions():
    request = PlanRequest(
        roi=square(0.0, 0.0, 4.0, 2.0),
        exclusions=(square(3.0, 0.5, 3.5, 1.5),),
        geometry_mode=GeometryMode.FUSED,
        op_width=1.0,
    )
    costmap = GridMap(
        width=4,
        height=2,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(0, 100, 0, 0, 0, 100, 0, 0),
    )

    components = EffectiveAreaBuilder().build(request, costmap)

    assert [component.area for component in components] == pytest.approx([3.5, 2.0])


def test_strict_geometry_rejects_self_intersection_without_repair():
    request = PlanRequest(
        roi=Polygon2D.from_tuples(
            [(0.0, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0)]),
        geometry_mode=GeometryMode.USER,
        op_width=1.0,
    )

    with pytest.raises(PlanningError) as error:
        EffectiveAreaBuilder().build(request)

    assert error.value.code == "INVALID_GEOMETRY"


def test_costmap_filters_cleanable_islands_below_configured_area():
    request = PlanRequest(
        roi=square(0.0, 0.0, 5.0, 2.0),
        geometry_mode=GeometryMode.COSTMAP,
        op_width=1.0,
    )
    costmap = GridMap(
        width=5,
        height=2,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=(0, 100, 0, 0, 0, 0, 100, 0, 0, 0),
    )

    components = EffectiveAreaBuilder(min_cleanable_area=3.0).build(
        request, costmap)

    assert len(components) == 1
    assert components[0].area == pytest.approx(6.0)


def test_strict_geometry_rejects_overlapping_exclusions():
    request = PlanRequest(
        roi=square(0.0, 0.0, 10.0, 10.0),
        exclusions=(
            square(2.0, 2.0, 5.0, 5.0),
            square(4.0, 4.0, 7.0, 7.0),
        ),
        geometry_mode=GeometryMode.USER,
        op_width=1.0,
    )

    with pytest.raises(PlanningError) as error:
        EffectiveAreaBuilder().build(request)

    assert error.value.code == "INVALID_GEOMETRY"
