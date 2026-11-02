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

import json
from pathlib import Path

import pytest
from shapely.geometry import Polygon

from opennav_cleaning_coverage.f2c_adapter import Fields2CoverSwathGenerator
import opennav_cleaning_coverage.f2c_adapter as adapter
from opennav_cleaning_coverage.models import (
    AngleMode,
    DecompositionMode,
    Objective,
    PlanRequest,
    PlanningError,
    Polygon2D,
    SegmentType,
)


def test_fixed_angle_returns_boustrophedon_sparse_swath_endpoints():
    polygon = Polygon([(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)])
    request = PlanRequest(
        roi=Polygon2D.from_tuples(polygon.exterior.coords[:-1]),
        op_width=1.0,
        angle_mode=AngleMode.FIXED,
        fixed_angle_deg=0.0,
    )

    segments = Fields2CoverSwathGenerator().generate(
        polygon, request, component_index=0)

    assert len(segments) == 2
    assert all(segment.segment_type == SegmentType.COVERAGE for segment in segments)
    assert (segments[0].start.x, segments[0].start.y) == pytest.approx((4.0, 1.5))
    assert (segments[0].end.x, segments[0].end.y) == pytest.approx((0.0, 1.5))
    assert (segments[1].start.x, segments[1].start.y) == pytest.approx((0.0, 0.5))
    assert (segments[1].end.x, segments[1].end.y) == pytest.approx((4.0, 0.5))


def test_auto_angle_checks_cancellation_between_candidates():
    polygon = Polygon([(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)])
    request = PlanRequest(
        roi=Polygon2D.from_tuples(polygon.exterior.coords[:-1]),
        op_width=1.0,
        angle_mode=AngleMode.AUTO,
        angle_step_deg=5.0,
    )
    calls = 0

    def checkpoint():
        nonlocal calls
        calls += 1
        if calls == 3:
            raise PlanningError("CANCELED", "planning was canceled")

    with pytest.raises(PlanningError) as error:
        Fields2CoverSwathGenerator().generate(
            polygon, request, component_index=0, checkpoint=checkpoint)

    assert error.value.code == "CANCELED"


def test_auto_angle_min_turns_prefers_long_axis_swaths_deterministically():
    polygon = Polygon([(0.0, 0.0), (4.0, 0.0), (4.0, 2.0), (0.0, 2.0)])
    request = PlanRequest(
        roi=Polygon2D.from_tuples(polygon.exterior.coords[:-1]),
        op_width=1.0,
        angle_mode=AngleMode.AUTO,
        angle_step_deg=90.0,
        objective=Objective.MIN_TURNS,
    )

    segments = Fields2CoverSwathGenerator().generate(
        polygon, request, component_index=0)

    assert len(segments) == 2
    assert (segments[0].start.x, segments[0].start.y) == pytest.approx((4.0, 1.5))
    assert (segments[0].end.x, segments[0].end.y) == pytest.approx((0.0, 1.5))


def test_balanced_uses_length_tie_breaker_while_min_turns_uses_angle():
    generator = Fields2CoverSwathGenerator()

    min_turns_shorter_angle = generator._objective_score(
        Objective.MIN_TURNS, turns=2, length=100.0, angle_deg=10.0)
    min_turns_shorter_length = generator._objective_score(
        Objective.MIN_TURNS, turns=2, length=10.0, angle_deg=20.0)
    balanced_shorter_angle = generator._objective_score(
        Objective.BALANCED, turns=2, length=100.0, angle_deg=10.0)
    balanced_shorter_length = generator._objective_score(
        Objective.BALANCED, turns=2, length=10.0, angle_deg=20.0)

    assert min_turns_shorter_angle < min_turns_shorter_length
    assert balanced_shorter_length < balanced_shorter_angle


def test_invalid_polygon_is_rejected_without_calling_native_bindings(monkeypatch):
    coordinates = json.loads(
        (Path(__file__).parent / "fixtures" / "invalid-polygon.json").read_text()
    )["field"]
    polygon = Polygon(coordinates)
    calls = []

    class ForbiddenNative:
        def __getattr__(self, name):
            calls.append(name)
            raise AssertionError("Invalid polygon reached Fields2Cover")

    monkeypatch.setattr(adapter, "f2c", ForbiddenNative())
    request = PlanRequest(
        roi=Polygon2D.from_tuples(coordinates),
        op_width=0.666,
        angle_mode=AngleMode.FIXED,
    )

    with pytest.raises(PlanningError) as error:
        Fields2CoverSwathGenerator().generate(polygon, request, component_index=0)

    assert error.value.code == "INVALID_GEOMETRY"
    assert "Self-intersection" in str(error.value)
    assert not calls


@pytest.mark.parametrize("op_width, limit", [(1.0, 1), (0.02, 500)])
def test_complex_route_is_rejected_before_allocating_native_graph(monkeypatch, op_width, limit):
    polygon = Polygon([(0, 0), (4, 0), (4, 2), (2, 2), (2, 4), (0, 4)])
    request = PlanRequest(
        roi=Polygon2D.from_tuples(polygon.exterior.coords[:-1]),
        op_width=op_width, angle_mode=AngleMode.FIXED,
        decomposition=DecompositionMode.NONE,
    )
    calls = []

    def forbidden_graph():
        calls.append(True)
        raise AssertionError("Oversized route reached native graph allocation")

    monkeypatch.setattr(adapter, "MAX_AUTO_ROUTE_NODES", limit)
    monkeypatch.setattr(adapter.f2c, "RP_RoutePlannerBase", forbidden_graph)
    with pytest.raises(PlanningError, match=f"limit {limit}") as error:
        Fields2CoverSwathGenerator().generate(polygon, request, component_index=0)
    assert error.value.code == "PLANNING_FAILED"
    assert not calls


def test_route_at_node_limit_still_uses_native_ordering(monkeypatch):
    polygon = Polygon([(0, 0), (4, 0), (4, 2), (0, 2)])
    request = PlanRequest(
        roi=Polygon2D.from_tuples(polygon.exterior.coords[:-1]),
        op_width=1.0, angle_mode=AngleMode.FIXED,
    )
    # Five closed-ring vertices plus four nodes for each of two swaths.
    monkeypatch.setattr(adapter, "MAX_AUTO_ROUTE_NODES", 13)
    native_planner = adapter.f2c.RP_RoutePlannerBase
    calls = []

    def tracked_graph():
        calls.append(True)
        return native_planner()

    monkeypatch.setattr(adapter.f2c, "RP_RoutePlannerBase", tracked_graph)
    assert len(Fields2CoverSwathGenerator().generate(polygon, request, component_index=0)) == 2
    assert calls == [True]


def test_large_convex_route_preserves_every_swath_without_native_graph(monkeypatch):
    polygon = Polygon([(0, 0), (20, 0), (20, 40), (0, 40)])
    request = PlanRequest(
        roi=Polygon2D.from_tuples(polygon.exterior.coords[:-1]),
        op_width=0.3, angle_mode=AngleMode.FIXED,
        decomposition=DecompositionMode.NONE,
    )

    def forbidden_graph():
        raise AssertionError("Oversized route reached native graph allocation")

    monkeypatch.setattr(adapter.f2c, "RP_RoutePlannerBase", forbidden_graph)
    segments = Fields2CoverSwathGenerator().generate(polygon, request, component_index=0)
    assert adapter.MAX_AUTO_ROUTE_NODES == 500
    assert len(segments) == 133
    assert len({segment.start.y for segment in segments}) == 133
    for segment in segments:
        assert sorted([segment.start.x, segment.end.x]) == pytest.approx([0, 20])
        assert segment.start.y == pytest.approx(segment.end.y)


def test_route_node_bound_includes_hole_boundaries(monkeypatch):
    polygon = Polygon([(0, 0), (8, 0), (8, 8), (0, 8)],
                      holes=[[(2, 2), (3, 2), (3, 3), (2, 3)]])
    generator = Fields2CoverSwathGenerator()
    cells = generator._to_cells(polygon)
    native_generator = adapter.f2c.SG_BruteForce()
    native_generator.setAllowOverlap(False)
    swaths = native_generator.generateSwaths(0.0, 1.0, cells)
    # A bound that omits the hole would incorrectly admit this native route.
    limit = cells.getGeometry(0).getGeometry(0).size() + 4 * swaths.at(0).size()
    monkeypatch.setattr(adapter, "MAX_AUTO_ROUTE_NODES", limit, raising=False)

    def forbidden_graph():
        raise AssertionError("Hole boundary was omitted from the graph bound")

    monkeypatch.setattr(adapter.f2c, "RP_RoutePlannerBase", forbidden_graph)
    request = PlanRequest(
        roi=Polygon2D.from_tuples(polygon.exterior.coords[:-1]),
        op_width=1.0, angle_mode=AngleMode.FIXED,
        decomposition=DecompositionMode.NONE,
    )
    with pytest.raises(PlanningError, match=f"limit {limit}"):
        generator.generate(polygon, request, component_index=0)
