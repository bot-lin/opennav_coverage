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
