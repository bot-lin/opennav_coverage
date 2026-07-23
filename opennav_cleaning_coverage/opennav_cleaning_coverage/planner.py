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

"""High-level, ROS-independent indoor cleaning coverage planner."""

from dataclasses import replace
from math import hypot, isfinite
from time import monotonic
from typing import Callable, Optional

from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union

from .edges import EdgeSegmentGenerator
from .f2c_adapter import Fields2CoverSwathGenerator
from .geometry import EffectiveAreaBuilder
from .models import (
    CleaningSegment,
    PlanComponent,
    PlanRequest,
    PlanResult,
    PlannerInfrastructure,
    PlanningError,
    Polygon2D,
    SegmentType,
)
from .safety import FootprintCollisionChecker


class CleaningCoveragePlanner:
    """Plan all components atomically and expose only sparse segment endpoints."""

    def __init__(
        self,
        infrastructure: Optional[PlannerInfrastructure] = None,
        area_builder: Optional[EffectiveAreaBuilder] = None,
        edge_generator: Optional[EdgeSegmentGenerator] = None,
        swath_generator: Optional[Fields2CoverSwathGenerator] = None,
    ) -> None:
        self._infrastructure = infrastructure or PlannerInfrastructure()
        self._area_builder = area_builder or EffectiveAreaBuilder(
            self._infrastructure.geometry_epsilon)
        self._edge_generator = edge_generator or EdgeSegmentGenerator()
        self._swath_generator = swath_generator or Fields2CoverSwathGenerator()

    def plan(
        self,
        request: PlanRequest,
        costmap=None,
        footprint=None,
        feedback: Optional[Callable[[str, float], None]] = None,
        cancel_check: Optional[Callable[[], bool]] = None,
        deadline: Optional[float] = None,
    ) -> PlanResult:
        started = monotonic()
        deadline = deadline or started + self._infrastructure.planning_timeout_sec
        self._validate_request(request)
        collision_checker = None
        if footprint is not None:
            if costmap is not None:
                collision_checker = FootprintCollisionChecker(
                    footprint,
                    costmap,
                    epsilon=self._infrastructure.geometry_epsilon,
                )
            else:
                self._validate_footprint(footprint)
        self._checkpoint(cancel_check, deadline)
        self._report(feedback, "EXTRACTING_GEOMETRY", 0.1)
        polygons = self._area_builder.build(request, costmap)

        components = []
        cleaned_geometries = []
        total_area = sum(polygon.area for polygon in polygons)
        total_cleaned_area = 0.0
        for component_index, polygon in enumerate(polygons):
            self._checkpoint(cancel_check, deadline)
            progress = 0.2 + 0.6 * component_index / max(1, len(polygons))
            self._report(feedback, "DECOMPOSING", progress)
            edges = self._edge_generator.generate(polygon, component_index)
            self._report(feedback, "GENERATING_SWATHS", progress + 0.05)
            coverage = self._swath_generator.generate(
                polygon,
                request,
                component_index,
                checkpoint=lambda: self._checkpoint(cancel_check, deadline),
            )
            self._report(feedback, "ORDERING", progress + 0.1)
            cleaning_sequence = edges + coverage
            self._validate_cleaning_segments(cleaning_sequence, polygon)
            if collision_checker is not None:
                collision_checker.validate(cleaning_sequence)
            transitions = self._transitions(
                cleaning_sequence, component_index)
            ordered_poses = tuple(
                pose
                for segment in cleaning_sequence
                for pose in (segment.start, segment.end)
            )

            cleaned = self._cleaned_geometry(
                cleaning_sequence, request.op_width).intersection(polygon)
            cleaned_geometries.append(cleaned)
            cleaned_area = cleaned.area
            total_cleaned_area += cleaned_area
            component_ratio = cleaned_area / polygon.area
            components.append(
                PlanComponent(
                    index=component_index,
                    outer=self._ring_to_polygon2d(polygon.exterior.coords),
                    exclusions=self._ordered_holes(polygon),
                    edge_segments=tuple(edges),
                    coverage_segments=tuple(coverage),
                    transition_segments=tuple(transitions),
                    ordered_poses=ordered_poses,
                    coverage_ratio=component_ratio,
                )
            )

        self._add_cross_component_transitions(components)
        self._checkpoint(cancel_check, deadline)
        self._report(feedback, "VALIDATING_RESULT", 0.85)
        effective = unary_union(polygons)
        cleaned = unary_union(cleaned_geometries)
        uncovered = effective.difference(cleaned)
        coverage_ratio = total_cleaned_area / total_area
        uncovered_regions = tuple(
            self._ring_to_polygon2d(value.exterior.coords)
            for value in self._polygons(uncovered)
            if value.area > self._infrastructure.geometry_epsilon ** 2
        )
        result = PlanResult(
            components=tuple(components),
            coverage_ratio=coverage_ratio,
            uncovered_regions=uncovered_regions,
            planning_duration_sec=monotonic() - started,
        )
        if coverage_ratio + self._infrastructure.geometry_epsilon < (
            request.min_coverage_ratio
        ):
            error = PlanningError(
                "COVERAGE_BELOW_THRESHOLD",
                (
                    f"coverage ratio {coverage_ratio:.6f} is below "
                    f"{request.min_coverage_ratio:.6f}"
                ),
            )
            error.result = result
            raise error
        self._report(feedback, "VALIDATING_RESULT", 1.0)
        return result

    @staticmethod
    def _validate_request(request: PlanRequest) -> None:
        if (
            not isfinite(request.min_coverage_ratio)
            or not 0.0 < request.min_coverage_ratio <= 1.0
        ):
            raise PlanningError(
                "INVALID_PARAMETERS", "min_coverage_ratio must be in (0, 1]")
        if not isfinite(request.headland_width) or request.headland_width < 0.0:
            raise PlanningError(
                "INVALID_PARAMETERS",
                "headland_width must be finite and nonnegative",
            )
        if (
            not isfinite(request.angle_step_deg)
            or request.angle_step_deg <= 0.0
            or request.angle_step_deg >= 180.0
        ):
            raise PlanningError(
                "INVALID_PARAMETERS", "angle_step_deg must be in (0, 180)")

    @staticmethod
    def _validate_footprint(footprint):
        polygon = Polygon(
            [(point.x, point.y) for point in footprint.vertices])
        if (
            polygon.is_empty
            or not polygon.is_valid
            or polygon.area <= 0.0
            or not polygon.covers(Point(0.0, 0.0))
        ):
            raise PlanningError(
                "FOOTPRINT_UNAVAILABLE",
                "Nav2 footprint must be a valid polygon containing base_link",
            )

    def _validate_cleaning_segments(self, segments, safe_polygon):
        tolerance = self._infrastructure.geometry_epsilon
        permitted = safe_polygon.buffer(tolerance)
        for segment in segments:
            line = LineString(
                [
                    (segment.start.x, segment.start.y),
                    (segment.end.x, segment.end.y),
                ]
            )
            if line.length <= tolerance:
                raise PlanningError(
                    "PLANNING_FAILED",
                    f"generated segment {segment.swath_index} is degenerate",
                )
            if not permitted.covers(line):
                raise PlanningError(
                    "PLANNING_FAILED",
                    (
                        f"generated segment {segment.swath_index} leaves "
                        "the robot-center-safe area"
                    ),
                )

    def _transitions(self, segments, component_index):
        transitions = []
        for previous, current in zip(segments, segments[1:]):
            transition = self._transition_between(
                previous, current, component_index)
            if transition is not None:
                transitions.append(transition)
        return transitions

    def _add_cross_component_transitions(self, components):
        for index in range(1, len(components)):
            previous = components[index - 1]
            current = components[index]
            previous_segments = (
                previous.edge_segments + previous.coverage_segments)
            current_segments = current.edge_segments + current.coverage_segments
            transition = self._transition_between(
                previous_segments[-1], current_segments[0], current.index)
            if transition is not None:
                components[index] = replace(
                    current,
                    transition_segments=(
                        transition,
                    ) + current.transition_segments,
                )

    def _transition_between(self, previous, current, component_index):
        distance = hypot(
            current.start.x - previous.end.x,
            current.start.y - previous.end.y,
        )
        if distance <= self._infrastructure.geometry_epsilon:
            return None
        return CleaningSegment(
            segment_type=SegmentType.TRANSITION,
            component_index=component_index,
            boundary_index=-1,
            swath_index=-1,
            start=previous.end,
            end=current.start,
        )

    @staticmethod
    def _cleaned_geometry(segments, op_width):
        return unary_union(
            [
                LineString(
                    [(segment.start.x, segment.start.y), (segment.end.x, segment.end.y)]
                ).buffer(op_width / 2.0, cap_style=2, join_style=2)
                for segment in segments
                if segment.segment_type != SegmentType.TRANSITION
            ]
        )

    @staticmethod
    def _ring_to_polygon2d(coordinates):
        return Polygon2D.from_tuples(list(coordinates)[:-1])

    def _ordered_holes(self, polygon):
        holes = sorted(
            (Polygon(ring) for ring in polygon.interiors),
            key=lambda value: (value.centroid.x, value.centroid.y, value.area),
        )
        return tuple(
            self._ring_to_polygon2d(hole.exterior.coords) for hole in holes)

    @staticmethod
    def _polygons(geometry):
        if geometry.is_empty:
            return []
        if isinstance(geometry, Polygon):
            return [geometry]
        if isinstance(geometry, MultiPolygon):
            return list(geometry.geoms)
        return [
            value for value in getattr(geometry, "geoms", ())
            if isinstance(value, Polygon)
        ]

    @staticmethod
    def _checkpoint(cancel_check, deadline):
        if cancel_check is not None and cancel_check():
            raise PlanningError("CANCELED", "planning was canceled")
        if monotonic() > deadline:
            raise PlanningError("TIMEOUT", "planning exceeded its deadline")

    @staticmethod
    def _report(feedback, stage, progress):
        if feedback is not None:
            feedback(stage, progress)
