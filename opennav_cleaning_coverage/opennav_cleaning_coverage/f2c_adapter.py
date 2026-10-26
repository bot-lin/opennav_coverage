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

"""Narrow, tested adapter around the Fields2Cover v2 Python binding."""

from math import atan2, hypot, isfinite, radians
from typing import List

import fields2cover as f2c
from shapely.geometry import Polygon
from shapely.validation import explain_validity

from .models import (
    AngleMode,
    CleaningSegment,
    DecompositionMode,
    Objective,
    PlanRequest,
    PlanningError,
    Pose2D,
    SegmentType,
)


class Fields2CoverSwathGenerator:
    """Generate and order endpoint-only swaths for one effective component."""

    def generate(
        self,
        polygon: Polygon,
        request: PlanRequest,
        component_index: int,
        checkpoint=None,
    ) -> List[CleaningSegment]:
        try:
            return self._generate(
                polygon, request, component_index, checkpoint)
        except PlanningError:
            raise
        except Exception as error:
            raise PlanningError(
                "PLANNING_FAILED", f"Fields2Cover failed: {error}") from error

    def _generate(
        self,
        polygon: Polygon,
        request: PlanRequest,
        component_index: int,
        checkpoint=None,
    ) -> List[CleaningSegment]:
        checkpoint = checkpoint or (lambda: None)
        if not isfinite(request.fixed_angle_deg):
            raise PlanningError("INVALID_PARAMETERS", "fixed_angle_deg must be finite")

        checkpoint()
        cells = self._to_cells(polygon)
        cells = self._decompose(cells, polygon, request.decomposition)
        checkpoint()
        if request.headland_width > 0.0:
            cells = f2c.HG_Const_gen().generateHeadlands(
                cells, request.headland_width)
            checkpoint()

        generator = f2c.SG_BruteForce()
        generator.setAllowOverlap(False)
        if request.angle_mode == AngleMode.FIXED:
            angle = radians(request.fixed_angle_deg % 180.0)
            swaths_by_cell = generator.generateSwaths(
                angle, request.op_width, cells)
        elif request.angle_mode == AngleMode.AUTO:
            swaths_by_cell = self._best_swaths(
                generator, cells, request, checkpoint)
        else:
            raise PlanningError("INVALID_PARAMETERS", "angle_mode is invalid")

        segments = []
        swath_index = 0
        checkpoint()
        route = f2c.RP_RoutePlannerBase().genRoute(cells, swaths_by_cell)
        ordered_groups = route.getVectorSwaths()
        for cell_index in range(ordered_groups.size()):
            checkpoint()
            ordered = ordered_groups[cell_index]
            for index in range(ordered.size()):
                swath = ordered.at(index)
                start = swath.startPoint()
                end = swath.endPoint()
                yaw = atan2(end.Y() - start.Y(), end.X() - start.X())
                segments.append(
                    CleaningSegment(
                        segment_type=SegmentType.COVERAGE,
                        component_index=component_index,
                        boundary_index=-1,
                        swath_index=swath_index,
                        start=Pose2D(start.X(), start.Y(), yaw),
                        end=Pose2D(end.X(), end.Y(), yaw),
                    )
                )
                swath_index += 1
        if not segments:
            raise PlanningError(
                "PLANNING_FAILED", "Fields2Cover generated no coverage swaths")
        return segments

    def _best_swaths(self, generator, cells, request, checkpoint):
        if (
            not isfinite(request.angle_step_deg)
            or request.angle_step_deg <= 0.0
            or request.angle_step_deg >= 180.0
        ):
            raise PlanningError(
                "INVALID_PARAMETERS", "angle_step_deg must be in (0, 180)")

        candidates = []
        angle_deg = 0.0
        while angle_deg < 180.0 - 1.0e-9:
            checkpoint()
            swaths = generator.generateSwaths(
                radians(angle_deg), request.op_width, cells)
            count, length = self._swath_metrics(swaths)
            if count:
                turns = max(0, count - swaths.size())
                score = self._objective_score(
                    request.objective, turns, length, angle_deg)
                candidates.append((score, swaths))
            angle_deg += request.angle_step_deg
        if not candidates:
            raise PlanningError(
                "PLANNING_FAILED", "Fields2Cover found no valid swath angle")
        candidates.sort(key=lambda value: value[0])
        return candidates[0][1]

    @staticmethod
    def _objective_score(objective, turns, length, angle_deg):
        if objective == Objective.MIN_LENGTH:
            return length, angle_deg
        if objective == Objective.MIN_TURNS:
            return turns, angle_deg
        if objective == Objective.BALANCED:
            return turns, length, angle_deg
        raise PlanningError(
            "INVALID_PARAMETERS", "optimization objective is invalid")

    @staticmethod
    def _swath_metrics(swaths_by_cell):
        count = 0
        length = 0.0
        for cell_index in range(swaths_by_cell.size()):
            swaths = swaths_by_cell[cell_index]
            count += swaths.size()
            for index in range(swaths.size()):
                swath = swaths.at(index)
                start = swath.startPoint()
                end = swath.endPoint()
                length += hypot(end.X() - start.X(), end.Y() - start.Y())
        return count, length

    @staticmethod
    def _decompose(cells, polygon, mode):
        if mode == DecompositionMode.AUTO:
            is_convex_simple = (
                not polygon.interiors and polygon.equals(polygon.convex_hull))
            mode = (
                DecompositionMode.NONE
                if is_convex_simple
                else DecompositionMode.BOUSTROPHEDON
            )
        if mode == DecompositionMode.NONE:
            return cells
        if mode == DecompositionMode.BOUSTROPHEDON:
            return f2c.DECOMP_Boustrophedon().decompose(cells)
        if mode == DecompositionMode.TRAPEZOIDAL:
            return f2c.DECOMP_TrapezoidalDecomp().decompose(cells)
        raise PlanningError("INVALID_PARAMETERS", "decomposition mode is invalid")

    @staticmethod
    def _to_cells(polygon: Polygon):
        if not isinstance(polygon, Polygon):
            raise PlanningError("INVALID_GEOMETRY", "coverage geometry must be a polygon")
        if not polygon.is_valid:
            raise PlanningError(
                "INVALID_GEOMETRY",
                f"coverage polygon is invalid: {explain_validity(polygon)}",
            )
        if polygon.is_empty or not isfinite(polygon.area) or polygon.area <= 0.0:
            raise PlanningError(
                "INVALID_GEOMETRY", "coverage polygon must have finite positive area")
        exterior = [
            f2c.Point(float(x), float(y)) for x, y in polygon.exterior.coords]
        cell = f2c.Cell(f2c.LinearRing(f2c.VectorPoint(exterior)))
        for interior in polygon.interiors:
            points = [f2c.Point(float(x), float(y)) for x, y in interior.coords]
            cell.addRing(f2c.LinearRing(f2c.VectorPoint(points)))
        return f2c.Cells(cell)
