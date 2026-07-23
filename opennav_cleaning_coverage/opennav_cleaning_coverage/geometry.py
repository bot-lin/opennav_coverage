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

"""Strict construction of the effective robot-center planning area."""

from math import hypot, isfinite
from typing import List, Sequence

from shapely import affinity
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union
from shapely.validation import explain_validity

from .models import GeometryMode, GridMap, PlanRequest, PlanningError, Polygon2D


class EffectiveAreaBuilder:
    """Build validated, deterministically ordered planning components."""

    def __init__(
        self,
        geometry_epsilon: float = 1.0e-6,
        min_obstacle_area=None,
        min_cleanable_area=None,
    ) -> None:
        self._epsilon = geometry_epsilon
        self._min_obstacle_area = min_obstacle_area
        self._min_cleanable_area = min_cleanable_area

    def build(self, request: PlanRequest, costmap=None) -> List[Polygon]:
        if request.op_width <= 0.0 or not isfinite(request.op_width):
            raise PlanningError("INVALID_PARAMETERS", "op_width must be finite and positive")

        roi = self._to_strict_polygon(request.roi, "ROI")
        exclusions = [
            self._to_strict_polygon(value, f"exclusion {index}")
            for index, value in enumerate(request.exclusions)
        ]

        if request.geometry_mode == GeometryMode.COSTMAP and exclusions:
            raise PlanningError(
                "INVALID_PARAMETERS",
                "COSTMAP mode does not accept explicit exclusion polygons",
            )
        if request.geometry_mode != GeometryMode.USER and costmap is None:
            raise PlanningError(
                "COSTMAP_UNAVAILABLE",
                "a costmap is required for COSTMAP and FUSED geometry modes",
            )

        for index, exclusion in enumerate(exclusions):
            if not roi.contains(exclusion):
                raise PlanningError(
                    "INVALID_GEOMETRY",
                    f"exclusion {index} must be strictly inside the ROI",
                )
            for other_index in range(index):
                if exclusion.intersects(exclusions[other_index]):
                    raise PlanningError(
                        "INVALID_GEOMETRY",
                        (
                            f"exclusions {other_index} and {index} overlap "
                            "or touch"
                        ),
                    )

        if request.geometry_mode == GeometryMode.USER:
            effective = roi
        else:
            traversable = self._costmap_traversable(
                costmap, request.max_traversable_cost, request.op_width)
            effective = roi.intersection(traversable)
        if exclusions:
            effective = effective.difference(unary_union(exclusions))
        polygons = self._sorted_polygons(effective)
        minimum_cleanable_area = self._min_cleanable_area
        if minimum_cleanable_area is None:
            minimum_cleanable_area = 0.5 * request.op_width * request.op_width
            if costmap is not None:
                minimum_cleanable_area = max(
                    minimum_cleanable_area,
                    costmap.resolution * costmap.resolution,
                )
        polygons = [
            polygon
            for polygon in polygons
            if polygon.area + self._epsilon >= minimum_cleanable_area
        ]
        if not polygons:
            raise PlanningError(
                "NO_CLEANABLE_AREA",
                "all cleanable components are below the minimum area",
            )
        return polygons

    def _costmap_traversable(
        self, grid: GridMap, max_cost: int, op_width: float
    ):
        if grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0.0:
            raise PlanningError("COSTMAP_UNAVAILABLE", "costmap metadata is invalid")
        if len(grid.data) != grid.width * grid.height:
            raise PlanningError("COSTMAP_UNAVAILABLE", "costmap data size is invalid")
        if max_cost < 0 or max_cost >= grid.unknown_value:
            raise PlanningError(
                "INVALID_PARAMETERS",
                "max_traversable_cost must be below the unknown cost value",
            )

        known_obstacles = self._costmap_cell_union(
            grid,
            lambda value: value != grid.unknown_value and value > max_cost,
        )
        unknown = self._costmap_cell_union(
            grid, lambda value: value == grid.unknown_value)
        minimum_area = self._min_obstacle_area
        if minimum_area is None:
            minimum_area = max(
                grid.resolution * grid.resolution,
                0.5 * op_width * grid.resolution,
            )
        retained = [
            polygon.simplify(
                0.5 * grid.resolution, preserve_topology=True)
            for polygon in self._geometry_polygons(known_obstacles)
            if polygon.area + self._epsilon >= minimum_area
        ]
        blocked_values = retained
        if not unknown.is_empty:
            blocked_values.append(
                unknown.simplify(
                    0.5 * grid.resolution, preserve_topology=True))
        blocked = (
            unary_union(blocked_values) if blocked_values else GeometryCollection()
        )

        map_bounds = box(
            0.0,
            0.0,
            grid.width * grid.resolution,
            grid.height * grid.resolution,
        )
        traversable = map_bounds.difference(blocked)
        return self._transform_costmap_geometry(traversable, grid)

    def _costmap_cell_union(self, grid, predicate):
        runs = []
        for row in range(grid.height):
            start = None
            for column in range(grid.width + 1):
                selected = False
                if column < grid.width:
                    value = grid.data[row * grid.width + column]
                    selected = predicate(value)
                if selected and start is None:
                    start = column
                elif not selected and start is not None:
                    runs.append(
                        box(
                            start * grid.resolution,
                            row * grid.resolution,
                            column * grid.resolution,
                            (row + 1) * grid.resolution,
                        )
                    )
                    start = None
        if not runs:
            return GeometryCollection()
        return unary_union(runs)

    @staticmethod
    def _transform_costmap_geometry(geometry, grid):
        if grid.origin_yaw:
            geometry = affinity.rotate(
                geometry, grid.origin_yaw, origin=(0.0, 0.0), use_radians=True)
        return affinity.translate(
            geometry, xoff=grid.origin_x, yoff=grid.origin_y)

    @staticmethod
    def _geometry_polygons(geometry):
        if isinstance(geometry, Polygon):
            return [geometry]
        if isinstance(geometry, MultiPolygon):
            return list(geometry.geoms)
        return [
            value
            for value in getattr(geometry, "geoms", ())
            if isinstance(value, Polygon)
        ]

    def _to_strict_polygon(self, value: Polygon2D, label: str) -> Polygon:
        coordinates = self._normalized_coordinates(value, label)
        polygon = Polygon(coordinates)
        if polygon.is_empty or polygon.area <= self._epsilon * self._epsilon:
            raise PlanningError("INVALID_GEOMETRY", f"{label} has no usable area")
        if not polygon.is_valid:
            raise PlanningError(
                "INVALID_GEOMETRY",
                f"{label} is invalid: {explain_validity(polygon)}",
            )
        return orient(polygon, sign=1.0)

    def _normalized_coordinates(self, value: Polygon2D, label: str):
        normalized = []
        for point in value.vertices:
            if not isfinite(point.x) or not isfinite(point.y):
                raise PlanningError(
                    "INVALID_GEOMETRY", f"{label} contains non-finite coordinates")
            if normalized:
                previous = normalized[-1]
                if hypot(point.x - previous[0], point.y - previous[1]) <= self._epsilon:
                    continue
            normalized.append((point.x, point.y))

        if len(normalized) > 1:
            if hypot(
                normalized[0][0] - normalized[-1][0],
                normalized[0][1] - normalized[-1][1],
            ) <= self._epsilon:
                normalized.pop()
        if len(normalized) < 3:
            raise PlanningError(
                "INVALID_GEOMETRY", f"{label} needs at least three distinct vertices")
        for index, coordinate in enumerate(normalized):
            following = normalized[(index + 1) % len(normalized)]
            if hypot(
                coordinate[0] - following[0], coordinate[1] - following[1]
            ) <= self._epsilon:
                raise PlanningError("INVALID_GEOMETRY", f"{label} has a near-zero edge")
        return normalized

    @staticmethod
    def _sorted_polygons(geometry) -> List[Polygon]:
        if isinstance(geometry, Polygon):
            polygons: Sequence[Polygon] = [geometry]
        elif isinstance(geometry, MultiPolygon):
            polygons = list(geometry.geoms)
        elif isinstance(geometry, GeometryCollection):
            polygons = [
                value for value in geometry.geoms if isinstance(value, Polygon)]
        else:
            polygons = []
        polygons = [orient(value, sign=1.0) for value in polygons if not value.is_empty]
        polygons.sort(key=lambda value: (-value.area, value.centroid.x, value.centroid.y))
        if not polygons:
            raise PlanningError("NO_CLEANABLE_AREA", "the effective area is empty")
        return polygons
