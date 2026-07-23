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

"""Final swept-footprint checks against lethal and unknown costmap cells."""

from math import cos, sin

from shapely import affinity
from shapely.geometry import GeometryCollection, Point, Polygon, box
from shapely.ops import unary_union

from .models import PlanningError


class FootprintCollisionChecker:
    """Validate straight cleaning strokes without applying inflation twice."""

    def __init__(
        self,
        footprint,
        costmap,
        lethal_cost: int = 254,
        epsilon: float = 1.0e-6,
    ) -> None:
        self._epsilon = epsilon
        self._coordinates = [
            (point.x, point.y) for point in footprint.vertices]
        footprint_polygon = Polygon(self._coordinates)
        if (
            footprint_polygon.is_empty
            or not footprint_polygon.is_valid
            or footprint_polygon.area <= epsilon * epsilon
            or not footprint_polygon.covers(Point(0.0, 0.0))
        ):
            raise PlanningError(
                "FOOTPRINT_UNAVAILABLE",
                "Nav2 footprint must be a valid polygon containing base_link",
            )
        self._validate_costmap(costmap)
        self._map_bounds = self._transform(
            box(
                0.0,
                0.0,
                costmap.width * costmap.resolution,
                costmap.height * costmap.resolution,
            ),
            costmap,
        )
        hard_cells = self._selected_cells(
            costmap, lambda value: value >= lethal_cost)
        self._hard_obstacles = self._transform(hard_cells, costmap)

    def validate(self, segments) -> None:
        for segment in segments:
            swept = self._swept_polygon(segment)
            if not self._map_bounds.buffer(self._epsilon).covers(swept):
                raise PlanningError(
                    "PLANNING_FAILED",
                    (
                        f"segment {segment.swath_index} moves the footprint "
                        "outside the global costmap"
                    ),
                )
            if (
                not self._hard_obstacles.is_empty
                and swept.intersects(self._hard_obstacles)
            ):
                raise PlanningError(
                    "PLANNING_FAILED",
                    (
                        f"segment {segment.swath_index} sweeps the footprint "
                        "through a lethal or unknown costmap cell"
                    ),
                )

    def _swept_polygon(self, segment):
        start_coordinates = self._pose_coordinates(
            segment.start.x, segment.start.y, segment.start.yaw)
        end_coordinates = self._pose_coordinates(
            segment.end.x, segment.end.y, segment.end.yaw)
        polygons = [Polygon(start_coordinates), Polygon(end_coordinates)]
        for index in range(len(self._coordinates)):
            following = (index + 1) % len(self._coordinates)
            strip = Polygon(
                [
                    start_coordinates[index],
                    start_coordinates[following],
                    end_coordinates[following],
                    end_coordinates[index],
                ]
            )
            if strip.area > self._epsilon * self._epsilon:
                polygons.append(strip)
        return unary_union(polygons)

    def _pose_coordinates(self, x, y, yaw):
        cosine = cos(yaw)
        sine = sin(yaw)
        return [
            (
                cosine * local_x - sine * local_y + x,
                sine * local_x + cosine * local_y + y,
            )
            for local_x, local_y in self._coordinates
        ]

    @staticmethod
    def _validate_costmap(costmap):
        if (
            costmap.width <= 0
            or costmap.height <= 0
            or costmap.resolution <= 0.0
            or len(costmap.data) != costmap.width * costmap.height
        ):
            raise PlanningError(
                "COSTMAP_UNAVAILABLE",
                "costmap metadata is invalid during footprint validation",
            )

    @staticmethod
    def _selected_cells(costmap, predicate):
        cells = []
        for row in range(costmap.height):
            for column in range(costmap.width):
                value = costmap.data[row * costmap.width + column]
                if predicate(value):
                    cells.append(
                        box(
                            column * costmap.resolution,
                            row * costmap.resolution,
                            (column + 1) * costmap.resolution,
                            (row + 1) * costmap.resolution,
                        )
                    )
        return unary_union(cells) if cells else GeometryCollection()

    @staticmethod
    def _transform(geometry, costmap):
        if costmap.origin_yaw:
            geometry = affinity.rotate(
                geometry,
                costmap.origin_yaw,
                origin=(0.0, 0.0),
                use_radians=True,
            )
        return affinity.translate(
            geometry, xoff=costmap.origin_x, yoff=costmap.origin_y)
