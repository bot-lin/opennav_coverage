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

"""Deterministic sparse edge-cleaning segment generation."""

from math import atan2, hypot
from typing import List

from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

from .models import CleaningSegment, Pose2D, SegmentType


class EdgeSegmentGenerator:
    """Convert polygon boundaries to endpoint-only cleaning segments."""

    def generate(
        self, polygon: Polygon, component_index: int
    ) -> List[CleaningSegment]:
        polygon = orient(polygon, sign=1.0)
        boundaries = [(0, polygon.exterior)]
        holes = sorted(
            (Polygon(ring) for ring in polygon.interiors),
            key=lambda value: (value.centroid.x, value.centroid.y, value.area),
        )
        boundaries.extend((index + 1, hole.exterior) for index, hole in enumerate(holes))

        segments = []
        for boundary_index, ring in boundaries:
            coordinates = self._rotate_to_deterministic_start(list(ring.coords)[:-1])
            for index, start in enumerate(coordinates):
                end = coordinates[(index + 1) % len(coordinates)]
                if hypot(end[0] - start[0], end[1] - start[1]) <= 1.0e-6:
                    continue
                yaw = atan2(end[1] - start[1], end[0] - start[0])
                segments.append(
                    CleaningSegment(
                        segment_type=SegmentType.EDGE,
                        component_index=component_index,
                        boundary_index=boundary_index,
                        swath_index=-1,
                        start=Pose2D(start[0], start[1], yaw),
                        end=Pose2D(end[0], end[1], yaw),
                    )
                )
        return segments

    @staticmethod
    def _rotate_to_deterministic_start(coordinates):
        start_index = min(
            range(len(coordinates)),
            key=lambda index: (coordinates[index][0], coordinates[index][1]),
        )
        return coordinates[start_index:] + coordinates[:start_index]
