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

"""Bounded composite PNG output for successful cleaning plans."""

import os
from pathlib import Path
from threading import Lock
from time import time_ns
from uuid import uuid4

import cv2
import numpy as np

from .models import PlanningError


class ArtifactWriter:
    """Render one composite image and retain only owned, bounded artifacts."""

    _PREFIX = "cleaning_plan_"

    def __init__(
        self,
        directory: str,
        max_files: int = 20,
        max_total_bytes: int = 100 * 1024 * 1024,
        max_dimension: int = 1600,
    ) -> None:
        if max_files <= 0 or max_total_bytes <= 0 or max_dimension < 64:
            raise ValueError("artifact limits must be positive and usable")
        self._directory = Path(directory)
        self._max_files = max_files
        self._max_total_bytes = max_total_bytes
        self._max_dimension = max_dimension
        self._lock = Lock()

    def write(self, result) -> str:
        with self._lock:
            self._directory.mkdir(parents=True, exist_ok=True)
            image, to_pixel = self._canvas(result)
            self._draw_components(image, result, to_pixel)
            cv2.putText(
                image,
                f"coverage: {result.coverage_ratio:.4f}",
                (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (20, 20, 20),
                1,
                cv2.LINE_AA,
            )

            stem = f"{self._PREFIX}{time_ns()}_{uuid4().hex[:8]}"
            final_path = self._directory / f"{stem}.png"
            temporary_path = self._directory / f".{stem}.tmp.png"
            if not cv2.imwrite(str(temporary_path), image):
                raise PlanningError(
                    "INTERNAL_ERROR", "failed to encode cleaning-plan PNG")
            os.replace(temporary_path, final_path)
            self._prune()
            if not final_path.exists():
                raise PlanningError(
                    "INTERNAL_ERROR",
                    "artifact limits are too small for a single cleaning-plan PNG",
                )
            return str(final_path)

    def _canvas(self, result):
        coordinates = []
        for component in result.components:
            coordinates.extend(
                (point.x, point.y) for point in component.outer.vertices)
            for exclusion in component.exclusions:
                coordinates.extend(
                    (point.x, point.y) for point in exclusion.vertices)
        if not coordinates:
            raise PlanningError("INTERNAL_ERROR", "cannot render an empty plan")

        min_x = min(value[0] for value in coordinates)
        max_x = max(value[0] for value in coordinates)
        min_y = min(value[1] for value in coordinates)
        max_y = max(value[1] for value in coordinates)
        span_x = max(max_x - min_x, 1.0e-6)
        span_y = max(max_y - min_y, 1.0e-6)
        margin = 36
        usable = self._max_dimension - 2 * margin
        if span_x >= span_y:
            width = self._max_dimension
            height = max(96, int(round(usable * span_y / span_x)) + 2 * margin)
        else:
            height = self._max_dimension
            width = max(96, int(round(usable * span_x / span_y)) + 2 * margin)
        scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

        def to_pixel(x, y):
            pixel_x = int(round(margin + (x - min_x) * scale))
            pixel_y = int(round(height - margin - (y - min_y) * scale))
            return pixel_x, pixel_y

        return np.full((height, width, 3), 255, dtype=np.uint8), to_pixel

    @staticmethod
    def _polyline(points, to_pixel):
        return np.asarray(
            [to_pixel(point.x, point.y) for point in points], dtype=np.int32)

    def _draw_components(self, image, result, to_pixel):
        for component in result.components:
            outer = self._polyline(component.outer.vertices, to_pixel)
            cv2.fillPoly(image, [outer], (242, 242, 242))
            cv2.polylines(image, [outer], True, (80, 80, 80), 2, cv2.LINE_AA)
            for exclusion in component.exclusions:
                points = self._polyline(exclusion.vertices, to_pixel)
                cv2.fillPoly(image, [points], (210, 210, 210))
                cv2.polylines(
                    image, [points], True, (80, 80, 80), 2, cv2.LINE_AA)
            for segment in component.edge_segments:
                cv2.line(
                    image,
                    to_pixel(segment.start.x, segment.start.y),
                    to_pixel(segment.end.x, segment.end.y),
                    (220, 90, 20),
                    2,
                    cv2.LINE_AA,
                )
            for segment in component.coverage_segments:
                cv2.line(
                    image,
                    to_pixel(segment.start.x, segment.start.y),
                    to_pixel(segment.end.x, segment.end.y),
                    (30, 160, 50),
                    2,
                    cv2.LINE_AA,
                )
            for segment in component.transition_segments:
                cv2.line(
                    image,
                    to_pixel(segment.start.x, segment.start.y),
                    to_pixel(segment.end.x, segment.end.y),
                    (160, 160, 160),
                    1,
                    cv2.LINE_AA,
                )

    def _prune(self):
        files = sorted(
            self._directory.glob(f"{self._PREFIX}*.png"),
            key=lambda path: (path.stat().st_mtime_ns, path.name),
            reverse=True,
        )
        total = 0
        for index, path in enumerate(files):
            size = path.stat().st_size
            if index >= self._max_files or total + size > self._max_total_bytes:
                path.unlink()
            else:
                total += size
