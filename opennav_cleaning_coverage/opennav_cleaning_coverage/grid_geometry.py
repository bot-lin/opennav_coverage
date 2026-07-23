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

"""Exact, compressed Shapely geometry construction from costmap cells."""

from shapely.geometry import GeometryCollection, box
from shapely.ops import unary_union


def union_selected_cells(grid, predicate, checkpoint=None):
    """Union selected cells after merging equal horizontal runs vertically."""
    active_runs = {}
    rectangles = []
    for row in range(grid.height):
        if checkpoint is not None:
            checkpoint()
        row_runs = []
        start = None
        for column in range(grid.width + 1):
            selected = (
                column < grid.width
                and predicate(grid.data[row * grid.width + column])
            )
            if selected and start is None:
                start = column
            elif not selected and start is not None:
                row_runs.append((start, column))
                start = None

        current_runs = set(row_runs)
        for run, start_row in active_runs.items():
            if run not in current_runs:
                rectangles.append(
                    _rectangle(grid, run, start_row, row))
        active_runs = {
            run: active_runs.get(run, row) for run in row_runs
        }

    for run, start_row in active_runs.items():
        rectangles.append(
            _rectangle(grid, run, start_row, grid.height))
    if not rectangles:
        return GeometryCollection()
    if checkpoint is not None:
        checkpoint()
    geometry = unary_union(rectangles)
    if checkpoint is not None:
        checkpoint()
    return geometry


def _rectangle(grid, run, start_row, end_row):
    return box(
        run[0] * grid.resolution,
        start_row * grid.resolution,
        run[1] * grid.resolution,
        end_row * grid.resolution,
    )
