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

"""ROS-independent domain models for cleaning coverage planning."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable, Optional, Tuple


class GeometryMode(IntEnum):
    USER = 0
    COSTMAP = 1
    FUSED = 2


class AngleMode(IntEnum):
    FIXED = 0
    AUTO = 1


class Objective(IntEnum):
    MIN_TURNS = 0
    MIN_LENGTH = 1
    BALANCED = 2


class DecompositionMode(IntEnum):
    AUTO = 0
    NONE = 1
    BOUSTROPHEDON = 2
    TRAPEZOIDAL = 3


class SegmentType(IntEnum):
    EDGE = 1
    COVERAGE = 2
    TRANSITION = 3


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Polygon2D:
    vertices: Tuple[Point2D, ...]

    @classmethod
    def from_tuples(cls, vertices: Iterable[Tuple[float, float]]) -> "Polygon2D":
        return cls(tuple(Point2D(float(x), float(y)) for x, y in vertices))


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Tuple[int, ...]
    unknown_value: int = 255


@dataclass(frozen=True)
class PlanRequest:
    roi: Polygon2D
    op_width: float
    exclusions: Tuple[Polygon2D, ...] = ()
    geometry_mode: GeometryMode = GeometryMode.USER
    angle_mode: AngleMode = AngleMode.AUTO
    fixed_angle_deg: float = 0.0
    angle_step_deg: float = 5.0
    objective: Objective = Objective.MIN_TURNS
    decomposition: DecompositionMode = DecompositionMode.AUTO
    headland_width: float = 0.0
    max_traversable_cost: int = 0
    min_coverage_ratio: float = 0.98


@dataclass(frozen=True)
class CleaningSegment:
    segment_type: SegmentType
    component_index: int
    boundary_index: int
    swath_index: int
    start: Pose2D
    end: Pose2D


@dataclass(frozen=True)
class PlanComponent:
    index: int
    outer: Polygon2D
    exclusions: Tuple[Polygon2D, ...]
    edge_segments: Tuple[CleaningSegment, ...]
    coverage_segments: Tuple[CleaningSegment, ...]
    transition_segments: Tuple[CleaningSegment, ...]
    ordered_poses: Tuple[Pose2D, ...]
    coverage_ratio: float


@dataclass(frozen=True)
class PlanResult:
    components: Tuple[PlanComponent, ...]
    coverage_ratio: float
    uncovered_regions: Tuple[Polygon2D, ...] = ()
    debug_image_path: str = ""
    planning_duration_sec: float = 0.0


class PlanningError(RuntimeError):
    """Expected planning failure that maps to an action result error code."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class PlannerInfrastructure:
    """Non-goal settings controlled by the lifecycle node."""

    geometry_epsilon: float = 1.0e-6
    min_obstacle_area: Optional[float] = None
    min_cleanable_area: Optional[float] = None
    planning_timeout_sec: float = 30.0
    debug_output_directory: str = ""
    debug_max_files: int = 20
    debug_max_total_bytes: int = 100 * 1024 * 1024
    debug_max_dimension: int = 1600
    publish_markers: bool = False
