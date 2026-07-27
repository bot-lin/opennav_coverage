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

"""Lifecycle ROS 2 action server for indoor cleaning coverage planning."""

from copy import deepcopy
from dataclasses import replace
from math import atan2, cos, sin
from threading import Event, Lock
from time import monotonic

import rclpy
from geometry_msgs.msg import Point, PolygonStamped, PoseStamped
from nav2_msgs.srv import GetCostmap
from opennav_cleaning_coverage_msgs.action import ComputeCleaningCoverage
from opennav_cleaning_coverage_msgs.msg import (
    CleaningPlanComponent as CleaningPlanComponentMsg,
)
from opennav_cleaning_coverage_msgs.msg import CleaningSegment as CleaningSegmentMsg
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .artifacts import ArtifactWriter
from .geometry import EffectiveAreaBuilder
from .models import (
    GeometryMode,
    GridMap,
    PlannerInfrastructure,
    PlanningError,
    Polygon2D,
    SegmentType,
)
from .planner import CleaningCoveragePlanner
from .ros_conversions import goal_to_request


class CleaningCoverageServer(LifecycleNode):
    """Lifecycle-managed, single-goal cleaning coverage action server."""

    def __init__(self, parameter_overrides=None) -> None:
        super().__init__(
            "cleaning_coverage_server",
            parameter_overrides=parameter_overrides or [],
        )
        self.declare_parameter("action_name", "/compute_cleaning_coverage")
        self.declare_parameter("planning_frame", "map")
        self.declare_parameter(
            "costmap_service", "/global_costmap/get_costmap")
        self.declare_parameter(
            "footprint_topic", "/global_costmap/published_footprint")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.5)
        self.declare_parameter("costmap_timeout_sec", 5.0)
        self.declare_parameter("planning_timeout_sec", 30.0)
        self.declare_parameter("default_angle_step_deg", 5.0)
        self.declare_parameter("default_min_coverage_ratio", 0.98)
        self.declare_parameter("geometry_epsilon", 1.0e-6)
        self.declare_parameter("min_obstacle_area", -1.0)
        self.declare_parameter("min_cleanable_area", -1.0)
        self.declare_parameter(
            "debug_output_directory", "/tmp/opennav_cleaning_coverage")
        self.declare_parameter("debug_max_files", 20)
        self.declare_parameter("debug_max_total_mib", 100.0)
        self.declare_parameter("debug_max_dimension", 1600)
        self.declare_parameter("publish_markers", False)

        self._active = False
        self._goal_lock = Lock()
        self._goal_in_progress = False
        self._footprint_lock = Lock()
        self._latest_footprint = None
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = None
        self._costmap_client = None
        self._footprint_subscription = None
        self._marker_publisher = None
        self._tf_buffer = None
        self._tf_listener = None
        self._planner = None
        self._artifact_writer = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        del state
        try:
            infrastructure = self._infrastructure()
            min_obstacle_area = self.get_parameter(
                "min_obstacle_area").value
            min_cleanable_area = self.get_parameter(
                "min_cleanable_area").value
            area_builder = EffectiveAreaBuilder(
                geometry_epsilon=infrastructure.geometry_epsilon,
                min_obstacle_area=(
                    None if min_obstacle_area < 0.0 else min_obstacle_area),
                min_cleanable_area=(
                    None if min_cleanable_area < 0.0 else min_cleanable_area),
            )
            self._planner = CleaningCoveragePlanner(
                infrastructure=infrastructure, area_builder=area_builder)
            self._artifact_writer = ArtifactWriter(
                directory=infrastructure.debug_output_directory,
                max_files=infrastructure.debug_max_files,
                max_total_bytes=infrastructure.debug_max_total_bytes,
                max_dimension=infrastructure.debug_max_dimension,
            )
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._costmap_client = self.create_client(
                GetCostmap,
                self.get_parameter("costmap_service").value,
                callback_group=self._callback_group,
            )
            footprint_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._footprint_subscription = self.create_subscription(
                PolygonStamped,
                self.get_parameter("footprint_topic").value,
                self._footprint_callback,
                footprint_qos,
                callback_group=self._callback_group,
            )
            if infrastructure.publish_markers:
                marker_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self._marker_publisher = self.create_publisher(
                    MarkerArray, "cleaning_coverage_markers", marker_qos)
            self._action_server = ActionServer(
                self,
                ComputeCleaningCoverage,
                self.get_parameter("action_name").value,
                execute_callback=self._execute,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._callback_group,
            )
        except Exception as error:  # Lifecycle must report configuration failure.
            self.get_logger().error(f"configuration failed: {error}")
            return TransitionCallbackReturn.FAILURE
        self.get_logger().info("cleaning coverage server configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = True
        self.get_logger().info("cleaning coverage server active")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False
        self.get_logger().info("cleaning coverage server inactive")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        del state
        self._active = False
        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None
        if self._footprint_subscription is not None:
            self.destroy_subscription(self._footprint_subscription)
            self._footprint_subscription = None
        if self._costmap_client is not None:
            self.destroy_client(self._costmap_client)
            self._costmap_client = None
        if self._marker_publisher is not None:
            self.destroy_publisher(self._marker_publisher)
            self._marker_publisher = None
        self._tf_listener = None
        self._tf_buffer = None
        self._planner = None
        self._artifact_writer = None
        with self._footprint_lock:
            self._latest_footprint = None
        self.get_logger().info("cleaning coverage server cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def _infrastructure(self):
        return PlannerInfrastructure(
            geometry_epsilon=self.get_parameter("geometry_epsilon").value,
            min_obstacle_area=(
                None
                if self.get_parameter("min_obstacle_area").value < 0.0
                else self.get_parameter("min_obstacle_area").value
            ),
            min_cleanable_area=(
                None
                if self.get_parameter("min_cleanable_area").value < 0.0
                else self.get_parameter("min_cleanable_area").value
            ),
            planning_timeout_sec=self.get_parameter(
                "planning_timeout_sec").value,
            debug_output_directory=self.get_parameter(
                "debug_output_directory").value,
            debug_max_files=self.get_parameter("debug_max_files").value,
            debug_max_total_bytes=int(
                self.get_parameter("debug_max_total_mib").value
                * 1024
                * 1024
            ),
            debug_max_dimension=self.get_parameter(
                "debug_max_dimension").value,
            publish_markers=self.get_parameter("publish_markers").value,
        )

    def _goal_callback(self, _goal_request):
        with self._goal_lock:
            if not self._active:
                self.get_logger().warning(
                    "rejecting cleaning coverage goal: server is inactive")
                return GoalResponse.REJECT
            if self._goal_in_progress:
                self.get_logger().warning(
                    "rejecting cleaning coverage goal: another goal is active")
                return GoalResponse.REJECT
            self._goal_in_progress = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().info("cleaning coverage goal cancellation requested")
        return CancelResponse.ACCEPT

    def _footprint_callback(self, message):
        with self._footprint_lock:
            self._latest_footprint = deepcopy(message)

    def _execute(self, goal_handle):
        started = monotonic()
        deadline = started + self.get_parameter("planning_timeout_sec").value
        result_message = ComputeCleaningCoverage.Result()
        try:
            raw_request = goal_handle.request
            self.get_logger().info(
                "cleaning coverage goal received: "
                f"geometry_mode={raw_request.geometry_mode} "
                f"roi_frame={raw_request.roi.header.frame_id or '<default>'} "
                f"roi_vertices={len(raw_request.roi.polygon.points)} "
                f"exclusions={len(raw_request.exclusions)}"
            )
            request = goal_to_request(
                raw_request,
                self._polygon_to_planning_frame,
                default_angle_step_deg=self.get_parameter(
                    "default_angle_step_deg").value,
                default_min_coverage_ratio=self.get_parameter(
                    "default_min_coverage_ratio").value,
            )
            self.get_logger().info(
                "cleaning coverage parameters: "
                f"op_width={request.op_width:.3f} "
                f"angle_mode={int(request.angle_mode)} "
                f"fixed_angle_deg={request.fixed_angle_deg:.3f} "
                f"angle_step_deg={request.angle_step_deg:.3f} "
                f"objective={int(request.objective)} "
                f"decomposition={int(request.decomposition)} "
                f"headland_width={request.headland_width:.3f} "
                f"max_traversable_cost={request.max_traversable_cost} "
                f"min_coverage_ratio={request.min_coverage_ratio:.6f}"
            )
            footprint = self._current_footprint()
            self.get_logger().debug(
                f"using Nav2 footprint with {len(footprint.vertices)} vertices")
            costmap = None
            if request.geometry_mode != GeometryMode.USER:
                self._publish_feedback(
                    goal_handle, "ACQUIRING_COSTMAP", 0.05)
                costmap = self._request_costmap(goal_handle, deadline)
                self.get_logger().info(
                    "global costmap acquired: "
                    f"size={costmap.width}x{costmap.height} "
                    f"resolution={costmap.resolution:.4f} "
                    f"origin=({costmap.origin_x:.3f},"
                    f"{costmap.origin_y:.3f},"
                    f"{costmap.origin_yaw:.3f})"
                )

            def feedback(stage, progress):
                self.get_logger().debug(
                    f"planning stage={stage} progress={progress:.3f}")
                self._publish_feedback(goal_handle, stage, progress)

            result = self._planner.plan(
                request,
                costmap=costmap,
                footprint=footprint,
                feedback=feedback,
                cancel_check=lambda: goal_handle.is_cancel_requested,
                deadline=deadline,
            )
            self._checkpoint(goal_handle, deadline)
            self._publish_feedback(goal_handle, "WRITING_ARTIFACT", 0.95)
            image_path = self._artifact_writer.write(result)
            edge_image_path, coverage_image_path = (
                self._artifact_writer.separated_paths(image_path)
            )
            result = replace(
                result,
                debug_image_path=image_path,
                planning_duration_sec=monotonic() - started,
            )
            self._fill_result_message(result_message, result)
            self._publish_markers(result)
            goal_handle.succeed()
            result_message.error_code = result_message.SUCCESS
            result_message.message = "cleaning coverage plan computed"
            if result.skipped_edge_count:
                result_message.message += (
                    f"; skipped {result.skipped_edge_count} unsafe edge "
                    "segments"
                )
            if result.skipped_coverage_count:
                result_message.message += (
                    f"; skipped {result.skipped_coverage_count} unsafe "
                    "coverage segments"
                )
            edge_count = sum(
                len(component.edge_segments)
                for component in result.components
            )
            coverage_count = sum(
                len(component.coverage_segments)
                for component in result.components
            )
            transition_count = sum(
                len(component.transition_segments)
                for component in result.components
            )
            self.get_logger().info(
                "cleaning coverage goal succeeded: "
                f"components={len(result.components)} "
                f"edge_segments={edge_count} "
                f"coverage_segments={coverage_count} "
                f"transition_segments={transition_count} "
                f"skipped_edge_segments={result.skipped_edge_count} "
                "skipped_coverage_segments="
                f"{result.skipped_coverage_count} "
                f"coverage_ratio={result.coverage_ratio:.6f} "
                f"duration_sec={result.planning_duration_sec:.3f} "
                f"debug_image={result.debug_image_path or '<disabled>'} "
                f"edge_image={edge_image_path} "
                f"coverage_image={coverage_image_path}"
            )
            return result_message
        except PlanningError as error:
            elapsed = monotonic() - started
            partial = getattr(error, "result", None)
            if partial is not None:
                result_message.coverage_ratio = partial.coverage_ratio
                result_message.uncovered_regions = [
                    self._polygon_message(value)
                    for value in partial.uncovered_regions
                ]
            result_message.error_code = self._error_code(result_message, error.code)
            result_message.message = str(error)
            self._set_duration(result_message, elapsed)
            if error.code == "CANCELED" or goal_handle.is_cancel_requested:
                goal_handle.canceled()
                outcome = "canceled"
            else:
                goal_handle.abort()
                outcome = "failed"
            self.get_logger().warning(
                f"cleaning coverage goal {outcome}: "
                f"code={error.code} "
                f"message={error} "
                f"coverage_ratio={result_message.coverage_ratio:.6f} "
                f"duration_sec={elapsed:.3f}"
            )
            return result_message
        except Exception as error:  # Never leak an exception through an action.
            elapsed = monotonic() - started
            self.get_logger().error(
                "cleaning coverage goal failed unexpectedly: "
                f"type={type(error).__name__} "
                f"message={error} "
                f"duration_sec={elapsed:.3f}"
            )
            result_message.error_code = result_message.INTERNAL_ERROR
            result_message.message = str(error)
            self._set_duration(result_message, elapsed)
            goal_handle.abort()
            return result_message
        finally:
            with self._goal_lock:
                self._goal_in_progress = False

    def _current_footprint(self):
        with self._footprint_lock:
            message = deepcopy(self._latest_footprint)
        if message is None:
            raise PlanningError(
                "FOOTPRINT_UNAVAILABLE",
                "no Nav2 global footprint has been received",
            )
        return self._polygon_to_frame(
            message, self.get_parameter("robot_base_frame").value)

    def _request_costmap(self, goal_handle, deadline):
        service_deadline = min(
            deadline,
            monotonic() + self.get_parameter("costmap_timeout_sec").value,
        )
        while not self._costmap_client.wait_for_service(timeout_sec=0.1):
            self._checkpoint(goal_handle, service_deadline)
        future = self._costmap_client.call_async(GetCostmap.Request())
        ready = Event()
        future.add_done_callback(lambda _future: ready.set())
        while not future.done():
            self._checkpoint(goal_handle, service_deadline)
            ready.wait(0.05)
        if future.exception() is not None:
            raise PlanningError(
                "COSTMAP_UNAVAILABLE",
                f"Nav2 GetCostmap failed: {future.exception()}",
            )
        response = future.result()
        if response is None:
            raise PlanningError(
                "COSTMAP_UNAVAILABLE", "Nav2 GetCostmap returned no response")
        return self._costmap_to_model(response.map)

    def _costmap_to_model(self, message):
        metadata = message.metadata
        frame_id = message.header.frame_id or self.get_parameter(
            "planning_frame").value
        origin_x = metadata.origin.position.x
        origin_y = metadata.origin.position.y
        origin_yaw = self._yaw(metadata.origin.orientation)
        planning_frame = self.get_parameter("planning_frame").value
        if frame_id != planning_frame:
            transform = self._lookup_transform(planning_frame, frame_id)
            transform_yaw = self._yaw(transform.transform.rotation)
            translated_x = (
                cos(transform_yaw) * origin_x
                - sin(transform_yaw) * origin_y
                + transform.transform.translation.x
            )
            translated_y = (
                sin(transform_yaw) * origin_x
                + cos(transform_yaw) * origin_y
                + transform.transform.translation.y
            )
            origin_x = translated_x
            origin_y = translated_y
            origin_yaw += transform_yaw
        return GridMap(
            width=metadata.size_x,
            height=metadata.size_y,
            resolution=metadata.resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
            data=tuple(message.data),
        )

    def _polygon_to_planning_frame(self, message):
        return self._polygon_to_frame(
            message, self.get_parameter("planning_frame").value)

    def _polygon_to_frame(self, message, target_frame):
        source_frame = message.header.frame_id or self.get_parameter(
            "planning_frame").value
        if source_frame == target_frame:
            return Polygon2D.from_tuples(
                (point.x, point.y) for point in message.polygon.points)
        transform = self._lookup_transform(target_frame, source_frame)
        yaw = self._yaw(transform.transform.rotation)
        translation = transform.transform.translation
        return Polygon2D.from_tuples(
            (
                cos(yaw) * point.x - sin(yaw) * point.y + translation.x,
                sin(yaw) * point.x + cos(yaw) * point.y + translation.y,
            )
            for point in message.polygon.points
        )

    def _lookup_transform(self, target_frame, source_frame):
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(
                    seconds=self.get_parameter("tf_timeout_sec").value),
            )
        except TransformException as error:
            raise PlanningError(
                "TF_UNAVAILABLE",
                f"cannot transform {source_frame} to {target_frame}: {error}",
            ) from error

    @staticmethod
    def _yaw(quaternion):
        return atan2(
            2.0 * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0 - 2.0 * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )

    @staticmethod
    def _checkpoint(goal_handle, deadline):
        if goal_handle.is_cancel_requested:
            raise PlanningError("CANCELED", "planning was canceled")
        if monotonic() > deadline:
            raise PlanningError("TIMEOUT", "planning exceeded its deadline")

    @staticmethod
    def _error_code(result_message, code):
        return getattr(result_message, code, result_message.INTERNAL_ERROR)

    def _publish_feedback(self, goal_handle, stage, progress):
        feedback = ComputeCleaningCoverage.Feedback()
        feedback.stage = getattr(feedback, stage)
        feedback.progress = float(progress)
        feedback.message = stage.replace("_", " ").lower()
        goal_handle.publish_feedback(feedback)

    def _fill_result_message(self, message, result):
        message.components = [
            self._component_message(component) for component in result.components]
        message.coverage_ratio = result.coverage_ratio
        message.uncovered_regions = [
            self._polygon_message(value) for value in result.uncovered_regions]
        message.debug_image_path = result.debug_image_path
        self._set_duration(message, result.planning_duration_sec)

    def _component_message(self, component):
        message = CleaningPlanComponentMsg()
        message.component_index = component.index
        message.outer_boundary = self._polygon_message(component.outer)
        message.exclusions = [
            self._polygon_message(value) for value in component.exclusions]
        message.edge_segments = [
            self._segment_message(value) for value in component.edge_segments]
        message.coverage_segments = [
            self._segment_message(value)
            for value in component.coverage_segments
        ]
        message.transition_segments = [
            self._segment_message(value)
            for value in component.transition_segments
        ]
        message.ordered_waypoints = [
            self._pose_message(value) for value in component.ordered_poses]
        message.coverage_ratio = component.coverage_ratio
        return message

    def _segment_message(self, segment):
        message = CleaningSegmentMsg()
        message.type = int(segment.segment_type)
        message.component_index = segment.component_index
        message.boundary_index = segment.boundary_index
        message.swath_index = segment.swath_index
        message.start = self._pose_message(segment.start)
        message.end = self._pose_message(segment.end)
        return message

    def _pose_message(self, pose):
        message = PoseStamped()
        message.header.frame_id = self.get_parameter("planning_frame").value
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose.x
        message.pose.position.y = pose.y
        message.pose.orientation.z = sin(pose.yaw / 2.0)
        message.pose.orientation.w = cos(pose.yaw / 2.0)
        return message

    def _polygon_message(self, polygon):
        message = PolygonStamped()
        message.header.frame_id = self.get_parameter("planning_frame").value
        message.header.stamp = self.get_clock().now().to_msg()
        message.polygon.points = [
            self._point32(value.x, value.y) for value in polygon.vertices]
        return message

    @staticmethod
    def _point32(x, y):
        from geometry_msgs.msg import Point32
        return Point32(x=float(x), y=float(y), z=0.0)

    @staticmethod
    def _set_duration(message, seconds):
        whole = int(seconds)
        message.planning_time.sec = whole
        message.planning_time.nanosec = int((seconds - whole) * 1.0e9)

    def _publish_markers(self, result):
        if self._marker_publisher is None:
            return
        markers = MarkerArray()
        marker_id = 0
        styles = (
            (SegmentType.EDGE, "edge", (0.1, 0.4, 0.9)),
            (SegmentType.COVERAGE, "coverage", (0.1, 0.7, 0.2)),
            (SegmentType.TRANSITION, "transition", (0.6, 0.6, 0.6)),
        )
        for segment_type, namespace, color in styles:
            marker = Marker()
            marker.header.frame_id = self.get_parameter("planning_frame").value
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = namespace
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.03
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            for component in result.components:
                segments = (
                    component.edge_segments
                    if segment_type == SegmentType.EDGE
                    else component.coverage_segments
                    if segment_type == SegmentType.COVERAGE
                    else component.transition_segments
                )
                for segment in segments:
                    marker.points.extend(
                        [
                            Point(x=segment.start.x, y=segment.start.y),
                            Point(x=segment.end.x, y=segment.end.y),
                        ]
                    )
            markers.markers.append(marker)
        self._marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CleaningCoverageServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
