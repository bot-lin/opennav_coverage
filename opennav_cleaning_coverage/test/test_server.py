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

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from nav2_msgs.srv import GetCostmap
from opennav_cleaning_coverage_msgs.action import ComputeCleaningCoverage
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from threading import Thread
from time import monotonic, sleep

from opennav_cleaning_coverage.artifacts import ArtifactWriter
from opennav_cleaning_coverage.server import CleaningCoverageServer


def test_server_declares_stable_ros_contract_defaults():
    rclpy.init()
    node = CleaningCoverageServer()
    try:
        assert node.get_parameter("action_name").value == "/compute_cleaning_coverage"
        assert node.get_parameter("planning_frame").value == "map"
        assert (
            node.get_parameter("costmap_service").value
            == "/global_costmap/get_costmap"
        )
        assert (
            node.get_parameter("footprint_topic").value
            == "/global_costmap/published_footprint"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def wait_for_future(future, timeout=10.0):
    deadline = monotonic() + timeout
    while not future.done() and monotonic() < deadline:
        sleep(0.02)
    assert future.done()
    return future.result()


def test_active_server_computes_user_mode_action_and_writes_artifact(tmp_path):
    rclpy.init()
    server = CleaningCoverageServer(
        parameter_overrides=[
            Parameter("debug_output_directory", value=str(tmp_path)),
        ]
    )
    client_node = Node("cleaning_coverage_action_test")
    footprint_publisher = client_node.create_publisher(
        PolygonStamped, "/global_costmap/published_footprint", 1)
    action_client = ActionClient(
        client_node, ComputeCleaningCoverage, "/compute_cleaning_coverage")
    executor = MultiThreadedExecutor(num_threads=4)
    spin_thread = None
    try:
        server.trigger_configure()
        server.trigger_activate()
        executor.add_node(server)
        executor.add_node(client_node)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        footprint = PolygonStamped()
        footprint.header.frame_id = "base_link"
        footprint.polygon.points = [
            Point32(x=-0.2, y=-0.2),
            Point32(x=0.2, y=-0.2),
            Point32(x=0.2, y=0.2),
            Point32(x=-0.2, y=0.2),
        ]
        footprint_publisher.publish(footprint)
        assert action_client.wait_for_server(timeout_sec=3.0)
        sleep(0.1)

        goal = ComputeCleaningCoverage.Goal()
        goal.geometry_mode = goal.USER
        goal.roi.header.frame_id = "map"
        goal.roi.polygon.points = [
            Point32(x=0.0, y=0.0),
            Point32(x=4.0, y=0.0),
            Point32(x=4.0, y=2.0),
            Point32(x=0.0, y=2.0),
        ]
        goal.op_width = 1.0
        goal.angle_mode = goal.FIXED
        goal.fixed_angle_deg = 0.0
        goal.angle_step_deg = 5.0
        goal.optimization_objective = goal.MIN_TURNS
        goal.decomposition_mode = goal.DECOMPOSITION_AUTO
        goal.min_coverage_ratio = 0.98

        goal_handle = wait_for_future(action_client.send_goal_async(goal))
        assert goal_handle.accepted
        wrapped_result = wait_for_future(goal_handle.get_result_async())

        assert wrapped_result.result.error_code == wrapped_result.result.SUCCESS
        assert len(wrapped_result.result.components) == 1
        assert wrapped_result.result.debug_image_path.endswith(".png")
        combined_path = wrapped_result.result.debug_image_path
        assert (tmp_path / combined_path.split("/")[-1]).exists()
        for separated_path in ArtifactWriter.separated_paths(combined_path):
            assert (tmp_path / separated_path.split("/")[-1]).exists()
    finally:
        executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()


def test_costmap_mode_uses_nav2_get_costmap_service(tmp_path):
    rclpy.init()
    server = CleaningCoverageServer(
        parameter_overrides=[
            Parameter("debug_output_directory", value=str(tmp_path)),
        ]
    )
    client_node = Node("cleaning_coverage_costmap_test")

    def serve_costmap(_request, response):
        response.map.header.frame_id = "map"
        response.map.metadata.resolution = 1.0
        response.map.metadata.size_x = 6
        response.map.metadata.size_y = 4
        response.map.metadata.origin.orientation.w = 1.0
        response.map.data = [0] * 24
        return response

    costmap_service = client_node.create_service(
        GetCostmap, "/global_costmap/get_costmap", serve_costmap)
    footprint_publisher = client_node.create_publisher(
        PolygonStamped, "/global_costmap/published_footprint", 1)
    action_client = ActionClient(
        client_node, ComputeCleaningCoverage, "/compute_cleaning_coverage")
    executor = MultiThreadedExecutor(num_threads=4)
    spin_thread = None
    try:
        server.trigger_configure()
        server.trigger_activate()
        executor.add_node(server)
        executor.add_node(client_node)
        spin_thread = Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        footprint = PolygonStamped()
        footprint.header.frame_id = "base_link"
        footprint.polygon.points = [
            Point32(x=-0.2, y=-0.2),
            Point32(x=0.2, y=-0.2),
            Point32(x=0.2, y=0.2),
            Point32(x=-0.2, y=0.2),
        ]
        footprint_publisher.publish(footprint)
        assert action_client.wait_for_server(timeout_sec=3.0)
        sleep(0.1)

        goal = ComputeCleaningCoverage.Goal()
        goal.geometry_mode = goal.COSTMAP
        goal.roi.header.frame_id = "map"
        goal.roi.polygon.points = [
            Point32(x=1.0, y=1.0),
            Point32(x=5.0, y=1.0),
            Point32(x=5.0, y=3.0),
            Point32(x=1.0, y=3.0),
        ]
        goal.op_width = 1.0
        goal.angle_mode = goal.FIXED
        goal.fixed_angle_deg = 0.0
        goal.angle_step_deg = 5.0
        goal.optimization_objective = goal.MIN_TURNS
        goal.decomposition_mode = goal.DECOMPOSITION_AUTO
        goal.min_coverage_ratio = 0.98

        goal_handle = wait_for_future(action_client.send_goal_async(goal))
        wrapped_result = wait_for_future(goal_handle.get_result_async())

        assert wrapped_result.result.error_code == wrapped_result.result.SUCCESS
        assert wrapped_result.result.coverage_ratio == 1.0
        assert len(wrapped_result.result.components[0].coverage_segments) == 2
    finally:
        client_node.destroy_service(costmap_service)
        executor.shutdown(timeout_sec=2.0)
        if spin_thread is not None:
            spin_thread.join(timeout=2.0)
        server.destroy_node()
        client_node.destroy_node()
        rclpy.shutdown()
