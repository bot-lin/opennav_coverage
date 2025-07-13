#! /usr/bin/env python3
# Copyright 2023 Open Navigation LLC
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

from enum import Enum
import time
import threading
import json
import logging
import os
import yaml
from flask import Flask, request, jsonify, Response

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from nav2_msgs.msg import BehaviorTreeLog
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
import math

import cv2
import numpy as np
from nav2_msgs.srv import GetCostmap
from nav2_msgs.msg import Costmap

# 配置日志记录
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageNavigatorTester(Node):
    # 这里保留您现有的CoverageNavigatorTester类的代码
    def __init__(self):
        super().__init__(node_name='coverage_navigator_tester')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None
        self.resume_required = False

        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log',
                                 self.behavior_tree_log_callback, 10)

        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')
        self.get_logger().info('Waiting for global costmap message...')
        self.get_costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        while not self.get_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GetCostmap service...')

    def call_get_costmap_service(self):
        try:
            # Create service request
            request = GetCostmap.Request()
            # Note: specs field can be left empty for default behavior
            
            # Call service asynchronously
            future = self.get_costmap_client.call_async(request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                costmap_response = future.result()
                free_contours_world = self.process_costmap(costmap_response.map)
                return free_contours_world[0]['world_coordinates']
            else:
                self.get_logger().error('Service call failed')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error calling service: {str(e)}')

    def pixel_to_world(self, pixel_x, pixel_y, resolution, origin_x, origin_y):
        """
        Convert pixel coordinates to world coordinates
        
        Args:
            pixel_x: Column in image (x-axis in image coordinates)
            pixel_y: Row in image (y-axis in image coordinates)
            resolution: Meters per pixel
            origin_x: World X coordinate of pixel (0,0)
            origin_y: World Y coordinate of pixel (0,0)
            
        Returns:
            (world_x, world_y): World coordinates in meters
        """
        world_x = origin_x + (pixel_x * resolution)
        world_y = origin_y + (pixel_y * resolution)
        return world_x, world_y

    def convert_contour_to_world(self, contour, resolution, origin_x, origin_y):
        """
        Convert a single contour from pixel coordinates to world coordinates
        
        Args:
            contour: OpenCV contour in pixel coordinates
            resolution: Meters per pixel
            origin_x, origin_y: World coordinates of pixel (0,0)
            
        Returns:
            world_contour: List of (x, y) tuples in world coordinates
        """
        world_contour = []
        for point in contour:
            pixel_x, pixel_y = point[0][0], point[0][1]  # OpenCV contour format
            world_x, world_y = self.pixel_to_world(pixel_x, pixel_y, resolution, origin_x, origin_y)
            world_contour.append([world_x, world_y])
        return world_contour

    def process_costmap(self, msg: Costmap):
        try:
            # Convert Costmap data to numpy array
            # Note: Costmap uses uint8 (0-255) instead of int8 (-128 to 127)
            data = np.array(msg.data, dtype=np.uint8).reshape((msg.metadata.size_y, msg.metadata.size_x))
            
            # Extract metadata for coordinate conversion
            resolution = msg.metadata.resolution
            origin_x = msg.metadata.origin.position.x
            origin_y = msg.metadata.origin.position.y
            
            self.get_logger().info(f'Costmap size: {msg.metadata.size_x} x {msg.metadata.size_y}')
            self.get_logger().info(f'Costmap resolution: {resolution} m/cell')
            self.get_logger().info(f'Costmap origin: ({origin_x}, {origin_y}) meters')

            # Create a uint8 image for visualization
            # Costmap values: 0 (free) -> 255 (occupied), so we need to invert for visualization
            image = np.zeros_like(data, dtype=np.uint8)
            
            # For costmap: 0 = free, 255 = occupied
            # For visualization: 0 = black (occupied), 255 = white (free)
            image = 255 - data  # Invert the costmap for better visualization
            
            # Alternative: Create a more detailed mapping if needed
            # image = np.zeros_like(data, dtype=np.uint8)
            # image[data == 0] = 255      # Free space -> white
            # image[data == 255] = 0      # Occupied -> black
            # image[(data > 0) & (data < 255)] = 255 - data[(data > 0) & (data < 255)]  # Interpolate

            # Save original costmap
            filename = 'global_costmap.png'
            cv2.imwrite(filename, image)
            self.get_logger().info(f'Saved costmap to {filename}')

            # Create better binary images for polygon detection
            # For costmap data (0-255), we need different thresholds
            
            # Free space: cells with low cost (0-50)
            free_space_binary = np.zeros_like(data, dtype=np.uint8)
            free_space_binary[data <= 50] = 255
            
            # Navigable space: cells with cost <= 100
            navigable_binary = np.zeros_like(data, dtype=np.uint8)
            navigable_binary[data <= 100] = 255
            
            # Obstacle boundaries: cells with cost >= 200
            obstacle_binary = np.zeros_like(data, dtype=np.uint8)
            obstacle_binary[data >= 200] = 255
            
            # Occupied space: cells with cost >= 250 (nearly occupied)
            occupied_binary = np.zeros_like(data, dtype=np.uint8)
            occupied_binary[data >= 250] = 255
            
            # Apply morphological operations to clean up the binary images
            kernel = np.ones((3,3), np.uint8)
            free_space_binary = cv2.morphologyEx(free_space_binary, cv2.MORPH_CLOSE, kernel)
            navigable_binary = cv2.morphologyEx(navigable_binary, cv2.MORPH_CLOSE, kernel)
            obstacle_binary = cv2.morphologyEx(obstacle_binary, cv2.MORPH_CLOSE, kernel)
            occupied_binary = cv2.morphologyEx(occupied_binary, cv2.MORPH_CLOSE, kernel)
            
            # Create a colored image for polygon visualization
            polygon_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            
            # Find and draw contours with different parameters for better fitting
            
            # 1. Free space polygons (low cost areas) with index labels
            free_contours, _ = cv2.findContours(free_space_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Convert free contours to world coordinates
            free_contours_world = []
            free_contour_index = 0
            
            for contour in free_contours:
                if cv2.contourArea(contour) > 700:  # Reduced threshold
                    # Convert contour to world coordinates
                    world_contour = self.convert_contour_to_world(contour, resolution, origin_x, origin_y)
                    free_contours_world.append({
                        'index': free_contour_index,
                        'area_pixels': cv2.contourArea(contour),
                        'area_meters_squared': cv2.contourArea(contour) * (resolution ** 2),
                        'pixel_coordinates': contour.tolist(),
                        'world_coordinates': world_contour
                    })
                    
                    # Use more precise approximation for visualization
                    epsilon = 0.005 * cv2.arcLength(contour, True)  # Much more precise
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (0, 255, 0), 1)  # Green, thinner line
                    
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Convert centroid to world coordinates
                        world_cx, world_cy = self.pixel_to_world(cx, cy, resolution, origin_x, origin_y)
                        
                        # Draw index number at centroid
                        cv2.putText(polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)  # White text with border
                        cv2.putText(polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)  # Black text
                        
                        # Log information about this contour
                        self.get_logger().info(f'Free contour {free_contour_index}: '
                                             f'Area={cv2.contourArea(contour):.1f} pixels '
                                             f'({cv2.contourArea(contour) * (resolution ** 2):.2f} m²), '
                                             f'Centroid=({world_cx:.2f}, {world_cy:.2f}) m')
                        
                        free_contour_index += 1
            
            # Save free contours as JSON file with world coordinates
            with open('free_contours_world_coordinates.json', 'w') as f:
                json.dump(free_contours_world, f, indent=2)
            self.get_logger().info(f'Saved {len(free_contours_world)} free contours with world coordinates to free_contours_world_coordinates.json')
            
            # 2. Navigable space polygons (broader navigable area)
            navigable_contours, _ = cv2.findContours(navigable_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in navigable_contours:
                if cv2.contourArea(contour) > 100:
                    epsilon = 0.008 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (0, 255, 255), 1)  # Yellow
            
            # 3. Obstacle boundaries
            obstacle_contours, _ = cv2.findContours(obstacle_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in obstacle_contours:
                if cv2.contourArea(contour) > 25:  # Even smaller threshold for obstacles
                    epsilon = 0.005 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (0, 0, 255), 1)  # Red
            
            # 4. Occupied space polygons (high cost areas)
            occupied_contours, _ = cv2.findContours(occupied_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in occupied_contours:
                if cv2.contourArea(contour) > 10:  # Very small threshold for precise obstacles
                    epsilon = 0.003 * cv2.arcLength(contour, True)  # Most precise
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    cv2.drawContours(polygon_image, [approx], -1, (255, 0, 0), 2)  # Blue, thicker line
            
            # Save polygon image
            polygon_filename = 'global_costmap_polygons.png'
            cv2.imwrite(polygon_filename, polygon_image)
            self.get_logger().info(f'Saved polygon visualization to {polygon_filename}')
            
            # Create a detailed version with just the most important polygons and indexed free contours
            detailed_polygon_image = np.ones_like(polygon_image) * 255  # White background
            
            # Draw only the most precise contours with indices
            free_contour_index = 0
            for contour in free_contours:
                if cv2.contourArea(contour) > 700:
                    # Use the original contour points for maximum precision
                    cv2.drawContours(detailed_polygon_image, [contour], -1, (0, 200, 0), 2)
                    
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw index number at centroid (larger font for white background)
                        cv2.putText(detailed_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 3)  # Red text with thick border
                        cv2.putText(detailed_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 2)  # Black text
                        
                        free_contour_index += 1
            
            for contour in occupied_contours:
                if cv2.contourArea(contour) > 25:
                    cv2.drawContours(detailed_polygon_image, [contour], -1, (0, 0, 200), 1)
            
            detailed_filename = 'global_costmap_detailed_polygons.png'
            cv2.imwrite(detailed_filename, detailed_polygon_image)
            self.get_logger().info(f'Saved detailed polygon visualization to {detailed_filename}')
            
            # Create a version with original contours (no approximation) with indices
            precise_polygon_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            
            # Draw exact contours without approximation
            cv2.drawContours(precise_polygon_image, free_contours, -1, (0, 255, 0), 1)
            cv2.drawContours(precise_polygon_image, occupied_contours, -1, (0, 0, 255), 1)
            
            # Add indices to free contours
            free_contour_index = 0
            for contour in free_contours:
                if cv2.contourArea(contour) > 50:
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw index number at centroid
                        cv2.putText(precise_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)  # White text with border
                        cv2.putText(precise_polygon_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)  # Black text
                        
                        free_contour_index += 1
            
            precise_filename = 'global_costmap_precise_contours.png'
            cv2.imwrite(precise_filename, precise_polygon_image)
            self.get_logger().info(f'Saved precise contour visualization to {precise_filename}')
            
            # Create a special version with ONLY indexed free contours for clarity
            indexed_free_image = np.ones_like(polygon_image) * 255  # White background
            
            free_contour_index = 0
            for contour in free_contours:
                if cv2.contourArea(contour) > 50:
                    # Draw contour in green
                    cv2.drawContours(indexed_free_image, [contour], -1, (0, 180, 0), 2)
                    
                    # Calculate centroid for text placement
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw index number at centroid (large, clear text)
                        cv2.putText(indexed_free_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 4)  # Red text with thick border
                        cv2.putText(indexed_free_image, str(free_contour_index), (cx, cy), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)  # White text
                        
                        free_contour_index += 1
            
            indexed_filename = 'global_costmap_indexed_free_contours.png'
            cv2.imwrite(indexed_filename, indexed_free_image)
            self.get_logger().info(f'Saved indexed free contours to {indexed_filename}')
            
            # Log detailed statistics
            self.get_logger().info(f'Found {len(free_contours)} free space regions')
            self.get_logger().info(f'Found {len(navigable_contours)} navigable regions')
            self.get_logger().info(f'Found {len(obstacle_contours)} obstacle boundary regions')
            self.get_logger().info(f'Found {len(occupied_contours)} occupied regions')
            
            # Log unique costmap values for debugging
            unique_values = np.unique(data)
            self.get_logger().info(f'Unique costmap values: {unique_values}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing costmap: {str(e)}')
        return free_contours_world




    def behavior_tree_log_callback(self, msg):
        for log in msg.event_log:
            if log.node_name == "Wait":
                self.resume_required = True
                return
        self.resume_required = False


    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def toPolygon(self, field):
        poly = Polygon()
        for coord in field:
            pt = Point32()
            pt.x = coord[0]
            pt.y = coord[1]
            poly.points.append(pt)
        return poly

    def navigateCoverage(self, field, swath_angle=0.0, mode='SET_ANGLE', step_angle=0.0, objective=""):
        """Send a `NavToPose` action request."""
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
        goal_msg.swath_angle = swath_angle
        goal_msg.mode = mode
        goal_msg.step_angle = step_angle
        goal_msg.objective = objective
        goal_msg.behavior_tree = "/data/behavior_trees/navigate_w_basic_complete_coverage_nav_to_start.xml"
        goal_msg.polygons.append(self.toPolygon(field))

        print('Navigating to with field of size: ' + str(len(field)) + '...')
        send_goal_future = self.coverage_client.send_goal_async(goal_msg,
                                                                self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            print('Navigate Coverage request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                print(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        print('Task succeeded!')
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def startup(self, node_name='bt_navigator'):
        # Waits for the node within the tester namespace to become active
        print(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            print(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                print(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.get_logger().info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return True


class CoverageNavigatorServer(CoverageNavigatorTester):
    """扩展CoverageNavigatorTester以提供HTTP API功能。"""
    
    def __init__(self):
        super().__init__()
        self.app = Flask(__name__)
        self.task_thread = None
        self.repeat_times = 1
        self.current_repeat = 0
        self.cancel_required = False
        self.config_file = '/data/params/coverage_params.yaml'  # Will be set when needed
        self.setup_routes()
        
    def setup_routes(self):
        """设置HTTP路由。"""
        @self.app.route('/navigate_coverage', methods=['POST'])
        def handle_navigate_coverage():
            try:
                logging.info(f"收到导航请求，内容类型: {request.content_type}")
                
                if not request.is_json:
                    logging.error(f"请求不是JSON格式: {request.data}")
                    return jsonify({"code": 1,"error": "请求必须是JSON格式"}), 400
                
                data = request.get_json(force=True)  # 使用force=True尝试强制解析JSON
                logging.info(f"解析的JSON数据: {data}")
                
                if 'field' not in data:
                    return jsonify({"code": 1,"error": "缺少'field'字段"}), 400
                    
                # field = data['field']
                field = self.call_get_costmap_service()
                if not isinstance(field, list) or len(field) < 3:
                    return jsonify({"code": 1,"error": "'field'必须是至少包含3个坐标点的列表"}), 400
                
                if "mode" in data:
                    mode = data['mode']
                    if mode not in ['SET_ANGLE', 'BRUTE_FORCE']:
                        return jsonify({"code": 1,"error": "'mode must be 'SET_ANGLE' or 'BRUTE_FORCE'"}), 400
                else:
                    mode = 'SET_ANGLE'
                
                if "best_angle" in data:
                    swath_angle = data['best_angle']
                    
                    if not isinstance(swath_angle, (int, float)):
                        return jsonify({"code": 1,"error": "'best_angle'必须是数字"}), 400
                    swath_angle = math.radians(swath_angle)  # 转换为弧度
                else:
                    swath_angle = 0.0

                if "step_angle" in data:
                    step_angle = data['step_angle']
                    if not isinstance(step_angle, (int, float)):
                        return jsonify({"code": 1,"error": "'step_angle'必须是数字"}), 400
                    step_angle = math.radians(step_angle)
                else:
                    step_angle = 0.0
                if "objective" in data:
                    objective = data['objective']
                    if objective not in ['LENGTH', 'NUMBER', 'COVERAGE']:
                        return jsonify({"code": 1,"error": "'objective'必须是'LENGTH', 'NUMBER'或'COVERAGE'"}), 400
                else:
                    objective = ''
                
                if "repeat_times" in data:
                    repeat_times = data['repeat_times']
                    if not isinstance(repeat_times, int) or repeat_times < 1:
                        return jsonify({"code": 1,"error": "'repeat_times'必须是正整数"}), 400
                else:
                    repeat_times = 1
                    
                # 如果有正在运行的任务，先取消它
                if self.task_thread and self.task_thread.is_alive():
                    return jsonify({"code": 1,"error": "已有导航任务正在运行中"}), 409
                    
                # 在新线程中启动导航任务
                self.task_thread = threading.Thread(target=self._run_navigation_task, args=(field, swath_angle, repeat_times, mode, objective, step_angle))
                self.task_thread.start()
                
                return jsonify({"code": 0,"status": "导航任务已启动"}), 202
            except Exception as e:
                logging.exception("处理导航请求时出错:")
                return jsonify({"code": 1, "error": f"服务器处理请求时发生错误: {str(e)}"}), 500

        @self.app.route('/navigate_coverage', methods=['OPTIONS'])
        def handle_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "POST, OPTIONS")
            return response

        @self.app.route('/status', methods=['GET'])
        def handle_status():
            """返回当前导航任务的状态。"""
            if self.task_thread and self.task_thread.is_alive():
                status = "运行中, 重复次数: {}/{}".format(self.current_repeat, self.repeat_times)
                if self.feedback:
                    remaining_time = Duration.from_msg(self.feedback.estimated_time_remaining).nanoseconds / 1e9
                    return jsonify({
                        "status": status,
                        "resume_required": self.resume_required,
                        "task_complete": False,
                        "estimated_time_remaining": f"{remaining_time:.1f} 秒"
                    })
                return jsonify({"status": status})
            else:
                result = self.getResult()
                status = "未知"
                if result == TaskResult.SUCCEEDED:
                    status = "成功"
                elif result == TaskResult.CANCELED:
                    status = "已取消"
                elif result == TaskResult.FAILED:
                    status = "失败"
                    
                return jsonify({
                    "status": status,
                    "task_complete": True
                })

        @self.app.route('/cancel', methods=['GET'])
        def handle_cancel():
            """取消当前正在执行的导航任务。"""
            try:
                logging.info("收到取消导航任务请求")
                
                # 检查是否有正在运行的任务
                if not self.task_thread or not self.task_thread.is_alive():
                    return jsonify({
                        "code": 1,
                        "error": "当前没有正在运行的导航任务"
                    }), 404
                
                # 尝试取消任务
                cancel_result = self.cancelTask()
                self.cancel_required = True
                
                if cancel_result:
                    # 等待任务线程结束
                    if self.task_thread:
                        self.task_thread.join(timeout=2.0)  # 等待最多2秒
                        
                    return jsonify({
                        "code": 0,
                        "status": "导航任务已取消"
                    }), 200
                else:
                    return jsonify({
                        "code": 1,
                        "error": "无法取消任务，可能已完成或发生错误"
                    }), 500
            except Exception as e:
                logging.exception("处理取消请求时出错:")
                return jsonify({
                    "code": 1, 
                    "error": f"服务器处理取消请求时发生错误: {str(e)}"
                }), 500

        # Simple robot parameter endpoints
        @self.app.route('/robot_width', methods=['GET'])
        def get_robot_width():
            """获取机器人宽度"""
            return self._get_parameter('robot_width')

        @self.app.route('/robot_width', methods=['POST'])
        def set_robot_width():
            """设置机器人宽度"""
            return self._set_parameter('robot_width')

        @self.app.route('/operation_width', methods=['GET'])
        def get_operation_width():
            """获取作业宽度"""
            return self._get_parameter('operation_width')

        @self.app.route('/operation_width', methods=['POST'])
        def set_operation_width():
            """设置作业宽度"""
            return self._set_parameter('operation_width')

        @self.app.route('/min_turning_radius', methods=['GET'])
        def get_min_turning_radius():
            """获取最小转弯半径"""
            return self._get_parameter('min_turning_radius')

        @self.app.route('/min_turning_radius', methods=['POST'])
        def set_min_turning_radius():
            """设置最小转弯半径"""
            return self._set_parameter('min_turning_radius')

        @self.app.route('/headland_width', methods=['GET'])
        def get_headland_width():
            """获取地头宽度"""
            return self._get_parameter('default_headland_width')

        @self.app.route('/headland_width', methods=['POST'])
        def set_headland_width():
            """设置地头宽度"""
            return self._set_parameter('default_headland_width')

        @self.app.route('/swath_angle', methods=['GET'])
        def get_swath_angle():
            """获取扫描角度"""
            return self._get_parameter('default_swath_angle')

        @self.app.route('/swath_angle', methods=['POST'])
        def set_swath_angle():
            """设置扫描角度"""
            return self._set_parameter('default_swath_angle')

        @self.app.route('/allow_overlap', methods=['GET'])
        def get_allow_overlap():
            """获取是否允许重叠"""
            return self._get_parameter('default_allow_overlap')

        @self.app.route('/allow_overlap', methods=['POST'])
        def set_allow_overlap():
            """设置是否允许重叠"""
            return self._set_parameter('default_allow_overlap')

        @self.app.route('/config_file', methods=['POST'])
        def set_config_file():
            """设置配置文件路径"""
            try:
                if not request.is_json:
                    return jsonify({"code": 1, "error": "请求必须是JSON格式"}), 400
                
                data = request.get_json()
                file_path = data.get('file_path')
                
                if not file_path:
                    return jsonify({"code": 1, "error": "缺少'file_path'字段"}), 400
                
                if not os.path.exists(file_path):
                    return jsonify({"code": 1, "error": f"文件不存在: {file_path}"}), 404
                
                self.config_file = file_path
                return jsonify({"code": 0, "message": "配置文件路径设置成功", "file_path": file_path})
                
            except Exception as e:
                return jsonify({"code": 1, "error": f"设置配置文件时出错: {str(e)}"}), 500

        # OPTIONS for all parameter endpoints
        @self.app.route('/robot_width', methods=['OPTIONS'])
        def handle_robot_width_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            return response

        @self.app.route('/operation_width', methods=['OPTIONS'])
        def handle_operation_width_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            return response

        @self.app.route('/min_turning_radius', methods=['OPTIONS'])
        def handle_min_turning_radius_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            return response

        @self.app.route('/headland_width', methods=['OPTIONS'])
        def handle_headland_width_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            return response

        @self.app.route('/swath_angle', methods=['OPTIONS'])
        def handle_swath_angle_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            return response

        @self.app.route('/allow_overlap', methods=['OPTIONS'])
        def handle_allow_overlap_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            return response

        @self.app.route('/config_file', methods=['OPTIONS'])
        def handle_config_file_options():
            response = Response()
            response.headers.add("Access-Control-Allow-Origin", "*")
            response.headers.add("Access-Control-Allow-Headers", "Content-Type")
            response.headers.add("Access-Control-Allow-Methods", "POST, OPTIONS")
            return response

    def _get_parameter(self, param_name):
        """获取参数值的通用方法"""
        try:
            if not self.config_file:
                return jsonify({"code": 1, "error": "请先设置配置文件路径"}), 400
            
            if not os.path.exists(self.config_file):
                return jsonify({"code": 1, "error": f"配置文件不存在: {self.config_file}"}), 404
            
            with open(self.config_file, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file)
            
            value = config.get('coverage_server', {}).get('ros__parameters', {}).get(param_name)
            
            if value is None:
                return jsonify({"code": 1, "error": f"参数不存在: {param_name}"}), 404
            
            return jsonify({
                "code": 0,
                "parameter": param_name,
                "value": value
            })
            
        except Exception as e:
            return jsonify({"code": 1, "error": f"获取参数时出错: {str(e)}"}), 500

    def _set_parameter(self, param_name):
        """设置参数值的通用方法"""
        try:
            if not self.config_file:
                return jsonify({"code": 1, "error": "请先设置配置文件路径"}), 400
            
            if not request.is_json:
                return jsonify({"code": 1, "error": "请求必须是JSON格式"}), 400
            
            data = request.get_json()
            new_value = data.get('value')
            
            if new_value is None:
                return jsonify({"code": 1, "error": "缺少'value'字段"}), 400
            
            if not os.path.exists(self.config_file):
                return jsonify({"code": 1, "error": f"配置文件不存在: {self.config_file}"}), 404
            
            # 读取配置文件
            with open(self.config_file, 'r', encoding='utf-8') as file:
                config = yaml.safe_load(file) or {}
            
            # 确保结构存在
            if 'coverage_server' not in config:
                config['coverage_server'] = {}
            if 'ros__parameters' not in config['coverage_server']:
                config['coverage_server']['ros__parameters'] = {}
            
            # 设置新值
            old_value = config['coverage_server']['ros__parameters'].get(param_name)
            config['coverage_server']['ros__parameters'][param_name] = new_value
            
            # 写回文件
            with open(self.config_file, 'w', encoding='utf-8') as file:
                yaml.dump(config, file, default_flow_style=False, allow_unicode=True)
            
            return jsonify({
                "code": 0,
                "message": "参数更新成功",
                "parameter": param_name,
                "old_value": old_value,
                "new_value": new_value
            })
            
        except Exception as e:
            return jsonify({"code": 1, "error": f"设置参数时出错: {str(e)}"}), 500
    
    def _run_navigation_task(self, field, swath_angle, repeat_times=1, mode='SET_ANGLE', objective='', step_angle=0.0):
        """在单独的线程中运行导航任务。"""
        self.repeat_times = repeat_times
        for i in range(repeat_times):
            self.current_repeat = i + 1
            if self.cancel_required:
                logging.info("取消请求已收到，停止导航任务。")
                self.cancel_required = False
                return
            """在单独的线程中运行导航任务。"""
            logging.info(f"开始导航任务，区域: {field}, 扫描角度: {swath_angle}")
            self.navigateCoverage(field, swath_angle, mode=mode, step_angle=step_angle, objective=objective)
            
            while not self.isTaskComplete():
                feedback = self.getFeedback()
                time.sleep(1)
            if self.cancel_required:
                logging.info("取消请求已收到，停止导航任务。")
                self.cancel_required = False

            logging.info(f"导航任务完成，结果: {self.getResult()}")
        
    def run_server(self, host='0.0.0.0', port=1235):  # 修改端口为1235
        """启动HTTP服务器。"""
        logging.info(f"开启HTTP服务器在 http://{host}:{port}/")
        # 启用CORS支持，设置线程模式
        self.app.run(host=host, port=port, debug=False, threaded=True)


def main():
    rclpy.init()
    
    # 创建ROS节点和服务器
    navigator_server = CoverageNavigatorServer()
    navigator_server.startup()
    
    # 在单独的线程中处理ROS循环
    ros_thread = threading.Thread(target=rclpy.spin, args=(navigator_server,))
    ros_thread.daemon = True
    ros_thread.start()
    
    # 启动HTTP服务器(在主线程中)
    try:
        navigator_server.run_server()
    except KeyboardInterrupt:
        logging.info("服务器正在关闭...")
    finally:
        navigator_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()