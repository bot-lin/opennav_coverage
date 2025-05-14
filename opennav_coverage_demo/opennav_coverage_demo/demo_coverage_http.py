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
from flask import Flask, request, jsonify, Response

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

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

        self.coverage_client = ActionClient(self, NavigateCompleteCoverage,
                                            'navigate_complete_coverage')

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

    def navigateCoverage(self, field):
        """Send a `NavToPose` action request."""
        print("Waiting for 'NavigateCompleteCoverage' action server")
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            print('"NavigateCompleteCoverage" action server not available, waiting...')

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = 'map'
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


class CoverageNavigatorServer(CoverageNavigatorTester):
    """扩展CoverageNavigatorTester以提供HTTP API功能。"""
    
    def __init__(self):
        super().__init__()
        self.app = Flask(__name__)
        self.task_thread = None
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
                    
                field = data['field']
                if not isinstance(field, list) or len(field) < 3:
                    return jsonify({"code": 1,"error": "'field'必须是至少包含3个坐标点的列表"}), 400
                    
                # 如果有正在运行的任务，先取消它
                if self.task_thread and self.task_thread.is_alive():
                    return jsonify({"code": 1,"error": "已有导航任务正在运行中"}), 409
                    
                # 在新线程中启动导航任务
                self.task_thread = threading.Thread(target=self._run_navigation_task, args=(field,))
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
                status = "运行中"
                if self.feedback:
                    remaining_time = Duration.from_msg(self.feedback.estimated_time_remaining).nanoseconds / 1e9
                    return jsonify({
                        "status": status,
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
    
    def _run_navigation_task(self, field):
        """在单独的线程中运行导航任务。"""
        logging.info(f"开始导航任务，区域: {field}")
        self.navigateCoverage(field)
        
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            time.sleep(1)
            
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