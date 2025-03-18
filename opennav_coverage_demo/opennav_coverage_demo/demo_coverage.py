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

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageNavigatorTester(Node):

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


def main():
    rclpy.init()

    navigator = CoverageNavigatorTester()
    navigator.startup()

    # Some example field
    field = [
        [14.3, 13.4],
        [14.3, 13.3],
        [12.0, 13.3],
        [11.9, 13.3],
        [10.7, 13.3],
        [10.7, 13.3],
        [9.2, 13.3],
        [9.2, 13.3],
        [7.9, 13.3],
        [7.9, 13.3],
        [5.7, 13.3],
        [5.7, 12.3],
        [5.8, 12.2],
        [5.8, 11.9],
        [5.8, 11.9],
        [5.8, 11.5],
        [5.9, 11.4],
        [5.9, 10.9],
        [5.9, 10.8],
        [5.9, 10.5],
        [5.9, 10.5],
        [5.9, 10.3],
        [5.8, 10.3],
        [5.8, 10.0],
        [5.8, 9.9],
        [5.8, 9.0],
        [5.7, 8.9],
        [5.7, 8.5],
        [5.8, 8.5],
        [5.8, 8.3],
        [5.8, 8.2],
        [5.8, 8.1],
        [5.9, 8.1],
        [5.9, 8.0],
        [5.8, 8.0],
        [5.9, 7.9],
        [5.9, 7.4],
        [5.9, 7.4],
        [5.9, 7.3],
        [5.9, 7.3],
        [5.9, 7.2],
        [5.9, 7.2],
        [5.8, 7.2],
        [5.8, 5.9],
        [5.8, 5.9],
        [5.8, 5.6],
        [5.7, 5.5],
        [5.7, 5.3],
        [5.7, 5.2],
        [5.7, 5.1],
        [6.5, 5.1],
        [6.5, 5.1],
        [6.9, 5.1],
        [6.9, 5.0],
        [7.5, 5.0],
        [7.7, 4.8],
        [7.7, 2.7],
        [7.7, 2.6],
        [7.9, 2.6],
        [8.0, 2.6],
        [8.0, 2.5],
        [7.9, 2.4],
        [7.9, 2.3],
        [7.9, 2.2],
        [7.9, 2.1],
        [7.8, 2.0],
        [8.1, 2.0],
        [8.2, 2.1],
        [8.3, 2.1],
        [8.4, 2.1],
        [8.7, 2.1],
        [8.7, 2.5],
        [8.7, 2.4],
        [8.7, 2.3],
        [8.4, 2.3],
        [8.3, 2.4],
        [8.4, 2.5],
        [8.4, 2.6],
        [8.4, 2.6],
        [8.5, 2.6],
        [8.5, 2.7],
        [8.6, 2.7],
        [8.7, 2.6],
        [8.7, 2.8],
        [8.6, 2.9],
        [8.6, 5.0],
        [8.6, 5.0],
        [8.6, 5.5],
        [8.5, 5.5],
        [8.5, 6.6],
        [8.5, 6.6],
        [8.5, 6.8],
        [8.4, 6.9],
        [8.4, 7.4],
        [8.5, 7.4],
        [8.5, 7.8],
        [8.5, 7.8],
        [8.5, 8.8],
        [8.6, 8.9],
        [8.6, 9.0],
        [8.6, 9.1],
        [8.6, 9.4],
        [8.7, 9.4],
        [8.7, 10.1],
        [8.7, 10.2],
        [8.7, 10.2],
        [8.9, 10.4],
        [9.3, 10.4],
        [9.4, 10.5],
        [9.6, 10.5],
        [9.7, 10.5],
        [10.3, 10.5],
        [10.3, 10.6],
        [10.6, 10.6],
        [10.7, 10.6],
        [11.0, 10.6],
        [11.1, 10.7],
        [12.4, 10.7],
        [12.5, 10.7],
        [12.7, 10.7],
        [12.8, 10.8],
        [13.0, 10.8],
        [13.0, 10.8],
        [14.0, 10.8],
        [14.0, 10.8],
        [14.3, 10.8],
        [14.4, 10.7],
        [14.4, 10.7],
        [14.5, 10.7],
        [14.5, 10.7],
        [14.5, 11.2],
        [14.5, 11.2],
        [14.5, 11.4],
        [14.4, 11.5],
        [14.4, 12.7],
        [14.5, 12.7],
        [14.5, 12.9],
        [14.5, 13.0],
        [14.5, 13.1],
        [14.6, 13.2],
        [14.6, 13.2],
        [14.6, 13.3],
        [14.6, 13.3],
        [14.5, 13.3],
        [14.4, 13.4],
        [14.3, 13.4]
    ]
    navigator.navigateCoverage(field)

    i = 0
    while not navigator.isTaskComplete():
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        time.sleep(1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')


if __name__ == '__main__':
    main()
