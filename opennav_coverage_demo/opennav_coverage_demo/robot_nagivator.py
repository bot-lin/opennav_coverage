#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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
import math
import numpy as np

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from zbot_interfaces.srv import SwitchLight
from std_srvs.srv import Trigger as RosTrigger
from nav2_msgs.action import BackUp, Spin, DriveOnHeading
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose
from nav2_msgs.action import FollowPath, FollowWaypoints, NavigateThroughPoses, NavigateToPose
# from nav2_msgs.action import SmoothPath
from nav2_msgs.srv import ClearEntireCostmap, GetCostmap, LoadMap, ManageLifecycleNodes
from zbot_interfaces.srv import LineSegmentListSrv
from slam_toolbox.srv import Pause as SlamPause
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Twist
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinates, Coordinate
import sys


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class BasicNavigator(Node):

    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.linear_velocity = 0.04  # m/s
        self.angular_velocity = 0.2  # rad/s
        # Declare distance metrics in meters
        self.distance_goal_tolerance = 0.03
        self.reached_distance_goal = False
        self.cancel_docking = False
        # Declare angle metrics in radians
        self.heading_tolerance = 0.1
        self.yaw_goal_tolerance = 0.1
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.robot_pose = None
        self.init_pose_with_cov = False

        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self,
                                                     NavigateThroughPoses,
                                                     'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose,
                                                        'compute_path_to_pose')
        self.compute_full_cooverage_path_client = ActionClient(self, ComputeCoveragePath,
                                                              'compute_coverage_path')
        
        self.compute_path_through_poses_client = ActionClient(self, ComputePathThroughPoses,
                                                              'compute_path_through_poses')
        # self.smoother_client = ActionClient(self, SmoothPath, 'smooth_path')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.backup_client = ActionClient(self, BackUp, 'backup')
        self.driveonheading_client = ActionClient(
            self, DriveOnHeading, 'drive_on_heading')
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.ekf_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                  'set_pose',
                                                  10)
        self.path_pub = self.create_publisher(Path, 'plan', 10)
        self.change_maps_srv = self.create_client(
            LoadMap, '/map_server/load_map')
        self.change_mask_srv = self.create_client(
            LoadMap, '/filter_mask_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.pause_slam_srv = self.create_client(
            SlamPause, '/slam_toolbox/pause_new_measurements')
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.get_costmap_global_srv = self.create_client(
            GetCostmap, '/global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(
            GetCostmap, '/local_costmap/get_costmap')
        self.reset_imu_cli = self.create_client(SwitchLight, 'reset_imu')
        self.get_charging_status_cli = self.create_client(
            RosTrigger, 'get_charging_status_srv')
        self.get_laser_line_srv = self.create_client(
            LineSegmentListSrv, 'get_line_from_laser')
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

    def do_init(self):
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

    def destroyNode(self):
        self.destroy_node()

    def destroy_node(self):
        self.nav_through_poses_client.destroy()
        self.nav_to_pose_client.destroy()
        self.follow_waypoints_client.destroy()
        self.follow_path_client.destroy()
        self.compute_path_to_pose_client.destroy()
        self.compute_full_cooverage_path_client.destroy()
        self.compute_path_through_poses_client.destroy()
        self.smoother_client.destroy()
        self.spin_client.destroy()
        self.backup_client.destroy()
        self.driveonheading_client.destroy()
        super().destroy_node()

    def setInitialPose(self, initial_pose):
        """Set the initial pose to the localization system."""
        self.initial_pose_received = True
        self.initial_pose = initial_pose
        self._setInitialPose()

    def goThroughPoses(self, poses, behavior_tree=''):
        """Send a `NavThroughPoses` action request."""
        self.debug("Waiting for 'NavigateThroughPoses' action server")
        while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info(
                "'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = behavior_tree

        self.info(f'Navigating with {len(goal_msg.poses)} goals....')
        send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg,
                                                                         self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error(f'Goal with {len(poses)} poses was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):
        """Send a `FollowWaypoints` action request."""
        try:
            self.debug("Waiting for 'FollowWaypoints' action server")
            while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
                self.info(
                    "'FollowWaypoints' action server not available, waiting...")

            goal_msg = FollowWaypoints.Goal()
            goal_msg.waypoints = poses

            self.info(f'Following {len(goal_msg.waypoints)} goals....')
            send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                            self._feedbackCallback)
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:
            exception_type, exception_object, exception_traceback = sys.exc_info()
            filename = exception_traceback.tb_frame.f_code.co_filename
            line_number = exception_traceback.tb_lineno
            self.info("Exception type: {}".format(exception_type))
            self.info("File name: {}".format(filename))
            self.info("Line number: {}".format(line_number))
            return False

        if not self.goal_handle.accepted:
            self.error(
                f'Following {len(poses)} waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57, time_allowance=10):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        # goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def backup(self, backup_dist=0.15, backup_speed=0.025, time_allowance=20, acc=0.15, dec=0.3):
        self.debug("Waiting for 'Backup' action server")
        while not self.backup_client.wait_for_server(timeout_sec=1.0):
            self.info("'Backup' action server not available, waiting...")
        goal_msg = BackUp.Goal()
        goal_msg.target = Point(x=float(backup_dist))
        goal_msg.speed = backup_speed
        goal_msg.acc = acc
        goal_msg.dec = dec
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(
            f'Backing up {goal_msg.target.x} m at {goal_msg.speed} m/s....')
        send_goal_future = self.backup_client.send_goal_async(
            goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Backup request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def driveonheading(self, driveon_dist=0.40, driveon_speed=0.04, time_allowance=20, acc=0.15, dec=0.3):
        self.debug("Waiting for 'Driveon' action server")
        while not self.driveonheading_client.wait_for_server(timeout_sec=1.0):
            self.info("'Driveon' action server not available, waiting...")
        goal_msg = DriveOnHeading.Goal()
        goal_msg.target = Point(x=float(driveon_dist))
        goal_msg.speed = driveon_speed
        goal_msg.acc = acc
        goal_msg.dec = dec
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(
            f'Drive on {goal_msg.target.x} m at {goal_msg.speed} m/s....')
        send_goal_future = self.driveonheading_client.send_goal_async(
            goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Backup request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def followPath(self, path, controller_id='', goal_checker_id=''):
        """Send a `FollowPath` action request."""
        self.debug("Waiting for 'FollowPath' action server")
        while not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowPath' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = controller_id
        goal_msg.goal_checker_id = goal_checker_id
        self.path_pub.publish(path)
        self.info('Executing path...')
        send_goal_future = self.follow_path_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Follow path was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(
            self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            self.status = None
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            self.status = None
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            self.status = None
            return TaskResult.CANCELED
        else:
            self.status = None
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl', is_localization=True):
        """Block until the full navigation system is up and running."""
        if is_localization:
            self._waitForNodeToActivate(localizer)
            if localizer == 'amcl':
                self._waitForInitialPose()
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def getPath(self, start, goal, planner_id='', use_start=False):
        """Send a `ComputePathToPose` action request."""
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info(
                "'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = planner_id
        goal_msg.use_start = use_start

        self.info('Getting path...')
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(
            goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result.path


    def getFullCoveragePath(self,polygons, best_angle=0.0, step_angle=0.0, swath_mode="SET_ANGLE", swath_objective="COVERAGE"):
        """Send a `ComputePathToPose` action request."""
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_full_cooverage_path_client.wait_for_server(timeout_sec=1.0):
            self.info(
                "'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputeCoveragePath.Goal()
        p = []
        for polygon in polygons:
            coords = Coordinates()
            for point in polygon.points:
                coord = Coordinate()
                coord.axis1 = point.x
                coord.axis2 = point.y
                coords.coordinates.append(coord)
            p.append(coords)
        goal_msg.polygons = p
        goal_msg.swath_mode.best_angle = best_angle
        goal_msg.swath_mode.step_angle = step_angle
        goal_msg.swath_mode.mode = swath_mode
        goal_msg.swath_mode.objective = swath_objective

        self.info('Getting Full Coverage path...')
        send_goal_future = self.compute_full_cooverage_path_client.send_goal_async(
            goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None
        
        result = self.result_future.result().result
        coverage_path = result.coverage_path
        swaths = coverage_path.swaths
        turns = coverage_path.turns
        self.info(f'Got {len(swaths)} swaths and {len(turns)} turns in the coverage path.')
        self.info(f'——————————————————————')
        for i, swath in enumerate(swaths):
            self.info(f'Swath {i}: {swath}')
        for i, turn in enumerate(turns):
            self.info(f'Turn {i}: {turn}')


        return result

    def getPathThroughPoses(self, start, goals, planner_id='', use_start=False):
        """Send a `ComputePathThroughPoses` action request."""
        self.debug("Waiting for 'ComputePathThroughPoses' action server")
        while not self.compute_path_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.info(
                "'ComputePathThroughPoses' action server not available, waiting...")

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.start = start
        goal_msg.goals = goals
        goal_msg.planner_id = planner_id
        goal_msg.use_start = use_start

        self.info('Getting path...')
        send_goal_future = self.compute_path_through_poses_client.send_goal_async(
            goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn(f'Getting path failed with status code: {self.status}')
            return None

        return self.result_future.result().result.path

    # def smoothPath(self, path, smoother_id='', max_duration=2.0, check_for_collision=False):
    #     """Send a `SmoothPath` action request."""
    #     self.debug("Waiting for 'SmoothPath' action server")
    #     while not self.smoother_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'SmoothPath' action server not available, waiting...")

    #     goal_msg = SmoothPath.Goal()
    #     goal_msg.path = path
    #     goal_msg.max_smoothing_duration = rclpyDuration(
    #         seconds=max_duration).to_msg()
    #     goal_msg.smoother_id = smoother_id
    #     goal_msg.check_for_collisions = check_for_collision

    #     self.info('Smoothing path...')
    #     send_goal_future = self.smoother_client.send_goal_async(goal_msg)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     if not self.goal_handle.accepted:
    #         self.error('Smooth path was rejected!')
    #         return None

    #     self.result_future = self.goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(self, self.result_future)
    #     self.status = self.result_future.result().status
    #     if self.status != GoalStatus.STATUS_SUCCEEDED:
    #         self.warn(f'Getting path failed with status code: {self.status}')
    #         return None

    #     return self.result_future.result().result.path

    def changeMap(self, map_filepath):
        """Change the current static map in the map server."""
        while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.error('Change map request failed!')
            return False
        else:
            self.info('Change map request was successful!')
            return True

    def changeMask(self, map_filepath):
        """Change the current static map in the map server."""
        while not self.change_mask_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.change_mask_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.error('Change mask request failed!')
            return False
        else:
            self.info('Change mask request was successful!')
            return True

    def clearAllCostmaps(self):
        """Clear all costmaps."""
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        """Clear local costmap."""
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        """Clear global costmap."""
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def getGlobalCostmap(self):
        """Get the global costmap."""
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get global costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def pauseSlam(self):
        while not self.pause_slam_srv.wait_for_service(timeout_sec=1.0):
            self.info('Pause SLAM service not available, waiting...')
        req = SlamPause.Request()
        future = self.pause_slam_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().status

    def getLaserLine(self):
        while not self.get_laser_line_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get laser line service not available, waiting...')
        req = LineSegmentListSrv.Request()
        req.request = True
        future = self.get_laser_line_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().line_segments

    def resetIMU(self):
        while not self.reset_imu_cli.wait_for_service(timeout_sec=1.0):
            self.info('Reset IMU service not available, waiting...')
        req = SwitchLight.Request()
        req.command = True
        future = self.reset_imu_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().response

    def getChargingStatus(self):
        while not self.get_charging_status_cli.wait_for_service(timeout_sec=1.0):
            self.info('Get charging status service not available, waiting...')
        req = RosTrigger.Request()
        future = self.get_charging_status_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def getLocalCostmap(self):
        """Get the local costmap."""
        while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get local costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def lifecycleStartup(self):
        """Startup nav2 lifecycle system."""
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Starting up {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(
                        self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        """Shutdown nav2 lifecycle system."""
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        for srv_name, srv_type in self.get_service_names_and_types():
            if srv_type[0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                self.info(f'Shutting down {srv_name}')
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(f'{srv_name} service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose.pose
        if self.init_pose_with_cov:
            msg.pose.covariance = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.5]
        msg.header.frame_id = self.initial_pose.header.frame_id
        msg.header.stamp = self.initial_pose.header.stamp
        self.info('Publishing Initial Pose')
        self.info("x: {}".format(self.initial_pose.pose.position.x))
        self.info("y: {}".format(self.initial_pose.pose.position.y))
        self.info("z: {}".format(self.initial_pose.pose.orientation.z))
        self.info("w: {}".format(self.initial_pose.pose.orientation.w))
        self.initial_pose_pub.publish(msg)

        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    def find_docking_spot(self, spots):
        return

    def euler_from_quaternion(self, x=0.0, y=0.0, z=0.0, w=1.0):
        return

    def pub_velocity(self, v=0.0, w=0.0):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = v
        cmd_vel_msg.angular.z = w
        self.publisher_cmd_vel.publish(cmd_vel_msg)

    """
    goal: [float, float]
    yaw: float
    """

    def go_to_goal(self, goal, target_yaw=0.0):
        distance_to_goal = self.get_distance_to_goal(goal)
        heading_error = self.get_heading_error_to_goal(goal)
        yaw_goal_error = self.get_radians_to_goal(target_yaw)
        cmd_vel_msg = Twist()

        # If we are not yet at the position goal
        if (math.fabs(distance_to_goal) > self.distance_goal_tolerance and self.reached_distance_goal == False):

            # If the robot's heading is off, fix it
            if (math.fabs(heading_error) > self.heading_tolerance):
                # print("Heading error: {}".format(heading_error))
                # self.get_logger().info("Heading error: " + str(heading_error))
                # cmd_vel_msg.angular.z = 0.15 * heading_error
                cmd_vel_msg.linear.x = 0.005

                if heading_error > 0:
                    cmd_vel_msg.angular.z = self.angular_velocity
                else:
                    cmd_vel_msg.angular.z = -self.angular_velocity
            # else:
                # print("D to goal: {}".format(distance_to_goal))
            else:
                cmd_vel_msg.linear.x = self.linear_velocity

        # Orient towards the yaw goal angle
        elif (math.fabs(yaw_goal_error) > self.yaw_goal_tolerance):
            # print("yaw error: {}".format(yaw_goal_error))
            cmd_vel_msg.angular.z = 0.3 * yaw_goal_error

            # if yaw_goal_error > 0:
            # cmd_vel_msg.angular.z = -self.angular_velocity
            # else:
            # cmd_vel_msg.angular.z = self.angular_velocity
            self.reached_distance_goal = True
        # Goal achieved, go to the next goal
        else:
            # Go to the next goal
            self.reached_distance_goal = False
            self.publisher_cmd_vel.publish(cmd_vel_msg)
            return "done"
        # Publish the velocity message
        self.publisher_cmd_vel.publish(cmd_vel_msg)
        return "running"

    def get_distance_to_goal(self, goal):
        current_robot_x = self.robot_pose.position.x
        current_robot_y = self.robot_pose.position.y
        goal_x = goal[0]
        goal_y = goal[1]
        return math.sqrt((goal_x - current_robot_x) ** 2 + (goal_y - current_robot_y) ** 2)

    def get_heading_error_to_goal(self, goal):

        return

    def get_radians_to_goal(self, target_yaw=0.0):

        return
