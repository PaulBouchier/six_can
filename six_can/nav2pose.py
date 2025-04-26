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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import sys
import math

"""
Basic navigation demo to go to pose.
"""

class Nav2Pose:
    def __init__(self):
        self.navigator = BasicNavigator()

    def waitForNav2Active(self):
        print("Waiting for Nav2 to be active...")
        self.navigator.waitUntilNav2Active()
        print("Nav2 is active!")

    def waitForComplete(self):
        try:
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
        except KeyboardInterrupt:
            print('Navigation interrupted by user!')
            self.navigator.cancelTask()
        finally:
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                return True
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')
            return False

    def goToPose(self, target_x, target_y, target_orientation):
        """Navigate the robot to a target pose in the map frame.
        
        This method commands the robot to move to a specified position and orientation
        in the map frame. It uses ROS 2 Navigation2 (Nav2) for path planning and execution.
        The method blocks until the navigation is complete or fails.
        
        Args:
            target_x (float): Target X coordinate in meters in the map frame
            target_y (float): Target Y coordinate in meters in the map frame
            target_orientation (float): Target orientation in degrees. 0 degrees points along 
                the positive X axis, and angles increase counterclockwise.
        
        Returns:
            bool: True if navigation succeeded, False if it failed or was interrupted
        
        Example:
            >>> navigator = Nav2Pose()
            >>> # Move to (1.0, 2.0) facing 90 degrees (along +Y axis)
            >>> success = navigator.goToPose(1.0, 2.0, 90.0)
        """
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.orientation.w = round(math.cos(math.radians(target_orientation) / 2), 3)
        target_pose.pose.orientation.z = round(math.sin(math.radians(target_orientation) / 2), 3)
        self.navigator.goToPose(target_pose)
        rv = self.waitForComplete()
        if not rv:
            print('Goal failed to complete!')
            return False
        else:
            print('Goal completed successfully!')
            return True

def main():
    if len(sys.argv) != 4:
        print("Usage: nav_to_point.py <x_coordinate> <y_coordinate> <orientation>")
        exit(1)

    try:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_orientation = float(sys.argv[3])
    except ValueError:
        print("Error: Coordinates must be valid floating-point numbers.")
        exit(1)

    rclpy.init()

    nav = Nav2Pose()
    nav.waitForNav2Active()

    print(f"Navigating to: x={goal_x}, y={goal_y}, orientation={goal_orientation} degrees")
    
    try:
        nav.goToPose(goal_x, goal_y, goal_orientation)
    except Exception as e:
        print(f"Navigation failed with error: {e}")

    exit(0)

if __name__ == '__main__':
    main()