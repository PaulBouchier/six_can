import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.node import Node

import os
import yaml
from enum import Enum
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import time # For potential delays or timing
from typing import Tuple

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseStamped, Pose
from scripted_bot_driver import single_move_client


class RunningTask(Enum):
    NONE = 0
    NAVIGATE_TO_POSE = 1
    TEST_MOVES = 2

class NavNode(BasicNavigator):
    """
    ROS2 Node to manage the process of navigating to a series of poses.
    It navigates to predefined search poses
    """
    def __init__(self):
        super().__init__(node_name='test_nav_and_move') # Initialize BasicNavigator with a node name
        self.get_logger().info('NavNode node starting...')

        # Get parameter for search poses YAML filename
        default_search_poses_path = os.path.join(
            get_package_share_directory('six_can'),
            'config',
            'search_poses.yaml'
        )
        self.declare_parameter('search_poses_file', default_search_poses_path)
        self.search_poses_file_path = self.get_parameter('search_poses_file').get_parameter_value().string_value
        self.get_logger().info(f"Using search poses file: {self.search_poses_file_path}")

        # Read and parse the YAML file for search poses
        try:
            self.search_poses = self._load_search_poses(self.search_poses_file_path)
            if not self.search_poses:
                self.get_logger().error("No search poses loaded. Shutting down.")
                # Propagate error to main to allow clean shutdown
                raise RuntimeError("Failed to load search poses or no poses found.") 
            self.get_logger().info(f"Loaded {len(self.search_poses)} search poses.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize SixCanRunner due to YAML parsing error: {e}")
            raise # Re-raise to be caught by main() for shutdown
        self.current_search_pose_index = 0

        self.get_logger().info('NavNode initialization complete.')

    def _quaternion_to_yaw(self, quaternion_msg: Pose.orientation) -> float:
        """Converts a geometry_msgs/Quaternion to a yaw angle in radians."""
        q = quaternion_msg
        # Ensure q is the orientation part of a Pose message or a Quaternion message
        orientation_list = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def _load_search_poses(self, yaml_file_path: str) -> list[Pose]:
        """
        Loads search poses from a YAML file.
        The YAML file is expected to contain a 'poses' key, which is a list of lists.
        Each inner list should be [x, y, yaw_degrees].
        """
        search_poses = []
        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"YAML file not found: {yaml_file_path}")
            raise FileNotFoundError(f"YAML file not found: {yaml_file_path}")

        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)

        if not yaml_data or 'poses' not in yaml_data:
            self.get_logger().error(f"YAML file '{yaml_file_path}' is empty or does not contain 'poses' key.")
            return [] # Return empty list, which will be caught in __init__

        for item in yaml_data['poses']:
            if not isinstance(item, list) or len(item) != 3:
                self.get_logger().warn(f"Skipping invalid pose entry: {item}. Expected [x, y, yaw_degrees].")
                continue

            search_pose = PoseStamped()
            search_pose.header.frame_id = 'map'
            search_pose.header.stamp = self.get_clock().now().to_msg()
            try:
                search_pose.pose.position.x = float(item[0])
                search_pose.pose.position.y = float(item[1])
                search_pose.pose.position.z = 0.0  # Assuming 2D navigation, z is 0

                yaw_degrees = float(item[2])
                yaw_radians = math.radians(yaw_degrees)
                q = quaternion_from_euler(0.0, 0.0, yaw_radians)  # roll, pitch, yaw
                search_pose.pose.orientation.x = q[0]
                search_pose.pose.orientation.y = q[1]
                search_pose.pose.orientation.z = q[2]
                search_pose.pose.orientation.w = q[3]
                search_poses.append(search_pose)
            except ValueError as e:
                self.get_logger().warn(f"Skipping pose entry due to conversion error: {item} ({e}).")
        
        if not search_poses:
            self.get_logger().warn(f"No valid poses were loaded from {yaml_file_path}.")
        return search_poses

    def _spin_and_sleep(self, duration_sec: float):
        """Helper to spin for callbacks and sleep."""
        start_time = self.get_clock().now()
        end_time = start_time + rclpy.duration.Duration(seconds=duration_sec)
        while rclpy.ok() and self.get_clock().now() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05) # Process callbacks
            time.sleep(0.01) # Python sleep to yield CPU

class MoveNode(Node):
    """
    ROS2 Node to manage the process of moving the robot.
    It uses the SingleMoveClient to execute moves.
    """
    def __init__(self):
        super().__init__('move_node')
        self.get_logger().info('MoveNode node starting...')
        self.single_move_client = single_move_client.SingleMoveClient(self)
        self.get_logger().info('MoveNode initialization complete.')

    def execute_move(self, move_type: str, move_spec: list[str]) -> bool:
        """
        Executes a move using the SingleMoveClient.
        Blocks until the move is complete.
        """
        self.get_logger().info(f"Executing move: {move_type} {move_spec}")
        try:
            success = self.single_move_client.execute_move(move_type, move_spec)
            if success:
                self.get_logger().info(f"Move completed successfully: {move_type} {move_spec}")
            else:
                self.get_logger().error(f"Move failed: {move_type} {move_spec}")
            return success
        except Exception as e:
            self.get_logger().error(f"Error executing move: {e}")
            return False

def run_move_tests():
    #global move_node
    move_node.get_logger().info('Starting move_tests sequence...')
    moves = [
        ("rotate_odom", str(1.57)),
        ("rotate_odom", str(-1.57)),
        ("drive_straight_odom", str(0.1)),
        ("drive_straight_odom", str(-0.1)),
    ]

    for move_type, move_command_str in moves:
        move_spec = []
        move_spec.append(move_command_str)
        move_node.get_logger().info(f"Executing move: type='{move_type}', command='{move_spec}'")
        try:
            move_node.single_move_client.execute_move(move_type, move_spec)
            move_node.get_logger().info(f"Move completed: type='{move_type}', command='{move_spec}'")
        except Exception as e:
            move_node.get_logger().error(f"Error executing move: {e}")

    move_node.get_logger().info('run_move_tests completed.')

def run_loops():
    """
    Main operational loop for the NavNode.
    Iterates through search poses
    """
    #global nav_node
    running_task = RunningTask.NONE

    while rclpy.ok():
        if running_task == RunningTask.NONE:
            nav_node.get_logger().info("No task is currently running, starting nav.")

            current_target_search_pose = nav_node.search_poses[nav_node.current_search_pose_index]
            nav_node.get_logger().info(f"--- Mission: Processing search pose "
                                f"{nav_node.current_search_pose_index + 1}/{len(nav_node.search_poses)} ---")
        
            # Command BasicNavigator superclass to go to the desired pose
            rv = nav_node.goToPose(current_target_search_pose)
            if not rv:
                nav_node.get_logger().error(f"goToPose returned {rv}")
                continue
            else:
                running_task = RunningTask.NAVIGATE_TO_POSE

        elif running_task == RunningTask.NAVIGATE_TO_POSE:
            if not nav_node.isTaskComplete():
                feedback = nav_node.getFeedback()
                #nav_node.get_logger().info("Navigation still running, feedback: ")
                print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )
            else:
                result = nav_node.getResult()
                if result == TaskResult.SUCCEEDED:
                    nav_node.get_logger().info("Navigation task completed successfully.")
                elif result == TaskResult.CANCELED:
                    nav_node.get_logger().info("Navigation task was canceled.")
                elif result == TaskResult.FAILED:
                    nav_node.get_logger().info("Navigation task failed.")
                else:
                    nav_node.get_logger().info("Navigation task has an invalid return status!")

                nav_node.get_logger().info("Navigation task completed, setting next search pose.")
                nav_node.current_search_pose_index = (nav_node.current_search_pose_index + 1) % len(nav_node.search_poses)

                running_task = RunningTask.TEST_MOVES  # Set to next task
                time.sleep(1)  # Optional delay before starting the next task

        elif running_task == RunningTask.TEST_MOVES:
            run_move_tests()
            running_task = RunningTask.NONE

        else:
            nav_node.get_logger().info("Unknown task state. Resetting task.")
            running_task = RunningTask.NONE
            nav_node.current_search_pose_index = (nav_node.current_search_pose_index + 1) % len(nav_node.search_poses)

        time.sleep(1.0)  # Adjust as needed for responsiveness 

def main(args=None):
    rclpy.init(args=args)

    global nav_node
    global move_node

    try:
        nav_node = NavNode()
        move_node = MoveNode()
        #executor = MultiThreadedExecutor(num_threads=4)
        #executor.add_node(nav_node)
        nav_node.get_logger().info("Waiting for Nav2 to become active before starting mission loop...")
        nav_node.waitUntilNav2Active()
        nav_node.get_logger().info("Nav2 is active. Starting mission loop...")
        if not nav_node.search_poses:
            nav_node.get_logger().error("No search poses available. Mission cannot start.")
            return

        # Start the main loop
        run_loops()
        #rclpy.spin(nav_node)  # Spin the node to process callbacks
        #executor.spin()  # This will block until the node is shut down
        #rclpy.spin(nav_node, executor=executor)  # Spin the node with the executor
    except KeyboardInterrupt:
        if nav_node:
            nav_node.get_logger().info('Keyboard interrupt, shutting down NavNode.')
    except RuntimeError as e:
        # This can catch errors from __init__, like failing to load poses
        if nav_node:
            nav_node.get_logger().fatal(f"Critical runtime error in NavNode: {e}")
        else:
            # Logger might not be available if __init__ failed early
            print(f"Failed to initialize SixCanRunner node: {e}")
    except Exception as e:
        if nav_node:
            nav_node.get_logger().error(f"Unhandled exception in NavNode: {e}", exc_info=True)
        else:
            print(f"Unhandled exception before SixCanRunner node fully initialized: {e}")
    finally:
        #executor.shutdown()
        if nav_node:
            nav_node.get_logger().info("Destroying NavNode node.")
            nav_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("NavNode main finished.")

if __name__ == '__main__':
    main()
