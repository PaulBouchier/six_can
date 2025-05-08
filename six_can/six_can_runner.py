#!/usr/bin/env python3

"""
ROS2 Node for orchestrating the Six Can Challenge.

This node drives the robot to various search poses, detects cans,
and uses the CaptureCan class to pick up cans and deliver them to a goal area.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from example_interfaces.msg import Bool
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math
import time
import sys # For sys.exit in main

# Local imports
from .nav2pose import Nav2Pose
from .capture_can import CaptureCan
# YamlParserNode is not instantiated directly; direct YAML parsing is used.

class SixCanRunner(Node):
    """
    Manages the high-level logic for the Six Can Challenge.

    The robot navigates to predefined search poses. If a can is detected within
    the arena from a search pose, it commands the CaptureCan system to retrieve
    and deliver the can to the goal. This process repeats, cycling through search
    poses until the mission is stopped or an error occurs.
    """

    def __init__(self):
        """Initialize the SixCanRunner node."""
        super().__init__('six_can_runner')
        self.get_logger().info('SixCanRunner node starting...')

        # Declare and get parameters
        self.declare_parameter('arena_max_x', 2.0)
        self.declare_parameter('arena_max_y', 2.0)
        # Default search_poses_file is relative to the package share's resource directory
        self.declare_parameter('search_poses_file', 'resource/search_poses.yaml')

        self.arena_max_x = self.get_parameter('arena_max_x').get_parameter_value().double_value
        self.arena_max_y = self.get_parameter('arena_max_y').get_parameter_value().double_value
        search_poses_file_name = self.get_parameter('search_poses_file').get_parameter_value().string_value

        self.get_logger().info(f"Arena dimensions: max_x={self.arena_max_x}, max_y={self.arena_max_y}")
        self.get_logger().info(f"Search poses file (relative to package share): {search_poses_file_name}")

        # Construct full path to search poses YAML file
        try:
            package_share_dir = get_package_share_directory('six_can')
            self.search_poses_file_path = os.path.join(package_share_dir, search_poses_file_name)
        except Exception as e: # More specific: PackageNotFoundError from ament_index_python.packages
            self.get_logger().error(f"Error getting package share directory for 'six_can': {e}")
            # This is a critical failure, rclpy.shutdown() might be too abrupt here,
            # consider raising to be caught by main.
            raise RuntimeError(f"Failed to get package share directory: {e}")

        # Read and parse search poses
        try:
            self.search_poses = self._load_search_poses(self.search_poses_file_path)
            if not self.search_poses:
                self.get_logger().error("No search poses loaded. Check YAML file content and path.")
                raise ValueError("No search poses loaded. YAML file might be empty or incorrectly formatted.")
        except Exception as e:
            self.get_logger().error(f"Failed to load or parse search poses from '{self.search_poses_file_path}': {e}")
            raise # Propagate to halt initialization

        # Initialize state variables
        self.can_detected = False
        self.closest_can_pose = None  # Stores geometry_msgs.msg.Pose
        self.search_poses_idx = 0

        # Subscribers
        self.can_detected_subscription = self.create_subscription(
            Bool,
            '/can_detected',
            self._can_detected_callback,
            10)

        self.can_positions_subscription = self.create_subscription(
            PoseArray,
            '/can_positions',
            self._can_positions_callback,
            10)

        # Instantiate helper classes
        self.nav2pose = Nav2Pose()
        self.capture_can = CaptureCan(self) # CaptureCan requires the node instance

        # Wait for Nav2 to be active
        self.get_logger().info("Waiting for Nav2 services to become active...")
        if not self.nav2pose.waitForNav2Active(): # Assuming Nav2Pose has this method
             self.get_logger().error("Nav2 failed to become active. SixCanRunner cannot proceed.")
             # This is a critical failure.
             raise RuntimeError("Nav2 failed to become active.")
        self.get_logger().info("Nav2 is active.")


        self.get_logger().info('SixCanRunner node initialized successfully.')

    def _load_search_poses(self, yaml_file_path: str) -> list:
        """
        Load search poses from a YAML file.
        The YAML file is expected to have a top-level key 'poses',
        which contains a list of pose definitions. Each pose definition
        is a list of three numbers: [x, y, theta_degrees].
        This method transforms these into a list of dictionaries,
        each with 'name', 'x', 'y', and 'theta' keys.

        Args:
            yaml_file_path: The absolute path to the YAML file.

        Returns:
            A list of pose dictionaries (e.g., [{'name': 'SearchPose0', 'x': 0.5, ...}]).

        Raises:
            FileNotFoundError: If the YAML file cannot be found.
            yaml.YAMLError: If there's an error parsing the YAML file.
            ValueError: If the YAML content is not in the expected format.
        """
        self.get_logger().info(f"Loading search poses from: {yaml_file_path}")
        try:
            with open(yaml_file_path, 'r') as file:
                raw_data = yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"Search poses file not found: {yaml_file_path}")
            raise
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing search poses YAML file '{yaml_file_path}': {e}")
            raise
        
        if not isinstance(raw_data, dict) or 'poses' not in raw_data:
            self.get_logger().error(
                f"Search poses YAML file '{yaml_file_path}' must be a dictionary with a 'poses' key."
            )
            raise ValueError(f"Invalid YAML format in '{yaml_file_path}': expected dict with 'poses' key.")

        poses_list_from_yaml = raw_data['poses']

        if not isinstance(poses_list_from_yaml, list):
            self.get_logger().error(
                f"The 'poses' key in YAML file '{yaml_file_path}' must contain a list of poses."
            )
            raise ValueError(f"Invalid YAML format in '{yaml_file_path}': 'poses' key must contain a list.")

        if not poses_list_from_yaml:
            self.get_logger().warn(f"Search poses list in YAML file '{yaml_file_path}' is empty.")
            return [] # Handled by caller

        transformed_poses = []
        for i, pose_values in enumerate(poses_list_from_yaml):
            if not (isinstance(pose_values, list) and len(pose_values) == 3):
                self.get_logger().error(
                    f"Pose entry {i} in '{yaml_file_path}' is not a list of 3 numbers: {pose_values}"
                )
                raise ValueError(f"Invalid format for pose entry {i} in '{yaml_file_path}': expected [x, y, theta]")
            
            try:
                x = float(pose_values[0])
                y = float(pose_values[1])
                theta = float(pose_values[2]) # Assuming theta is in degrees
            except (TypeError, ValueError) as e:
                self.get_logger().error(
                    f"Invalid numeric value in pose entry {i} ({pose_values}) in '{yaml_file_path}': {e}"
                )
                raise ValueError(f"Invalid numeric value in pose entry {i} in '{yaml_file_path}': {e}")

            transformed_poses.append({
                'name': f"SearchPose{i}",
                'x': x,
                'y': y,
                'theta': theta
            })
        
        self.get_logger().info(
            f"Successfully loaded and transformed {len(transformed_poses)} search poses from '{yaml_file_path}'."
        )
        return transformed_poses

    def _can_detected_callback(self, msg: Bool):
        """Callback for /can_detected topic. Stores the detection status."""
        self.can_detected = msg.data
        # self.get_logger().debug(f"Can detected status updated: {self.can_detected}")

    def _can_positions_callback(self, msg: PoseArray):
        """Callback for /can_positions topic. Stores the first pose if available."""
        if msg.poses:
            self.closest_can_pose = msg.poses[0]
            # self.get_logger().debug(f"Received closest_can_pose: x={self.closest_can_pose.position.x:.2f}")
        else:
            self.closest_can_pose = None
            # self.get_logger().debug("Received empty PoseArray for can_positions.")

    def is_can_in_arena(self, can_pose: Pose) -> bool:
        """
        Check if the given can pose is within the defined arena boundaries.
        Arena boundaries: 0 < x < arena_max_x and 0 < y < arena_max_y.

        Args:
            can_pose: The geometry_msgs.msg.Pose of the can.

        Returns:
            True if the can is within the arena, False otherwise.
        """
        if can_pose is None:
            return False
        
        x = can_pose.position.x
        y = can_pose.position.y
        
        in_arena = (0 < x < self.arena_max_x) and \
                   (0 < y < self.arena_max_y)
        
        if not in_arena:
            self.get_logger().debug(f"Can at ({x:.2f}, {y:.2f}) is outside arena "
                                   f"(max_x={self.arena_max_x}, max_y={self.arena_max_y}).")
        return in_arena

    def run_mission(self):
        """
        Execute the main mission logic for finding and capturing cans.
        Cycles through search poses, and for each pose, attempts to find and
        capture cans within the arena.
        """
        self.get_logger().info("Starting Six Can mission...")
        if not self.search_poses: # Should have been caught in __init__
            self.get_logger().fatal("No search poses available. Aborting mission.")
            return

        self.search_poses_idx = 0 # Start with the first search pose

        while rclpy.ok():
            current_search_pose_data = self.search_poses[self.search_poses_idx]
            pose_name = current_search_pose_data.get('name', f"UnnamedPose{self.search_poses_idx}")
            target_x = float(current_search_pose_data['x'])
            target_y = float(current_search_pose_data['y'])
            target_theta = float(current_search_pose_data['theta']) # Degrees

            self.get_logger().info(f"--- Mission: Driving to search pose: {pose_name} "
                                   f"({target_x:.2f}, {target_y:.2f}, {target_theta:.1f} deg) ---")
            
            # Drive to the current search pose
            # nav2pose.goToPose is blocking and returns True on success
            nav_success = self.nav2pose.goToPose(target_x, target_y, target_theta)

            if not nav_success:
                self.get_logger().warn(f"Failed to reach search pose {pose_name}. Moving to next search pose.")
                # Advance to next search pose and continue outer loop
                self.search_poses_idx = (self.search_poses_idx + 1) % len(self.search_poses)
                if not rclpy.ok(): break
                time.sleep(1.0) # Brief pause before next navigation attempt
                continue

            self.get_logger().info(f"Arrived at search pose {pose_name}. Looking for cans.")

            # Inner loop: attempt to capture cans from this current_search_pose
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2) # Process callbacks for can_detected, closest_can_pose
                self.get_logger().debug(f"At {pose_name} - Can detected: {self.can_detected}, "
                                       f"Closest can pose valid: {self.closest_can_pose is not None}")

                if self.can_detected and self.closest_can_pose and self.is_can_in_arena(self.closest_can_pose):
                    self.get_logger().info(f"Can detected in arena from {pose_name}. Attempting capture.")
                    
                    # Command CaptureCan to handle the can (blocking call)
                    capture_successful = self.capture_can.start_capture()

                    if capture_successful:
                        self.get_logger().info(f"Can captured and delivered successfully from {pose_name}.")
                        # Robot is now at the goal. Drive back to the current search pose to check for more.
                        self.get_logger().info(f"Driving back to search pose {pose_name} to check for more cans.")
                        
                        nav_back_success = self.nav2pose.goToPose(target_x, target_y, target_theta)
                        
                        if not nav_back_success:
                            self.get_logger().warn(f"Failed to return to search pose {pose_name} after capture. "
                                                   "Moving to next search pose in list.")
                            break # Break inner loop (stop trying at this pose), go to next search pose in outer loop
                        
                        self.get_logger().info(f"Returned to search pose {pose_name}. Re-evaluating for cans.")
                        # Loop back in this inner `while` to check for more cans from the same spot
                        if not rclpy.ok(): break
                        time.sleep(0.5) # Small delay to allow sensors to update if needed
                    else: # capture_can.start_capture() returned False
                        self.get_logger().info(f"Failed to capture can from {pose_name}. Will try next search pose.")
                        break # Break inner loop, go to next search pose
                else: # No can detected, or pose data missing, or can not in arena
                    self.get_logger().info(f"No suitable can detected at {pose_name} (or can outside arena). "
                                           "Moving to next search pose.")
                    break # Break inner loop, go to next search pose
            
            if not rclpy.ok(): break # Check before advancing index and sleeping

            # Advance to the next search pose for the outer loop
            self.search_poses_idx = (self.search_poses_idx + 1) % len(self.search_poses)
            self.get_logger().info(f"--- Mission: Advancing to next search pose index: {self.search_poses_idx} ---")
            if not rclpy.ok(): break
            time.sleep(0.5) # Small delay before starting next navigation cycle

        self.get_logger().info("Mission loop ended or ROS shutdown requested.")


def main(args=None):
    """Main function to initialize and run the SixCanRunner node."""
    rclpy.init(args=args)
    six_can_runner = None
    exit_code = 0
    try:
        six_can_runner = SixCanRunner()
        six_can_runner.run_mission()
    except KeyboardInterrupt:
        if six_can_runner:
            six_can_runner.get_logger().info('Keyboard interrupt, shutting down SixCanRunner.')
        else:
            print('Keyboard interrupt during SixCanRunner initialization.')
        exit_code = 1
    except RuntimeError as e: # Catch specific errors like Nav2 not active or YAML loading issues
        if six_can_runner:
            six_can_runner.get_logger().fatal(f"Runtime error in SixCanRunner: {e}")
        else:
            print(f"Runtime error during SixCanRunner initialization: {e}")
        exit_code = 1
    except Exception as e:
        if six_can_runner:
            six_can_runner.get_logger().fatal(f"Unhandled exception in SixCanRunner: {e}")
        else:
            print(f"Unhandled exception during SixCanRunner initialization: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 1
    finally:
        if six_can_runner:
            six_can_runner.get_logger().info('Destroying SixCanRunner node.')
            six_can_runner.destroy_node()
        
        # Ensure rclpy.shutdown() is called, but avoid errors if already shut down.
        # rclpy.try_shutdown() is not a public API.
        # A common pattern is to check rclpy.ok() before calling shutdown.
        if rclpy.ok():
            rclpy.shutdown()
        
        if six_can_runner: # Log after shutdown attempt if node existed
             six_can_runner.get_logger().info('SixCanRunner shutdown complete.')
        else:
             print('SixCanRunner shutdown complete (node might not have been fully initialized).')

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
