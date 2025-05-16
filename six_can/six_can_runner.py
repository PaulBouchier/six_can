import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String # For PoseArray subscriber, though String itself isn't used for logic
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import time # For potential delays or timing

from .nav2pose import Nav2Pose
from .capture_can import CaptureCan
from .can_chooser import CanChooser
# YamlParserNode is not instantiated here; SixCanRunner parses its own YAML.

class SixCanRunner(Node):
    """
    ROS2 Node to manage the process of finding and moving six cans to a goal area.
    It navigates to predefined search poses, uses CanChooser to identify cans,
    and CaptureCan to retrieve and deposit them.
    """
    def __init__(self):
        """
        Initializes the SixCanRunner node, parameters, helper classes, and loads search poses.
        """
        super().__init__('six_can_runner')
        self.get_logger().info('SixCanRunner node starting...')

        # Get parameters for arena size
        self.declare_parameter('arena_max_x', 2.0)
        self.declare_parameter('arena_max_y', 2.0)
        self.arena_max_x = self.get_parameter('arena_max_x').get_parameter_value().double_value
        self.arena_max_y = self.get_parameter('arena_max_y').get_parameter_value().double_value
        self.get_logger().info(f"Arena dimensions: max_x={self.arena_max_x}, max_y={self.arena_max_y}")

        # Get parameter for search poses YAML file
        default_search_poses_path = os.path.join(
            get_package_share_directory('six_can'), # Assumes package name is 'six_can'
            'resource',
            'search_poses.yaml' # Corrected filename
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

        # Subscriber to /can_positions (mirroring minimal subscriber example)
        # CanChooser also subscribes to this, this one is mostly for fulfilling the requirement
        # or for potential direct use/logging by SixCanRunner if needed in future.
        self.can_positions_subscription = self.create_subscription(
            PoseArray,
            '/can_positions',
            self._can_positions_callback,
            10)
        # self.can_positions_subscription  # prevent unused variable warning if not used elsewhere

        # Instantiate helper classes
        self.nav2pose = Nav2Pose() # Not a ROS Node, constructor takes no args
        self.capture_can = CaptureCan(self) # Pass self (the SixCanRunner node)

        self.current_search_pose_index = 0
        self.get_logger().info('SixCanRunner initialization complete.')

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
        poses = []
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

            pose = Pose()
            try:
                pose.position.x = float(item[0])
                pose.position.y = float(item[1])
                pose.position.z = 0.0  # Assuming 2D navigation, z is 0

                yaw_degrees = float(item[2])
                yaw_radians = math.radians(yaw_degrees)
                q = quaternion_from_euler(0.0, 0.0, yaw_radians)  # roll, pitch, yaw
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                poses.append(pose)
            except ValueError as e:
                self.get_logger().warn(f"Skipping pose entry due to conversion error: {item} ({e}).")
        
        if not poses:
            self.get_logger().warn(f"No valid poses were loaded from {yaml_file_path}.")
        return poses

    def _can_positions_callback(self, msg: PoseArray):
        """
        Callback for the /can_positions subscriber.
        Currently logs the number of received can positions. CanChooser handles detailed processing.
        """
        # self.get_logger().info(f'SixCanRunner received {len(msg.poses)} can positions on /can_positions.')
        # This subscription is primarily to fulfill the "MIRROR" instruction from aider_six_can_runner.md.
        # CanChooser has its own subscription and handles the logic.
        pass

    def _navigate_to_pose(self, target_pose: Pose) -> bool:
        """
        Navigates the robot to the given target_pose using Nav2Pose.
        Returns True on success, False on failure.
        """
        self.get_logger().info(f"Attempting to navigate to pose: "
                               f"x={target_pose.position.x:.2f}, y={target_pose.position.y:.2f}")
        
        # Ensure Nav2 is active before commanding. waitForNav2Active is blocking.
        # Nav2Pose uses print statements; ideally, it would use logging.
        if not self.nav2pose.waitForNav2Active(): # This might have its own timeout or block indefinitely
            self.get_logger().error("Nav2 is not active. Cannot navigate.")
            return False

        yaw_rad = self._quaternion_to_yaw(target_pose.orientation)
        yaw_deg = math.degrees(yaw_rad)
        
        self.get_logger().info(f"Commanding Nav2: x={target_pose.position.x:.2f}, "
                               f"y={target_pose.position.y:.2f}, yaw={yaw_deg:.1f} degrees")
        success = self.nav2pose.goToPose(target_pose.position.x, target_pose.position.y, yaw_deg)
        
        if success:
            self.get_logger().info("Navigation to pose reported success.")
        else:
            self.get_logger().warn("Navigation to pose reported failure or was cancelled.")
        return success

    def run_mission(self):
        """
        Main operational loop for the SixCanRunner.
        Iterates through search poses, finds cans, and captures them.
        """
        if not self.search_poses:
            self.get_logger().error("No search poses available. Mission cannot start.")
            return

        self.get_logger().info("Waiting for Nav2 to become active before starting mission loop...")
        if not self.nav2pose.waitForNav2Active():
            self.get_logger().error("Nav2 failed to become active. Aborting mission.")
            return
        self.get_logger().info("Nav2 is active. Starting mission loop...")

        while rclpy.ok():
            current_target_search_pose = self.search_poses[self.current_search_pose_index]
            self.get_logger().info(f"--- Mission: Processing search pose "
                                   f"{self.current_search_pose_index + 1}/{len(self.search_poses)} ---")
            
            # Drive to the current search pose
            if not self._navigate_to_pose(current_target_search_pose):
                self.get_logger().warn(f"Failed to navigate to search pose {self.current_search_pose_index + 1}. "
                                       f"Skipping this pose and moving to the next.")
                # Advance to the next search pose and continue the outer loop
                self.current_search_pose_index = (self.current_search_pose_index + 1) % len(self.search_poses)
                self._spin_and_sleep(1.0) # Brief pause before next attempt
                continue 

            self.get_logger().info(f"Arrived at search pose. Looking for cans...")

            # Inner loop: attempt to find and capture cans at the current_target_search_pose
            while rclpy.ok():
                self.can_chooser = CanChooser(self) # Pass self (the SixCanRunner node)
                # Spin a few times to allow CanChooser to process fresh sensor data
                # (e.g., /can_positions and /odom callbacks)
                self._spin_and_sleep(5.0) # Allow time for can chooser to process and choose a can
                if not rclpy.ok(): break

                try:
                    self.get_logger().info("Attempting to choose a can...")
                    chosen_can_pose_odom = self.can_chooser.choose_can() # Raises RuntimeError if no can
                    self.get_logger().info(f"Can chosen at odom coordinates: "
                                           f"x={chosen_can_pose_odom.position.x:.2f}, "
                                           f"y={chosen_can_pose_odom.position.y:.2f}")

                    # A can was chosen, now try to capture it
                    self.get_logger().info("Attempting to capture the chosen can...")
                    # CaptureCan.start_capture is blocking and returns True on success
                    capture_success = self.capture_can.start_capture(chosen_can_pose_odom)

                    if capture_success:
                        self.get_logger().info("Successfully captured and delivered can.")
                        # Robot is now in goal area. Navigate back to the *same* search pose
                        # to check for more cans at this location.
                        self.get_logger().info("Returning to current search pose to check for more cans.")
                        if not self._navigate_to_pose(current_target_search_pose):
                            self.get_logger().warn("Failed to navigate back to current search pose after capture. "
                                                   "Moving to next search pose in list.")
                            break # Exit inner loop, move to next search pose in outer loop
                        self.get_logger().info("Arrived back at search pose. Looking for more cans...")
                        # Continue in this inner loop (while rclpy.ok():) to find more cans
                        # at current_target_search_pose
                    else:
                        self.get_logger().warn("Failed to capture/deliver chosen can. "
                                               "Moving to next search pose in list.")
                        break # Exit inner loop, move to next search pose in outer loop

                except RuntimeError as e:
                    # This exception typically means CanChooser.choose_can() found no suitable cans
                    self.get_logger().info(f"No suitable cans found at current search pose (or error in choose_can): {str(e)}")
                    self.get_logger().info("Moving to next search pose in list.")
                    break # Exit inner loop, move to next search pose in outer loop
                except Exception as e:
                    self.get_logger().error(f"Unexpected error during can choosing/capturing: {e}", exc_info=True)
                    self.get_logger().info("Due to unexpected error, moving to next search pose in list.")
                    break # Exit inner loop on unexpected error
            
            if not rclpy.ok():
                self.get_logger().info("ROS shutdown requested, exiting mission loop.")
                break # Exit outer (while rclpy.ok():) loop

            # Move to the next search pose in the list for the next iteration of the outer loop
            self.current_search_pose_index = (self.current_search_pose_index + 1) % len(self.search_poses)
            self.get_logger().info("Advanced to next search pose index.")
            self._spin_and_sleep(1.0) # Brief pause before starting next major cycle

        self.get_logger().info("SixCanRunner mission loop ended.")

    def _spin_and_sleep(self, duration_sec: float):
        """Helper to spin for callbacks and sleep."""
        start_time = self.get_clock().now()
        end_time = start_time + rclpy.duration.Duration(seconds=duration_sec)
        while rclpy.ok() and self.get_clock().now() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05) # Process callbacks
            time.sleep(0.01) # Python sleep to yield CPU


def main(args=None):
    rclpy.init(args=args)
    six_can_runner_node = None
    try:
        six_can_runner_node = SixCanRunner()
        # run_mission contains the main operational loop and internal spinning.
        six_can_runner_node.run_mission()
    except KeyboardInterrupt:
        if six_can_runner_node:
            six_can_runner_node.get_logger().info('Keyboard interrupt, shutting down SixCanRunner.')
    except RuntimeError as e:
        # This can catch errors from __init__, like failing to load poses
        if six_can_runner_node:
            six_can_runner_node.get_logger().fatal(f"Critical runtime error in SixCanRunner: {e}")
        else:
            # Logger might not be available if __init__ failed early
            print(f"Failed to initialize SixCanRunner node: {e}")
    except Exception as e:
        if six_can_runner_node:
            six_can_runner_node.get_logger().error(f"Unhandled exception in SixCanRunner: {e}", exc_info=True)
        else:
            print(f"Unhandled exception before SixCanRunner node fully initialized: {e}")
    finally:
        if six_can_runner_node:
            six_can_runner_node.get_logger().info("Destroying SixCanRunner node.")
            six_can_runner_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("SixCanRunner main finished.")

if __name__ == '__main__':
    main()

# End of six_can/six_can_runner.py
