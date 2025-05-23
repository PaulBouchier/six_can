import rclpy
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
import time
import math
import sys

from example_interfaces.msg import Int32
from example_interfaces.srv import SetBool
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Local imports
from scripted_bot_driver.single_move_client import SingleMoveClient
from .can_chooser_client import CanChooserClient
from .basic_navigator_child import BasicNavigatorChild


class CaptureCan:
    """
    A class to manage the state machine for capturing a can and delivering it to a goal.

    States:
        IDLE: Waiting for a command.
        ROTATE: Rotate towards the closest can.
        SEEK2CAN: Move towards the can using lidar guidance.
        GRASPCAN: Close jaws and verify capture.
        DRIVE2GOAL: Navigate to the drop-off goal location.
        DROP_IN_GOAL: Release the can at the goal.
        FAIL_RETREAT: Abort sequence, open jaws, back up, and return to IDLE.
    """
    def __init__(self, node: BasicNavigatorChild, mt_executor):
        """
        Initialize the CaptureCan class.

        Args:
            node: The ROS2 node to use for communication and navigation.
        """
        self.node = node
        self.mt_executor = mt_executor
        self.logger = self.node.get_logger()

        # Declare and get parameters
        self.node.declare_parameter('goal_x', 1.1)
        self.node.declare_parameter('goal_y', -0.25)
        self.goal_x = self.node.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.node.get_parameter('goal_y').get_parameter_value().double_value
        self.logger.info(f"Goal position set to: x={self.goal_x}, y={self.goal_y}")

        # Servo publisher
        self.servo_pub = self.node.create_publisher(Int32, '/servo', 10)

        # Closest can subscriber
        self.range = None
        self.bearing = None
        self.closest_can_sub = self.node.create_subscription(
            Point,
            '/closest_range_bearing',
            self._closest_can_callback,
            10)

        # Blank forward sector service client
        self.blank_fwd_sector_client = self.node.create_client(SetBool, '/blank_fwd_sector')
        self.future: Future = None

        # Navigation and movement clients
        self.move_client = SingleMoveClient(self.node)

        # State machine variable
        self.capture_sm_state = 'IDLE'

        # Add these new initializations:
        self.target_can_pose = None  # Will store the pose of the can to capture
        self.current_robot_pose = None # Will store the robot's current pose from /odom

        # Add odom subscriber
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        # Initial setup
        self._open_jaws() # Start with jaws open
        if not self._call_blank_fwd_sector(False): # Clear blanking initially
             self.logger.warn("Initial call to clear blank_fwd_sector failed. Proceeding anyway.")
        else:
             self.logger.info("Initial call to clear blank_fwd_sector succeeded.")


    def _closest_can_callback(self, msg: Point):
        """Callback for the /closest_range_bearing topic."""
        self.range = msg.x
        self.bearing = msg.y
        # self.logger.debug(f"Received closest can data: range={self.range}, bearing={self.bearing}")

    def _odom_callback(self, msg: Odometry):
        """Callback for the /odom topic to store the robot's current pose."""
        self.current_robot_pose = msg.pose.pose
        # self.logger.debug(f"CaptureCan Odom updated: x={self.current_robot_pose.position.x:.2f}")

    def _open_jaws(self):
        """Publish command to open jaws."""
        msg = Int32()
        msg.data = 1050
        self.servo_pub.publish(msg)
        self.logger.info("Commanding jaws to OPEN (1050)")

    def _close_jaws(self):
        """Publish command to close jaws."""
        msg = Int32()
        msg.data = 1400
        self.servo_pub.publish(msg)
        self.logger.info("Commanding jaws to CLOSE (1400)")

    def _ros_sleep(self, seconds):
        """Sleeps for a given duration while spinning the ROS node."""
        start_time = self.node.get_clock().now()
        duration = rclpy.duration.Duration(seconds=seconds)
        while rclpy.ok() and (self.node.get_clock().now() - start_time) < duration:
            self.mt_executor.spin_once(timeout_sec=0.01)  # Spin the executor to process callbacks
            # Optional small Python sleep to prevent high CPU usage if needed
            # time.sleep(0.01)

    def _call_blank_fwd_sector(self, value: bool) -> bool:
        """
        Calls the /blank_fwd_sector service.

        Args:
            value: The boolean value to send (True to blank, False to unblank).

        Returns:
            True if the service call was successful, False otherwise.
        """

        # Check if the service is available and any previous future is done
        if not self.blank_fwd_sector_client.wait_for_service(timeout_sec=2.0):
            self.logger.error('/blank_fwd_sector service not available')
            return False
        if self.future is not None and not self.future.done():
            self.future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.get_logger().info("Service Future canceled. The Node took too long to process the service call."
                                   "Is the Service Server still alive?")

        # call the service to blank or unblank the forward sector
        self.logger.info(f"Calling /blank_fwd_sector with value: {value}")
        req = SetBool.Request()
        req.data = value
        self.future = self.blank_fwd_sector_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)

        try:
            if self.future.done():
                try:
                    response = self.future.result()
                    if response.success:
                        self.logger.info(f"/blank_fwd_sector service call successful (data={value})")
                        return True
                    else:
                        self.logger.error(f"/blank_fwd_sector service call failed: {response.message}")
                        return False
                except Exception as e:
                    self.logger.error(f"Service call failed with exception: {e}")
                    return False
            else:
                self.logger.error(f"Service call '/blank_fwd_sector' timed out (data={value})")
                # Attempt to cancel the future? Might not be necessary or effective.
                self.future.cancel()
                return False

        except Exception as e:
            self.get_logger().error(f"service_response_callback: Service call failed or future error: {e!r}")
            return False
        finally:
            # Clean up the future to prevent memory leaks
            self.future = None

    def navToEulerPose(self, target_x, target_y, target_orientation):
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
        """
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.orientation.w = round(math.cos(math.radians(target_orientation) / 2), 3)
        target_pose.pose.orientation.z = round(math.sin(math.radians(target_orientation) / 2), 3)

        self.node.goToPose(target_pose)

        try:
            while not self.node.isTaskComplete():
                feedback = self.node.getFeedback()
                print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
                )
                time.sleep(1.0)

        except KeyboardInterrupt:
            self.logger.info('Navigation interrupted by user!')
            self.node.cancelTask()
        finally:
            result = self.node.getResult()
            if result == TaskResult.SUCCEEDED:
                self.logger.info('Goal succeeded!')
                return True
            elif result == TaskResult.CANCELED:
                self.logger.info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.logger.info('Goal failed!')
            else:
                self.logger.info('Goal has an invalid return status!')
            return False

    # --- State Machine Methods ---

    def _state_idle(self) -> str:
        """IDLE state: Does nothing, waits for start_capture."""
        # This state is primarily entered, not executed actively in the loop.
        # The transition *out* happens in start_capture.
        return 'IDLE' # Should not be called directly in the loop normally

    def _state_rotate(self) -> str:
        """ROTATE state: Rotate towards the target can."""
        if self.target_can_pose is None:
            self.logger.error("Target can pose not set. Cannot rotate.")
            return 'FAIL_RETREAT'

        # Wait for current robot pose if not available
        wait_count = 0
        while self.current_robot_pose is None and wait_count < 20 and rclpy.ok(): # Increased wait time
             self.logger.info("Waiting for current robot pose from /odom...")
             self._ros_sleep(0.1) # Spin and sleep
             wait_count += 1

        if self.current_robot_pose is None:
             self.logger.error("Failed to get current robot pose after waiting.")
             return 'FAIL_RETREAT'

        # Calculate bearing to the target can
        robot_x = self.current_robot_pose.position.x
        robot_y = self.current_robot_pose.position.y
        can_x = self.target_can_pose.position.x
        can_y = self.target_can_pose.position.y

        # Get current robot orientation (yaw)
        _, _, current_robot_yaw = euler_from_quaternion([
            self.current_robot_pose.orientation.x,
            self.current_robot_pose.orientation.y,
            self.current_robot_pose.orientation.z,
            self.current_robot_pose.orientation.w
        ])

        # Calculate desired yaw to face the can
        angle_to_can = math.atan2(can_y - robot_y, can_x - robot_x)

        # Calculate rotation needed (difference between desired yaw and current yaw)
        # Normalize angles to be within -pi to pi if necessary, though SingleMoveClient might handle large angles.
        # The rotation amount is angle_to_can - current_robot_yaw.
        # Positive is counter-clockwise.
        rotation_amount_rad = angle_to_can - current_robot_yaw
        
        # Normalize rotation_amount_rad to the range [-pi, pi]
        while rotation_amount_rad > math.pi:
            rotation_amount_rad -= 2 * math.pi
        while rotation_amount_rad < -math.pi:
            rotation_amount_rad += 2 * math.pi

        self.logger.info(f"Current robot pose: x={robot_x:.2f}, y={robot_y:.2f}, yaw={math.degrees(current_robot_yaw):.1f} deg")
        self.logger.info(f"Target can pose: x={can_x:.2f}, y={can_y:.2f}")
        self.logger.info(f"Angle to can: {math.degrees(angle_to_can):.1f} deg. Rotation needed: {math.degrees(rotation_amount_rad):.1f} deg.")

        # The 'rotate_odom' command likely expects radians
        success = self.move_client.execute_move('rotate_odom', [str(rotation_amount_rad)])

        if success:
            self.logger.info("Rotation towards target can successful.")
            return 'SEEK2CAN'
        else:
            self.logger.error("Rotation towards target can failed.")
            return 'FAIL_RETREAT'

    def _state_seek2can(self) -> str:
        """SEEK2CAN state: Move towards the can using lidar."""
        self.logger.info("Executing seek2can move.")
        success = self.move_client.execute_move('seek2can', [])

        if success:
            self.logger.info("seek2can successful. Closing jaws.")
            self._close_jaws()
            self._ros_sleep(1.0) # Wait for jaws to physically close
            return 'GRASPCAN'
        else:
            self.logger.error("seek2can failed.")
            return 'FAIL_RETREAT'

    def _state_graspcan(self) -> str:
        """GRASPCAN state: Verify can is captured, blank lidar sector."""
        self.logger.info("Waiting for lidar update after grasp...")
        self._ros_sleep(0.5)

        # Check bearing again after grasp attempt
        wait_count = 0
        bearing_after_grasp = self.bearing # Get current bearing
        while bearing_after_grasp is None and wait_count < 5 and rclpy.ok():
             self.logger.info("Waiting for bearing data post-grasp...")
             self._ros_sleep(0.2)
             bearing_after_grasp = self.bearing
             wait_count += 1

        if bearing_after_grasp is None:
             self.logger.error("Failed to get bearing data post-grasp.")
             return 'FAIL_RETREAT'

        self.logger.info(f"Bearing after grasp attempt: {bearing_after_grasp:.3f} radians.")
        if abs(bearing_after_grasp) < 0.25: # Check if can is centered (approx 14 degrees)
            self.logger.info("Can appears centered after grasp. Blanking forward sector.")
            if self._call_blank_fwd_sector(True):
                return 'DRIVE2GOAL'
            else:
                self.logger.error("Failed to blank forward sector.")
                return 'FAIL_RETREAT' # Treat service failure as critical
        else:
            self.logger.error(f"Can not centered after grasp (bearing {bearing_after_grasp:.3f} >= 0.25). Aborting.")
            return 'FAIL_RETREAT'

    def _state_drive2goal(self) -> str:
        """DRIVE2GOAL state: Navigate to the goal location."""
        self.logger.info(f"Navigating to goal: x={self.goal_x}, y={self.goal_y}, orientation=-90.0 deg")
        try:
            success = self.navToEulerPose(self.goal_x, self.goal_y, -90.0)
            if success:
                self.logger.info("Navigation to goal successful.")
                return 'DROP_IN_GOAL'
            else:
                self.logger.error("Navigation to goal failed.")
                return 'FAIL_RETREAT'
        except Exception as e:
            self.logger.error(f"Navigation failed with exception: {e}")
            return 'FAIL_RETREAT'

    def _state_drop_in_goal(self) -> str:
        """DROP_IN_GOAL state: Open jaws, back up, clear blanking, rotate."""
        self.logger.info("Reached goal. Opening jaws.")
        self._open_jaws()
        self._ros_sleep(1.0) # Wait for jaws to open

        self.logger.info("Backing up 0.15m.")
        # Failure here is less critical, but log it.
        if not self.move_client.execute_move('drive_straight_odom', ['-0.4']):
             self.logger.warn("Post-drop backup move failed.")

        self.logger.info("Clearing forward sector blanking.")
        if not self._call_blank_fwd_sector(False):
             self.logger.warn("Failed to clear forward sector blanking.")

        self.logger.info("Rotating 180 degrees (pi radians).")
        # Failure here is less critical, but log it.
        if not self.move_client.execute_move('rotate_odom', [str(math.pi)]):
             self.logger.warn("Post-drop rotation move failed.")

        self.logger.info("Drop sequence complete.")
        return 'IDLE' # Signal successful completion

    def _state_fail_retreat(self) -> str:
        """FAIL_RETREAT state: Open jaws, back up, clear blanking."""
        self.logger.warn("Entering FAIL_RETREAT state.")
        self._open_jaws()
        self._ros_sleep(1.0) # Wait for jaws to open

        self.logger.info("Backing up 0.15m due to failure.")
        # Try to back up, log if it fails but continue cleanup.
        if not self.move_client.execute_move('drive_straight_odom', ['-0.15']):
             self.logger.warn("Retreat backup move failed.")

        self.logger.info("Clearing forward sector blanking after failure.")
        if not self._call_blank_fwd_sector(False):
             self.logger.warn("Failed to clear forward sector blanking during retreat.")

        self.logger.info("Retreat sequence complete.")
        return 'IDLE' # Signal failure completion

    # --- Public Methods ---

    def start_capture(self, target_can_pose: Pose) -> bool:
        """
        Starts the capture sequence state machine for a specific target can.

        Args:
            target_can_pose (Pose): The pose of the can to capture in the odom frame.

        Returns:
            True if the sequence completes successfully (ends via DROP_IN_GOAL),
            False otherwise (ends via FAIL_RETREAT or initial error).
        """
        self.logger.info(f"CaptureCan: Starting capture sequence for can at "
                         f"x={target_can_pose.position.x:.2f}, y={target_can_pose.position.y:.2f}.")
        if self.capture_sm_state != 'IDLE':
            self.logger.warn(f"CaptureCan: Cannot start, already in state '{self.capture_sm_state}'.")
            return False

        self.target_can_pose = target_can_pose # Store the target can pose

        # Initial state transition
        next_state = 'ROTATE'
        last_state_before_idle = None # Track the state that led to IDLE

        while rclpy.ok() and next_state != 'IDLE':
            current_state = next_state
            self.logger.info(f"CaptureCan: Entering state '{current_state}'")
            self.capture_sm_state = current_state # Update internal state

            if current_state == 'ROTATE':
                next_state = self._state_rotate()
            elif current_state == 'SEEK2CAN':
                next_state = self._state_seek2can()
            elif current_state == 'GRASPCAN':
                next_state = self._state_graspcan()
            elif current_state == 'DRIVE2GOAL':
                next_state = self._state_drive2goal()
            elif current_state == 'DROP_IN_GOAL':
                next_state = self._state_drop_in_goal()
            elif current_state == 'FAIL_RETREAT':
                next_state = self._state_fail_retreat()
            else:
                self.logger.error(f"CaptureCan: Unknown state '{current_state}'")
                next_state = 'FAIL_RETREAT' # Force failure state

            # Track the state *before* the transition to IDLE
            if next_state == 'IDLE':
                last_state_before_idle = current_state

            # Small sleep to prevent tight loop if a state returns immediately
            # Only needed if states don't inherently block/sleep/spin
            # self._ros_sleep(0.05)

        # Reached IDLE state
        self.capture_sm_state = 'IDLE'
        self.logger.info("CaptureCan: State machine finished, returning to IDLE.")

        # Determine success based on the state that transitioned to IDLE
        # We check the state *before* the final transition to IDLE
        if last_state_before_idle == 'DROP_IN_GOAL':
            self.logger.info("CaptureCan: Capture sequence completed successfully.")
            return True
        else:
            # This includes FAIL_RETREAT or potentially initial failures
            self.logger.warn(f"CaptureCan: Capture sequence failed or was aborted (last active state: {last_state_before_idle}).")
            return False


def main(args=None):
    """Main function to run the CaptureCan node standalone for testing."""
    rclpy.init(args=args)

    node = BasicNavigatorChild(node_name='capture_can')
    node.get_logger().info("capture_can node starting...")

    # Create a MultiThreadedExecutor for handling multiple threads
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(node)

    # Wait for Nav2 to become active (CRITICAL!)
    node.get_logger().info("Waiting for Nav2 services to become active...")
    node.waitUntilNav2Active()
    node.get_logger().info("Nav2 is active.")

    capture_can = CaptureCan(node, mt_executor)

    # Optional: Add checks for other dependencies like move_client services
    # if not capture_can.move_client.wait_for_servers(timeout_sec=5.0):
    #      node.get_logger().error("Move client action servers not available. Exiting.")
    #      node.destroy_node()
    #      rclpy.shutdown()
    #      sys.exit(1)
    # node.get_logger().info("Move client servers are active.")

    can_chooser_client = CanChooserClient(node)
    node.get_logger().info("Waiting for can chooser service to become available...")
    while not can_chooser_client.client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service /can_chooser_rqst not available, waiting again...')
    node.get_logger().info("Can chooser service is available.")
    node.get_logger().info("Calling can chooser service to select a can...")
    can_chosen, chosen_can_pose_odom = can_chooser_client.choose_can()
    if can_chosen:
        node.get_logger().info(f"Chosen can pose: x={chosen_can_pose_odom.position.x}, y={chosen_can_pose_odom.position.y}")
    else:
        node.get_logger().info("No can chosen. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    node.get_logger().info("Can chooser service call completed.")
    node.get_logger().info("Calling capture_can.start_capture()...")
    # Start the capture sequence with the chosen can pose
    # Note: The chosen can pose is in the odom frame, which is expected by CaptureCan
    # Ensure the pose is valid before passing it to CaptureCan
    if chosen_can_pose_odom is None:
        node.get_logger().error("Chosen can pose is None. Cannot start capture.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info("Starting capture sequence via start_capture()...")
    try:
        success = capture_can.start_capture(chosen_can_pose_odom)

        if success:
            node.get_logger().info("Main: CaptureCan sequence reported SUCCESS.")
            exit_code = 0
        else:
            node.get_logger().error("Main: CaptureCan sequence reported FAILURE.")
            exit_code = 1

    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception during capture sequence: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 2
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C caught, shutting down.")
        exit_code = 1 # Treat interrupt as failure/incomplete


    node.get_logger().info("Shutting down capture_can node.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
