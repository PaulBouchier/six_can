import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor

from example_interfaces.msg import Int32
from example_interfaces.srv import SetBool
from geometry_msgs.msg import Point

import time
import math
import sys

# Local imports
from .nav2pose import Nav2Pose
from scripted_bot_driver.single_move_client import SingleMoveClient

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
    def __init__(self, node: Node):
        """
        Initialize the CaptureCan class.

        Args:
            node: The ROS2 node to use for communication.
        """
        self.node = node
        self.logger = self.node.get_logger()

        # Declare and get parameters
        self.node.declare_parameter('goal_x', 0.75)
        self.node.declare_parameter('goal_y', 0.3)
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

        # Navigation and movement clients
        self.nav = Nav2Pose(self.node) # Pass the node to Nav2Pose
        self.move_client = SingleMoveClient(self.node)

        # State machine variable
        self.capture_sm_state = 'IDLE'

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
            rclpy.spin_once(self.node, timeout_sec=0.05)
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
        if not self.blank_fwd_sector_client.wait_for_service(timeout_sec=2.0):
            self.logger.error('/blank_fwd_sector service not available')
            return False

        req = SetBool.Request()
        req.data = value
        future = self.blank_fwd_sector_client.call_async(req)

        # Use executor to spin until future is complete
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        try:
             executor.spin_until_future_complete(future, timeout_sec=3.0) # Adjust timeout if needed
        finally:
             executor.remove_node(self.node)


        if future.done():
            try:
                response = future.result()
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
            # future.cancel()
            return False

    # --- State Machine Methods ---

    def _state_idle(self) -> str:
        """IDLE state: Does nothing, waits for start_capture."""
        # This state is primarily entered, not executed actively in the loop.
        # The transition *out* happens in start_capture.
        return 'IDLE' # Should not be called directly in the loop normally

    def _state_rotate(self) -> str:
        """ROTATE state: Rotate towards the closest can."""
        # Wait briefly for bearing data if not available
        wait_count = 0
        while self.bearing is None and wait_count < 10 and rclpy.ok():
             self.logger.info("Waiting for bearing data...")
             self._ros_sleep(0.2)
             wait_count += 1

        if self.bearing is None:
             self.logger.error("Failed to get bearing data after waiting.")
             return 'FAIL_RETREAT'

        bearing_rad_str = str(self.bearing)
        self.logger.info(f"Rotating by {bearing_rad_str} radians.")
        success = self.move_client.execute_move('rotate_odom', [bearing_rad_str])

        if success:
            self.logger.info("Rotation successful.")
            # Reset bearing after use? Maybe not necessary if sub keeps updating.
            # self.bearing = None
            return 'SEEK2CAN'
        else:
            self.logger.error("Rotation failed.")
            return 'FAIL_RETREAT'

    def _state_seek2can(self) -> str:
        """SEEK2CAN state: Move towards the can using lidar."""
        self.logger.info("Executing seek2can move.")
        success = self.move_client.execute_move('seek2can', ['']) # Argument needed, even if empty

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
        self.logger.info(f"Navigating to goal: x={self.goal_x}, y={self.goal_y}, orientation=90.0 deg")
        try:
            # Ensure Nav2 is ready before commanding
            # waitForNav2Active might be better placed before starting the whole sequence,
            # but double-checking here can't hurt if it's quick.
            # self.nav.waitForNav2Active(timeout_sec=5.0) # Optional check with timeout

            success = self.nav.goToPose(self.goal_x, self.goal_y, 90.0)
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
        if not self.move_client.execute_move('drive_straight_odom', ['-0.15']):
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

    def start_capture(self) -> bool:
        """
        Starts the capture sequence state machine.

        Returns:
            True if the sequence completes successfully (ends via DROP_IN_GOAL),
            False otherwise (ends via FAIL_RETREAT or initial error).
        """
        self.logger.info("CaptureCan: Starting capture sequence.")
        if self.capture_sm_state != 'IDLE':
            self.logger.warn(f"CaptureCan: Cannot start, already in state '{self.capture_sm_state}'.")
            return False

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
    node = rclpy.create_node('capture_can_node')
    node.get_logger().info("CaptureCan node starting...")

    # Allow time for node discovery and service/topic setup, especially Nav2
    node.get_logger().info("Waiting a few seconds for system startup...")
    time.sleep(3.0) # Simple delay, replace with more robust checks if needed

    capture_can = CaptureCan(node)

    # Wait for Nav2 to become active (CRITICAL!)
    node.get_logger().info("Waiting for Nav2 services to become active...")
    # Assuming Nav2Pose has waitForNav2Active as shown in example
    # Need to ensure Nav2Pose spins the node while waiting
    if not capture_can.nav.waitForNav2Active(timeout_sec=30.0): # Generous timeout
         node.get_logger().error("Nav2 failed to become active. Exiting.")
         node.destroy_node()
         rclpy.shutdown()
         sys.exit(1)
    node.get_logger().info("Nav2 is active.")

    # Optional: Add checks for other dependencies like move_client services
    # if not capture_can.move_client.wait_for_servers(timeout_sec=5.0):
    #      node.get_logger().error("Move client action servers not available. Exiting.")
    #      node.destroy_node()
    #      rclpy.shutdown()
    #      sys.exit(1)
    # node.get_logger().info("Move client servers are active.")


    node.get_logger().info("Starting capture sequence via start_capture()...")
    try:
        success = capture_can.start_capture()

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


    node.get_logger().info("Shutting down CaptureCan node.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
