
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Point
from example_interfaces.msg import Int32
from example_interfaces.srv import SetBool

import math

# Import required local modules
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
        DRIVE2GOAL: Navigate to the predefined goal location.
        DROP_IN_GOAL: Release the can at the goal.
        FAIL_RETREAT: Back away if capture or navigation fails.
    """
    # Define states
    IDLE = 0
    ROTATE = 1
    SEEK2CAN = 2
    GRASPCAN = 3
    DRIVE2GOAL = 4
    DROP_IN_GOAL = 5
    FAIL_RETREAT = 6

    def __init__(self, node: Node):
        """
        Initialize the CaptureCan class.

        Args:
            node: The ROS2 node to use for communication.
        """
        self.node = node
        self.logger = self.node.get_logger()

        # --- Task 1: Initialization ---

        # Get goal parameters
        goal_x_descriptor = ParameterDescriptor(description='X coordinate of the goal pose in the map frame.')
        self.node.declare_parameter('goal_x', 0.75, goal_x_descriptor)
        goal_y_descriptor = ParameterDescriptor(description='Y coordinate of the goal pose in the map frame.')
        self.node.declare_parameter('goal_y', 0.3, goal_y_descriptor)
        self.goal_x = self.node.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.node.get_parameter('goal_y').get_parameter_value().double_value
        self.logger.info(f"Goal position set to: x={self.goal_x}, y={self.goal_y}")

        # Create servo publisher
        self.servo_pub = self.node.create_publisher(Int32, '/servo', 10)
        self.logger.info("Servo publisher created.")

        # Open jaws initially
        self._open_jaws()

        # Create closest_range_bearing subscriber
        self.range = 0.0
        self.bearing = 0.0
        self.range_bearing_sub = self.node.create_subscription(
            Point,
            '/closest_range_bearing',
            self._range_bearing_callback,
            10)
        self.logger.info("Closest range/bearing subscriber created.")

        # Create blank_fwd_sector service client
        self.blank_fwd_sector_cli = self.node.create_client(SetBool, '/blank_fwd_sector')
        while not self.blank_fwd_sector_cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('\'/blank_fwd_sector\' service not available, waiting again...')
        self.logger.info("Blank forward sector service client created.")

        # Clear blank_fwd_sector initially
        self._clear_blank_fwd_sector()

        # Instantiate Nav2Pose
        self.nav = Nav2Pose()
        self.logger.info("Nav2Pose instantiated.")

        # Instantiate SingleMoveClient
        self.move_client = SingleMoveClient(self.node)
        self.logger.info("SingleMoveClient instantiated.")

        # Initialize state machine
        self.capture_sm_state = self.IDLE
        self.logger.info("CaptureCan initialized, state machine in IDLE.")
        self._last_bearing_update_time = self.node.get_clock().now()


    def _spin_sleep(self, duration_sec: float):
        """Spin the node for a specified duration, allowing callbacks."""
        start_time = self.node.get_clock().now()
        end_time = start_time + rclpy.duration.Duration(seconds=duration_sec)
        while self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.01) # Spin with a small timeout


    def _range_bearing_callback(self, msg: Point):
        """Callback for the /closest_range_bearing topic."""
        self.range = msg.x
        self.bearing = msg.y
        self._last_bearing_update_time = self.node.get_clock().now()
        self.logger.info(f"Received range: {self.range}, bearing: {self.bearing}", throttle_duration_sec=0.2)


    def _open_jaws(self):
        """Publish command to open jaws."""
        msg = Int32()
        msg.data = 1050
        self.servo_pub.publish(msg)
        self.logger.info("Commanding jaws to OPEN.")
        self._spin_sleep(1.0) # Allow time for servo to move


    def _close_jaws(self):
        """Publish command to close jaws."""
        msg = Int32()
        msg.data = 1400
        self.servo_pub.publish(msg)
        self.logger.info("Commanding jaws to CLOSE.")
        self._spin_sleep(1.0) # Allow time for servo to move


    def _call_blank_fwd_sector_service(self, value: bool):
        """Call the /blank_fwd_sector service."""
        req = SetBool.Request()
        req.data = value
        future = self.blank_fwd_sector_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if future.done() and future.result() is not None:
            self.logger.info(f"Blank forward sector service call successful: set to {value}")
            return future.result().success
        else:
            self.logger.error(f"Failed to call blank forward sector service: set to {value}")
            return False


    def _set_blank_fwd_sector(self):
        """Set the blank_fwd_sector flag to True."""
        return self._call_blank_fwd_sector_service(True)


    def _clear_blank_fwd_sector(self):
        """Set the blank_fwd_sector flag to False."""
        return self._call_blank_fwd_sector_service(False)


    def _back_up(self, distance: float = -0.15):
        """Command the robot to back up."""
        self.logger.info(f"Backing up {abs(distance)}m.")
        return self.move_client.execute_move('drive_straight_odom', [str(distance)])


    def _rotate_in_place(self, angle_rad: float):
        """Command the robot to rotate in place."""
        self.logger.info(f"Rotating in place by {angle_rad:.2f} radians.")
        return self.move_client.execute_move('rotate_odom', [str(angle_rad)])


    def start_capture(self) -> bool:
        """
        Starts the can capture state machine.

        Returns:
            True if the can was successfully captured and delivered, False otherwise.
        """
        self.logger.info("Starting can capture sequence.")
        if self.capture_sm_state != self.IDLE:
            self.logger.warn("Capture sequence requested but state machine not idle.")
            return False # Or handle re-entry differently?

        # --- Task 2: Start in IDLE, transition to ROTATE ---
        self.capture_sm_state = self.ROTATE # Initial state transition
        final_success = False

        while self.capture_sm_state != self.IDLE:
            next_state = self.IDLE # Default transition if a state fails unexpectedly

            if self.capture_sm_state == self.ROTATE:
                next_state = self._do_rotate()
            elif self.capture_sm_state == self.SEEK2CAN:
                next_state = self._do_seek2can()
            elif self.capture_sm_state == self.GRASPCAN:
                next_state = self._do_graspcan()
            elif self.capture_sm_state == self.DRIVE2GOAL:
                next_state = self._do_drive2goal()
            elif self.capture_sm_state == self.DROP_IN_GOAL:
                next_state = self._do_drop_in_goal()
                final_success = True # Reached successful end state
            elif self.capture_sm_state == self.FAIL_RETREAT:
                next_state = self._do_fail_retreat()
                final_success = False # Reached failure end state
            else:
                self.logger.error(f"Unknown state: {self.capture_sm_state}")
                next_state = self.IDLE # Bail out

            self.capture_sm_state = next_state
            # Optional: Add a small sleep to prevent busy-waiting if needed
            # time.sleep(0.1)

        self.logger.info(f"Capture sequence finished. Success: {final_success}")
        return final_success

    # --- State Implementation Methods ---

    def _do_idle(self):
        """State: IDLE. Does nothing, waits for start_capture."""
        # This state is technically handled by the start_capture loop exit condition
        self.logger.debug("State: IDLE")
        return self.IDLE # Should not be called directly in the loop

    # --- Task 3: ROTATE state ---
    def _do_rotate(self):
        """State: ROTATE. Rotate towards the closest can."""
        self.logger.info("State: ROTATE")
        # Wait briefly for fresh bearing data if needed
        if (self.node.get_clock().now() - self._last_bearing_update_time).nanoseconds / 1e9 > 0.5:
             self.logger.warn("Bearing data is stale, waiting briefly...")
             self._spin_sleep(0.5)
             # Check again
             if (self.node.get_clock().now() - self._last_bearing_update_time).nanoseconds / 1e9 > 0.5:
                 self.logger.error("Bearing data did not update. Aborting rotate.")
                 return self.FAIL_RETREAT # Or maybe just IDLE?

        bearing_to_can = self.bearing
        self.logger.info(f"Bearing to can: {bearing_to_can:.2f} radians.")

        if abs(bearing_to_can) < 0.05: # Already pointing close enough
             self.logger.info("Already pointing towards can. Skipping rotation.")
             return self.SEEK2CAN

        if self._rotate_in_place(bearing_to_can):
            self.logger.info("Rotation successful.")
            return self.SEEK2CAN
        else:
            self.logger.error("Rotation failed.")
            # Don't retreat yet, maybe seek will still work if close? Or go straight to fail?
            # Let's try seek first. If seek fails, it will retreat.
            # If strict adherence is needed, change to FAIL_RETREAT
            return self.SEEK2CAN # Or self.FAIL_RETREAT

    # --- Task 4: SEEK2CAN state ---
    def _do_seek2can(self):
        """State: SEEK2CAN. Move towards the can using lidar."""
        self.logger.info("State: SEEK2CAN")
        if self.move_client.execute_move('seek2can', ['']):
            self.logger.info("Seek2Can successful.")
            self._close_jaws()
            return self.GRASPCAN
        else:
            self.logger.error("Seek2Can failed.")
            # No need to open jaws here, FAIL_RETREAT handles it
            return self.FAIL_RETREAT # Transition to failure state

    # --- Task 5: GRASPCAN state ---
    def _do_graspcan(self):
        """State: GRASPCAN. Verify capture and blank lidar sector."""
        self.logger.info("State: GRASPCAN")
        self._spin_sleep(0.5) # Wait for lidar/bearing update after closing jaws/moving

        # Check if bearing data is recent enough
        if (self.node.get_clock().now() - self._last_bearing_update_time).nanoseconds / 1e9 > 1.0:
             self.logger.warn("Bearing data is stale after grasp attempt, assuming failure.")
             # Treat as failure if no recent bearing data
             return self.FAIL_RETREAT

        current_bearing = self.bearing
        self.logger.info(f"Checking bearing after grasp: {current_bearing:.2f} radians.")

        # Check if can is centered (bearing close to zero)
        if abs(current_bearing) >= 0.25:
            self.logger.error(f"Can not centered after grasp (bearing {current_bearing:.2f} >= 0.25). Aborting.")
            # FAIL_RETREAT will open jaws and back up
            return self.FAIL_RETREAT
        else:
            self.logger.info("Can appears centered in jaws.")
            if self._set_blank_fwd_sector():
                self.logger.info("Forward lidar sector blanked.")
                return self.DRIVE2GOAL
            else:
                self.logger.error("Failed to blank forward lidar sector.")
                # If blanking fails, we probably shouldn't navigate
                return self.FAIL_RETREAT

    # --- Task 6: DRIVE2GOAL state ---
    def _do_drive2goal(self):
        """State: DRIVE2GOAL. Navigate to the goal location."""
        self.logger.info("State: DRIVE2GOAL")
        self.logger.info(f"Navigating to goal: x={self.goal_x}, y={self.goal_y}, orientation=90.0 deg")

        try:
            # Wait for Nav2 to be ready if it wasn't already
            # self.nav.waitForNav2Active() # Optional: Add if needed, Nav2Pose constructor might handle this
            success = self.nav.goToPose(self.goal_x, self.goal_y, 90.0)
            if success:
                self.logger.info("Navigation to goal successful.")
                return self.DROP_IN_GOAL
            else:
                self.logger.error("Navigation to goal failed.")
                return self.FAIL_RETREAT
        except Exception as e:
            self.logger.error(f"Navigation failed with exception: {e}")
            return self.FAIL_RETREAT

    # --- Task 7: DROP_IN_GOAL state ---
    def _do_drop_in_goal(self):
        """State: DROP_IN_GOAL. Release can and back away."""
        self.logger.info("State: DROP_IN_GOAL")
        self._open_jaws()
        # Wait handled within _open_jaws
        self._back_up(-0.15) # Back up 0.15m
        self._clear_blank_fwd_sector()
        self._rotate_in_place(math.pi) # Rotate 180 degrees (3.14 radians)
        self.logger.info("Can dropped, backed up, sector cleared, rotated.")
        return self.IDLE # Successful completion, return to IDLE

    # --- Task 8: FAIL_RETREAT state ---
    def _do_fail_retreat(self):
        """State: FAIL_RETREAT. Open jaws, back up, clear sector."""
        self.logger.info("State: FAIL_RETREAT")
        self._open_jaws()
        # Wait handled within _open_jaws
        self._back_up(-0.15) # Back up 0.15m
        self._clear_blank_fwd_sector()
        self.logger.info("Failure retreat sequence completed.")
        return self.IDLE # Return to IDLE after failure


# --- Standalone Execution / Testing ---
def main(args=None):
    """Main function to run the CaptureCan node standalone for testing."""
    rclpy.init(args=args)
    node = rclpy.create_node('capture_can_node')
    node.get_logger().info("CaptureCan node starting for standalone test.")

    capture_can_logic = CaptureCan(node)

    # Example: Start the capture sequence immediately for testing
    # In a real scenario, this might be triggered by another node or event.
    node.get_logger().info("Waiting 2 seconds before starting capture...")
    # Spin for 2 seconds to allow subscriptions etc. to establish
    start_time = node.get_clock().now()
    end_time = start_time + rclpy.duration.Duration(seconds=2.0)
    while node.get_clock().now() < end_time:
        rclpy.spin_once(node, timeout_sec=0.01)

    # Keep node alive while capture runs (spin_until_future_complete handles spinning)
    success = capture_can_logic.start_capture()

    if success:
        node.get_logger().info("Standalone test: CaptureCan sequence completed successfully.")
    else:
        node.get_logger().info("Standalone test: CaptureCan sequence failed or was aborted.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
