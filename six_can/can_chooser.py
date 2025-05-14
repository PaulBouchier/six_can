import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import math
import collections

class CanTracker:
    """
    Tracks the state and location of a single can, averaging its position
    and managing its persistence.
    """
    def __init__(self, initial_pose: Pose, persistence_ticks: int, window_size: int = 4):
        self.logger = get_logger(f'CanTracker_{id(self)}') # Unique logger name
        self.poses_buffer = collections.deque(maxlen=window_size)
        self.poses_buffer.append(initial_pose) # Store initial pose
        self.max_persistence = persistence_ticks
        self.current_persistence = persistence_ticks
        # seen_in_current_scan is a flag managed by CanChooser
        self.seen_in_current_scan = True # Initialized as seen because it's just created

    def update_pose(self, new_pose: Pose):
        """Adds a new observed pose and resets persistence."""
        self.poses_buffer.append(new_pose)
        self.current_persistence = self.max_persistence
        self.seen_in_current_scan = True # Mark as seen in the current scan processing

    def get_averaged_pose(self) -> Pose:
        """Calculates the averaged pose from the buffer."""
        if not self.poses_buffer:
            # This case should ideally not be reached if the tracker is active
            # and part of self.tracked_cans, as it's initialized with a pose.
            # However, to be safe, return a default or log an error.
            # For now, let's assume it always has poses if active.
            # If it could be empty, a more robust handling (e.g. returning None)
            # and checking in the caller would be needed.
            # Given current logic, it should always have at least one pose.
            self.logger.error("get_averaged_pose called on an empty poses_buffer. This should not happen.")
            # Fallback to a zero pose to prevent crashes, though this indicates a logic flaw.
            return Pose()


        avg_x = sum(p.position.x for p in self.poses_buffer) / len(self.poses_buffer)
        avg_y = sum(p.position.y for p in self.poses_buffer) / len(self.poses_buffer)
        
        averaged_pose = Pose()
        averaged_pose.position.x = avg_x
        averaged_pose.position.y = avg_y
        # Use the orientation of the most recent pose in the buffer.
        if self.poses_buffer:
            averaged_pose.orientation = self.poses_buffer[-1].orientation 
        return averaged_pose

    def decrement_persistence(self) -> bool:
        """
        Decrements persistence. Called by CanChooser if not seen in a scan.
        Returns True if still alive (persistence > 0), False if expired.
        """
        self.current_persistence -= 1
        return self.current_persistence > 0

class CanChooser:
    """
    Subscribes to can positions, tracks them, and provides a method
    to choose the closest can.
    """
    def __init__(self, node: Node):
        self.node = node
        self.logger = self.node.get_logger()

        # Get parameters
        self.can_persistence = self.node.declare_parameter(
            'can_persistence', 3
        ).get_parameter_value().integer_value
        self.can_pose_variance = self.node.declare_parameter(
            'can_pose_variance', 0.1  # meters
        ).get_parameter_value().double_value
        self.can_pose_variance_sq = self.can_pose_variance ** 2 # For squared distance comparison

        self.tracked_cans = []  # List of CanTracker objects
        self.current_robot_pose = None  # Stores nav_msgs.msg.Odometry.pose.pose (which is a Pose)
        self.last_chosen_can_pose = None # Stores the pose of the last can chosen by choose_can()

        # Subscribe to /can_positions
        self.can_positions_sub = self.node.create_subscription(
            PoseArray,
            '/can_positions',
            self._can_positions_callback,
            10
        )
        # Subscribe to /odom
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )
        self.logger.info(
            f"CanChooser initialized. Persistence ticks: {self.can_persistence}, "
            f"Pose variance for matching: {self.can_pose_variance}m"
        )

    def _odom_callback(self, msg: Odometry):
        """Stores the current robot pose from odometry."""
        self.current_robot_pose = msg.pose.pose
        # self.logger.debug(f"Odom updated: x={self.current_robot_pose.position.x:.2f}, y={self.current_robot_pose.position.y:.2f}")

    def _can_positions_callback(self, msg: PoseArray):
        """
        Processes observed can poses, updates trackers, and manages persistence.
        - A can is new if its x,y pose is more than 'can_pose_variance' from any currently
          tracked can, otherwise associate the pose to the closest already-tracked can.
        - Can poses that are already being tracked should have their x and y positions
          updated by averaging with previous positions with a sliding window of length 4.
        - New cans should be added to the list of tracked cans.
        """
        # self.logger.debug(f"Received {len(msg.poses)} can poses.")

        # Mark all existing trackers as not seen in this current scan pass
        for tracker in self.tracked_cans:
            tracker.seen_in_current_scan = False

        current_observed_poses = msg.poses
        
        for obs_pose in current_observed_poses:
            closest_tracker = None
            min_dist_sq = self.can_pose_variance_sq # Use squared distance for efficiency

            # Try to match obs_pose with an existing tracker
            for tracker in self.tracked_cans:
                tracker_avg_pose = tracker.get_averaged_pose()
                # tracker_avg_pose should not be None if tracker is valid
                dist_sq = (obs_pose.position.x - tracker_avg_pose.position.x)**2 + \
                            (obs_pose.position.y - tracker_avg_pose.position.y)**2
                
                if dist_sq < min_dist_sq:
                        # This observed pose is close enough to this tracker
                        # and it's the closest one found so far for this obs_pose
                        min_dist_sq = dist_sq
                        closest_tracker = tracker
            
            if closest_tracker:
                # This observed pose updates an existing tracker
                closest_tracker.update_pose(obs_pose) # This also sets seen_in_current_scan = True and resets persistence
                # self.logger.debug(f"Updated existing can tracker with pose x={obs_pose.position.x:.2f}, y={obs_pose.position.y:.2f}")
            else:
                # This observed pose does not match any existing tracker closely enough
                # Create a new tracker for this new can
                new_tracker = CanTracker(obs_pose, self.can_persistence)
                self.tracked_cans.append(new_tracker)
                # self.logger.info(f"Added new can tracker at x={obs_pose.position.x:.2f}, y={obs_pose.position.y:.2f}")

        # Manage persistence: decrement for unseen cans and remove if expired (leaky bucket)
        surviving_cans = []
        for tracker in self.tracked_cans:
            if tracker.seen_in_current_scan:
                surviving_cans.append(tracker)
            else: # Not seen in this scan
                if tracker.decrement_persistence(): # Decrement and check if still alive
                    surviving_cans.append(tracker)
                    # avg_p = tracker.get_averaged_pose()
                    # self.logger.debug(f"Can tracker (last at x={avg_p.position.x:.2f}, y={avg_p.position.y:.2f}) not seen, persistence now {tracker.current_persistence}")
                else:
                    avg_p = tracker.get_averaged_pose()
                    self.logger.info(f"Removed can tracker (last at x={avg_p.position.x:.2f}, y={avg_p.position.y:.2f}) due to persistence timeout.")
        self.tracked_cans = surviving_cans
        # self.logger.debug(f"Total tracked cans after persistence update: {len(self.tracked_cans)}")

    def choose_can(self) -> Pose:
        """
        Find the closest tracked can to the current pose saved by the odom callback
        and return its pose to the caller.
        If there are no tracked cans, raise a runtime exception.
        """
        if self.current_robot_pose is None:
            self.logger.warn("Robot pose not yet available from odometry.")
            raise RuntimeError("Robot pose not yet available to choose a can.")

        if not self.tracked_cans:
            # self.logger.info("No cans currently tracked to choose from.")
            raise RuntimeError("No cans available to choose from.")

        closest_can_pose = None
        min_dist_sq_to_robot = float('inf')

        robot_x = self.current_robot_pose.position.x
        robot_y = self.current_robot_pose.position.y

        for tracker in self.tracked_cans:
            can_avg_pose = tracker.get_averaged_pose()
            if can_avg_pose: # Should always be true for an active tracker
                dist_sq = (can_avg_pose.position.x - robot_x)**2 + \
                          (can_avg_pose.position.y - robot_y)**2
                if dist_sq < min_dist_sq_to_robot:
                    min_dist_sq_to_robot = dist_sq
                    closest_can_pose = can_avg_pose
        
        if closest_can_pose:
            # self.logger.info(f"Chose can at x={closest_can_pose.position.x:.2f}, y={closest_can_pose.position.y:.2f}")
            return closest_can_pose
        else:
            # This case should ideally not be reached if self.tracked_cans is not empty
            # and trackers always return a valid averaged pose.
            self.logger.error("Could not determine closest can, though cans are tracked. This indicates an issue.")
            raise RuntimeError("Failed to choose a can despite tracked cans existing.")

    def get_all_tracked_cans_poses(self) -> list[Pose]:
        """Helper method to get all current (averaged) can poses for testing/logging."""
        poses = []
        for tracker in self.tracked_cans:
            pose = tracker.get_averaged_pose()
            if pose:
                poses.append(pose)
        return poses

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('can_chooser_tester_node') # Node for testing CanChooser
    
    # Instantiate CanChooser, passing it the node reference
    can_chooser = CanChooser(node)
    
    # Loop forever, printing (x,y) for all the tracked cans every 5 seconds,
    # ordered by increasing x within increasing y.
    # If there are no tracked cans and the class raises an exception, catch it and print "No Cans"
    
    timer_period = 5.0  # seconds
    
    def timer_callback():
        node.get_logger().info("--- Timer Tick ---")
        
        # 1. Print all tracked cans
        all_cans_poses = can_chooser.get_all_tracked_cans_poses() 
        if all_cans_poses:
            sorted_cans = sorted(all_cans_poses, key=lambda p: (p.position.y, p.position.x))
            node.get_logger().info(f"Currently tracked cans ({len(sorted_cans)}):")
            for i, pose in enumerate(sorted_cans):
                node.get_logger().info(
                    f"  Can {i+1}: (x={pose.position.x:.2f}, y={pose.position.y:.2f})"
                )
        else:
            node.get_logger().info("No cans currently tracked.")

        # 2. Call choose_can()
        chosen_can_pose_from_call = None
        try:
            node.get_logger().info("Calling choose_can()...")
            chosen_can_pose_from_call = can_chooser.choose_can() 
            # choose_can() logs its own details. Main logs the returned pose as per spec.
            if chosen_can_pose_from_call:
                 node.get_logger().info(
                     f"Main: choose_can() returned pose: x={chosen_can_pose_from_call.position.x:.2f}, "
                     f"y={chosen_can_pose_from_call.position.y:.2f}"
                 )
        except RuntimeError as e:
            if "No cans available" in str(e):
                node.get_logger().info(f"Main: No Cans exception from choose_can(): {e}")
            else:
                node.get_logger().warn(f"Main: RuntimeError from choose_can(): {e}")
        
        # 3. Call get_choice_range_bearing() and print its result
        try:
            node.get_logger().info("Calling get_choice_range_bearing()...")
            r, b = can_chooser.get_choice_range_bearing()
            node.get_logger().info(
                f"Main: get_choice_range_bearing() returned: Range={r:.2f}m, Bearing={math.degrees(b):.1f} deg."
            )
        except RuntimeError as e:
            node.get_logger().warn(f"Main: RuntimeError from get_choice_range_bearing(): {e}")
        
        node.get_logger().info("--- End Timer Tick ---")

    node.create_timer(timer_period, timer_callback)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
