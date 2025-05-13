import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from nav_msgs.msg import Odometry
import math
from collections import deque
import numpy as np

# Helper class to track individual cans based on aider_can_chooser.md Task 1 & 3
class CanTracker:
    def __init__(self, initial_pose: Pose, persistence: int, window_size: int = 4):
        self.positions = deque(maxlen=window_size)
        # Store position as tuple (x, y) for easier averaging
        self.positions.append((initial_pose.position.x, initial_pose.position.y))
        # Store the full pose, primarily for orientation from the latest observation
        self.latest_pose = initial_pose
        self._update_averaged_position() # Initialize averaged position
        self.persistence_max = persistence
        self.persistence_count = persistence

    def _update_averaged_position(self):
        """Calculates the average x, y from the deque."""
        avg_x = np.mean([p[0] for p in self.positions])
        avg_y = np.mean([p[1] for p in self.positions])
        # Update the position part of the latest_pose with the averaged values
        self.latest_pose.position.x = avg_x
        self.latest_pose.position.y = avg_y
        # Orientation is kept from the latest observation in add_observation

    def add_observation(self, pose: Pose):
        """Adds a new observation, updates average, and resets persistence."""
        self.positions.append((pose.position.x, pose.position.y))
        self.latest_pose = pose # Update to the latest pose (including orientation)
        self._update_averaged_position() # Recalculate average position
        self.persistence_count = self.persistence_max # Reset persistence

    def get_pose(self) -> Pose:
        """Returns the pose with averaged position and latest orientation."""
        return self.latest_pose

    def get_position(self) -> Point:
        """Returns the averaged position."""
        return self.latest_pose.position

    def distance_to(self, x: float, y: float) -> float:
        """Calculates distance from the tracker's averaged position."""
        avg_pos = self.get_position()
        return math.sqrt((avg_pos.x - x)**2 + (avg_pos.y - y)**2)

    def decrement_persistence(self) -> bool:
        """Decrements persistence count. Returns True if count reaches zero or less."""
        self.persistence_count -= 1
        return self.persistence_count <= 0

# Main class based on aider_can_chooser.md Task 1
class CanChooser(Node):
    def __init__(self):
        super().__init__('can_chooser')

        # Parameters (Task 1)
        self.declare_parameter('can_persistence', 3)
        self.declare_parameter('can_pose_variance', 0.1) # Meters
        self.can_persistence = self.get_parameter('can_persistence').get_parameter_value().integer_value
        self.can_pose_variance = self.get_parameter('can_pose_variance').get_parameter_value().double_value
        # Use squared distance for comparison efficiency
        self.can_pose_variance_sq = self.can_pose_variance ** 2

        self.get_logger().info(f"CanChooser node started.")
        self.get_logger().info(f"  can_persistence: {self.can_persistence}")
        self.get_logger().info(f"  can_pose_variance: {self.can_pose_variance} m")

        # Internal state
        self.tracked_cans: list[CanTracker] = [] # List of active CanTracker objects
        self.current_odom_pose: Pose | None = None # Store Pose from Odometry

        # Subscriptions (Task 1)
        self.can_sub = self.create_subscription(
            PoseArray,
            '/can_positions',
            self._can_positions_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10)

    # Odom Callback (Task 2)
    def _odom_callback(self, msg: Odometry):
        """Stores the robot's current pose from odometry."""
        self.current_odom_pose = msg.pose.pose
        # self.get_logger().debug(f"Received odom: {self.current_odom_pose.position.x:.2f}, {self.current_odom_pose.position.y:.2f}")

    # Can Positions Callback (Task 3)
    def _can_positions_callback(self, msg: PoseArray):
        """Processes observed can poses, updates trackers, and manages persistence."""
        # self.get_logger().debug(f"Received {len(msg.poses)} can poses.")
        observed_poses = msg.poses
        matched_tracker_indices = set() # Keep track of which trackers were updated this cycle

        # --- Association Phase ---
        # For each observed pose, find the best matching existing tracker (if any)
        potential_matches = [] # List of (obs_idx, track_idx, dist_sq)
        for obs_idx, obs_pose in enumerate(observed_poses):
            best_match_track_idx = -1
            min_dist_sq = self.can_pose_variance_sq # Only consider matches within variance

            for track_idx, tracker in enumerate(self.tracked_cans):
                # Calculate squared distance from observed pose to tracker's avg position
                tracker_pos = tracker.get_position()
                dist_sq = (tracker_pos.x - obs_pose.position.x)**2 + \
                          (tracker_pos.y - obs_pose.position.y)**2

                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    best_match_track_idx = track_idx

            if best_match_track_idx != -1:
                potential_matches.append((obs_idx, best_match_track_idx, min_dist_sq))

        # Resolve conflicts: if multiple observations match the same tracker,
        # choose the closest one. If one observation matches multiple trackers,
        # it's already handled by choosing the one with min_dist_sq.
        # We need to ensure one observation updates at most one tracker,
        # and one tracker is updated by at most one observation (the closest one).

        final_matches = {} # track_idx -> (obs_idx, dist_sq)
        matched_obs_indices = set()

        # Sort by distance to prioritize closer matches
        potential_matches.sort(key=lambda x: x[2])

        for obs_idx, track_idx, dist_sq in potential_matches:
            if obs_idx not in matched_obs_indices and track_idx not in final_matches:
                final_matches[track_idx] = (obs_idx, dist_sq)
                matched_obs_indices.add(obs_idx)
                matched_tracker_indices.add(track_idx) # Mark tracker as updated

        # --- Update Phase ---
        # Update matched trackers
        for track_idx, (obs_idx, _) in final_matches.items():
            self.tracked_cans[track_idx].add_observation(observed_poses[obs_idx])
            # self.get_logger().debug(f"Updated tracker {track_idx} with observation {obs_idx}")

        # Add new trackers for unmatched observations
        for obs_idx, obs_pose in enumerate(observed_poses):
            if obs_idx not in matched_obs_indices:
                new_tracker = CanTracker(obs_pose, self.can_persistence)
                self.tracked_cans.append(new_tracker)
                # self.get_logger().debug(f"Created new tracker for observation {obs_idx}")

        # --- Persistence Phase ---
        # Decrement persistence for trackers that were *not* matched/updated
        cans_to_remove = []
        for i, tracker in enumerate(self.tracked_cans):
            if i not in matched_tracker_indices:
                if tracker.decrement_persistence():
                    cans_to_remove.append(tracker)
                    # self.get_logger().debug(f"Removing stale tracker {i} (persistence {tracker.persistence_count})")

        # Remove stale trackers
        for tracker in cans_to_remove:
            self.tracked_cans.remove(tracker)

        # self.get_logger().debug(f"Tracked cans after update: {len(self.tracked_cans)}")


    # Choose Can Method (Task 4)
    def choose_can(self) -> Pose:
        """
        Finds the closest tracked can to the robot's current odometry pose.

        Returns:
            Pose: The pose (averaged position, latest orientation) of the closest can.

        Raises:
            RuntimeError: If no cans are tracked or odometry is unavailable.
        """
        if not self.tracked_cans:
            raise RuntimeError("No cans are currently tracked.")
        if self.current_odom_pose is None:
            # Wait briefly for odom, maybe it just hasn't arrived yet
            # This isn't ideal in a callback-driven system but helps initial startup
            # A better approach might involve waiting externally or using state machines
            self.get_logger().warn("choose_can called before odometry received. Waiting briefly...")
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_odom_pose is None:
                 raise RuntimeError("Current odometry pose not available.")

        robot_x = self.current_odom_pose.position.x
        robot_y = self.current_odom_pose.position.y

        min_dist_sq = float('inf')
        closest_can_pose: Pose | None = None

        for tracker in self.tracked_cans:
            can_pos = tracker.get_position() # Use averaged position
            dist_sq = (can_pos.x - robot_x)**2 + (can_pos.y - robot_y)**2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest_can_pose = tracker.get_pose() # Get pose with avg pos / latest orientation

        if closest_can_pose is None:
             # This should not happen if tracked_cans is not empty and odom is available
             raise RuntimeError("Could not determine closest can despite having tracked cans.")

        # self.get_logger().debug(f"Chose can at ({closest_can_pose.position.x:.2f}, {closest_can_pose.position.y:.2f})")
        return closest_can_pose

    def get_tracked_cans(self) -> list[CanTracker]:
        """Returns the current list of CanTracker objects."""
        # Ensure thread safety if accessed from multiple callbacks/timers, though
        # in this simple case it might be okay. Consider locks if complexity grows.
        return list(self.tracked_cans) # Return a copy

# Main function for testing (Task 5)
def main(args=None):
    rclpy.init(args=args)
    can_chooser = CanChooser()

    # Timer callback for periodic status printing
    def print_status():
        try:
            # Get current trackers directly instead of calling choose_can for printing
            tracked_cans = can_chooser.get_tracked_cans()

            if not tracked_cans:
                can_chooser.get_logger().info("No Cans")
            else:
                # Sort cans by y, then x for consistent output
                # Access position via get_position() method of CanTracker
                sorted_cans = sorted(tracked_cans, key=lambda c: (c.get_position().y, c.get_position().x))

                can_chooser.get_logger().info("Tracked Cans:")
                for i, can_tracker in enumerate(sorted_cans):
                    pos = can_tracker.get_position() # Get averaged position
                    can_chooser.get_logger().info(
                        f"  {i}: ({pos.x:.2f}, {pos.y:.2f}) "
                        f"Persist: {can_tracker.persistence_count}/{can_tracker.persistence_max}"
                    )
        except Exception as e:
            # Catch potential errors during status printing
            can_chooser.get_logger().error(f"Error in print_status: {e}")

    timer_period = 5.0  # seconds
    timer = can_chooser.create_timer(timer_period, print_status)

    try:
        rclpy.spin(can_chooser)
    except KeyboardInterrupt:
        can_chooser.get_logger().info("Shutting down CanChooser node.")
    finally:
        # Cleanup
        if rclpy.ok():
            can_chooser.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
