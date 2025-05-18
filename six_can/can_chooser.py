#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point, Pose
from nav_msgs.msg import Odometry
from six_can_interfaces.msg import CanChooserDebug
from six_can_interfaces.srv import CanChooserRqst
import math
import collections

class CanTracker:
    def __init__(self, can_id: int, initial_x: float, initial_y: float, max_persistence: int, window_size: int):
        self.id = can_id
        self.positions_x = collections.deque(maxlen=window_size)
        self.positions_y = collections.deque(maxlen=window_size)
        self.avg_x = 0.0
        self.avg_y = 0.0
        self.max_persistence = max_persistence
        self.persistence_left = max_persistence 
        self.window_size = window_size
        self.update_pose(initial_x, initial_y) # Initialize with first pose

    def update_pose(self, x: float, y: float):
        self.positions_x.append(x)
        self.positions_y.append(y)
        if self.positions_x: 
            self.avg_x = sum(self.positions_x) / len(self.positions_x)
        if self.positions_y: 
            self.avg_y = sum(self.positions_y) / len(self.positions_y)
        self.reset_persistence()

    def get_averaged_point(self) -> Point:
        point = Point()
        point.x = self.avg_x
        point.y = self.avg_y
        point.z = 0.0
        return point

    def decrement_persistence(self):
        self.persistence_left -= 1

    def reset_persistence(self):
        self.persistence_left = self.max_persistence

    def is_stale(self) -> bool:
        return self.persistence_left <= 0

class CanChooser(Node):
    def __init__(self):
        super().__init__('can_chooser') # Node name as per typical convention

        self.declare_parameter('can_persistence', 3) 
        self.declare_parameter('can_pose_variance', 0.1) # meters
        self.declare_parameter('sliding_window_size', 4) # As per requirement doc (implicitly, for averaging)

        self.can_persistence_param = self.get_parameter('can_persistence').get_parameter_value().integer_value
        self.can_pose_variance_param = self.get_parameter('can_pose_variance').get_parameter_value().double_value
        self.sliding_window_size_param = self.get_parameter('sliding_window_size').get_parameter_value().integer_value

        self.tracked_cans: list[CanTracker] = []
        self.current_odom_pose: Pose = None
        self.chosen_can_tracker: CanTracker = None
        self.next_can_id = 0

        self.can_positions_sub = self.create_subscription(
            PoseArray,
            '/can_positions',
            self._can_positions_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10)
        
        self.debug_pub = self.create_publisher(
            CanChooserDebug,
            '/can_chooser_debug',
            10)
        
        self.choose_can_srv = self.create_service(
            CanChooserRqst,
            '/can_chooser_rqst',
            self._choose_can_service_callback)
        
        self.get_logger().info(
            f"CanChooser node started. Persistence: {self.can_persistence_param} cycles, "
            f"Pose Variance Thresh: {self.can_pose_variance_param}m, "
            f"Sliding Window: {self.sliding_window_size_param} samples."
        )

    def _odom_callback(self, msg: Odometry):
        self.current_odom_pose = msg.pose.pose

    def _calculate_distance_points(self, p1: Point, p2: Point) -> float:
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def _can_positions_callback(self, msg: PoseArray):
        updated_trackers_in_current_cycle = set()

        for detected_can_pose in msg.poses:
            detected_point = detected_can_pose.position
            best_match_tracker: CanTracker = None
            min_dist_to_existing = float('inf')

            for tracker in self.tracked_cans:
                dist = self._calculate_distance_points(detected_point, tracker.get_averaged_point())
                if dist < self.can_pose_variance_param and dist < min_dist_to_existing:
                    min_dist_to_existing = dist
                    best_match_tracker = tracker
            
            if best_match_tracker:
                best_match_tracker.update_pose(detected_point.x, detected_point.y)
                updated_trackers_in_current_cycle.add(best_match_tracker)
            else:
                new_tracker = CanTracker(
                    can_id=self.next_can_id,
                    initial_x=detected_point.x,
                    initial_y=detected_point.y,
                    max_persistence=self.can_persistence_param,
                    window_size=self.sliding_window_size_param
                )
                self.next_can_id += 1
                self.tracked_cans.append(new_tracker)
                updated_trackers_in_current_cycle.add(new_tracker)
                self.get_logger().debug(f"New can tracked: ID {new_tracker.id} at ({detected_point.x:.2f}, {detected_point.y:.2f})")

        cans_to_remove = []
        for tracker in self.tracked_cans:
            if tracker not in updated_trackers_in_current_cycle:
                tracker.decrement_persistence()
                if tracker.is_stale():
                    cans_to_remove.append(tracker)
                    self.get_logger().debug(f"Can ID {tracker.id} became stale and will be removed.")

        for stale_tracker in cans_to_remove:
            self.tracked_cans.remove(stale_tracker)
            if self.chosen_can_tracker == stale_tracker:
                self.get_logger().info(f"Chosen can ID {stale_tracker.id} removed as it became stale. Clearing chosen can.")
                self.chosen_can_tracker = None
        
        debug_msg = CanChooserDebug()
        debug_msg.tracked_can_cnt = len(self.tracked_cans)
        
        if self.current_odom_pose:
            debug_msg.robot_x = self.current_odom_pose.position.x
            debug_msg.robot_y = self.current_odom_pose.position.y
        else:
            debug_msg.robot_x = 0.0
            debug_msg.robot_y = 0.0

        if self.chosen_can_tracker and self.current_odom_pose:
            chosen_can_point = self.chosen_can_tracker.get_averaged_point()
            debug_msg.chosen_x = chosen_can_point.x
            debug_msg.chosen_y = chosen_can_point.y
            dx = chosen_can_point.x - self.current_odom_pose.position.x
            dy = chosen_can_point.y - self.current_odom_pose.position.y
            debug_msg.chosen_range = math.hypot(dx, dy)
            debug_msg.chosen_bearing = math.atan2(dy, dx)
        elif self.chosen_can_tracker: 
            chosen_can_point = self.chosen_can_tracker.get_averaged_point()
            debug_msg.chosen_x = chosen_can_point.x
            debug_msg.chosen_y = chosen_can_point.y
            debug_msg.chosen_range = 0.0 # Range unknown without robot pose
            debug_msg.chosen_bearing = 0.0 # Bearing unknown without robot pose
        else: 
            debug_msg.chosen_x = 0.0
            debug_msg.chosen_y = 0.0
            debug_msg.chosen_range = 0.0
            debug_msg.chosen_bearing = 0.0
            
        self.debug_pub.publish(debug_msg)

    def _choose_can_service_callback(self, request: CanChooserRqst.Request, response: CanChooserRqst.Response):
        if request.choose_can:
            if not self.tracked_cans or not self.current_odom_pose:
                self.chosen_can_tracker = None 
                self.get_logger().info("Choose request: No cans currently tracked or robot pose unknown. Cannot choose a new can.")
            else:
                closest_can_tracker_candidate: CanTracker = None
                min_dist_to_robot = float('inf')
                robot_pos = self.current_odom_pose.position

                active_tracked_cans = [tc for tc in self.tracked_cans if not tc.is_stale()]

                if not active_tracked_cans:
                    self.chosen_can_tracker = None
                    self.get_logger().info("Choose request: No active (non-stale) cans available to choose from.")
                else:
                    for tracker in active_tracked_cans:
                        can_point = tracker.get_averaged_point()
                        dist = self._calculate_distance_points(robot_pos, can_point)
                        if dist < min_dist_to_robot:
                            min_dist_to_robot = dist
                            closest_can_tracker_candidate = tracker
                    
                    self.chosen_can_tracker = closest_can_tracker_candidate
                    if self.chosen_can_tracker:
                        chosen_pt = self.chosen_can_tracker.get_averaged_point()
                        self.get_logger().info(
                            f"Choose request: Actively chose can ID {self.chosen_can_tracker.id} "
                            f"at ({chosen_pt.x:.2f}, {chosen_pt.y:.2f}), dist: {min_dist_to_robot:.2f}m."
                        )
                    else: 
                        self.get_logger().warn("Choose request: No suitable can found to choose despite active cans present.")
        
        # Check if the chosen_can_tracker (either newly chosen or pre-existing) is still valid
        if self.chosen_can_tracker:
            if self.chosen_can_tracker.is_stale() or self.chosen_can_tracker not in self.tracked_cans:
                self.get_logger().info(
                    f"Chosen can ID {self.chosen_can_tracker.id} is no longer valid (stale or removed). Clearing chosen can."
                )
                self.chosen_can_tracker = None

        if self.chosen_can_tracker:
            response.can_chosen = True
            chosen_can_point = self.chosen_can_tracker.get_averaged_point()
            response.can_x = chosen_can_point.x
            response.can_y = chosen_can_point.y

            if self.current_odom_pose:
                dx = chosen_can_point.x - self.current_odom_pose.position.x
                dy = chosen_can_point.y - self.current_odom_pose.position.y
                response.range = math.hypot(dx, dy)
                response.bearing_rad = math.atan2(dy, dx)
                self.get_logger().info(
                    f"Service Response: Reporting chosen Can ID {self.chosen_can_tracker.id}. "
                    f"Pose: ({response.can_x:.2f}, {response.can_y:.2f}). "
                    f"Range: {response.range:.2f}m, Bearing: {math.degrees(response.bearing_rad):.1f}deg."
                )
            else: 
                response.range = 0.0
                response.bearing_rad = 0.0
                self.get_logger().info(
                    f"Service Response: Reporting chosen Can ID {self.chosen_can_tracker.id}. "
                    f"Pose: ({response.can_x:.2f}, {response.can_y:.2f}). "
                    f"Range/Bearing: Unknown (no robot odom)."
                )
        else:
            response.can_chosen = False
            response.can_x = 0.0
            response.can_y = 0.0
            response.range = 0.0
            response.bearing_rad = 0.0
            self.get_logger().info("Service Response: No can is currently chosen or available.")
            
        return response

def main(args=None):
    rclpy.init(args=args)
    can_chooser_node = CanChooser()
    try:
        rclpy.spin(can_chooser_node)
    except KeyboardInterrupt:
        can_chooser_node.get_logger().info("CanChooser node shutting down...")
    finally:
        can_chooser_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
