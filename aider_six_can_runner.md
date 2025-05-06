# ROS2 Jazzy Node: Find cans in lidar scandata

Ingest the information from this file, implement the low-level tasks, and generate
the code that will satisfy the high and mid-level objectives

## High-level Objective

Create a ROS2 node which drives a robot so as to move six soda cans
from a competition arena to beyond a goal line at the opposite end of
the arena from where the robot started.  The arena size is 3.05 meters
by 2.13 meters in size, with a 0.90 meter wide goal line in the middle
of the narrow side of the arena.

## Implementation Notes
- The node is to be a ROS2 python node
- Be sure to follow the low-level tasks in order and in detail
- Use the current git branch
- add DocStrings;
- comment the code;
- This node should implement a state machine which switches between states
as it goes through the steps to move all six cans to the goal. The state
variable should be called 'six_can_sm_state'. Make each state call a
method that performs the work and returns the next state, so as to aid
testing. Whenever the state machine transitions to a new state print a
diagnostic message.

## Context

This is the highest level module in a system that moves six soda cans to a goal area.
It uses the following resources to accomplish the goal:
- ROS2 navigator class to drive the robot to different poses from where
'can_finder should be called to find if there are any detected cans that
are within the arena
- 'can_finder' class which finds cans in a 360 degree lidar scan and
publishes data about the positions of cans (in the map frame) in a
geometry_msgs/msg/PoseArray message on topic '/can_positions'. The
'can_finder' also publishes the range and bearing to the closest can in
a geometry_msgs/msg/Point message on topic '/closest_range_bearing'.
- 'CaptureCan' class to move to the nearest can and grasp it and carry it to the
goal, drop it, and return success or fail status

### Beginning context

This module will read a list of search coordinates from a yaml file, and
instruct the navigator to drive the robot to each point in order before checking for
cans near that point and moving any found cans to the goal

## Reference code snippets

The following code snippets give examples of how to use the various services that this node will use

### Reference: Minimal ROS2 subscriber code

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Nav2pose use example

This snippet shows how to use the Nav2pose class, which uses ROS Nav2 navigation to drive
the robot to the requested pose. Note that Nav2pose is not a ROS Node, so do not pass
a node to it.

``` python
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
```

## Mid-level Objective

Create a state machine which performs the steps below using these resources:
- The Nav2pose class is called to perform driving tasks
- The ROS2 message '/goal_can_pos' with message type 'geometry_msgs/msg/PointStamped'
contains the position of the nearest can. Only cans inside the arena shall be considered
as targets.
- Cans are inside the arena if their x and y pose locations are greater than zero
and 

The strategy to be used to move the cans to the goal shall be:

1. Read and parse the yaml file containing the list of search coordinates
2. Drive from the start position to the next search coordinate
3. Find the nearest can that is inside the arena
4. Drive to a 'can_target_pose' position that is 20 cm from the can,
facing the can, positioned on a line from the middle of the arena to the can.
5. Use the drive_waypoints action to drive to a waypoint that is 5cm before the can
6. Close the gripper and call the blank_fwd_scan service passing data = true
7. Drive to the goal end, beyond the goal line. This will carry the can beyond the goal line.
8. Command the 'can_dropper' to release the can and back away from it
9. Repeat from step 2 until no more cans are found  

## Low-level tasks

These tasks are ordered from start to finish

1. Create SixCanRunner class

```aider
/add 'six_can/six_can_runner.py'
/read-only 'six_can/CaptureCan.py'
  - from .nav2pose import Nav2Pose
  - Define the class SixCanRunner, whose constructor is passed a reference to the
  node. The constructor should:
    - Get the parameters 'arena_max_x' and 'arena_max_y' which define the maximum extents of the arena
    - Get the parameter 'search_coords_file' which defines the locations the robot should
    drive to try and find cans. The default should be 'package_share_directory/resource/search_coords.yaml'
    - Read and parse the yaml file containing the list of search poses and save them
    in a list of Pose messages called 'search_poses'. Throw an exception if this operation fails.
    - MIRROR the reference minimal subscriber code to create a subcriber which subscribes
    to topic '/can_detected' and receives messages of type example_interfaces/msg/Bool and
    stores the result in 'can_detected'.
    - MIRROR the provided Minimal Subscriber code to add a subscriber to topic
    '/can_positions' with message type geometry_msgs/msg/PoseArray. The callback should
    save the first pose from the array in a 'closest_can_pose' variable
```

2. Drive to search coordinates

```aider
  - Drive to each coordinate in 'search_poses' in turn and check for cans near that location.
  When all locations in the list have been visited, start over at the beginning of the list.
  - At each location in 'search_poses', if 'can_detected' is true and if the first element
  in 'closest_can_pose' is within the arena, call the 'capture_can'
  - 

6. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```

