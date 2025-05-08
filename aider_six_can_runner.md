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

/read-only 'six_can/nav2pose.py'

This file's main() shows how to use the Nav2pose class, which uses ROS
Nav2 navigation to drive the robot to the requested pose. Note that
Nav2pose is not a ROS Node, so do not pass a node to its constructor.

## Mid-level Objective

Create a state machine which performs the steps below using these resources:
- The Nav2pose class is called to perform driving tasks
- The ROS2 message '/goal_can_pos' with message type 'geometry_msgs/msg/PointStamped'
contains the position of the nearest can. Only cans inside the arena shall be considered
as targets.
- Cans are inside the arena if their x and y pose locations are greater than zero
and 

The strategy to be used to move the cans to the goal shall be:

1. Read and parse the yaml file containing the list of search poses
2. Drive from the start position to the next search pose
3. Find the nearest can and check that it is inside the arena
4. Tell the CaptureCan class to move the can to the goal area
5. Drive back to the last search pose from step 2 and repeat steps 3 and 4
until no more cans are found at that search pose, then drive to the next
search pose. If there are no more poses in the list, start over from the
beginning of the search list.

## Low-level tasks

These tasks are ordered from start to finish

In these tasks:
- Drive to a place means use the Nav2Pose class as shown in the 'Nave2Pose use example'
to move the robot from its current pose to the requested pose

1. Create SixCanRunner class

This class will be a ROS2 node.

```aider
/add 'six_can/six_can_runner.py'
/read-only 'six_can/capture_can.py'
  - from .nav2pose import Nav2Pose
  - from .capture_can import CaptureCan
  - from .yaml_parser_node import YamlParserNode
  - Define the class SixCanRunner, whose constructor is passed a reference to the
  node. The constructor should:
    - Get the parameters 'arena_max_x' and 'arena_max_y' which define the maximum extents of the arena.
    The default arena_max_x should be 2.0, default arena_max_y should be 2.0
    - Get the parameter 'search_poses_file' which defines the locations the robot should
    drive to try and find cans. The default should be 'package_share_directory/resource/search_coords.yaml'
    - Read and parse the yaml file containing the list of search poses and save them
    in a list of Pose messages called 'search_poses'. Throw an exception if this operation fails.
    - MIRROR the reference minimal subscriber code to create a subcriber which subscribes
    to topic '/can_detected' and receives messages of type example_interfaces/msg/Bool and
    stores the result in 'can_detected'.
    - MIRROR the provided Minimal Subscriber code to add a subscriber to topic
    '/can_positions' with message type geometry_msgs/msg/PoseArray. The callback should
    save the first pose from the array in a 'closest_can_pose' variable
    - Instantiate the 'CaptureCan' class as 'capture_can'
    - Instantiate the 'Nav2Pose' class as 'nav2pose'
    - Instantiate the 'YamlParserNode' class as 'yaml_parser_node'
```

2. Drive to search poses, look for cans, capture any found

Within the arean means that for the current pose, x and y are > 0 and x < arena_max_x
and y < arena_max_y

```aider
  - Run the following loop forever. When all poses in the list have been visited,
  start over at the beginning of the list.
    - Drive to the next pose in 'search_poses'
    - Check for cans at that location by testing 'can_detected'.
    - If 'can_detected' is true and if the first element
    in 'closest_can_pose' is within the arena, call 'capture_can.start_capture()' which will
    grab the can, deposit it in the goal, and leave the robot facing the arena in the goal area.
    - If 'can_detected' is false, drive to the next pose in 'search_poses'
    - If 'capture_can.start_capture()' returns false, mdrive to the next location in 'search_poses',
    otherwise check 'can_detected' again.

3. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```

