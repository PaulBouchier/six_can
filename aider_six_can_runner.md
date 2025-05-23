# ROS2 Jazzy Node: Find cans in lidar scandata

Ingest the information from this file, implement the low-level tasks, and generate
the code that will satisfy the high and mid-level objectives

## High-level Objective

Create a ROS2 node which drives a robot so as to move six soda cans
from a competition arena to beyond a goal line at the opposite end of
the arena from where the robot started.  The arena size is 3.05 meters
by 2.13 meters in size, with a 0.90 meter wide goal line in the middle
of the narrow side of the arena. For testing, allow parameters to change
the arena size.

## Implementation Notes
- The node is to be a ROS2 node in a python file called six_can_runner.py
- Be sure to follow the low-level tasks in order and in detail
- Use the current git branch
- add DocStrings
- comment the code

## Context

This is the highest level module in a system that moves six soda cans to a goal area.
It uses the following resources to accomplish the goal:
- 'BasicNavigator' parent class to drive the robot to different poses from where
'CanTracker.choose_can()' is called to chose a can to grab.
- 'CaptureCan' class to move to the chosen can and grasp it and carry it to the
goal, drop it, and return success or fail status

There is a 'can_finder' node in the system which finds cans in a
360 degree lidar scan and publishes data about the positions of cans
(in the odom frame) in a geometry_msgs/msg/PoseArray message on topic
'/can_positions'. The 'can_finder' also publishes the range and bearing
to the closest can in a geometry_msgs/msg/Point message on topic
'/closest_range_bearing'. It also publishes a example_msgs/msg/Bool
message on topic '/can_detected' which indicates whether a can was
detected at this location.

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

### Nav2Pose use example

/read-only 'six_can/nav2pose.py'

This file's main() shows how to use the Nav2Pose class, which uses ROS
Nav2 navigation to drive the robot to the requested pose. Note that
Nav2pose is a ROS Node because it subclasses the 'BasicNavigator', which
subclasses 'Node'

## Mid-level Objective

Create a routine which moves cans to the goal using these steps:

1. Read and parse the yaml file containing the list of search poses
2. Drive from the start position to the next search pose
3. Use the 'CanChooser' class to choose a can (if one is seen)
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
Create 'six_can/six_can_runner.py', which is the file that will contain the SixCanRunner class
/read-only 'six_can/capture_can.py'
  - from .capture_can import CaptureCan
  - from six_can_interfaces.srv import CanChooserRqst
  - Define the class SixCanRunner, whose constructor calls super.__init__()
  The constructor should:
    - Get the parameters 'arena_max_x' and 'arena_max_y' which define the maximum extents of the arena.
    The default arena_max_x should be 2.0, default arena_max_y should be 2.0
    - Get the parameter 'search_poses_file' which defines the locations the robot should
    drive to try and find cans. The default should be 'package_share_directory/resource/search_coords.yaml'
    - Read and parse the yaml file containing the list of search poses and save them
    in a list of Pose messages called 'search_poses'. Throw an exception if this operation fails.
    - Create a service client for the '/can_chooser_rqst' service and check that the service is available
    - Instantiate the 'CaptureCan' class as 'capture_can'
```

2. Define 'choose_can' method

```aider
Define a method which makes a service call to the '/choose_can_rqst' requesting
the can chooser node to choose a can. Return the resulting can position as a pose,
along with a flag which indicates whether a can was chosen
```

3. Drive to search poses, look for cans, capture any found

Within the arean means that for the current pose, x and y are > 0 and x < arena_max_x
and y < arena_max_y

```aider
  - Run the following loop forever. When all poses in the list have been visited,
  start over at the beginning of the list.
    - Drive to the next pose in 'search_poses'
    - Check for cans at that location by calling 'choose_can()'.
    - Check that the return status indicates a can was chosen.
    - 'choose_can()' returns the pose in the odom frame of a can that should be captured.
    - If no can was chosen, drive to the next pose in 'search_poses'
    - If a can was chosen, call 'capture_can.start_capture()' passing the pose in the odom frame.
    This will grab the can, deposit it in the goal, and leave the robot facing the arena in the goal area.
    - If 'capture_can.start_capture()' returns false, drive to the next location in 'search_poses',
    otherwise go back to the current pose and check for any more cans
```

3. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```

