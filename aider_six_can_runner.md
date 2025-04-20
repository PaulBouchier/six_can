# ROS2 Jazzy Node: Find cans in lidar scandata

Ingest the information from this file, implement the low-level tasks, and generate
the code that will satisfy the high and mid-level objectives

## High-level Objective

Create a ROS2 node which drives a robot so as to move soda cans from a competition
arena to beyond a goal line at the opposite end of the arena from where the robot started.
The arena size is 3.05 meters by 2.13 meters in size, with a 0.90 meter wide goal line in
the middle of the narrow side of the arena.

The strategy to be used to move the cans to the goal shall be:

1. Drive from the start end of the arena to the goal end 
2. Turn around to face the arena
3. Find the nearest can that is inside the arena
4. Drive to a 'can_target_pose' position that is 20 cm from the can,
facing the can, positioned on a line from the middle of the arena to the can.
5. Use the drive_waypoints action to drive to a waypoint that is 5cm before the can
6. Close the gripper and call the blank_fwd_scan service passing data = true
7. Drive to the goal end, beyond the goal line. This will carry the can beyond the goal line.
8. Command the 'can_dropper' to release the can and back away from it
9. Repeat from step 2 until no more cans are found  

## Mid-level Objective

Create a state machine which performs the steps above using these resources:
- The ROS2 navigator is called to perform driving tasks
- The ROS2 message '/goal_can_pos' with message type 'geometry_msgs/msg/PointStamped'
contains the position of the nearest can. Only cans inside the arena shall be considered
as targets.

## Implementation Notes
- The node is to be a ROS2 python node
- Be sure to follow the low-level tasks in order and in detail
- Use the current git branch
- add DocStrings;
- comment the code;

### Beginning context

/add six_can/six_can_runner.py

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

### Reference: Minimal ROS2 publisher code

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()  return 0;
```

## Low-level tasks

These tasks are ordered from start to finish

1. Create SixCanRunner class

```aider
- MIRROR the reference minimal publisher code to create a python ROS2 class called
  SixCanRunner
```


6. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```

