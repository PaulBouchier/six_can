# Capture a can and take it to the goal

Ingest the information from this file, implement the low-level tasks, and generate
the code for a ROS2 Jazzy node that will satisfy the high and mid-level objectives

## High level goal

Move the robot to the closest can and close the jaws to capture it.
Then use ROS2 navigation to navigate to the goal. Then open the jaws
to release the can, and back away from it.

## Implementation Notes
- Be sure to follow the low-level tasks in order and in detail
- Use the current git branch
- This will be a python node
- If instantiated by another module, this module will be passed a reference to the ROS
node. If running stand-alone it must initialize ROS and create a node.
- This node should implement a state machine which switches between states as it goes
through the steps to capture a can
- add DocStrings
- comment the code

## Context

This module is used by a higher-level caller which moves several soda cans in an arena
to a goal area. This module is responsible for moving the closest can in front
of the robot to the goal, and reporting success or failure to the caller.

This module is part of a system which includes a 'can_finder' which finds cans
in a 360 degree lidar scan and publishes data about the positions
of cans (in the map frame) in a geometry_msgs/msg/PoseArray message on topic
'/can_positions'. The 'can_finder' also publishes the range and bearing to
the closest can in a geometry_msgs/msg/Point message on topic '/closest_range_bearing'.

### Beginning context

This module will be called when the can it should capture is in front of the robot
and less than 0.4 meters away.

## Reference Code Snippets

Here are examples of how to use the various services that will be required to achieve
the goal

### Service Client

```python
from example_interfaces.srv import AddTwoInts

import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(AddTwoInts, 'add_two_ints')

    req = AddTwoInts.Request()
    req.a = 41
    req.b = 1
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    node.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (req.a, req.b, result.sum))

    node.destroy_node()
    rclpy.shutdown()
```

### SingleMoveClient use example

This snippet shows a class which uses the ScriptedBotClient to rotate 1.57 radians
then drive forward 0.2 meters then backward 0.1 meters.

```python
from scripted_bot_driver.single_move_client import SingleMoveClient


class ScriptedBotRoto:
    def __init__(self, node):
        self.node = node
        self.move_client = SingleMoveClient(node)

    def execute(self):
        self.move_client.execute_move('rotate_odom', "1.57r")  # Example angle in radians
        self.move_client.execute_move('drive_straight_odom', "0.2")  # Example distance in meters
        self.move_client.execute_move('drive_straight_odom', "-0.1")  # Example distance in meters
```

## Mid-level goals

Create a module which can be imported by another module and asked to capture a can and take
it to the goal. This module should make requests to the navigator and to modules in
'scripted_bot_driver' package as specified below to move the robot.

The module will also have a 'main()' method which will activate the state machine to
perform the tasks specified, assuming the robot is close to a can.

This module will have a state machine that steps through the various steps involved in
capturing a can, and handles failures. The steps to capture a can are:

1. Read the bearing to the nearest can from the /closest_range_bearing topic
and rotate to point to the nearest can using the roto action server
2. Seek to the nearest can using the 'seek2can' action server. This should
move the robot so that the can is in the open jaws, but the can may not end up
in the jaws
3. Close the jaws to grasp the can and wait 1 second for the jaws to actuate.
4. Check that the closest can is roughly in the center of the lidar scan.
If not, open the jaws and back away for 0.1 meters and return failure to the caller.
5. Call the '/blank_fwd_sector' service, which removes the sector of lidar data
obscured by the can
6. Command the navigator to drive to the goal pose
7. Open the jaws to release the can and wait 1 second for the jaws to actuate
8. Back away from the can 0.1 meters using the 'drive_straight_odom' action server
9. Return success to the caller

## Low-level tasks

These tasks are ordered from start to finish

1. Create CaptureCan class.

```aider
  /add 'six_can/CaptureCan.py'
  - from scripted_bot_driver.single_move_client import SingleMoveClient
  - Define the class CaptureCan, whose constructor is passed a reference to the
  node. The constructor should:
    - Get the parameters goal_x and goal_y which define the position in the map frame
    of the goal where cans should be dropped, and save them
    - MIRROR the provided Minimal Publisher to create a publisher which publishes
    messages of type 'example_interfaces/msg/Int32' to topic '/servo'
    - MIRROR the provided Minimal Subscriber to create a subscriber which subscribes
    to topic '/closest_range_bearing' and receives messages of type geometry_msgs/msg/Point.
    The callback of this subscriber should save the x value in a class variable 'range' and
    the y value in a class variable 'bearing'.
    - MIRROR the provided Minimal Service client code to add a service client to
    the CaptureCan class. It will use the service '/blank_fwd_sector' with
    type 'example_interfaces/srv/SetBool'.
    - Create a state machine which will step through the steps described below
    to accomplish the high-level goal. The state variable should be called 'capture_sm_state'.
    - Define a method called 'start_capture()' which will start the capture sequence and
    return True if it succeeds, False if it fails.
  - Outside the class definition add a ROS main() function which initializes
  ROS and creates the node and calls start_capture(). This is to test this part of the system.
```

2. Add idle state

```aider
- Define the 'IDLE' state, which transitions to the 'ROTATE' state when 'start_capture()' is called.
```

3. Add 'ROTATE' state

```aider
- Create the 'ROTATE' state.
- Get the bearing to the closest can and call<br>
self.move_client.execute_move('rotate_odom', "1.57r")<br>
to rotate the robot to point at the can.
- When execute_move returns set the next state to 'IDLE' and return, passing the return
value from execute_move() back to the caller
```

## TO DO
- Add example of calling navigator