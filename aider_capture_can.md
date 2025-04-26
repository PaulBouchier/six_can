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
through the steps to capture a can. The state variable should be called 'capture_sm_state'.
Make each state call a method that performs the work and returns the next state, so
as to aid testing.
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
then drive forward 0.2 meters then backward 0.1 meters then to seek to a can.

The execute_move() method blocks until the move is complete or fails, and returns
True for success, False otherwise.

```python
import rclpy
from rclpy.node import Node

from scripted_bot_driver.single_move_client import SingleMoveClient

'''
This script uses the SingleMoveClient to execute a rotation move, followed
by two straight moves then a seek2can.
'''

def main(argv=None):
    rclpy.init()
    node = Node('scripted_bot_roto')

    move_client = SingleMoveClient(node)
    move_client.execute_move('rotate_odom', ['1.57'])  # Example angle in radians
    move_client.execute_move('drive_straight_odom', ['0.2'])  # Drive forward, example distance in meters
    move_client.execute_move('drive_straight_odom', ['-0.1'])  # Drive backward, example distance in meters
    move_client.execute_move('seek2can', [])  # Seek to can by tracking with lidar
    rclpy.spin_once(node, timeout_sec=1.0)  # Allow time for the action to complete

    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    main()
```

### Nav2Pose use example

```python
def main():
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
3. Close the jaws to grasp the can
4. Check that the closest can is roughly in the center of the lidar scan.
If not, open the jaws and back away for 0.1 meters and return failure to the caller.
5. Call the '/blank_fwd_sector' service, which removes the sector of lidar data
obscured by the can
6. Command the navigator to drive to the goal pose
7. Open the jaws to release the can and wait 1 second for the jaws to actuate
8. Back away from the can 0.1 meters using the 'drive_straight_odom' action server
9. Return success to the caller

## Low-level tasks

These tasks are ordered from start to finish.

In these tasks:
- "Set blank_fwd_sector" means call the /blank_fwd_sector with data=True
- "Clear blank_fwd_sector" means call the /blank_fwd_sector with data=False
- "Open jaws" means publish a value of 1050 to /servo
- "Close jaws" means publish a value of 1400 to /servo
- "Back up 0.15m" means call self.move_client.execute_move('drive_straight_odom', [-0.15']) as shown
in the move_client example
- "Rotate in place by x radians" means call self.move_client.execute_move('rotate_odom', [x]) where
x must be a string containing the number of radians to rotate.

1. Create CaptureCan class.

```aider
  /add 'six_can/CaptureCan.py'
  - from .nav2pose import Nav2Pose
  - from scripted_bot_driver.single_move_client import SingleMoveClient
  - Define the class CaptureCan, whose constructor is passed a reference to the
  node. The constructor should:
    - Get the parameters goal_x and goal_y which define the position in the map frame
    of the goal where cans should be dropped, and save them. The defaults should be
    goal_x=0.75, goal_y=0.3
    - MIRROR the provided Minimal Publisher to create a publisher which publishes
    messages of type 'example_interfaces/msg/Int32' to topic '/servo'
    - Open jaws
    - MIRROR the provided Minimal Subscriber to create a subscriber which subscribes
    to topic '/closest_range_bearing' and receives messages of type geometry_msgs/msg/Point.
    The callback of this subscriber should save the x value in a class variable 'range' and
    the y value in a class variable 'bearing'.
    - MIRROR the provided Minimal Service client code to add a service client to
    the CaptureCan class. It will use the service '/blank_fwd_sector' with
    type 'example_interfaces/srv/SetBool'.
    - Clear blank_fwd_sector
    - Instantiate Nav2Pose as nav
    - Create a state machine which will step through the steps described below
    to accomplish the high-level goal. 
    - Define a method called 'start_capture()' which will start the capture state machine and
    return True if it succeeds, False if it fails.
  - Outside the class definition add a ROS main() function which initializes
  ROS and creates the node and calls start_capture(). This is to test this part of the system.
```

2. Add idle state

```aider
- Define the 'IDLE' state, which transitions to the 'ROTATE' state when 'start_capture()' is called.
- The state machine starts in the 'IDLE' state, and returns to it when it completes or abandons
the goal.
```

3. Add 'ROTATE' state

Rotate to point at can, because initial position might not be pointing directly at the can.

```aider
- Create the 'ROTATE' state.
- Get the bearing to the closest can
- Rotate in place bearing radians. This causes the robot to point at the can.
Note that bearing must be a string containing a float that represents the number of radians to rotate
- If execute_move returns True, transition to 'SEEK2CAN', otherwise print an error and transition to 'IDLE'
```

4. Add 'SEEK2CAN' state

Command execute_move to use lidar to drive directly to the can and stop when can is in
the jaws.

```aider
- Create the 'SEEK2CAN' state.
- Call self.move_client.execute_move('seek2can', ['']) to drive the robot to the can.
- If execute_move() returns False, print an error, transition to 'IDLE' state and return False to caller
- If execute_move() returns True, Close jaws, and transition to 'GRASPCAN'
```

5. Add 'GRASPCAN' state

Make sure the can is inside the jaws and not alongside them then close jaws

```aider
- wait 0.5 seconds for lidar data to update, then get 'bearing' and check that its absolute
value is less than 0.25. If >= 0.25 do the following:
    - print an error message
    - Transition to 'FAIL_RETREAT' state
- Set blank_fwd_sector
- Transition to 'DRIVE2GOAL' state
```

6. Add 'DRIVE2GOAL' state

```aider
- Drive to the goal_x, goal_y, with heading 90.0 using the nav object as shown in
the usage example
- If goToPose() returns false or an exception, print an error message and
transition to 'FAIL_RETREAT' state
- Transition to 'DROP_IN_GOAL' state
```

7. Add 'DROP_IN_GOAL' state

```aider
- Open jaws
- Back up 0.15m
- Clear blank_fwd_sector
- Rotate in place 3.14 radians
```

8. Add 'FAIL_RETREAT' state

```aider
- Open jaws
- Back up 0.15m
- Clear blank_fwd_sector
- Transition to the 'IDLE' state and return False to caller

```

## TO DO