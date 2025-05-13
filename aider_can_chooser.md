# ROS2 Jazzy Node: Choose a can (if one can be found) for capture

## High-level objective

Create a class that can be called by a higher-level module to choose one among
zero or more soda cans found by a can_finder node, which publishes observed
can positions. Inspect the array of observed soda can poses,
track them and filter each of them with a leaky bucket filter to persist
them for a short while if they should drop out. Average the can positions.
Provide a method for choosing the can that is closest and returning its pose.
Raise an exception if no cans are found.

## Context

This node subscribes to topic '/can_positions' in which each geometry_msgs/msgs/PoseArray
message describes the poses in the odom frame of any cans found in the current lidar scan.

## Low level tasks

These tasks are ordered from start to finish

1. Create CanChooser class

This class will be a ROS2 node

```aider
/add 'six_can/can_chooser.py'
  - Define a helper class CanTracker, which tracks the state and location of each can
  - Define the class CanChooser, whose constructor is passed a reference to the node.
  The constructor should:
    - Get the parameter 'can_persistence' with default value 3. This is the leaky bucket
    persistence for a can
    - Get the parameter 'can_pose_variance' with default value 0.1 meters.
    - Subscribe to '/can_positions' with message type geometry_msgs/msgs/PoseArray
    - Subscribe to '/odom'
```

2. Create odom callback

Save the current pose for later use

3. Create callback for '/can_positions'.

```aider
  - The callback should check whether any can poses are in the PoseArray message,
  and if so, whether they represent new cans.
  - A can is new if its x,y pose is more than 'can_pose_variance' from any currently
  tracked can, otherwise associate the pose to the closest already-tracked can
  - Can poses that are already being tracked should have their x and y positions
  updated by averaging with previous positions with a sliding window of length 4
  - New cans should be added to the list of tracked cans
```

3. Create 'choose_can' method

```aider
  - Find the closest tracked can to the current pose saved by the odom callback
  and return its pose to the caller
  - If there are no tracked cans, raise a runtime exception
```

4. Create a 'main()' function

Create a main that helps test the class by looping forever, printing (x,y) for all the tracked cans
every 5 seconds, ordered by increasing x within increasing y. If there are
no tracked cans and the class raises an exception, catch it and print "No Cans"

5. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```