# ROS2 Jazzy Node: Choose a can (if one can be found) for capture

## High-level objective

Create a class that can be called by a higher-level node to choose one among
zero or more soda cans found by a can_finder node, which publishes observed
can positions. Inspect the array of observed soda can poses,
track them and filter each of them with a leaky bucket filter to persist
them for a short while if they should drop out. Average the can positions.
Provide a method for choosing the can that is closest and returning its pose.
Raise an exception if no cans are found. Remember the last chosen can's pose
and allow querying its range and bearing.

## Context

This class will be called by a higher-level node and passed the node handle at initialization.
It subscribes to topic '/can_positions' in which each geometry_msgs/msgs/PoseArray
message describes the poses in the odom frame of any cans found in the current lidar scan.

## Low level tasks

These tasks are ordered from start to finish

In these steps, only the position.x, position.y value of cans matters, orientation 
does not matter.

1. Create CanChooser class

This class will be called by a ROS2 node

```aider
Create the file 'six_can/can_chooser.py' which will contain these classes
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
  - Choose the closest tracked can to the current pose saved by the odom callback
  - Print the x, y pose of the chosen can and the range and bearing to it from
  the current pose saved by the odom callback
  - Return the pose of the chosen can to the caller
  - Save the pose of the chosen can for later use by clients
  - If there are no tracked cans, raise a runtime exception
```

4. Create 'get_choice_range_bearing' method

```aider
  - Calculate the range and bearing from the current odom position to the previously-chosen
  can and return them as a tuple 'choice_range_bearing' = (range, bearing) to the caller
  - If the pose of the chosen can is not in the list of tracked cans (with a tolerance of
  'can_pose_variance'), raise a Runtime exception
```
)
5. Create a 'main()' function

Create a main function that helps test the class

```aider
Create a main function that instantiates this class and passes it the node reference,
then loops forever, performing the following steps every 5 seconds:
  - print (x,y) for all the tracked cans ordered by increasing x within increasing y.
  - If there are no tracked cans and the class raises an exception, catch it and print "No Cans exception".
  - Call the 'choose_can' method and print the x, y pose of the chosen can.
  - Call the 'get_choice_range_bearing' method and print the range and bearing from the current odom
  position to the chosen can
```

6. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```