# ROS2 Jazzy Node: Choose a can (if one can be found) for capture

## High-level objective

Create a node to choose one among
zero or more soda cans found by a can_finder node, which publishes observed
can poses. Inspect the array of observed soda can poses,
track them and filter each of them with a leaky bucket filter to persist
them for a short while if they should drop out. Average the can positions.
Provide a ROS service for choosing the can that is closest and returning its pose.
The service should return whether a can has been chosen, false if no cans are found.
Remember the last chosen can's pose and allow querying its range and bearing.

## Context

This node subscribes to topic '/can_positions' in which each geometry_msgs/msgs/PoseArray
message describes the poses in the odom frame of any cans found in the current lidar scan.

### can_chooser_rqst service request definition

The ROS service can_chooser_rqst is defined as:

```aider
bool choose_can
---
bool can_chosen
float64 can_x
float64 can_y
float64 range
float64 bearing_rad
```

### CanChooserDebug message definition

The CanChooserDebug message is define as:

```aider
int64 tracked_can_cnt
float64 robot_x
float64 robot_y
float64 chosen_x
float64 chosen_y
float64 chosen_range
float64 chosen_bearing
```

## Low level tasks

These tasks are ordered from start to finish

In these steps, only the position.x, position.y value of cans matters, orientation 
does not matter.

1. Create CanChooser class

This class will be a ROS2 node

```aider
Create the file 'six_can/can_chooser.py' which will contain these classes
  - Define a helper class CanTracker, which tracks the state and location of each can
  - Define the class CanChooser. The constructor should:
    - Get the parameter 'can_persistence' with default value 3. This is the leaky bucket
    persistence for a can
    - Get the parameter 'can_pose_variance' with default value 0.1 meters.
    - Subscribe to '/can_positions' with message type geometry_msgs/msgs/PoseArray
    - Subscribe to '/odom' with message type nav_msgs/msg/Odometry
    - Create a publisher to '/can_chooser_debug' with message type six_can_interfaces/msg/CanChooserDebug
    - Provide a service server on '/can_chooser_rqst' with service type
    six_can_interfaces/srv/CanChooserRqst
```

2. Create odom callback

Save the current pose for later use

3. Create callback for '/can_positions'.

Track cans and filter for short dropouts using a leaky bucket

```aider
  - The callback should check whether any can poses are in the PoseArray message,
  and if so, whether they represent new cans.
  - A can is new if its x,y pose is more than 'can_pose_variance' from any currently
  tracked can, otherwise associate the pose to the closest already-tracked can
  - Can poses that are already being tracked should have their x and y positions
  updated by averaging with previous positions with a sliding window of length 4
  - New cans should be added to the list of tracked cans
  - If there is a tracked can, but it disappears after filtering, it should be removed from the
  list of tracked cans. If the tracked can being removed is a chosen can, clear the chosen can state
  - Fill the CanChooserDebug message per field names and publish it
```

3. Create 'can_chooser_rqst' service method

Select the closest can if there are any cans being tracked and remember it.

```aider
  - If the 'choose_can' request is true, choose the closest tracked can to the current
  pose saved by the odom callback, else do not change the closest tracked can
  - If there are no tracked cans, or a can has not been chosen, or the chosen can is
  no longer being tracked, return false in the 'can_chosen' response field
  - If there is a chosen can, return its x, y, range and bearing in the odom frame in the response to the caller, else
  set those values to 0.0
  - If there is a chosen can, print the x, y pose of the chosen can and the range and bearing to it from
  the current pose saved by the odom callback, 
  - Save the pose of the chosen can for later use by clients
```

5. Create a 'main()' function that launches the node

6. Create a test node to exercise this node.

Create a node in file 'can_chooser_test.py that helps test the node

```aider
Create a service client for '/can_chooser_rqst' that loops forever, performing the following steps every 10 seconds:
  - Call the 'can_chooser_rqst' service with choose_can true. If can_chosen is true in the response, print
  the x, y pose and range and bearing of the chosen can, else print "No can chosen"
  - Call the 'can_chooser_rqst' service with choose_can false. If can_chosen is true in the response, print
  the x, y pose and range and bearing of the chosen can, else print "No can chosen"
```

6. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```