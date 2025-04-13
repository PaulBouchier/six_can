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

```cpp
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### Reference: Minimal ROS2 publisher code

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## Low-level tasks

These tasks are ordered from start to finish

1. Create SixCanRunner class

```aider

```


6. Update build files

```aider
/add setup.py
/add package.xml
update the ROS2 build control files
```

