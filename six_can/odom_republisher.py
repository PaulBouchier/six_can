import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import numpy as np

"""
This node republishes /odom/unfiltered from the microros agent
as /odom, and publishes the tf transform between the odom frame
and the base_footprint frame. It exists because the EKF filter
in the robot_localization package gets confused when comms from
the robot drop out.
"""
class OdomRepublisher(Node):

    def __init__(self):
        super().__init__('odom_republisher')
        self.get_logger().info('Starting odom_republisher')
        # set up subscription to /odom/unfiltered
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/unfiltered', self.odom_listener_cb, 10)
        # set up publication to /odom and tf
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('OdomRepublisher initialized')

    def odom_listener_cb(self, msg):
        # received odom message from uros, publish data in an Odometry msg on /odom topic
        self.odom_pub.publish(msg)

        # publish transform in tf
        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # use the same timestamp as the incoming message
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id  # typically 'base_footprint' or 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    odom_republisher = OdomRepublisher()

    try:
        rclpy.spin(odom_republisher)
    except KeyboardInterrupt:
        print('KeyboardInterrupt - shutting down')


if __name__ == '__main__':
    main()
