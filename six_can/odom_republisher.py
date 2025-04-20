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

It also publishes the base_footprint pose in the map frame as a PoseStamped
for use by the waypoint navigation node.
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
        self.last_odom_time = self.get_clock().now().nanoseconds  # initialize last odom time to now
        self.odom_msg_cnt = 0  # for debugging purposes, count the number of odom messages received

        # set up transform & publishing of base_footprint pose in map frame
        self.map_pose_pub = self.create_publisher(PoseStamped, 'map_pose', 10)
        self.odom_pose = PoseStamped()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.get_logger().info('OdomRepublisher initialized')

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    
    def odom_listener_cb(self, msg):
        # check if we missed an odom message
        now = self.get_clock().now().nanoseconds  # get the current time in nanoseconds
        time_since_odom_msg_ms = (now - self.last_odom_time) / 1e6
        if (time_since_odom_msg_ms > 200):  # if more than rate since last odom message, log a warning
            self.get_logger().warn(
                f"Missed odom message, time since last odom message: {time_since_odom_msg_ms:.2f} ms")
        # update the last odom time
        self.last_odom_time = now
        self.odom_msg_cnt += 1  # increment the odom message count for debugging purposes
        if self.odom_msg_cnt % 1000 == 0:  # log every 10th message
            self.get_logger().info(
                f"Received odom message count: {self.odom_msg_cnt}")

        # received odom message from uros, publish data in an Odometry msg
        mb_odom = Odometry()
        mb_odom.header = msg.header  # use the same header as the incoming message
        mb_odom.child_frame_id = msg.child_frame_id
        # planar & angular pose
        mb_odom.pose.pose.position = msg.pose.pose.position
        mb_odom.pose.pose.orientation = msg.pose.pose.orientation

        self.odom_pub.publish(mb_odom)

        # publish transform in tf
        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # use the same timestamp as the incoming message
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id  # typically 'base_footprint' or 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        # publish base_footprint pose in map frame on /map_pose topic
        self.odom_pose.header.stamp = Time().to_msg() # blank timestamp uses latest transform
        self.odom_pose.header.frame_id = msg.child_frame_id
        self.odom_pose.pose = msg.pose.pose
        
        try:
            if not self.buffer.can_transform('map', msg.child_frame_id, Time()):
                return
            map_pose = self.buffer.transform(self.odom_pose, 'map')
        except Exception as e:
            self.get_logger().info(f'Exception transforming to map pose: {e}')
            return
        self.map_pose_pub.publish(map_pose)

def main(args=None):
    rclpy.init(args=args)

    odom_republisher = OdomRepublisher()

    try:
        rclpy.spin(odom_republisher)
    except KeyboardInterrupt:
        print('KeyboardInterrupt - shutting down')


if __name__ == '__main__':
    main()
