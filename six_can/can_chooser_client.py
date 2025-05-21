'''
Call the can_chooser service and return the result.
'''
import rclpy
from rclpy.node import Node

from typing import Tuple
from six_can_interfaces.srv import CanChooserRqst
from geometry_msgs.msg import Pose

class CanChooserClient():
    def __init__(self, node: Node):
        """
        Initializes the CanChooserClient.

        Args:
            node: The ROS 2 Node instance to use for creating action clients and logging.
        """
        self.node = node
        self.client = self.node.create_client(CanChooserRqst, '/can_chooser_rqst')
        self.logger = node.get_logger()
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.info('Service /can_chooser_rqst not available, waiting again...')
        
        self.req = CanChooserRqst.Request()
        self.future: Future = None

    def choose_can(self) -> Tuple[bool, Pose]:
        """
        Use CanChooser service '/can_chooser_rqst' to select a can from the detected cans.
        Service returns the chosen can's pose.
        """
        can_chosen = False
        chosen_can_pose_odom = Pose()

        if self.future is not None and not self.future.done():
            self.future.cancel()  # Cancel the future. The callback will be called with Future.result == None.
            self.logger.info("Service Future canceled. The Node took too long to process the service call."
                                   "Is the Service Server still alive?")

        self.req.choose_can = True  # Set the request to choose a can
        self.logger.info(f"choose_can: Calling service to choose_can")
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)
        response = self.future.result()
        if response is None:
            self.logger.error("choose_can: Service call failed or future error: response was None.")
        elif not response.can_chosen:
            self.logger.info("choose_can: No can chosen.")
        else:
            # If a can was chosen, convert the response to a Pose
            can_chosen = True
            chosen_can_pose_odom.position.x = response.can_x
            chosen_can_pose_odom.position.y = response.can_y

            self.logger.info(f"Can chosen at odom coordinates: "
                                    f"x={chosen_can_pose_odom.position.x:.2f}, "
                                    f"y={chosen_can_pose_odom.position.y:.2f}")
        return can_chosen, chosen_can_pose_odom

def main(args=None):
    rclpy.init(args=args)
    node = Node('can_chooser_client')
    can_chooser_client = CanChooserClient(node)

    # Call the service and get the result
    can_chosen, chosen_can_pose_odom = can_chooser_client.choose_can()

    if can_chosen:
        node.get_logger().info(f"Chosen can pose: x={chosen_can_pose_odom.position.x}, y={chosen_can_pose_odom.position.y}")
    else:
        node.get_logger().info("No can chosen.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()