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

        # --- BEGIN Manual Spin ---
        # rclpy.spin_until_future_complete(self.node, self.future) # OLD METHOD

        node_executor_ref = self.node.executor
        if node_executor_ref is None:
            self.logger.error(
                f"Node '{self.node.get_name()}' is not associated with an executor. "
                "Falling back to rclpy.spin_until_future_complete."
            )
            # This situation itself would be a symptom if node was expected to be in mt_executor
            rclpy.spin_until_future_complete(self.node, self.future, timeout_sec=10.0) # Added timeout
        else:
            target_executor = node_executor_ref()  # Dereference weakref
            if target_executor is None:
                self.logger.error(
                    f"Node '{self.node.get_name()}'s executor weakref is dead. "
                    "Falling back to rclpy.spin_until_future_complete."
                )
                rclpy.spin_until_future_complete(self.node, self.future, timeout_sec=10.0) # Added timeout
            else:
                self.logger.info(
                    f"CHOOSE_CAN_MANUAL_SPIN: Manually spinning executor (id: {id(target_executor)}, "
                    f"type: {type(target_executor).__name__}) for node '{self.node.get_name()}'."
                )
                try:
                    nodes_before_spin = target_executor.get_nodes()
                    node_names_before_spin = [n.get_name() for n in nodes_before_spin]
                    self.logger.info(
                        f"CHOOSE_CAN_MANUAL_SPIN_PRE: Executor (id: {id(target_executor)}) nodes: {node_names_before_spin}. "
                        f"Target node '{self.node.get_name()}' in list: {self.node in nodes_before_spin}"
                    )
                except Exception as e_log:
                    self.logger.warn(f"CHOOSE_CAN_MANUAL_SPIN_PRE: Error logging executor nodes: {e_log}")

                loop_timeout_sec = 10.0  # Timeout for the manual spin loop
                spin_interval_sec = 0.1   # How often to call spin_once
                start_time = self.node.get_clock().now()
                timeout_occurred = False

                while rclpy.ok() and not self.future.done():
                    current_time = self.node.get_clock().now()
                    if (current_time - start_time).nanoseconds / 1e9 > loop_timeout_sec:
                        self.logger.warn(
                            f"CHOOSE_CAN_MANUAL_SPIN: Loop timed out after {loop_timeout_sec}s waiting for future."
                        )
                        timeout_occurred = True
                        break
                    target_executor.spin_once(timeout_sec=spin_interval_sec)
                
                if not self.future.done() and not timeout_occurred and not rclpy.ok():
                    self.logger.warn("CHOOSE_CAN_MANUAL_SPIN: rclpy.ok() became false during spin.")

                try:
                    nodes_after_spin = target_executor.get_nodes()
                    node_names_after_spin = [n.get_name() for n in nodes_after_spin]
                    self.logger.info(
                        f"CHOOSE_CAN_MANUAL_SPIN_POST: Executor (id: {id(target_executor)}) nodes: {node_names_after_spin}. "
                        f"Target node '{self.node.get_name()}' in list: {self.node in nodes_after_spin}"
                    )
                except Exception as e_log:
                    self.logger.warn(f"CHOOSE_CAN_MANUAL_SPIN_POST: Error logging executor nodes: {e_log}")
        # --- END Manual Spin ---

        response = self.future.result() # This will raise if future is not done or has exception
        if response is None: # Should only happen if future.result() itself returns None, not on error
            self.logger.error("choose_can: Service call future.result() returned None.")
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
