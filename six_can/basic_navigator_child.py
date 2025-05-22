import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import MultiThreadedExecutor


class BasicNavigatorChild(BasicNavigator):
    """
    BasicNavigatorChild is a subclass of BasicNavigator that fixes initialization issues
    """

    def __init__(self, node_name):
        super().__init__(node_name=node_name)

    def _waitForInitialPose(self):
        self.get_logger().info("overridden _waitForInitialPose() called")
        return

def main(args=None):
    """Main function to run the CaptureCan node standalone for testing."""
    rclpy.init(args=args)

    node = BasicNavigatorChild(node_name='basic_navigator_child')
    node.get_logger().info("capture_can node starting...")

    # Create a MultiThreadedExecutor for handling multiple threads
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(node)

    # Wait for Nav2 to become active (CRITICAL!)
    node.get_logger().info("Waiting for Nav2 services to become active...")
    node.waitUntilNav2Active()
    node.get_logger().info("Nav2 is active.")

    node.get_logger().info("Shutting down capture_can node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
