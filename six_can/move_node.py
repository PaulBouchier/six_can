import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from scripted_bot_driver import single_move_client


class MoveNode(Node):
    """
    ROS2 Node to manage the process of moving the robot.
    It uses the SingleMoveClient to execute moves.
    """
    def __init__(self, executor: MultiThreadedExecutor):
        super().__init__('move_node')
        self.executor = executor
        self.single_move_client = single_move_client.SingleMoveClient(self)
        self.get_logger().info('MoveNode initialization complete.')

    def execute_move(self, move_type: str, move_spec: list[str]) -> bool:
        """
        Executes a move using the SingleMoveClient.
        Blocks until the move is complete.
        """
        self.get_logger().info(f"Executing move: {move_type} {move_spec}")
        try:
            success = self.single_move_client.execute_move(move_type, move_spec)
            if success:
                self.get_logger().info(f"Move completed successfully: {move_type} {move_spec}")
            else:
                self.get_logger().error(f"Move failed: {move_type} {move_spec}")
            return success
        except Exception as e:
            self.get_logger().error(f"Error executing move: {e}")
            return False

def run_move_tests():
    move_node.get_logger().info('Starting move_tests sequence...')
    moves = [
        ("rotate_odom", str(1.57)),
        ("rotate_odom", str(-1.57)),
        ("drive_straight_odom", str(0.1)),
        ("drive_straight_odom", str(-0.1)),
    ]

    for move_type, move_command_str in moves:
        move_spec = []
        move_spec.append(move_command_str)
        move_node.get_logger().info(f"Executing move: type='{move_type}', command='{move_spec}'")
        try:
            move_node.execute_move(move_type, move_spec)
            move_node.get_logger().info(f"Move completed: type='{move_type}', command='{move_spec}'")
        except Exception as e:
            move_node.get_logger().error(f"Error executing move: {e}")

    move_node.get_logger().info('run_move_tests completed.')

def main(args=None):
    rclpy.init(args=args)

    try:
        global move_node
        executor = MultiThreadedExecutor(num_threads=4)
        move_node = MoveNode(executor)
        executor.add_node(move_node)

        # Start the main loop
        run_move_tests()
    except KeyboardInterrupt:
        move_node.get_logger().info('Keyboard interrupt, shutting down MoveNode.')
    except Exception as e:
        move_node.get_logger().error(f"Unhandled exception in MoveNode: {e}", exc_info=True)
    finally:
        executor.shutdown()
        if move_node:
            move_node.get_logger().info("Destroying MoveNode node.")
            move_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("MoveNode main finished.")

if __name__ == '__main__':
    main()
