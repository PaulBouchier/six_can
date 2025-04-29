import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
import sys
import time

def main(args=None):
    """
    ROS2 node to control the robot's jaws via the /servo topic.

    Accepts 'open' or 'close' as a command-line argument.
    'open': Publishes 1050 to /servo.
    'close': Publishes 1400 to /servo.
    """
    rclpy.init(args=args)
    node = rclpy.create_node('jaws_control_node')
    logger = node.get_logger()

    if len(sys.argv) < 2 or sys.argv[1] not in ['open', 'close']:
        logger.error("Usage: ros2 run six_can jaws <open|close>")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    command = sys.argv[1]

    # Create publisher
    servo_pub = node.create_publisher(Int32, '/servo', 10)

    # Create message
    msg = Int32()

    if command == 'open':
        msg.data = 1050
        logger.info("Commanding jaws to OPEN (1050)")
    elif command == 'close':
        msg.data = 1400
        logger.info("Commanding jaws to CLOSE (1400)")

    # Allow time for publisher to connect and ensure discovery
    logger.info("Waiting 2 seconds for publisher connection...")
    start_time = time.time()
    while time.time() - start_time < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    logger.info("Wait complete.")

    # Publish the message the first time
    servo_pub.publish(msg)
    logger.info(f"Published {msg.data} to /servo (1st time)")

    # Wait 1 second before publishing again
    logger.info("Waiting 1 second before second publish...")
    start_time = time.time()
    while time.time() - start_time < 1.0:
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as e:
            logger.error(f"Error during spin_once: {e}")
            break # Exit loop on error
    logger.info("Wait complete.")

    # Publish the message the second time
    servo_pub.publish(msg)
    logger.info(f"Published {msg.data} to /servo (2nd time)")

    # Give time for the message to be sent/processed before shutting down
    logger.info("Waiting 1 second before shutdown...")
    start_time = time.time()
    while time.time() - start_time < 1.0:
        try:
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception as e:
            logger.error(f"Error during spin_once: {e}")
            break # Exit loop on error
    logger.info("Wait complete.")

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    logger.info("Jaws node shut down.")
    sys.exit(0)

if __name__ == '__main__':
    main()
