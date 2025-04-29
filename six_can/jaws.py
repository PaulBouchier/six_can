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

    # Allow time for publisher to connect (optional but good practice)
    time.sleep(0.5) # Or use rate/spin_once for more robust connection check

    # Publish the message
    servo_pub.publish(msg)
    logger.info(f"Published {msg.data} to /servo")

    # Give a brief moment for the message to be sent before shutting down
    # Spin once allows callbacks to process if needed, and ensures publish occurs
    try:
        rclpy.spin_once(node, timeout_sec=0.5)
    except Exception as e:
        logger.error(f"Error during spin_once: {e}")


    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    logger.info("Jaws node shut down.")
    sys.exit(0)

if __name__ == '__main__':
    main()
