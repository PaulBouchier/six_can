import rclpy
from rclpy.node import Node
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class YamlParserNode(Node):
    def __init__(self):
        super().__init__('yaml_parser_node')
        self.declare_parameter('yaml_file', 'search_poses.yaml')
        self.parse_yaml()

    def parse_yaml(self):
        yaml_file_name = self.get_parameter('yaml_file').get_parameter_value().string_value
        # Construct the full path to the YAML file within the package's share directory
        package_share_directory = get_package_share_directory('six_can')
        yaml_file_path = os.path.join(package_share_directory, 'resource', yaml_file_name)
        self.get_logger().info(f"Attempting to load YAML file from: {yaml_file_path}")

        try:
            with open(yaml_file_path, 'r') as file:
                data = yaml.safe_load(file)
                if data and 'poses' in data:
                    self.get_logger().info('Successfully parsed YAML file:')
                    for i, pose in enumerate(data['poses']):
                        self.get_logger().info(f"Pose {i+1}: {pose}")
                else:
                    self.get_logger().warn(f"YAML file '{yaml_file_path}' is empty or does not contain 'coordinates' key.")
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found at {yaml_file_path}. "
                                    f"Ensure the file exists and the path is correct. "
                                    f"Check setup.py data_files installation path.")
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file {yaml_file_path}: {e}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")


def main(args=None):
    rclpy.init(args=args)
    yaml_parser_node = YamlParserNode()

    try:
        # Keep the node alive briefly to ensure logs are flushed.
        rclpy.spin_once(yaml_parser_node, timeout_sec=1.0) # Spin briefly
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        yaml_parser_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
