from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scripted_bot_driver',
            executable='stop',
            name='stop',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='drive_straight_odom',
            name='drive_straight_odom',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='rotate_odom',
            name='rotate_odom',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='scripted_bot_driver',
            executable='seek2can',
            name='seek2can',
            output='screen',
            emulate_tty=True,
            parameters= [{
                    'speed_default_param':          0.15,
                    'rot_speed_default_param':      0.25,
            }]
        )
    ])
