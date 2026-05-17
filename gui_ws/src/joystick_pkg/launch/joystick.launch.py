from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/input/js0',
                              description='Joystick device path'),
        DeclareLaunchArgument('deadzone',   default_value='0.05'),
        DeclareLaunchArgument('max_steps',  default_value='10'),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device': LaunchConfiguration('device')}],
        ),
        Node(
            package='joystick_pkg',
            executable='joy_to_cmdvel',
            name='joy_to_cmd_vel',
            parameters=[{
                'deadzone':  LaunchConfiguration('deadzone'),
                'max_steps': LaunchConfiguration('max_steps'),
            }],
        ),
    ])
