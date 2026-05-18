from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Button presets (ros2 run joy joy_node, Linux hidraw driver)
#   DS5:       btn_arm=10 (PS)     btn_mode=8 (Share)
#   Xbox One S: btn_arm=8 (Xbox)  btn_mode=6 (View)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device',    default_value='/dev/input/js0'),
        DeclareLaunchArgument('deadzone',  default_value='0.05'),
        DeclareLaunchArgument('max_steps', default_value='10'),
        DeclareLaunchArgument('btn_arm',   default_value='8',
                              description='Arm/disarm button index (Xbox: 8, DS5: 10)'),
        DeclareLaunchArgument('btn_mode',  default_value='6',
                              description='Mode toggle button index (Xbox: 6, DS5: 8)'),

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
                'btn_arm':   LaunchConfiguration('btn_arm'),
                'btn_mode':  LaunchConfiguration('btn_mode'),
            }],
        ),
    ])
