from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nereo_sensors_pkg',
            executable='imu_pub',
            name='imu_pub'
        ),
        Node(
            package='nereo_sensors_pkg',
            executable='barometer_pub',
            name='barometer_pub'
        )
    ])
