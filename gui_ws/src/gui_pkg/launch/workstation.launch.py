from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Button presets (ros2 run joy joy_node, Linux hidraw driver)
#   DS5:        btn_arm=10 (PS)    btn_mode=8 (Share)
#   Xbox One S: btn_arm=8  (Xbox)  btn_mode=6 (View)


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device': LaunchConfiguration('device')}],
    )
    joy_to_cmdvel_node = Node(
        package='joystick_pkg',
        executable='joy_to_cmdvel',
        name='joy_to_cmd_vel',
        parameters=[{
            'deadzone':  LaunchConfiguration('deadzone'),
            'max_steps': LaunchConfiguration('max_steps'),
            'btn_arm':   LaunchConfiguration('btn_arm'),
            'btn_mode':  LaunchConfiguration('btn_mode'),
        }],
    )
    gui_node = Node(
        package='gui_pkg',
        executable='gui_node',
        name='gui_node',
    )
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],
    )
    web_server_node = Node(
        package='web_pkg',
        executable='web_server_node',
        name='web_server_node',
    )
    safety_node = Node(
        package='web_pkg',
        executable='safety_node',
        name='safety_node',
    )

    return LaunchDescription([
        DeclareLaunchArgument('device',    default_value='/dev/input/js0',
                              description='Joystick device path'),
        DeclareLaunchArgument('deadzone',  default_value='0.05'),
        DeclareLaunchArgument('max_steps', default_value='10'),
        DeclareLaunchArgument('btn_arm',   default_value='8',
                              description='Arm/disarm button index (Xbox: 8, DS5: 10)'),
        DeclareLaunchArgument('btn_mode',  default_value='6',
                              description='Mode toggle button index (Xbox: 6, DS5: 8)'),

        joy_node,
        joy_to_cmdvel_node,
        gui_node,
        rosbridge_node,
        web_server_node,
        safety_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gui_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])
