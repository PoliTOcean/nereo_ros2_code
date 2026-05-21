import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Button presets (ros2 run joy joy_node, Linux hidraw driver)
#   DS5:        btn_arm=10 (PS)    btn_mode=8 (Share)
#   Xbox One S: btn_arm=8  (Xbox)  btn_mode=6 (View)

# Controller package lives in a separate workspace; we source its install
# overlay automatically so the user doesn't have to source two workspaces.
CONTROLLER_OVERLAY = os.path.expanduser(
    '~/Documents/PoliTOcean/RD/ros2_controller_tuning_aid/install'
)


def _source_controller_overlay():
    """Source the controller workspace's local_setup.bash and import the
    environment changes into the current process. Equivalent to running
    `source <CONTROLLER_OVERLAY>/local_setup.bash` in the parent shell —
    covers AMENT_PREFIX_PATH, LD_LIBRARY_PATH, PYTHONPATH, PATH, etc.

    Silently no-ops if the overlay isn't built yet, so the launcher can
    fail loudly later with a clear "package not found" message.
    """
    setup = os.path.join(CONTROLLER_OVERLAY, 'local_setup.bash')
    if not os.path.isfile(setup):
        return

    # Run bash, source the setup, then print the resulting env as NUL-delimited
    # KEY=VALUE pairs. NUL is the only safe separator (values can contain newlines).
    try:
        out = subprocess.check_output(
            ['bash', '-c', f'set -a; source {setup!r}; env -0'],
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f'[workstation.launch] warning: failed to source controller '
              f'overlay ({exc}); controller node may not be found.',
              file=sys.stderr)
        return

    for entry in out.split(b'\0'):
        if not entry:
            continue
        key, _, val = entry.decode('utf-8', 'replace').partition('=')
        if key:
            os.environ[key] = val


def generate_launch_description():
    _source_controller_overlay()

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
        parameters=[{
            'port': 9090,
            # Adopt the Jazzy-era defaults proactively so rosbridge stops
            # spamming the three deprecation warnings on every launch.
            'default_call_service_timeout': 5.0,
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
        }],
        arguments=['--ros-args', '--log-level', 'rosbridge_websocket:=WARN'],
        sigterm_timeout='2',
        sigkill_timeout='3',
    )
    web_server_node = Node(
        package='web_pkg',
        executable='web_server_node',
        name='web_server_node',
        sigterm_timeout='2',
        sigkill_timeout='3',
    )
    safety_node = Node(
        package='web_pkg',
        executable='safety_node',
        name='safety_node',
    )
    nereo_controller_node = Node(
        package='nereo_controller_node',
        executable='nereo_controller_node',
        name='nereo_controller_node',
        parameters=[{
            'control_mode': LaunchConfiguration('control_mode'),
        }],
        output='screen',
    )

    return LaunchDescription([
        # Compact, colored console output. rcutils only supports `{time}` as
        # raw `seconds.nanoseconds`, so we drop it from the format — the launch
        # system already prints its own wall-clock timestamps for lifecycle
        # events, and individual log lines don't need them.
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_OUTPUT_FORMAT',
            '[{severity}] [{name}]: {message}',
        ),

        DeclareLaunchArgument('device',    default_value='/dev/input/js0',
                              description='Joystick device path'),
        DeclareLaunchArgument('deadzone',  default_value='0.05'),
        DeclareLaunchArgument('max_steps', default_value='10'),
        DeclareLaunchArgument('btn_arm',   default_value='8',
                              description='Arm/disarm button index (Xbox: 8, DS5: 10)'),
        DeclareLaunchArgument('btn_mode',  default_value='6',
                              description='Mode toggle button index (Xbox: 6, DS5: 8)'),
        DeclareLaunchArgument('control_mode', default_value='0',
                              description='Nereo controller start mode (0=passthrough,1=PID,2=PID-AW,3=CS)'),

        joy_node,
        joy_to_cmdvel_node,
        gui_node,
        rosbridge_node,
        web_server_node,
        safety_node,
        nereo_controller_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=gui_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])
