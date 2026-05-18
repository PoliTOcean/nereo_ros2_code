"""
ROV command velocity monitor — subscribes to /nereo_cmd_vel and renders
a live terminal dashboard showing the 6-DOF command vector as bar graphs.

Usage:
    ros2 run joystick_pkg rov_cmd_monitor
"""
import sys
import rclpy
from rclpy.node import Node
from nereo_interfaces.msg import CommandVelocity

LABELS = ['SURGE ', 'SWAY  ', 'HEAVE ', 'PITCH ', 'ROLL  ', 'YAW   ']
BAR_W  = 20   # half-bar width in characters


def _bar(value: float) -> str:
    """Render a centered bar for a value in [-1, 1]."""
    filled = int(abs(value) * BAR_W)
    if value >= 0:
        left  = ' ' * BAR_W
        right = '█' * filled + ' ' * (BAR_W - filled)
    else:
        left  = ' ' * (BAR_W - filled) + '█' * filled
        right = ' ' * BAR_W
    sign  = '+' if value > 0 else ('-' if value < 0 else ' ')
    return f'[{left}|{right}] {sign}{abs(value):.3f}'


class RovCmdMonitor(Node):
    def __init__(self):
        super().__init__('rov_cmd_monitor')
        self._last: list[float] = [0.0] * 6
        self.create_subscription(CommandVelocity, '/nereo_cmd_vel', self._cb, 10)
        self.get_logger().info('Monitoring /nereo_cmd_vel  (Ctrl+C to quit)')
        self._render()

    def _cb(self, msg: CommandVelocity) -> None:
        self._last = list(msg.cmd_vel)
        self._render()

    def _render(self) -> None:
        lines = ['\033[2J\033[H']   # clear screen + move cursor home
        lines.append('  PoliTOcean Nereo — CMD_VEL monitor\n')
        lines.append(f"  {'':20}  negative          zero          positive\n")
        lines.append('  ' + '─' * 58 + '\n')
        for label, val in zip(LABELS, self._last):
            lines.append(f'  {label}  {_bar(val)}\n')
        lines.append('  ' + '─' * 58 + '\n')
        sys.stdout.write(''.join(lines))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = RovCmdMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print()   # restore terminal newline after clear


if __name__ == '__main__':
    main()
