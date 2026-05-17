import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from nereo_interfaces.msg import CommandVelocity

# ── DS5 axis/button map (ros2 run joy joy_node, Linux hidraw driver) ──────
#
# axes:
#   0  Left stick X     (+left  / -right)
#   1  Left stick Y     (+up    / -down)
#   2  L2 analog        (+1 released → -1 fully pressed)
#   3  Right stick X    (+left  / -right)
#   4  Right stick Y    (+up    / -down)
#   5  R2 analog        (+1 released → -1 fully pressed)
#   6  D-Pad X          (+1 left / -1 right)
#   7  D-Pad Y          (+1 up   / -1 down)
#
# buttons:
#   0  Cross      1  Circle    2  Square    3  Triangle
#   4  L1         5  R1        6  L2        7  R2
#   8  Share      9  Options  10  PS       11  L3  12  R3

AXIS_LEFT_X  = 0
AXIS_LEFT_Y  = 1
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_DPAD_X  = 6   # +1 = left,  -1 = right
AXIS_DPAD_Y  = 7   # +1 = up,    -1 = down

DEADZONE    = 0.05
MAX_STEPS   = 10
JOY_TIMEOUT = 0.25   # seconds without joy message → zero analog axes


class JoyToCmdVelNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_to_cmd_vel')

        self.declare_parameter('deadzone',  DEADZONE)
        self.declare_parameter('max_steps', MAX_STEPS)

        self._deadzone  = self.get_parameter('deadzone').value
        self._max_steps = self.get_parameter('max_steps').value
        self._timeout   = Duration(seconds=JOY_TIMEOUT)

        self._pitch = 0.0
        self._roll  = 0.0
        self._prev_dpad_x = 0.0
        self._prev_dpad_y = 0.0

        self._latest_joy: Joy | None = None
        self._last_joy_stamp: Time | None = None

        self._pub = self.create_publisher(CommandVelocity, '/nereo_cmd_vel', 10)
        self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        self.create_timer(1 / 20, self._publish)

        self.get_logger().info('joy_to_cmd_vel ready — subscribed to /joy')

    # ── joy subscriber ────────────────────────────────────────────────────

    def _joy_callback(self, msg: Joy) -> None:
        self._latest_joy = msg
        self._last_joy_stamp = self.get_clock().now()
        self._update_pitch_roll(msg)

    # ── D-pad pitch/roll accumulator ──────────────────────────────────────

    def _update_pitch_roll(self, msg: Joy) -> None:
        if len(msg.axes) <= max(AXIS_DPAD_X, AXIS_DPAD_Y):
            return

        dpad_x = msg.axes[AXIS_DPAD_X]
        dpad_y = msg.axes[AXIS_DPAD_Y]

        # increment only on rising edge (0 → ±1), not while held
        if dpad_y != 0 and self._prev_dpad_y == 0:
            self._pitch = max(-1.0, min(1.0, self._pitch + dpad_y / self._max_steps))

        if dpad_x != 0 and self._prev_dpad_x == 0:
            self._roll = max(-1.0, min(1.0, self._roll - dpad_x / self._max_steps))

        self._prev_dpad_x = dpad_x
        self._prev_dpad_y = dpad_y

    # ── publish at 20 Hz ─────────────────────────────────────────────────

    def _publish(self) -> None:
        msg = CommandVelocity()
        now = self.get_clock().now()

        joy_active = (
            self._latest_joy is not None
            and self._last_joy_stamp is not None
            and (now - self._last_joy_stamp) < self._timeout
        )

        if joy_active:
            axes = self._latest_joy.axes

            def a(i: int) -> float:
                return self._dz(axes[i]) if i < len(axes) else 0.0

            # surge:  left  Y  — joy +up = +1
            msg.cmd_vel[0] = a(AXIS_LEFT_Y)
            # sway:   left  X  — joy +left = +1, invert so +right = positive
            msg.cmd_vel[1] = -a(AXIS_LEFT_X)
            # heave:  right Y  — joy +up = +1
            msg.cmd_vel[2] = a(AXIS_RIGHT_Y)
            # pitch:  D-pad accumulator (up = positive)
            msg.cmd_vel[3] = self._pitch
            # roll:   D-pad accumulator (right = positive)
            msg.cmd_vel[4] = self._roll
            # yaw:    right X  — joy +left = +1, invert so +right = clockwise
            msg.cmd_vel[5] = -a(AXIS_RIGHT_X)

        self._pub.publish(msg)

    def _dz(self, value: float) -> float:
        return 0.0 if abs(value) < self._deadzone else value


def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
