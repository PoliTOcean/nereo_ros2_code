import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from nereo_interfaces.msg import CommandVelocity

# ── Axis map — identical for DS5 and Xbox One S with joy_node ─────────────
#
#   0  Left stick X     (+left  / -right)
#   1  Left stick Y     (+up    / -down)
#   2  L2/LT analog     (+1 released → -1 fully pressed)
#   3  Right stick X    (+left  / -right)
#   4  Right stick Y    (+up    / -down)
#   5  R2/RT analog     (+1 released → -1 fully pressed)
#   6  D-Pad X          (+1 left / -1 right)
#   7  D-Pad Y          (+1 up   / -1 down)
#
# ── Button map ────────────────────────────────────────────────────────────
#
#   DS5:       0=Cross  1=Circle  2=Square  3=Triangle  4=L1  5=R1
#              6=L2     7=R2      8=Share   9=Options  10=PS  11=L3  12=R3
#
#   Xbox One S: 0=A  1=B  2=X  3=Y  4=LB  5=RB
#               6=View  7=Menu  8=Xbox  9=L3  10=R3
#
# ── Control mapping ───────────────────────────────────────────────────────
#   Left  Y/X        → surge / sway
#   Right Y/X        → heave / yaw
#   D-pad up/down    → pitch trim (incremental, rising edge)
#   D-pad left/right → roll  trim (incremental, rising edge)
#   btn_arm          → arm / disarm toggle  (DS5: PS=10  / Xbox: Xbox=8)
#   btn_mode         → direct↔controller mode toggle (DS5: Share=8 / Xbox: View=6)

AXIS_LEFT_X  = 0
AXIS_LEFT_Y  = 1
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_DPAD_X  = 6
AXIS_DPAD_Y  = 7

# Default button indices — Xbox One S. Override via ROS parameters for DS5.
BTN_ARM_DEFAULT    = 8    # Xbox: Xbox / DS5: PS(10)
BTN_ENABLE_DEFAULT = 6    # Xbox: View / DS5: Share(8)

DEADZONE    = 0.05
MAX_STEPS   = 10
JOY_TIMEOUT = 0.25


class JoyToCmdVelNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_to_cmd_vel')

        self.declare_parameter('deadzone',  DEADZONE)
        self.declare_parameter('max_steps', MAX_STEPS)
        self.declare_parameter('btn_arm',   BTN_ARM_DEFAULT)
        self.declare_parameter('btn_mode',  BTN_ENABLE_DEFAULT)

        self._deadzone  = self.get_parameter('deadzone').value
        self._max_steps = self.get_parameter('max_steps').value
        self._btn_arm   = self.get_parameter('btn_arm').value
        self._btn_mode  = self.get_parameter('btn_mode').value
        self._timeout   = Duration(seconds=JOY_TIMEOUT)

        self._pitch = 0.0
        self._roll  = 0.0
        self._prev_dpad_x = 0.0
        self._prev_dpad_y = 0.0
        self._prev_btn_arm    = 0
        self._prev_btn_enable = 0

        self._controller_mode = False   # False = direct, True = through controller node
        self._latest_joy = None
        self._last_joy_stamp = None

        self._pub_direct     = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_joy',   10)
        self._pub_controller = self.create_publisher(CommandVelocity, '/nereo_cmd_vel_no_fb', 10)
        self._pub_active     = self.create_publisher(Bool, '/joy_control_active', 10)
        self._arm_client     = self.create_client(SetBool, '/set_rov_arm_mode')

        self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        self.create_timer(1 / 20, self._publish)

        self.get_logger().info('joy_to_cmd_vel ready — mode DIRECT (/nereo_cmd_vel_joy) | Share to toggle controller mode')

    # ── joy subscriber ────────────────────────────────────────────────────

    def _joy_callback(self, msg: Joy) -> None:
        self._latest_joy = msg
        self._last_joy_stamp = self.get_clock().now()
        self._handle_buttons(msg)
        self._update_pitch_roll(msg)

    # ── button handling (rising edge) ─────────────────────────────────────

    def _handle_buttons(self, msg: Joy) -> None:
        if len(msg.buttons) <= max(self._btn_arm, self._btn_mode):
            return

        # Mode toggle
        if msg.buttons[self._btn_mode] == 1 and self._prev_btn_enable == 0:
            self._controller_mode = not self._controller_mode
            state_msg = Bool()
            state_msg.data = self._controller_mode
            self._pub_active.publish(state_msg)
            mode = 'CONTROLLER (/nereo_cmd_vel_no_fb)' if self._controller_mode else 'DIRECT (/nereo_cmd_vel_joy)'
            self.get_logger().info(f'Mode → {mode}')

        # Arm toggle
        if msg.buttons[self._btn_arm] == 1 and self._prev_btn_arm == 0:
            self._send_arm_command()

        self._prev_btn_enable = msg.buttons[self._btn_mode]
        self._prev_btn_arm    = msg.buttons[self._btn_arm]

    def _send_arm_command(self) -> None:
        if not self._arm_client.service_is_ready():
            self.get_logger().warn('Arm service not ready')
            return
        self._armed = not getattr(self, '_armed', False)
        req = SetBool.Request()
        req.data = self._armed
        future = self._arm_client.call_async(req)
        future.add_done_callback(self._arm_response_cb)
        self.get_logger().info(f'Requesting {"ARM" if self._armed else "DISARM"}')

    def _arm_response_cb(self, future) -> None:
        try:
            if not future.result().success:
                self._armed = not self._armed   # revert on failure
                self.get_logger().warn('Arm service returned failure')
        except Exception as e:
            self._armed = not self._armed
            self.get_logger().warn(f'Arm service error: {e}')

    # ── D-pad pitch/roll accumulator ──────────────────────────────────────

    def _update_pitch_roll(self, msg: Joy) -> None:
        if len(msg.axes) <= max(AXIS_DPAD_X, AXIS_DPAD_Y):
            return

        dpad_x = msg.axes[AXIS_DPAD_X]
        dpad_y = msg.axes[AXIS_DPAD_Y]

        if dpad_y != 0 and self._prev_dpad_y == 0:
            self._pitch = max(-1.0, min(1.0, self._pitch + dpad_y / self._max_steps))

        if dpad_x != 0 and self._prev_dpad_x == 0:
            self._roll = max(-1.0, min(1.0, self._roll - dpad_x / self._max_steps))

        self._prev_dpad_x = dpad_x
        self._prev_dpad_y = dpad_y

    # ── publish at 20 Hz ─────────────────────────────────────────────────

    def _publish(self) -> None:
        now = self.get_clock().now()
        joy_active = (
            self._latest_joy is not None
            and self._last_joy_stamp is not None
            and (now - self._last_joy_stamp) < self._timeout
        )

        if not joy_active:
            return

        msg = CommandVelocity()
        axes = self._latest_joy.axes

        def a(i: int) -> float:
            return self._dz(axes[i]) if i < len(axes) else 0.0

        msg.cmd_vel[0] = a(AXIS_LEFT_Y)
        msg.cmd_vel[1] = -a(AXIS_LEFT_X)
        msg.cmd_vel[2] = a(AXIS_RIGHT_Y)
        msg.cmd_vel[3] = self._pitch
        msg.cmd_vel[4] = self._roll
        msg.cmd_vel[5] = -a(AXIS_RIGHT_X)

        if self._controller_mode:
            self._pub_controller.publish(msg)
        else:
            self._pub_direct.publish(msg)

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
