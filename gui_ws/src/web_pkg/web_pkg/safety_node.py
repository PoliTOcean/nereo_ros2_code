"""
Safety arbitration node — merges commands from the physical controller and
the web interface, enforces priority and timeout-based stop.

Topic layout:

  Physical controller (joy_to_cmdvel):
    /nereo_cmd_vel_joy      → direct mode  (priority 1 — highest)

  Controller node (nereo_controller_node, closed-loop):
    /nereo_cmd_vel_ctrl     → controller mode (priority 1 — highest)

  Web interface:
    /web_cmd_vel            → web joystick  (priority 2)

  Output (to ROV firmware — microROS subscribes to this):
    /nereo_cmd_vel          → arbitrated, timeout-guarded command

Note: /nereo_cmd_vel_no_fb is NOT subscribed here — it is the joystick output
that goes into nereo_controller_node, which then republishes the closed-loop
command on /nereo_cmd_vel_ctrl. This guarantees the safety node remains the
single publisher of /nereo_cmd_vel.

Priority rules:
  - If the physical controller has sent a command within CONTROLLER_TIMEOUT,
    it wins — web commands are ignored.
  - If only the web is active, its commands are forwarded.
  - If no source has sent a command within its respective timeout, zeros are sent.
  - On web disconnect (no message for WEB_HOLD_SECS), hold the last command
    for WEB_HOLD_SECS then ramp to zero over RAMP_SECS.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nereo_interfaces.msg import CommandVelocity

CONTROLLER_TIMEOUT = 0.5   # s — controller considered lost after this
WEB_HOLD_SECS      = 1.0   # s — hold last web command before stopping
RAMP_SECS          = 0.5   # s — ramp down to zero after hold expires
PUBLISH_RATE       = 20    # Hz
WEB_ACTIVE_WINDOW  = 2.0   # s — web considered "recently active" within this window


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        self._controller_cmd  = CommandVelocity()
        self._web_cmd         = CommandVelocity()
        self._last_controller = None
        self._last_web        = None
        self._web_hold_start  = None   # when the web timeout began

        self._pub = self.create_publisher(
            CommandVelocity, '/nereo_cmd_vel', 10)

        # controller sources — both treated as highest priority:
        #   - /nereo_cmd_vel_joy:  raw joystick passthrough (DIRECT mode)
        #   - /nereo_cmd_vel_ctrl: closed-loop output from nereo_controller_node
        self.create_subscription(
            CommandVelocity, '/nereo_cmd_vel_joy',
            lambda m: self._on_controller(m), 10)
        self.create_subscription(
            CommandVelocity, '/nereo_cmd_vel_ctrl',
            lambda m: self._on_controller(m), 10)

        # web source
        self.create_subscription(
            CommandVelocity, '/web_cmd_vel',
            self._on_web, 10)

        self.create_timer(1.0 / PUBLISH_RATE, self._publish)
        self.get_logger().info('Safety node active — publishing on /nereo_cmd_vel (firmware topic)')

    # ── callbacks ─────────────────────────────────────────────────────────

    def _on_controller(self, msg: CommandVelocity):
        self._controller_cmd = msg
        self._last_controller = self.get_clock().now()

    def _on_web(self, msg: CommandVelocity):
        self._web_cmd = msg
        self._last_web = self.get_clock().now()
        self._web_hold_start = None

    # ── arbitration ───────────────────────────────────────────────────────

    def _publish(self):
        now = self.get_clock().now()
        out = CommandVelocity()

        controller_alive = (
            self._last_controller is not None and
            (now - self._last_controller) < Duration(seconds=CONTROLLER_TIMEOUT)
        )

        if controller_alive:
            # controller has priority — pass through directly
            out = self._controller_cmd
            self._web_hold_start = None  # reset web hold while controller active
            # if web is actively publishing while controller is alive, keep
            # _last_web fresh so handover is immediate when controller drops
            if self._last_web is not None:
                web_age_now = (now - self._last_web).nanoseconds / 1e9
                if web_age_now < WEB_ACTIVE_WINDOW:
                    self._last_web = now   # pretend web just sent a message

        else:
            # no controller — use web with hold + ramp-down
            web_age = (
                (now - self._last_web).nanoseconds / 1e9
                if self._last_web is not None else float('inf')
            )

            if web_age < WEB_HOLD_SECS:
                # web is fresh or within hold window
                out = self._web_cmd
                self._web_hold_start = None

            elif web_age < WEB_HOLD_SECS + RAMP_SECS:
                # ramp down — linearly interpolate toward zero
                if self._web_hold_start is None:
                    self._web_hold_start = now
                ramp_elapsed = (now - self._web_hold_start).nanoseconds / 1e9
                scale = max(0.0, 1.0 - ramp_elapsed / RAMP_SECS)
                out = self._scale_cmd(self._web_cmd, scale)

            else:
                # fully timed out — send zeros
                if web_age < float('inf'):
                    self.get_logger().warn_once(
                        'Web client timed out — sending zero command')

        self._pub.publish(out)

    @staticmethod
    def _scale_cmd(msg: CommandVelocity, scale: float) -> CommandVelocity:
        out = CommandVelocity()
        for i in range(6):
            out.cmd_vel[i] = msg.cmd_vel[i] * scale
        return out


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
