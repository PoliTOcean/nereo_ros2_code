import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, Int32, Float32MultiArray, MultiArrayDimension

from brping import Ping1D


sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class SafePing1D(Ping1D):
    """
    Wraps Ping1D forcing all cached fields to None so every set_* call
    actually sends the serial command instead of being silently skipped
    when the value matches the cached one.
    """
    def __init__(self):
        super().__init__()
        self._speed_of_sound = None
        self._scan_start     = None
        self._scan_length    = None
        self._scan_end       = None
        self._gain_setting   = None
        self._mode_auto      = None
        self._ping_interval  = None


class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')

        self.declare_parameter('serial_port',    '/dev/ttyUSB0')
        self.declare_parameter('baud_rate',       115200)
        self.declare_parameter('speed_of_sound',  1500)   # m/s in water
        self.declare_parameter('mode_auto',       True)
        self.declare_parameter('gain',            6)
        self.declare_parameter('scan_start_mm',   0)
        self.declare_parameter('scan_length_mm',  10000)
        self.declare_parameter('ping_interval_ms', 50)

        self._pub_distance   = self.create_publisher(Float32,          'sonar/distance',   sensor_qos)
        self._pub_confidence = self.create_publisher(Int32,            'sonar/confidence', sensor_qos)
        self._pub_profile    = self.create_publisher(Float32MultiArray,'sonar/profile',    sensor_qos)

        self._ping   = SafePing1D()
        self._lock   = threading.Lock()
        self._running = False
        self._thread  = None

        self._connect()

    # ── connection ────────────────────────────────────────────────────────

    def _connect(self):
        port     = self.get_parameter('serial_port').value
        baud     = self.get_parameter('baud_rate').value
        sos      = self.get_parameter('speed_of_sound').value

        self.get_logger().info(f'Connecting to Ping1D on {port} @ {baud}')
        try:
            self._ping.connect_serial(port, baud)
            if not self._ping.initialize():
                self.get_logger().error('Ping1D initialize() failed — check serial port')
                return

            self._ping.set_speed_of_sound(sos)
            self._apply_params()

            self._running = True
            self._thread  = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self.get_logger().info('Ping1D connected and running')
        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')

    def _apply_params(self):
        auto     = self.get_parameter('mode_auto').value
        gain     = self.get_parameter('gain').value
        start    = self.get_parameter('scan_start_mm').value
        length   = self.get_parameter('scan_length_mm').value
        interval = self.get_parameter('ping_interval_ms').value

        self._ping.set_mode_auto(1 if auto else 0)
        if not auto:
            self._ping.set_gain_setting(gain)
            self._ping.set_range(start, length)
        self._ping.set_ping_interval(interval)

    # ── read loop ─────────────────────────────────────────────────────────

    def _read_loop(self):
        while self._running:
            try:
                data = self._ping.get_profile()
                if data:
                    self._publish(data)
            except Exception as e:
                self.get_logger().warn(f'Read error: {e}')

            time.sleep(0.08)

    def _publish(self, data):
        dist_m       = data['distance']    / 1000.0
        confidence   = int(data['confidence'])
        scan_start_m = data['scan_start']  / 1000.0
        scan_len_m   = data['scan_length'] / 1000.0
        raw_profile  = list(data['profile_data'])
        n            = len(raw_profile)

        dist_msg = Float32()
        dist_msg.data = dist_m
        self._pub_distance.publish(dist_msg)

        conf_msg = Int32()
        conf_msg.data = confidence
        self._pub_confidence.publish(conf_msg)

        if n > 0:
            profile_msg = Float32MultiArray()

            # layout: dim[0] = profile samples, dim[1] = depth scale (start + length)
            dim0 = MultiArrayDimension()
            dim0.label  = 'profile'
            dim0.size   = n
            dim0.stride = n + 2

            dim1 = MultiArrayDimension()
            dim1.label  = 'scale'
            dim1.size   = 2
            dim1.stride = 2

            profile_msg.layout.dim = [dim0, dim1]

            # data: [scan_start_m, scan_length_m, p0, p1, ..., pN-1]
            profile_msg.data = [scan_start_m, scan_len_m] + [float(v) for v in raw_profile]
            self._pub_profile.publish(profile_msg)

    # ── cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if hasattr(self._ping, 'iodev') and self._ping.iodev is not None:
            try:
                self._ping.iodev.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
