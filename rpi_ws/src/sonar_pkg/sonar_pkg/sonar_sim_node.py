import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32, Int32, Float32MultiArray, MultiArrayDimension

sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

PROFILE_SAMPLES = 200


class SonarSimNode(Node):
    """
    Simulates a Ping1D sonar by publishing fake distance, confidence and
    profile data on the same topics as sonar_node.

    The simulated scene: a slowly oscillating bottom (2–8 m range) with a
    gaussian echo peak that tracks the distance, plus additive noise.
    Confidence drops to ~10% every ~8 s to let you test the threshold filter.
    """

    def __init__(self):
        super().__init__('sonar_sim_node')

        self.declare_parameter('publish_rate_hz', 12.5)   # ~80 ms period
        self.declare_parameter('scan_start_m',    0.0)
        self.declare_parameter('scan_length_m',   10.0)

        rate = self.get_parameter('publish_rate_hz').value
        self._scan_start  = self.get_parameter('scan_start_m').value
        self._scan_length = self.get_parameter('scan_length_m').value

        self._pub_distance   = self.create_publisher(Float32,          'sonar/distance',   sensor_qos)
        self._pub_confidence = self.create_publisher(Int32,            'sonar/confidence', sensor_qos)
        self._pub_profile    = self.create_publisher(Float32MultiArray,'sonar/profile',    sensor_qos)

        self._t = 0.0
        self._dt = 1.0 / rate

        self.create_timer(self._dt, self._tick)
        self.get_logger().info('Sonar simulator running')

    def _tick(self):
        self._t += self._dt

        # slowly oscillating distance: 2–8 m over ~20 s
        distance_m = 5.0 + 3.0 * math.sin(self._t * 2 * math.pi / 20.0)

        # confidence: mostly 85–95, dips to ~10 for 1.5 s every 8 s
        phase = self._t % 8.0
        if phase < 1.5:
            confidence = 10 + int(5 * abs(math.sin(self._t * 4)))
        else:
            confidence = 85 + int(10 * abs(math.sin(self._t * 0.3)))

        profile = self._make_profile(distance_m)

        dist_msg = Float32()
        dist_msg.data = distance_m
        self._pub_distance.publish(dist_msg)

        conf_msg = Int32()
        conf_msg.data = confidence
        self._pub_confidence.publish(conf_msg)

        self._publish_profile(profile)

    def _make_profile(self, distance_m: float) -> list:
        n = PROFILE_SAMPLES
        depth_step = self._scan_length / n

        # gaussian echo centred at distance_m
        sigma = n * 0.03
        centre = (distance_m - self._scan_start) / depth_step

        import random
        profile = []
        for i in range(n):
            echo  = 220 * math.exp(-0.5 * ((i - centre) / sigma) ** 2)
            noise = random.gauss(0, 6)
            # faint background reverberation
            bg    = 10 * math.exp(-i / (n * 0.4))
            val   = max(0.0, min(255.0, echo + bg + noise))
            profile.append(val)

        return profile

    def _publish_profile(self, profile: list):
        msg = Float32MultiArray()

        dim0 = MultiArrayDimension()
        dim0.label  = 'profile'
        dim0.size   = len(profile)
        dim0.stride = len(profile) + 2

        dim1 = MultiArrayDimension()
        dim1.label  = 'scale'
        dim1.size   = 2
        dim1.stride = 2

        msg.layout.dim = [dim0, dim1]
        msg.data = [self._scan_start, self._scan_length] + [float(v) for v in profile]
        self._pub_profile.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SonarSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
