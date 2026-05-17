#!/usr/bin/env python3
import math
import random
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure, Joy
from std_srvs.srv import SetBool

CAM_PIPELINES = [
    ('Main Camera', 5001, 'smpte',  30),
    ('Camera 1',    5002, 'ball',   30),
    ('Camera 2',    5003, 'snow',   15),
]


class RovSimNode(Node):
    def __init__(self):
        super().__init__('rov_sim_node')

        self.declare_parameter('simulate_cameras', True)

        self._imu_pub  = self.create_publisher(Imu,           'imu_data',            10)
        self._baro_pub = self.create_publisher(FluidPressure, 'barometer_pressure',   10)
        self._joy_pub  = self.create_publisher(Joy,           'joy',                  10)
        self.create_service(SetBool, '/set_rov_arm_mode', self._handle_arm)

        self._armed = False
        self._t     = 0.0
        self._cam_procs = []

        self.create_timer(0.1, self._tick)

        if self.get_parameter('simulate_cameras').value:
            self._start_cameras()

        self.get_logger().info('=== ROV SIMULATOR ACTIVE ===')

    # ── camera subprocesses ───────────────────────────────────────────────

    def _start_cameras(self):
        for name, port, pattern, fps in CAM_PIPELINES:
            cmd = (
                f'gst-launch-1.0 videotestsrc pattern={pattern} ! '
                f'video/x-raw,width=640,height=480,framerate={fps}/1 ! videoconvert ! '
                f'x264enc tune=zerolatency bitrate=500 byte-stream=true ! '
                f'rtph264pay config-interval=1 ! '
                f'udpsink host=127.0.0.1 port={port}'
            )
            proc = subprocess.Popen(
                cmd, shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self._cam_procs.append(proc)
            self.get_logger().info(f'{name} → port {port} (pattern: {pattern})')

    def _stop_cameras(self):
        for proc in self._cam_procs:
            proc.terminate()
        self._cam_procs.clear()

    # ── arm service ───────────────────────────────────────────────────────

    def _handle_arm(self, request, response):
        self._armed = request.data
        self.get_logger().info(f'ARM → {"ARMED" if self._armed else "DISARMED"}')
        response.success = True
        response.message = f'ROV {"ARMED" if self._armed else "DISARMED"}'
        return response

    # ── telemetry tick ────────────────────────────────────────────────────

    def _tick(self):
        self._t += 0.05

        # joystick heartbeat
        joy = Joy()
        joy.axes   = [0.0, 0.0, 0.0, 0.0]
        joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        self._joy_pub.publish(joy)

        # IMU
        roll  = math.sin(self._t) * 0.15
        pitch = math.cos(self._t * 0.7) * 0.1
        yaw   = (self._t * 0.1) % (2 * math.pi)
        cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)

        imu = Imu()
        imu.orientation.w = cr * cp * cy + sr * sp * sy
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy
        self._imu_pub.publish(imu)

        # barometer
        depth = 3.5 + math.sin(self._t * 0.3) * 0.8
        baro = FluidPressure()
        baro.fluid_pressure = depth * 9806.65 + 101325.0 + random.uniform(-20.0, 20.0)
        self._baro_pub.publish(baro)

    # ── cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_cameras()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RovSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
