import sys
import os
import gi
import rclpy
import threading
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

from PyQt6.QtGui import QGuiApplication, QImage
from PyQt6.QtQml import QQmlApplicationEngine
from PyQt6.QtQuick import QQuickImageProvider
from PyQt6.QtCore import QObject, pyqtSignal, pyqtProperty, QTimer, pyqtSlot

gi.require_version('Gst', '1.0')
from gi.repository import Gst

from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Imu, FluidPressure, Joy
from std_srvs.srv import SetBool
from . import PoliciesUtils


class CamStreamer:
    def __init__(self, port: int):
        self.port = port
        self.processing_frame = False
        self.pipeline = None
        self.appsink = None
        self.lock = threading.Lock()

        self.latest_image = QImage(640, 480, QImage.Format.Format_RGB888)
        self.latest_image.fill(0x1a252c)
        
        self.pipeline = self.create_pipeline(port)
        self.pipeline.set_state(Gst.State.PLAYING)

    def create_pipeline(self, port: int):
        pipeline_str = (
            f"udpsrc address=0.0.0.0 port={port} reuse=true ! application/x-rtp, payload=96 ! "
            f"rtph264depay ! avdec_h264 ! videoconvert ! "
            f"video/x-raw, format=RGB ! appsink name=sink"
        )
        pipeline = Gst.parse_launch(pipeline_str)
        
        self.appsink = pipeline.get_by_name("sink")
        self.appsink.set_property("emit-signals", True)
        self.appsink.connect("new-sample", self.on_new_sample)
        return pipeline

    def on_new_sample(self, sink):
        try:
            if self.processing_frame:
                return Gst.FlowReturn.OK

            self.processing_frame = True
            sample = sink.emit("pull-sample")
            if not sample:
                self.processing_frame = False
                return Gst.FlowReturn.ERROR

            buffer = sample.get_buffer()
            if not buffer:
                self.processing_frame = False
                return Gst.FlowReturn.ERROR

            width, height = self.get_frame_resolution(sample)
            image = self.extract_image_from_buffer(buffer, width, height)

            if image:
                with self.lock:
                    self.latest_image = image

            self.processing_frame = False
            return Gst.FlowReturn.OK

        except Exception as e:
            self.processing_frame = False
            print(f"[Cam {self.port}] Errore processing frame: {e}")
            return Gst.FlowReturn.ERROR

    def get_frame_resolution(self, sample):
        caps = sample.get_caps()
        width = caps.get_structure(0).get_int("width")[1]
        height = caps.get_structure(0).get_int("height")[1]
        return width, height

    def extract_image_from_buffer(self, buffer, width, height):
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return None

        try:
            if map_info.data:
                return QImage(map_info.data, width, height, QImage.Format.Format_RGB888).copy()
            return None
        finally:
            buffer.unmap(map_info)

    def stop(self):
        if self.pipeline:
            print(f"Chiusura pipeline porta {self.port}")
            self.pipeline.set_state(Gst.State.NULL)


class VideoImageProvider(QQuickImageProvider):
    def __init__(self):
        super().__init__(QQuickImageProvider.ImageType.Image)
        self.cameras = {
            "main_cam": CamStreamer(port=5001),
            "cam_1": CamStreamer(port=5002),
            "cam_2": CamStreamer(port=5003)
        }

    def requestImage(self, id_str, size):
        cleaned_id = id_str.lstrip('/')
        cam_name = cleaned_id.split("?")[0]

        if cam_name in self.cameras:
            streamer = self.cameras[cam_name]
            with streamer.lock:
                if streamer.latest_image and not streamer.latest_image.isNull():
                    return streamer.latest_image, streamer.latest_image.size()
        
        fail_img = QImage(640, 480, QImage.Format.Format_RGB888)
        fail_img.fill(0x333333) 
        return fail_img, fail_img.size()
    
    def cleanup(self):
        for cam in self.cameras.values():
            cam.stop()


class ROSQmlBridge(QObject):
    roll_changed = pyqtSignal(float)
    pitch_changed = pyqtSignal(float)
    yaw_changed = pyqtSignal(float)
    depth_changed = pyqtSignal(float)
    joy_status_changed = pyqtSignal(bool)
    rov_armed_changed = pyqtSignal(bool)
    rov_connected_changed = pyqtSignal(bool)

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._depth = 0.0
        self._joystick_connected = False
        self._rov_armed = False
        self._rov_connected = False
        
        self.last_joy_time = None
        self.last_imu_time = None 

        self.arm_client = self.node.create_client(SetBool, '/set_rov_arm_mode')
        
        self.sub_imu = self.node.create_subscription(
            Imu, 'imu_data', self.imu_callback, PoliciesUtils.sensor_qos)
        self.sub_barometer = self.node.create_subscription(
            FluidPressure, 'barometer_pressure', self.barometer_callback, PoliciesUtils.sensor_qos)
        self.sub_joystick = self.node.create_subscription(
            Joy, 'joy', self.joystick_callback, PoliciesUtils.sensor_qos)

    @pyqtProperty(float, notify=roll_changed)
    def rovRoll(self): return self._roll

    @pyqtProperty(float, notify=pitch_changed)
    def rovPitch(self): return self._pitch

    @pyqtProperty(float, notify=yaw_changed)
    def rovYaw(self): return self._yaw

    @pyqtProperty(float, notify=depth_changed)
    def rovDepth(self): return self._depth

    @pyqtProperty(bool, notify=joy_status_changed)
    def joystickConnected(self): return self._joystick_connected

    @pyqtProperty(bool, notify=rov_armed_changed)
    def rovArmed(self): return self._rov_armed

    @pyqtProperty(bool, notify=rov_connected_changed)
    def rovConnected(self): return self._rov_connected

    def imu_callback(self, msg: Imu):
        self.last_imu_time = self.node.get_clock().now()
        if not self._rov_connected:
            self._rov_connected = True
            self.rov_connected_changed.emit(True)

        try:
            angles = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(angles)
            self._roll = roll * 180.0 / 3.14159
            self._pitch = pitch * 180.0 / 3.14159
            self._yaw = yaw * 180.0 / 3.14159
            
            self.roll_changed.emit(self._roll)
            self.pitch_changed.emit(self._pitch)
            self.yaw_changed.emit(self._yaw)
        except Exception:
            pass

    def barometer_callback(self, msg: FluidPressure):
        self._depth = (msg.fluid_pressure - 101325) / 9806.65
        self.depth_changed.emit(self._depth)

    def joystick_callback(self, msg: Joy):
        self.last_joy_time = self.node.get_clock().now()
        if not self._joystick_connected:
            self._joystick_connected = True
            self.joy_status_changed.emit(True)
        
        if len(msg.buttons) > 6 and msg.buttons[6] == 1:
            self.sendArmCommand(not self._rov_armed)

    def check_joystick_timeout(self):
        if self.last_joy_time is None:
            return
        current_time = self.node.get_clock().now()
        if (current_time - self.last_joy_time) > Duration(seconds=1):
            if self._joystick_connected:
                self._joystick_connected = False
                self.joy_status_changed.emit(False)

    def check_rov_timeout(self):
        if self.last_imu_time is None:
            return
        current_time = self.node.get_clock().now()
        if (current_time - self.last_imu_time) > Duration(seconds=1.5):
            if self._rov_connected:
                self._rov_connected = False
                self.rov_connected_changed.emit(False)

    @pyqtSlot(bool)
    def sendArmCommand(self, arm_request: bool):
        if not self.arm_client.service_is_ready():
            return
        request = SetBool.Request()
        request.data = arm_request
        future = self.arm_client.call_async(request)
        future.add_done_callback(lambda fut: self.arm_service_response_callback(fut, arm_request))

    def arm_service_response_callback(self, future, requested_state):
        try:
            response = future.result()
            if response and response.success:
                self._rov_armed = requested_state
                self.rov_armed_changed.emit(self._rov_armed)
        except Exception:
            pass


def main():
    rclpy.init(args=sys.argv)
    node = Node('gui_node')
    
    app = QGuiApplication(sys.argv)
    Gst.init(None)

    engine = QQmlApplicationEngine()

    video_provider = VideoImageProvider()
    engine.addImageProvider("videostream", video_provider)

    bridge = ROSQmlBridge(node)
    engine.rootContext().setContextProperty("RosBridge", bridge)

    package_share_dir = get_package_share_directory('gui_pkg')
    qml_file_path = os.path.join(package_share_dir, 'qml', 'main.qml')

    engine.load(qml_file_path)

    if not engine.rootObjects():
        sys.exit(-1)

    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(15)

    joy_monitor_timer = QTimer()
    joy_monitor_timer.timeout.connect(bridge.check_joystick_timeout)
    joy_monitor_timer.timeout.connect(bridge.check_rov_timeout)
    joy_monitor_timer.start(500)

    exit_code = app.exec()

    video_provider.cleanup()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()