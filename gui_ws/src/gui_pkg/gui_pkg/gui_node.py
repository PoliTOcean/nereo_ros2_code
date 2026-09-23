import sys
import os
import math
import gi
import rclpy
import threading
import numpy as np
from rclpy.node import Node
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory

from PyQt6.QtGui import QGuiApplication, QImage
from PyQt6.QtQml import QQmlApplicationEngine
from PyQt6.QtQuick import QQuickImageProvider
from PyQt6.QtCore import QObject, pyqtSignal, pyqtProperty, QTimer, pyqtSlot, QMetaObject, Qt, Q_ARG

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float32, Int32, Float32MultiArray, Bool, Float64MultiArray
from std_srvs.srv import Trigger
from rcl_interfaces.msg import ParameterType, ParameterValue, Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters, SetParameters
from . import PoliciesUtils


class CamStreamer:
    def __init__(self, port: int):
        self.port = port
        self._processing_frame = threading.Event()
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
            if self._processing_frame.is_set():
                return Gst.FlowReturn.OK

            self._processing_frame.set()
            sample = sink.emit("pull-sample")
            if not sample:
                self._processing_frame.clear()
                return Gst.FlowReturn.ERROR

            buffer = sample.get_buffer()
            if not buffer:
                self._processing_frame.clear()
                return Gst.FlowReturn.ERROR

            width, height = self.get_frame_resolution(sample)
            image = self.extract_image_from_buffer(buffer, width, height)

            if image:
                with self.lock:
                    self.latest_image = image

            self._processing_frame.clear()
            return Gst.FlowReturn.OK

        except Exception as e:
            self._processing_frame.clear()
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


SONAR_W          = 500   # waterfall image width  (px)
SONAR_H          = 400   # waterfall image height (px)
ASCAN_W          = 140   # A-scan image width     (px)
WATERFALL_ROWS   = 120   # number of history rows


class SonarRenderer:
    """
    Renders the waterfall and A-scan plot into QImages using matplotlib/numpy.
    Thread-safe: rendering happens under self.lock, Qt reads under the same lock.
    """

    def __init__(self):
        self.lock            = threading.Lock()
        self._waterfall_data = np.zeros((WATERFALL_ROWS, 200), dtype=np.float32)
        self._profile        = np.array([], dtype=np.float32)
        self._depth_vector   = np.array([], dtype=np.float32)

        self._wf_image       = self._blank_image(SONAR_W, SONAR_H)
        self._ascan_image    = self._blank_image(ASCAN_W, SONAR_H)

        self._colormap       = cm.get_cmap('turbo')

    @staticmethod
    def _blank_image(w, h):
        img = QImage(w, h, QImage.Format.Format_RGB888)
        img.fill(0x000010)
        return img

    def update(self, profile_data: list[float], scan_start_m: float, scan_len_m: float,
               confidence: int, conf_threshold: int):
        n = len(profile_data)
        if n == 0:
            return

        profile = np.array(profile_data, dtype=np.float32)
        depth_v = scan_start_m + np.arange(n) * (scan_len_m / n)

        attenuated = confidence < conf_threshold
        render_profile = profile * 0.3 if attenuated else profile

        with self.lock:
            # ── waterfall ──────────────────────────────────────────────
            if self._waterfall_data.shape[1] != n:
                self._waterfall_data = np.zeros((WATERFALL_ROWS, n), dtype=np.float32)

            self._waterfall_data = np.roll(self._waterfall_data, -1, axis=0)
            self._waterfall_data[-1, :] = render_profile

            self._profile      = render_profile
            self._depth_vector = depth_v

            self._wf_image    = self._render_waterfall(depth_v, scan_start_m, scan_start_m + scan_len_m)
            self._ascan_image = self._render_ascan(render_profile, depth_v,
                                                    scan_start_m, scan_start_m + scan_len_m)

    def _render_waterfall(self, depth_v, min_d, max_d) -> QImage:
        rgba = self._colormap(self._waterfall_data / 255.0)
        rgb  = (rgba[:, :, :3] * 255).astype(np.uint8)
        # flip rows so newest data is at the bottom
        rgb  = np.ascontiguousarray(np.flipud(rgb))
        h, w = rgb.shape[:2]
        qimg = QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        return qimg.scaled(SONAR_W, SONAR_H)

    def _render_ascan(self, profile, depth_v, min_d, max_d) -> QImage:
        fig = Figure(figsize=(ASCAN_W / 100, SONAR_H / 100), dpi=100)
        fig.patch.set_facecolor('#0055a4')
        ax = fig.add_axes([0.0, 0.0, 1.0, 1.0])
        ax.set_facecolor('#0055a4')
        ax.plot(profile, depth_v, color='#00ff00', linewidth=1.2)
        ax.set_xlim(0, 255)
        ax.set_ylim(max_d, min_d)
        ax.axis('off')

        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        buf = canvas.buffer_rgba()
        arr = np.frombuffer(buf, dtype=np.uint8).reshape(
            canvas.get_width_height()[::-1] + (4,))
        rgb = np.ascontiguousarray(arr[:, :, :3])
        plt.close(fig)

        h, w = rgb.shape[:2]
        return QImage(rgb.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()

    def get_waterfall_image(self):
        with self.lock:
            return self._wf_image, self._wf_image.size()

    def get_ascan_image(self):
        with self.lock:
            return self._ascan_image, self._ascan_image.size()


class SonarImageProvider(QQuickImageProvider):
    """
    Exposes sonar plots to QML via image://sonar/waterfall and image://sonar/ascan.
    Uses a frameCounter query param (appended by QML) to force reload.
    """
    def __init__(self, renderer: SonarRenderer):
        super().__init__(QQuickImageProvider.ImageType.Image)
        self._renderer = renderer

    def requestImage(self, id_str, size):
        view = id_str.lstrip('/').split('?')[0]
        if view == 'ascan':
            return self._renderer.get_ascan_image()
        return self._renderer.get_waterfall_image()


class SonarBridge(QObject):
    """
    Subscribes to sonar_node topics and exposes data to QML.
    Drives SonarRenderer on each profile update.
    """
    distance_changed        = pyqtSignal(float)
    confidence_changed      = pyqtSignal(int)
    conf_threshold_changed  = pyqtSignal(int)
    frame_ready             = pyqtSignal()   # tells QML to reload sonar images

    def __init__(self, node: Node, renderer: SonarRenderer):
        super().__init__()
        self._node      = node
        self._renderer  = renderer

        self._distance        = 0.0
        self._confidence      = 0
        self._conf_threshold  = 20

        node.create_subscription(Float32,          'sonar/distance',   self._cb_distance,   PoliciesUtils.sensor_qos)
        node.create_subscription(Int32,            'sonar/confidence', self._cb_confidence, PoliciesUtils.sensor_qos)
        node.create_subscription(Float32MultiArray,'sonar/profile',    self._cb_profile,    PoliciesUtils.sensor_qos)

    # ── properties ────────────────────────────────────────────────────────

    @pyqtProperty(float, notify=distance_changed)
    def sonarDistance(self): return self._distance

    @pyqtProperty(int, notify=confidence_changed)
    def sonarConfidence(self): return self._confidence

    @pyqtProperty(int, notify=conf_threshold_changed)
    def confidenceThreshold(self): return self._conf_threshold

    @confidenceThreshold.setter
    def confidenceThreshold(self, v: int):
        if self._conf_threshold != v:
            self._conf_threshold = v
            self.conf_threshold_changed.emit(v)

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _cb_distance(self, msg: Float32):
        self._distance = msg.data
        self.distance_changed.emit(self._distance)

    def _cb_confidence(self, msg: Int32):
        self._confidence = msg.data
        self.confidence_changed.emit(self._confidence)

    def _cb_profile(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 2:
            return
        scan_start_m = data[0]
        scan_len_m   = data[1]
        profile      = list(data[2:])

        self._renderer.update(profile, scan_start_m, scan_len_m,
                               self._confidence, self._conf_threshold)
        self.frame_ready.emit()


class ROSQmlBridge(QObject):
    roll_changed = pyqtSignal(float)
    pitch_changed = pyqtSignal(float)
    yaw_changed = pyqtSignal(float)
    depth_changed = pyqtSignal(float)
    temperature_changed = pyqtSignal(float)
    joy_status_changed     = pyqtSignal(bool)
    rov_armed_changed      = pyqtSignal(bool)
    rov_connected_changed  = pyqtSignal(bool)
    control_active_changed = pyqtSignal(bool)

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._depth = 0.0
        self._temperature = 0.0
        self._joystick_connected = False
        self._rov_armed          = False
        self._rov_connected      = False
        self._control_active     = False
        
        self.last_joy_time = None
        self.last_rov_armed_time = None

        self.arm_pub = self.node.create_publisher(Bool, '/set_arm_mode', 10)
        self.node.create_subscription(
            Bool, '/rov_armed', self.rov_armed_callback, PoliciesUtils.sensor_qos)

        self.sub_imu = self.node.create_subscription(
            Imu, 'imu_data', self.imu_callback, PoliciesUtils.sensor_qos)
        self.sub_barometer = self.node.create_subscription(
            Float32, 'barometer_depth_salt', self.barometer_callback, PoliciesUtils.sensor_qos)
        self.sub_temperature = self.node.create_subscription(
            Float32, 'barometer_temperature', self.temperature_callback, PoliciesUtils.sensor_qos)
        self.sub_joystick = self.node.create_subscription(
            Joy, 'joy', self.joystick_callback, PoliciesUtils.sensor_qos)
        self.node.create_subscription(
            Bool, '/joy_control_active', self.control_active_callback, 10)

    @pyqtProperty(float, notify=roll_changed)
    def rovRoll(self): return self._roll

    @pyqtProperty(float, notify=pitch_changed)
    def rovPitch(self): return self._pitch

    @pyqtProperty(float, notify=yaw_changed)
    def rovYaw(self): return self._yaw

    @pyqtProperty(float, notify=depth_changed)
    def rovDepth(self): return self._depth

    @pyqtProperty(float, notify=temperature_changed)
    def rovTemperature(self): return self._temperature

    @pyqtProperty(bool, notify=joy_status_changed)
    def joystickConnected(self): return self._joystick_connected

    @pyqtProperty(bool, notify=rov_armed_changed)
    def rovArmed(self): return self._rov_armed

    @pyqtProperty(bool, notify=rov_connected_changed)
    def rovConnected(self): return self._rov_connected

    @pyqtProperty(bool, notify=control_active_changed)
    def controlActive(self): return self._control_active

    def imu_callback(self, msg: Imu):
        try:
            angles = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(angles)
            self._roll = math.degrees(roll)
            self._pitch = math.degrees(pitch)
            self._yaw = math.degrees(yaw)

            self.roll_changed.emit(self._roll)
            self.pitch_changed.emit(self._pitch)
            self.yaw_changed.emit(self._yaw)
        except Exception as e:
            self.node.get_logger().warn(f'IMU callback error: {e}')

    def barometer_callback(self, msg: Float32):
        self._depth = msg.data
        self.depth_changed.emit(self._depth)

    def temperature_callback(self, msg: Float32):
        self._temperature = msg.data
        self.temperature_changed.emit(self._temperature)

    def control_active_callback(self, msg: Bool):
        self._control_active = msg.data
        self.control_active_changed.emit(self._control_active)

    def joystick_callback(self, msg: Joy):
        self.last_joy_time = self.node.get_clock().now()
        if not self._joystick_connected:
            self._joystick_connected = True
            self.joy_status_changed.emit(True)

    def check_joystick_timeout(self):
        if self.last_joy_time is None:
            return
        current_time = self.node.get_clock().now()
        if (current_time - self.last_joy_time) > Duration(seconds=1):
            if self._joystick_connected:
                self._joystick_connected = False
                self.joy_status_changed.emit(False)

    def check_rov_timeout(self):
        if self.last_rov_armed_time is None:
            return
        current_time = self.node.get_clock().now()
        if (current_time - self.last_rov_armed_time) > Duration(seconds=1.5):
            if self._rov_connected:
                self._rov_connected = False
                self.rov_connected_changed.emit(False)

    def rov_armed_callback(self, msg: Bool):
        self.last_rov_armed_time = self.node.get_clock().now()
        if not self._rov_connected:
            self._rov_connected = True
            self.rov_connected_changed.emit(True)
        if self._rov_armed != msg.data:
            self._rov_armed = msg.data
            QMetaObject.invokeMethod(
                self, '_emit_rov_armed',
                Qt.ConnectionType.QueuedConnection,
                Q_ARG(bool, self._rov_armed)
            )

    @pyqtSlot(bool)
    def sendArmCommand(self, arm_request: bool):
        self.node.get_logger().info(f'Sending arm command: {"ARM" if arm_request else "DISARM"}')
        msg = Bool()
        msg.data = arm_request
        self.arm_pub.publish(msg)

    @pyqtSlot()
    def resetDepthReference(self):
        client = self.node.create_client(Trigger, 'barometer_reset_reference')
        if client.wait_for_service(timeout_sec=1.0):
            client.call_async(Trigger.Request())
            self.node.get_logger().info('Depth reference reset requested')
        else:
            self.node.get_logger().warn('barometer_reset_reference service not available')

    @pyqtSlot()
    def resetAttitudeReference(self):
        client = self.node.create_client(Trigger, 'imu_reset_reference')
        if client.wait_for_service(timeout_sec=1.0):
            client.call_async(Trigger.Request())
            self.node.get_logger().info('Attitude reference reset requested')
        else:
            self.node.get_logger().warn('imu_reset_reference service not available')

    @pyqtSlot(bool)
    def _emit_rov_armed(self, armed: bool):
        self.rov_armed_changed.emit(armed)

    @pyqtSlot(int, result='QVariant')
    def getWebQrInfo(self, port: int):
        """Compute the workstation's outbound IP and a QR code PNG for
        http://<ip>:<port>. Called on Ctrl+Q from QML so the IP is fresh
        even if the Wi-Fi was connected after launching the GUI."""
        import socket, io, base64
        ip = None
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # No packet sent; the kernel picks the iface for the default route
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
        except OSError:
            ip = None
        finally:
            s.close()

        if not ip:
            return {'ok': False, 'url': '', 'qrDataUrl': '',
                    'error': 'Nessuna rete: connettiti al Wi-Fi e riprova.'}

        url = f'http://{ip}:{port}'
        try:
            import qrcode
            img = qrcode.make(url)
            buf = io.BytesIO()
            img.save(buf, format='PNG')
            data = base64.b64encode(buf.getvalue()).decode('ascii')
            qr_data_url = f'data:image/png;base64,{data}'
        except ImportError:
            return {'ok': False, 'url': url, 'qrDataUrl': '',
                    'error': 'Pacchetto python3-qrcode mancante.'}

        return {'ok': True, 'url': url, 'qrDataUrl': qr_data_url, 'error': ''}


CONTROLLER_PARAM_NAMES = [
    "control_mode",
    "kp", "ki", "kd",
    "manual_setpoint_depth", "manual_setpoint_roll",
    "manual_setpoint_pitch", "manual_setpoint_yaw",
    "setpoint_depth", "setpoint_roll", "setpoint_pitch", "setpoint_yaw",
    "cs_kx0", "cs_kx1", "cs_kx2",
    "cs_ki0", "cs_ki1", "cs_ki2",
    "cs_heave_min", "cs_heave_max", "cs_angle_min", "cs_angle_max",
]


class ControllerBridge(QObject):
    """
    Exposes /nereo_controller_node parameters and telemetry to QML.
    Uses rcl_interfaces services to GET/SET params; subscribes to
    /controller/setpoints, /controller/errors, /controller/pid_terms
    for live feedback.
    """
    connected_changed   = pyqtSignal(bool)
    status_changed      = pyqtSignal(str, str)   # (message, severity: info|ok|warn|err)
    params_loaded       = pyqtSignal('QVariant')  # dict name->value
    setpoints_changed   = pyqtSignal('QVariantList')
    errors_changed      = pyqtSignal('QVariantList')
    pid_terms_changed   = pyqtSignal('QVariantList')

    TARGET_NODE = '/nereo_controller_node'

    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        self._connected = False
        self._get_cli = node.create_client(
            GetParameters, f'{self.TARGET_NODE}/get_parameters')
        self._set_cli = node.create_client(
            SetParameters, f'{self.TARGET_NODE}/set_parameters')

        node.create_subscription(Float64MultiArray, '/controller/setpoints',
                                 self._cb_setpoints, 10)
        node.create_subscription(Float64MultiArray, '/controller/errors',
                                 self._cb_errors, 10)
        node.create_subscription(Float64MultiArray, '/controller/pid_terms',
                                 self._cb_pid_terms, 10)

        # poll connection
        self._poll_timer = QTimer()
        self._poll_timer.timeout.connect(self._check_connection)
        self._poll_timer.start(1000)

    @pyqtProperty(bool, notify=connected_changed)
    def controllerConnected(self):
        return self._connected

    # ── connection ────────────────────────────────────────────────────────
    def _check_connection(self):
        ok = (self._get_cli.wait_for_service(timeout_sec=0.0) and
              self._set_cli.wait_for_service(timeout_sec=0.0))
        if ok != self._connected:
            self._connected = ok
            self.connected_changed.emit(ok)
            if ok:
                self.status_changed.emit(
                    f'Connesso a {self.TARGET_NODE}', 'ok')
                self.loadParameters()
            else:
                self.status_changed.emit(
                    f'In attesa di {self.TARGET_NODE}…', 'warn')

    # ── slots invoked from QML ────────────────────────────────────────────
    @pyqtSlot()
    def loadParameters(self):
        if not self._connected:
            self.status_changed.emit('Servizi parametri non disponibili', 'err')
            return
        req = GetParameters.Request()
        req.names = CONTROLLER_PARAM_NAMES
        future = self._get_cli.call_async(req)
        future.add_done_callback(self._on_load_done)
        self.status_changed.emit('Lettura parametri…', 'info')

    def _on_load_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.status_changed.emit(f'Errore lettura: {exc}', 'err')
            return
        decoded = {}
        for name, value in zip(CONTROLLER_PARAM_NAMES, result.values):
            v = self._decode(value)
            if v is not None:
                decoded[name] = v
        self.params_loaded.emit(decoded)
        self.status_changed.emit('Parametri caricati', 'ok')

    @pyqtSlot('QVariant')
    def applyParameters(self, values):
        if not self._connected:
            self.status_changed.emit('Servizi parametri non disponibili', 'err')
            return
        # values is a QJSValue/dict from QML
        try:
            py = dict(values)
        except Exception:
            try:
                py = values.toVariant()
            except Exception:
                self.status_changed.emit('Payload non valido', 'err')
                return

        params = []
        try:
            for name in CONTROLLER_PARAM_NAMES:
                if name not in py:
                    continue
                params.append(self._encode(name, py[name]))
        except (ValueError, TypeError) as exc:
            self.status_changed.emit(f'Valore non valido: {exc}', 'err')
            return

        req = SetParameters.Request()
        req.parameters = params
        future = self._set_cli.call_async(req)
        future.add_done_callback(self._on_apply_done)
        self.status_changed.emit('Scrittura parametri…', 'info')

    def _on_apply_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.status_changed.emit(f'Errore scrittura: {exc}', 'err')
            return
        failed = [r.reason for r in response.results if not r.successful]
        if failed:
            self.status_changed.emit(
                f'Rifiutati: {"; ".join(failed)}', 'err')
        else:
            self.status_changed.emit('Applicati ✔', 'ok')

    # ── encode / decode helpers ───────────────────────────────────────────
    @staticmethod
    def _decode(pv):
        t = pv.type
        if t == ParameterType.PARAMETER_BOOL:
            return pv.bool_value
        if t == ParameterType.PARAMETER_INTEGER:
            return pv.integer_value
        if t == ParameterType.PARAMETER_DOUBLE:
            return pv.double_value
        if t == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(pv.double_array_value)
        if t == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(pv.integer_array_value)
        return None

    @staticmethod
    def _encode(name, value):
        msg = ParameterMsg()
        msg.name = name
        pv = ParameterValue()
        if name == 'control_mode':
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = int(value)
        elif name.startswith('manual_setpoint_'):
            pv.type = ParameterType.PARAMETER_BOOL
            pv.bool_value = bool(value)
        elif name in ('kp', 'ki', 'kd', 'cs_kx0', 'cs_kx1', 'cs_kx2'):
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(x) for x in value]
        else:
            pv.type = ParameterType.PARAMETER_DOUBLE
            pv.double_value = float(value)
        msg.value = pv
        return msg

    # ── telemetry callbacks ───────────────────────────────────────────────
    def _cb_setpoints(self, msg: Float64MultiArray):
        self.setpoints_changed.emit(list(msg.data))

    def _cb_errors(self, msg: Float64MultiArray):
        self.errors_changed.emit(list(msg.data))

    def _cb_pid_terms(self, msg: Float64MultiArray):
        self.pid_terms_changed.emit(list(msg.data))


def main():
    rclpy.init(args=sys.argv)
    node = Node('gui_node')

    app = QGuiApplication(sys.argv)
    Gst.init(None)

    engine = QQmlApplicationEngine()

    video_provider = VideoImageProvider()
    engine.addImageProvider("videostream", video_provider)

    sonar_renderer = SonarRenderer()
    sonar_provider = SonarImageProvider(sonar_renderer)
    engine.addImageProvider("sonar", sonar_provider)

    bridge            = ROSQmlBridge(node)
    sonar_bridge      = SonarBridge(node, sonar_renderer)
    controller_bridge = ControllerBridge(node)

    engine.rootContext().setContextProperty("RosBridge",        bridge)
    engine.rootContext().setContextProperty("SonarBridge",      sonar_bridge)
    engine.rootContext().setContextProperty("ControllerBridge", controller_bridge)

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