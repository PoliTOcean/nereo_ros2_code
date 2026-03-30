import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import threading
from queue import Queue
import message_filters

from PyQt6 import QtGui, QtCore, QtWidgets
from PyQt6.QtWidgets import QApplication, QTableWidgetItem, QMainWindow, QLabel
from PyQt6.QtGui import QPixmap, QShortcut, QKeySequence
from PyQt6.QtCore import QObject, QThread, pyqtSignal, Qt
from tf_transformations import euler_from_quaternion

# --- MESSAGGI ROS 2 (Aggiungi questi!) ---
from sensor_msgs.msg import Imu, FluidPressure, Joy
from diagnostic_msgs.msg import DiagnosticArray

# ------------------------------------------

# Gli altri import che avevi già:
from .gui_ui import Ui_MainWindow
from . import PoliciesUtils, CameraUtils, MainWindowUtils
from .Services import ROVArmDisarmServiceClient

class SensorProcessor(threading.Thread):
    def __init__(self, node, ui) -> None:
        super().__init__()
        self.node = node
        self.queue = Queue(maxsize=100)
        self.running = True
        self.ui = ui

    def run(self) -> None:
        while self.running:
            item = self.queue.get()
            if item is None: break
            topic, msg = item
            self.process_sensor_data(topic, msg)

    def rotate_image(self, angle: float, label: QLabel, path: str) -> None:
        if not path: return
        pix = QtGui.QPixmap(path)
        if pix.isNull(): return
        
        rotated = QtGui.QPixmap(pix.size())
        rotated.fill(QtGui.QColor(0, 0, 0, 0))
        painter = QtGui.QPainter(rotated)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.translate(pix.width()/2, pix.height()/2)
        painter.rotate(angle * 180 / 3.14159)
        painter.translate(-pix.width()/2, -pix.height()/2)
        painter.drawPixmap(0, 0, pix)
        painter.end()
        label.setPixmap(rotated)

    def process_sensor_data(self, topic: str, msg: any) -> None:
        if topic == 'imu_data':
            angles = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            self.ui.roll_value.setText(f"{angles[0]:.2f}°")
            self.ui.pitch_value.setText(f"{angles[1]:.2f}°")
            self.ui.yaw_value.setText(f"{angles[2]:.2f}°")
            self.rotate_image(angles[1], self.ui.side_image, getattr(self.ui, 'side_image_path', None))
            self.rotate_image(angles[2], self.ui.top_image, getattr(self.ui, 'top_image_path', None))
        elif topic == 'barometer_pressure':
            depth = (msg.fluid_pressure - 101325) / 9806.65
            self.ui.dept_value.setText(f"{depth:.2f} m")

class ROS2NodeThread(QThread):
    def __init__(self, node: Node) -> None:
        super().__init__()
        self.node = node
    def run(self) -> None:
        rclpy.spin(self.node)

class ROS2Node(Node, QObject):
    controller_status_signal = pyqtSignal(bool)
    arm_disarm_signal = pyqtSignal()

    def __init__(self, ui: Ui_MainWindow) -> None:
        Node.__init__(self, 'gui_node')
        QObject.__init__(self)
        self.ui = ui
        self.ui.logs = {"Camera": "", "Barometer": "", "IMU": "", "Thrusters": "", "Diagnostic MicroROS": ""}
        
        # Inizializziamo i Dialogs che prima erano in MainWindowUtils
        self.ui.control_panel_dialog = MainWindowUtils.ControlPanelDialog(self.ui.logs)
        self.ui.joy_config_dialog = MainWindowUtils.JoystickConfigDialog()

        # Collegamento pulsanti (I "CAVI" MANCANTI)
        self.ui.control_panel.clicked.connect(self.ui.control_panel_dialog.exec)
        self.ui.joystick.clicked.connect(self.ui.joy_config_dialog.exec)

        # Logica ARM/DISARM
        self.arm_disarm_client = ROVArmDisarmServiceClient()
        self.arm_disarm_signal.connect(
            self.ui.control_panel_dialog.arm_disarm_dialog.change_status,
            Qt.ConnectionType.QueuedConnection
        )

        # Camera
        self.image_receiver = CameraUtils.ImageReceiver(fps=15)
        self.image_receiver.image_received.connect(self.update_image)
        self.image_receiver.start()

        # Timer Joystick
        self.joy_timer = self.create_timer(0.5, self.check_joystick_connection)
        self.last_message_time = None

        # Subscriptions
        self.create_subscription(Imu, 'imu_data', self.imu_data_callback, 5)
        self.create_subscription(FluidPressure, 'barometer_pressure', lambda m: self.sensor_processor.queue.put(('barometer_pressure', m)), PoliciesUtils.sensor_qos)
        self.create_subscription(DiagnosticArray, 'barometer_diagnostic', lambda m: self.handle_diagnostics(m.status, "Barometer", 1), PoliciesUtils.sensor_qos)
        self.create_subscription(DiagnosticArray, 'imu_diagnostic', lambda m: self.handle_diagnostics(m.status, "IMU", 2), PoliciesUtils.sensor_qos)
        self.create_subscription(Joy, 'joy', self.joystick_callback, PoliciesUtils.sensor_qos)

        self.sensor_processor = SensorProcessor(self, self.ui)
        self.sensor_processor.start()

    def update_image(self, q_image: QtGui.QImage) -> None:
        pixmap = QPixmap.fromImage(q_image)
        self.ui.main_camera_image.setPixmap(pixmap.scaled(self.ui.main_camera_image.size(), Qt.AspectRatioMode.KeepAspectRatio))

    def imu_data_callback(self, msg: Imu) -> None:
        if not self.sensor_processor.queue.full():
            self.sensor_processor.queue.put(('imu_data', msg))

    def handle_diagnostics(self, status_array, name, index):
        # Semplificato per brevità, integra la logica colori se necessario
        self.ui.logs[name] = str(status_array)

    def joystick_callback(self, msg: Joy) -> None:
        self.last_message_time = self.get_clock().now()
        self.controller_status_signal.emit(True)
        if msg.buttons[6] == 1: self.arm_disarm_signal.emit()

    def check_joystick_connection(self) -> None:
        if self.last_message_time and (self.get_clock().now() - self.last_message_time) > Duration(seconds=1):
            self.controller_status_signal.emit(False)

    def destroy_node(self) -> None:
        self.image_receiver.stop()
        self.sensor_processor.running = False
        self.sensor_processor.queue.put(None)
        super().destroy_node()

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    win = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(win)
    ui.politocean_logo.setPixmap(QPixmap("gui_ws/images/logo_new.png"))
    ui.controller_status.setPixmap(QPixmap("gui_ws/images/red_controller.png"))
    ui.power_status.setPixmap(QPixmap("gui_ws/images/red_power.png"))
    ui.armed_status.setPixmap(QPixmap("gui_ws/images/red_shield.png"))

    # Definiamo i percorsi delle immagini per SensorProcessor
    ui.side_image_path = "gui_ws/images/sidePNG_white.png"
    ui.top_image_path = "gui_ws/images/upPNG_white.png"

    node = ROS2Node(ui)
    
    # Update icone controller
    node.controller_status_signal.connect(
        lambda s: ui.controller_status.setPixmap(QPixmap(f"gui_ws/images/{'green' if s else 'red'}_controller.png")),
        Qt.ConnectionType.QueuedConnection
    )

    t = ROS2NodeThread(node)
    t.start()
    win.show()
    
    code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)

if __name__ == '__main__': main()