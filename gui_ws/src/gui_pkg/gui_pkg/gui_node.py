import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import threading
from queue import Queue
import message_filters

from PyQt6 import QtGui
from PyQt6.QtWidgets import QApplication,  QTableWidgetItem, QMainWindow, QLabel
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import QObject, QThread, pyqtSignal, Qt
from tf_transformations import euler_from_quaternion

from . import MainWindowUtils, PoliciesUtils, CameraUtils
from .Services import ROVArmDisarmServiceClient

from sensor_msgs.msg import Imu, FluidPressure, Joy, Temperature
from diagnostic_msgs.msg import DiagnosticArray


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
            if item is None:
                break
            topic, msg = item
            self.process_sensor_data(topic, msg)


    def rotate_image(self, angle: float, label: QLabel, original_pixmap_path: str) -> None:

        original_pixmap = QtGui.QPixmap(original_pixmap_path)

        # Create a new pixmap for the rotated image
        rotated_pixmap = QtGui.QPixmap(original_pixmap.size())
        rotated_pixmap.fill(QtGui.QColor(0, 0, 0, 0))

        # Create a QPainter to draw the rotated image
        painter = QtGui.QPainter(rotated_pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)

        # Translate the painter to the center of the pixmap and rotate
        painter.translate(original_pixmap.width() / 2, original_pixmap.height() / 2)
        painter.rotate(angle * 180 / 3.14159)
        painter.translate(-original_pixmap.width() / 2, -original_pixmap.height() / 2)

        # Draw the scaled pixmap onto the rotated pixmap
        painter.drawPixmap(0, 0, original_pixmap)
        painter.end()

        # Set the rotated pixmap to the label
        label.setPixmap(rotated_pixmap)


    def process_sensor_data(self, topic: str, msg: any) -> None:
        """
        Function to process the sensor data based on the topic specified.
        """
        if topic == 'imu_data':
            self.update_imu(msg)

        elif topic == 'barometer_pressure':
            self.update_barometer(msg)


    def update_barometer(self, msg: FluidPressure) -> None:
        depth = (msg.fluid_pressure - 101325) / 9806.65
        self.ui.dept_value.setText(f"{depth:.2f} m")


    def update_imu(self, msg: Imu) -> None:

        angles = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        angles = euler_from_quaternion(angles)

        self.ui.roll_value.setText(f"{angles[0]:.2f}°")
        self.ui.pitch_value.setText(f"{angles[1]:.2f}°")
        self.ui.yaw_value.setText(f"{angles[2]:.2f}°")

        # ROTATION OF THE IMAGES
        self.rotate_image(angles[1], self.ui.side_image, self.ui.side_image_path)
        self.rotate_image(angles[2], self.ui.top_image, self.ui.top_image_path)


class ROS2NodeThread(QThread):

    def __init__(self, node: Node) -> None:
        super().__init__()
        self.node = node

    def run(self) -> None:
        # Start spinning the node in this thread
        rclpy.spin(self.node)


class ROS2Node(Node, QObject):

    controller_status_signal = pyqtSignal(bool)  # Define as class variable
    arm_disarm_signal = pyqtSignal()  # Signal for arm/disarm operations

    def __init__(self, ui: MainWindowUtils.Ui_MainWindow) -> None:
        """
        Function to initialize the ROS2 node.
        All the signals are defined here and the subscriptions are created.
        """
        Node.__init__(self, 'gui_node')
        QObject.__init__(self)
        self.last_frame_time = time.time()
        self.target_fps = 15
        self.last_message_time = None  # Initialize last_message_time

        self.ui = ui
        self.data_logged = 0
        
        # Initialize the arm/disarm service client
        self.arm_disarm_client = ROVArmDisarmServiceClient()
        
        # Connect arm/disarm signal
        self.arm_disarm_signal.connect(
            self.ui.control_panel_dialog.arm_disarm_dialog.change_status,
            Qt.ConnectionType.QueuedConnection
        )
        
        self.image_receiver = CameraUtils.ImageReceiver(fps=15)  # Reduced FPS
        self.image_receiver.image_received.connect(self.update_image)
        self.image_receiver.start()

        # Create joystick connection check timer
        self.joy_timer = self.create_timer(0.5, self.check_joystick_connection)  # Check every 0.5 seconds
        self.joy_timer.cancel()  # Start with timer cancelled

        self.subscription_imu_data = self.create_subscription(
                Imu,
                'imu_data',
                self.imu_data_callback,
                #PoliciesUtils.sensor_qos
                5)

        self.subscription_barometer_pressure = self.create_subscription(
                FluidPressure,
                'barometer_pressure',
                self.barometer_pressure_callback,
                PoliciesUtils.sensor_qos)

        """
        self.subscription_barometer_temperature = self.create_subscription(
                Temperature,
                'barometer_temperature',
                self.barometer_temperature_callback,
                PoliciesUtils.sensor_qos)
        """

        self.subscription_barometer_diagnostic = self.create_subscription(
                DiagnosticArray,
                'barometer_diagnostic',
                self.barometer_diagnostic_callback,
                PoliciesUtils.sensor_qos)

        self.subscription_imu_diagnostic = self.create_subscription(
                DiagnosticArray,
                'imu_diagnostic',
                self.imu_diagnostic_callback,
                PoliciesUtils.sensor_qos)

#        self.subscription_thrust_status = self.create_subscription(
#                ThrusterStatuses,
#                'thruster_status',
#                self.thrust_status_callback,
#                PoliciesUtils.reliable_qos)

        self.subscription_diagnostic_messages = self.create_subscription(
                DiagnosticArray,
                'diagnostic_messages',
                self.diagnostic_messages_callback,
                PoliciesUtils.sensor_qos)

        self.subscription_joystick = self.create_subscription(
                Joy,
                'joy',
                self.joystick_callback,
                PoliciesUtils.sensor_qos)

        self.last_frame_time = self.get_clock().now()
        self.frame_interval = 1.0 / 30  # 30 FPS frame rate limiter


        self.sensor_processor = SensorProcessor(self, self.ui)
        self.sensor_processor.start()

        imu_sub = message_filters.Subscriber(self, Imu, 'imu_data', qos_profile=PoliciesUtils.sensor_qos)
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub], 10, 0.1)
        ts.registerCallback(self.imu_data_callback)


    # CALLBACK FUNCTIONS =================================================================================================

    def destroy_node(self) -> None:
        self.image_receiver.stop()
        self.sensor_processor.running = False
        self.sensor_processor.queue.put(None)
        self.sensor_processor.join()
        super().destroy_node()

    def imu_data_callback(self, msg: Imu) -> None:

        if not self.sensor_processor.queue.full():
            self.sensor_processor.queue.put(('imu_data', msg))

    def barometer_pressure_callback(self, msg: FluidPressure) -> None:
        if not self.sensor_processor.queue.full():
            self.sensor_processor.queue.put(('barometer_pressure', msg))

    """
    def barometer_temperature_callback(self, msg: Temperature) -> None:
        pass
    """

    def handle_diagnostics(self, diagnostic_array: DiagnosticArray, peripheralName: str, index: int) -> None:
        """
        Function to handle the diagnostics of the peripheral.
        It will update the logs and the status of the peripheral in the UI.
        """
        errors = ["OK", "WARNING", "ERROR", "STALE", "UNKNOWN"]
        diagnostic_string = ""
        max_error = 0

        for diagnostic in diagnostic_array:

            if int.from_bytes(diagnostic.level, "big") > max_error:
                max_error = int.from_bytes(diagnostic.level, "big")

            level = errors[int.from_bytes(diagnostic.level, "big")]

            diagnostic_string += f"[{level}] {diagnostic.name}: {diagnostic.message}\n"

        self.ui.logs[peripheralName] = diagnostic_string
        self.ui.control_panel_dialog.peripherals_table.setItem(index, 1, QTableWidgetItem(errors[max_error]))
        self.ui.control_panel_dialog.peripherals_table.item(index, 1).setBackground(self.ui.control_panel_dialog.get_color(errors[max_error]))

    def update_image(self, q_image: QtGui.QImage) -> None:
        if not self.image_receiver.running:
            return

        # Convert QImage to QPixmap
        pixmap = QPixmap.fromImage(q_image)

        self.ui.main_camera_image.setPixmap(pixmap)


    def barometer_diagnostic_callback(self, msg: DiagnosticArray) -> None:
        self.handle_diagnostics(msg.status, "Barometer", 1)


    def imu_diagnostic_callback(self, msg: DiagnosticArray) -> None:
        self.handle_diagnostics(msg.status, "IMU", 2)


#    def thrust_status_callback(self, msg):
#        pass


    def diagnostic_messages_callback(self, msg: DiagnosticArray) -> None:

        self.handle_diagnostics(msg.status, "Diagnostic MicroROS", 4)


    def joystick_callback(self, msg: Joy) -> None:
        self.last_message_time = self.get_clock().now()
        
        # Reset the timer to start monitoring connection
        self.joy_timer.reset()
        
        # Emit signal instead of directly updating UI
        self.controller_status_signal.emit(True)
        
        # Check 7th button and if pressed update Service joy_button
        button = msg.buttons[6]
        if button == 1:
            self.arm_disarm_signal.emit()  # Emit signal instead of direct call
            self.get_logger().info("BUTTON PRESSED, ARM/DISARM CALLED")

    def check_joystick_connection(self) -> None:
        if self.last_message_time is None:
            return
            
        current_time = self.get_clock().now()
        delta_time = current_time - self.last_message_time

        if delta_time > Duration(seconds=1):
            self.controller_status_signal.emit(False)
        else:
            self.controller_status_signal.emit(True)

# MAIN FUNCTION ==========================================================================================================

def main() -> None:
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)

    main_window = QMainWindow()
    ui = MainWindowUtils.Ui_MainWindow()
    ui.setupUi(main_window)
    
    # Create the ROS2 node
    node = ROS2Node(ui)
    
    # Move the node to a separate thread
    ros2_thread = ROS2NodeThread(node)
    
    # Connect signals using Qt.QueuedConnection to ensure thread safety
    node.moveToThread(ros2_thread)
    node.controller_status_signal.connect(ui.update_controller_status, Qt.ConnectionType.QueuedConnection)
    
    ros2_thread.start()
    main_window.show()

    exit_code = app.exec()

    # Cleanup
    node.image_receiver.stop()
    node.destroy_node()
    rclpy.shutdown()

    ros2_thread.quit()
    ros2_thread.wait()

    sys.exit(exit_code)

if __name__ == '__main__':
    main()
