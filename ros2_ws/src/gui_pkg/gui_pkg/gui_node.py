import sys
import os
import time, cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import threading

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QApplication,  QTableWidgetItem, QMainWindow
from PyQt6.QtGui import QImage
from PyQt6.QtCore import QObject, QThread, pyqtSignal
from cv_bridge import CvBridge, CvBridgeError
from tf_transformations import euler_from_quaternion
from . import MainWindowUtils
from sensor_msgs.msg import Image, Imu, FluidPressure, Temperature, Joy
from diagnostic_msgs.msg import DiagnosticArray

class ROS2ImageNodeSignals(QObject):
    image_signal = pyqtSignal(QImage)

class ROS2ImageNode(Node):

    def __init__(self, ui):
        super().__init__('gui_node')
        self.ui = ui

        self.subscription_camera1 = self.create_subscription(
                Image,
                'camera1',
                self.image_callback,
                10)
        self.bridge = CvBridge()

        self.subscription_imu_data = self.create_subscription(
                Imu,
                'imu_data',
                self.imu_data_callback,
                10)

        self.subscription_barometer_pressure = self.create_subscription(
                FluidPressure,
                'barometer_pressure',
                self.barometer_pressure_callback,
                10)

        self.subscription_barometer_temperature = self.create_subscription(
                Temperature,
                'barometer_temperature',
                self.barometer_temperature_callback,
                10)


        self.subscription_barometer_diagnostic = self.create_subscription(
                DiagnosticArray,
                'barometer_diagnostic',
                self.barometer_diagnostic_callback,
                10)

        self.subscription_imu_diagnostic = self.create_subscription(
                DiagnosticArray,
                'imu_diagnostic',
                self.imu_diagnostic_callback,
                10)

#        self.subscription_thrust_status = self.create_subscription(
#                ThrusterStatuses, # Check if this is the correct message type
#                'thruster_status',
#                self.thrust_status_callback,
#                10)

        self.subscription_diagnostic_messages = self.create_subscription(
                DiagnosticArray,
                'diagnostic_messages',
                self.diagnostic_messages_callback,
                10)

        self.subscription_joystick = self.create_subscription(
                Joy,
                'joy',
                self.joystick_callback,
                10)

    def rotate_image(self, angle, label, original_pixmap_path):

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


    # CALLBACK FUNCTIONS =================================================================================================

    def image_callback(self, msg):
        self.get_logger().info('Received an image')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='8UC3')

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        self.get_logger().info("Starting to emit image")
        qt_image = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format.Format_RGB888).rgbSwapped()
        self.ui.image_signals.image_signal.emit(qt_image)
        self.get_logger().info("Image emitted")


    def imu_data_callback(self, msg):

        angles = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        angles = euler_from_quaternion(angles) # TF transformations

        self.ui.roll_value.setText(f"{angles[0]:.2f}°")
        self.ui.pitch_value.setText(f"{angles[1]:.2f}°")
        self.ui.yaw_value.setText(f"{angles[2]:.2f}°")

        # ROTATION OF THE IMAGES
        self.rotate_image(angles[0], self.ui.side_image, self.ui.side_image_path)
        self.rotate_image(angles[1], self.ui.top_image, self.ui.top_image_path)


    def barometer_pressure_callback(self, msg):
        self.ui.dept_value.setText(f"{msg.fluid_pressure:.2f} Pa")


    def barometer_temperature_callback(self, msg):
        pass


    def handle_diagnostics(self, diagnostic_array, peripheralName, index):
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


    def barometer_diagnostic_callback(self, msg):

        self.handle_diagnostics(msg.status, "Barometer", 1)


    def imu_diagnostic_callback(self, msg):

        self.handle_diagnostics(msg.status, "IMU", 2)


#    def thrust_status_callback(self, msg):
#        pass


    def diagnostic_messages_callback(self, msg):

        self.handle_diagnostics(msg.status, "Diagnostic MicroROS", 4)


    def joystick_callback(self, msg):
        self.last_message_time = self.get_clock().now()
        self.timer = self.create_timer(0.8, self.check_joystick_connection)


    def check_joystick_connection(self):
        current_time = self.get_clock().now()
        last_message_time = self.last_message_time
        delta_time = current_time - last_message_time
        if delta_time > Duration(seconds=1): # 1 second without messsages
            self.ui.controller_status.setPixmap(QtGui.QPixmap("./images/red_controller.png"))
        else:
            self.ui.controller_status.setPixmap(QtGui.QPixmap("./images/green_controller.png"))


# MAIN FUNCTION ==========================================================================================================

def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)

    main_window = QMainWindow()
    ui = MainWindowUtils.Ui_MainWindow()
    ui.setupUi(main_window)
    node = ROS2ImageNode(ui)

    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.1))
    timer.start(int(1000/30)) # 30 FPS

    main_window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
