import sys
import time, cv2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import threading
from queue import Queue
import message_filters

from PyQt6 import QtGui
from PyQt6.QtWidgets import QApplication,  QTableWidgetItem, QMainWindow
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import QObject, QThread, pyqtSignal
from cv_bridge import CvBridge, CvBridgeError
from tf_transformations import euler_from_quaternion
from . import MainWindowUtils, PoliciesUtils
from sensor_msgs.msg import Image, Imu, FluidPressure, Temperature, Joy
from diagnostic_msgs.msg import DiagnosticArray



class SensorProcessor(threading.Thread):
    def __init__(self, node, ui):
        super().__init__()
        self.node = node
        self.queue = Queue(maxsize=100)
        self.running = True
        self.ui = ui


    def run(self):
        while self.running:
            item = self.queue.get()
            if item is None:
                break
            topic, msg = item
            self.process_sensor_data(topic, msg)


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


    def process_sensor_data(self, topic, msg):
        if topic == 'imu_data':
            # Process IMU data
            pass
            # Update UI using signals

        elif topic == 'barometer_pressure':
            self.update_barometer(msg)


    def update_barometer(self, msg):
        self.ui.dept_value.setText(f"{msg.fluid_pressure:.2f} Pa")


    def imu_data_callback(self, msg):

        angles = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        angles = euler_from_quaternion(angles) # TF transformations

        self.ui.roll_value.setText(f"{angles[0]:.2f}°")
        self.ui.pitch_value.setText(f"{angles[1]:.2f}°")
        self.ui.yaw_value.setText(f"{angles[2]:.2f}°")

        # ROTATION OF THE IMAGES
        self.rotate_image(angles[0], self.ui.side_image, self.ui.side_image_path)
        self.rotate_image(angles[1], self.ui.top_image, self.ui.top_image_path)



class ImageProcessor(threading.Thread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.queue = Queue(maxsize=10)
        self.running = True


    def run(self):
        while self.running:
            msg = self.queue.get()
            if msg is None:
                break
            self.process_image(msg)


    def process_image(self, msg):
        try:
            cv_image = self.node.bridge.imgmsg_to_cv2(msg, desired_encoding='8UC3')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            self.node.get_logger().error(f"CvBridgeError: {e}")
            return

        height, width, channels = rgb_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.node.ui.image_signals.image_signal.emit(pixmap)



class ROS2NodeThread(QThread):

    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        # Start spinning the node in this thread
        rclpy.spin(self.node)

class ROS2ImageNodeSignals(QObject):
    image_signal = pyqtSignal(QPixmap)

class ROS2Node(Node):

    def __init__(self, ui):

        self.last_frame_time = time.time()
        self.target_fps = 30

        super().__init__('gui_node')
        self.ui = ui

        self.subscription_camera1 = self.create_subscription(
                Image,
                'camera1',
                self.image_callback,
                PoliciesUtils.sensor_qos)
        self.bridge = CvBridge()

        self.subscription_imu_data = self.create_subscription(
                Imu,
                'imu_data',
                self.imu_data_callback,
                PoliciesUtils.sensor_qos)

        self.subscription_barometer_pressure = self.create_subscription(
                FluidPressure,
                'barometer_pressure',
                self.barometer_pressure_callback,
                PoliciesUtils.sensor_qos)

        self.subscription_barometer_temperature = self.create_subscription(
                Temperature,
                'barometer_temperature',
                self.barometer_temperature_callback,
                PoliciesUtils.sensor_qos)


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

        self.image_processor = ImageProcessor(self)
        self.image_processor.start()

        self.sensor_processor = SensorProcessor(self, self.ui)
        self.sensor_processor.start()

        imu_sub = message_filters.Subscriber(self, Imu, 'imu_data', qos_profile=PoliciesUtils.sensor_qos)
        ts = message_filters.ApproximateTimeSynchronizer([imu_sub], 10, 0.1)
        ts.registerCallback(self.imu_data_callback)


    # CALLBACK FUNCTIONS =================================================================================================

    def image_callback(self, msg):

        if not self.image_processor.queue.full():
            self.image_processor.queue.put(msg)

    def destroy_node(self):
        self.sensor_processor.running = False
        self.sensor_processor.queue.put(None)
        self.sensor_processor.join()
        self.image_processor.running = False
        self.image_processor.queue.put(None)
        self.image_processor.join()
        super().destroy_node()

    def imu_data_callback(self, msg):

        if not self.sensor_processor.queue.full():
            self.sensor_processor.queue.put(('imu_data', msg))

    def barometer_pressure_callback(self, msg):

        if not self.sensor_processor.queue.full():
            self.sensor_processor.queue.put(('barometer_pressure', msg))


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
        self.timer = self.create_timer(1.1, self.check_joystick_connection)


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
    node = ROS2Node(ui)

    ros2_thread = ROS2NodeThread(node)
    ros2_thread.start()

    main_window.show()

    exit_code = app.exec()

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == '__main__':
    main()
