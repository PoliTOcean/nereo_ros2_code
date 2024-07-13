import sys
import time, cv2
import rclpy
from rclpy.node import Node

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import QApplication,  QTableWidgetItem, QMainWindow
# removed QHeaderView, QDialog, QDialogButtonBox, QTextEdit, QProgressBar, QSizePolicy, QWidget QLabel, QVBoxLayout, QPushButton, QTableWidget
from PyQt6.QtGui import QImage # REMOVED QKeyEvent, QPixmap, QKeySequence, QShortcut
from PyQt6.QtCore import QThread, pyqtSignal # removed Qt, QTimer, pyqtSlot
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion

from MainWindowUtils import Ui_MainWindow as MainWindowUi

class ROS2ImageNode(Node):
    image_signal = pyqtSignal(QImage)

    def __init__(self, ui):
        super().__init__('gui_node')

        self.ui = ui

        self.subscription_camera1 = self.create_subscription(
                Image,
                '/camera1',
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

        # In diagnostic I was thinking about collecting data, passing data to a function that would update the gui. The number of logs could vary, then we have to
        # calculate the dimension of the table and update it accordingly

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

        self.subscription_thrust_status = self.create_subscription(
                ThrusterStatuses, # Check if this is the correct message type
                'thruster_status',
                self.thrust_status_callback,
                10)

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

    def rotate_image(self, angle, label):

        # Get the original pixmap and size of the label
        original_pixmap = label.pixmap()
        label_size = label.size()

        # Scale the original pixmap to fit the label
        scale_factor = min(label_size.width() / original_pixmap.width(), label_size.height() / original_pixmap.height())
        scaled_pixmap = original_pixmap.scaled(int(original_pixmap.width() * scale_factor), int(original_pixmap.height() * scale_factor), QtCore.Qt.AspectRatioMode.KeepAspectRatio)

        # Create a new pixmap for the rotated image
        rotated_pixmap = QtGui.QPixmap(scaled_pixmap.size())
        rotated_pixmap.fill(QtGui.QColor(0, 0, 0, 0))

        # Create a QPainter to draw the rotated image
        painter = QtGui.QPainter(rotated_pixmap)
        painter.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)

        # Translate the painter to the center of the pixmap and rotate
        painter.translate(rotated_pixmap.width() / 2, rotated_pixmap.height() / 2)
        painter.rotate(angle * 180 / 3.14159)
        painter.translate(-rotated_pixmap.width() / 2, -rotated_pixmap.height() / 2)

        # Draw the scaled pixmap onto the rotated pixmap
        painter.drawPixmap(0, 0, scaled_pixmap)
        painter.end()
    
        # Set the rotated pixmap to the label
        label.setPixmap(rotated_pixmap)



    # CALLBACK FUNCTIONS =================================================================================================


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, channel = cv_image.shape
        bytesPerLine = 3 * width
        qt_image = QImage(cv_image.data, width, height, bytesPerLine, QImage.Format.Format_RGB888).rgbSwapped()
        self.image_signal.emit(qt_image)


    def imu_data_callback(self, msg):
        angles = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        angles = euler_from_quaternion(angles) # TF transformations

        self.ui.roll_value.setText(f"{angles[0]:.2f}°")
        self.ui.pitch_value.setText(f"{angles[1]:.2f}°")
        self.ui.yaw_value.setText(f"{angles[2]:.2f}°")


        # ROTATION OF THE IMAGES

        self.rotate_image(angles[0], self.ui.side_image)
        self.rotate_image(angles[1], self.ui.top_image)


    def barometer_pressure_callback(self, msg):
        self.ui.dept_value.setText(f"{msg.fluid_pressure:.2f} Pa")


    def barometer_temperature_callback(self, msg):
        pass


    def handle_diagnostics(self, diagnostic_array, peripheralName, index):
        errors = ["OK", "WARNING", "ERROR", "STALE", "UNKNOWN"]
        diagnostic_string = ""
        max_error = 0

        for diagnostic in diagnostic_array:

            if diagnostic.level > max_error:
                max_error = diagnostic.level

            level = errors[diagnostic.level]

            diagnostic_string += f"[{level}] {diagnostic.name}: {diagnostic.message}\n"

        self.ui.control_panel.logs[peripheralName] = diagnostic_string
        self.ui.peripherals_table.setItem(index, 1, QTableWidgetItem(errors[max_error]))
        self.ui.peripherals_table.item(index, 1).setBackground(self.ui.control_panel.get_color(errors[max_error]))


    def barometer_diagnostic_callback(self, msg):

        self.handle_diagnostics(msg.status, "Barometer", 1)


    def imu_diagnostic_callback(self, msg):

        self.handle_diagnostics(msg.status, "IMU", 2)


    def thrust_status_callback(self, msg):
        pass


    def diagnostic_messages_callback(self, msg):
        errors = ["OK", "WARNING", "ERROR", "STALE", "UNKNOWN"]
        diagnostic_string = ""
        max_error = 0

        for diagnostic in msg.status:

            if diagnostic.level > max_error:
                max_error = diagnostic.level

            level = errors[diagnostic.level]

            diagnostic_string += f"[{level}] {diagnostic.hardware_id}: {diagnostic.message}\n"

        self.ui.control_panel.logs["Thrusters"] = diagnostic_string
        self.ui.peripherals_table.setItem(3, 1, QTableWidgetItem(errors[max_error]))
        #self.handle_diagnostics(msg.status, "Thrusters", 3)


    def joystick_callback(self, msg):
        self.last_message_time = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.check_joystick_connection)


    def check_joystick_connection(self):
        current_time = self.get_clock().now()
        delta_time = current_time - self.last_message_time
        if delta_time > 1e9: # 1 second without messsages
            self.ui.controller_status.setPixmap(QtGui.QPixmap("images/red_controller.png"))
        else:
            self.ui.controller_status.setPixmap(QtGui.QPixmap("images/green_controller.png"))


# THREAD CLASS ============================================================================================================
class ImageThread(QThread):
    image_received = pyqtSignal(QImage)

    def __init__(self):
        super().__init__()
        self.ros2_node = ROS2ImageNode(ui)
        self.ros2_node.image_signal.connect(self.image_received) # Maybe should be self.image_received.emit

    def run(self):
        rclpy.spin(self.ros2_node)
        rclpy.shutdown()

    def stop(self):
        self.ros2_node.destroy_node()



# MAIN FUNCTION ==========================================================================================================
def main():
	rclpy.init(args=sys.argv)
	app = QApplication(sys.argv)
	
	main_window = QMainWindow()
	ui = MainWindowUi()
	ui.setupUi(main_window)
	
	image_thread = ImageThread()
	image_thread.start()
	
    image_thread.ros2_node.image_received.connect(ui.update_image_widget)

	main_window.show()
	sys.exit(app.exec())
	


if __name__ == "__main__":
    main()
