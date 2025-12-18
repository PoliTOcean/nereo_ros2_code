from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import (QLabel, QVBoxLayout, QPushButton,
                            QTableWidget, QTableWidgetItem, QHeaderView, QDialog, QDialogButtonBox, QTextEdit)
from PyQt6.QtGui import QShortcut, QKeySequence
from PyQt6.QtCore import Qt, QObject

from . import Services

class ControlPanelDialog(QDialog):
    def __init__(self, logs: dict[str, str]) -> None:
        super().__init__()
        self.setWindowTitle("Control Panel")
        self.setGeometry(300, 300, 600, 400)
        layout = QVBoxLayout()

        self.statuses = [] # Empty array for status at the beginning

        self.arm_button = QPushButton("ARM")
        self.arm_button.setCheckable(True)
        self.arm_button.clicked.connect(self.initiate_arm_disarm)
        layout.addWidget(self.arm_button)

        self.peripherals_table = QTableWidget(5, 2)
        self.peripherals_table.setHorizontalHeaderLabels(["Peripheral", "Status"])
        self.peripherals_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)

        self.peripherals = ["Camera", "Barometer", "IMU", "Thrusters", "Diagnostic MicroROS"]

        # Set peripherals name in the table
        for i in range(len(self.peripherals)):
            self.peripherals_table.setItem(i, 0, QTableWidgetItem(self.peripherals[i]))

        self.peripherals_table.cellDoubleClicked.connect(lambda row, column: self.show_logs(row, column, logs))
        self.peripherals_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)  # Stretch columns to fit
        layout.addWidget(self.peripherals_table)

        self.close_panel_shortcut = QShortcut(QKeySequence("Ctrl+W"), self)
        self.close_panel_shortcut.activated.connect(self.reject)

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        self.arm_disarm_dialog = ArmDisarmDialog()

        self.setLayout(layout)


    def get_color(self, status: str) -> Qt.GlobalColor:
        """
        Function to get the color of the status by the status string.
        """
        colors = {
                "OK": Qt.GlobalColor.green,
                "WARN": Qt.GlobalColor.yellow,
                "ERROR": Qt.GlobalColor.red,
                "STALE": Qt.GlobalColor.lightGray
                }
        return colors[status]


    def initiate_arm_disarm(self) -> None:
        self.arm_disarm_dialog.exec()


    def show_logs(self, row: int, column: int, logs: dict[str, str]) -> None:
        """
        Function to show the logs of the peripheral by the row and column of the table.
        """
        peripheral_name = self.peripherals_table.item(row, 0).text()
        dialog = PeripheralDialog(peripheral_name, logs)
        dialog.exec()



class PeripheralDialog(QDialog):
    def __init__(self, peripheral_name: str, logs: dict[str, str]) -> None:
        super().__init__()
        self.setWindowTitle(f"{peripheral_name} Logs")
        self.setGeometry(300, 300, 400, 300)
        layout = QVBoxLayout()

        self.logs_text_edit = QTextEdit()
        self.logs_text_edit.setReadOnly(True)

        self.update_logs(peripheral_name, logs)

        layout.addWidget(self.logs_text_edit)

        button_box = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        button_box.accepted.connect(self.accept)
        layout.addWidget(button_box)

        self.setLayout(layout)

    def update_logs(self, peripheral_name: str, logs: dict[str, str]) -> None:
        self.logs_text_edit.setText("Logs for " + peripheral_name + '\n\n' + logs[peripheral_name])


class ArmDisarmDialog(QDialog):
    status_changed = QtCore.pyqtSignal(bool)  # Signal to handle status changes

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Confirm")
        self.setGeometry(400, 400, 300, 150)
        self.ask_service = layout = QVBoxLayout()
        self.arm_status = False
        self.service_client = Services.ROVArmDisarmServiceClient()

        self.label = QLabel(self.get_text())
        layout.addWidget(self.label)

        self.enter_shortcut = QShortcut(QKeySequence("Return"), self)
        self.enter_shortcut.activated.connect(self.change_status)
        
        # Connect the status change signal to handle UI updates in the main thread
        self.status_changed.connect(self._handle_status_change, Qt.ConnectionType.QueuedConnection)

        self.setLayout(layout)

    def get_text(self) -> str:
        self.arm_status = self.service_client.get_current_value()
        if self.arm_status is not None:
            if self.arm_status:
                return "Press 'Enter' to DISARM the ROV."
            else:
                return "Press 'Enter' to ARM the ROV."
        else:
            return "Service not available."

    @QtCore.pyqtSlot()
    def _handle_status_change(self) -> None:
        """
        Handle UI updates in the main thread
        """
        self.label.setText(self.get_text())
        self.accept()

    def change_status(self) -> None:
        """
        Function to change the status of the ROV calling the std_srvs/SetBool service.
        """
        if self.service_client.service_available is False:    
            self.label.setText("Service not available.")
            self.status_changed.emit(False)
            return
    
        response = self.service_client.call_service(True if not self.arm_status else False)

        if response is not None:
            self.arm_status = not self.arm_status
            self.status_changed.emit(True)

    def get_armed_status(self) -> bool:
        return self.arm_status


class ImageSignals(QtCore.QObject):
    image_signal = QtCore.pyqtSignal(QtGui.QImage)

class ControllerStatusSignal(QtCore.QObject):
    controller_status_signal = QtCore.pyqtSignal(bool)


class JoystickConfigDialog(QDialog):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Joystick Mapping")
        self.setFixedSize(500, 500) 
        self.setStyleSheet("background-color: #212529; color: white;")
        
        layout = QVBoxLayout()

        # Joystick Image Slot
        self.joy_image = QLabel()
        pixmap = QtGui.QPixmap("images/joy.png") 
        
        if pixmap.isNull():
            self.joy_image.setText("IMAGE NOT FOUND")
        else:
            self.joy_image.setPixmap(pixmap.scaled(300, 180, 
                                     Qt.AspectRatioMode.KeepAspectRatio, 
                                     Qt.TransformationMode.SmoothTransformation))
        
        self.joy_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.joy_image)

        # Mapping Details (Text Area)
        mapping_area = QtWidgets.QTextEdit()
        mapping_area.setReadOnly(True)
        
        mapping_text = """
        <b>MOVEMENT & STEERING:</b><br>
        • <b>Left Stick:</b> Forward/Backward (ABS_Y) and Lateral Strafe (ABS_X)<br>
        • <b>Right Stick:</b> Turn Left/Right (Yaw) and Pitch Up/Down (ABS_RY)<br><br>
        
        <b>VERTICAL CONTROL (Depth):</b><br>
        • <b>Right Trigger (RT):</b> Thrust Down (Descend)<br>
        • <b>Left Trigger (LT):</b> Thrust Up (Ascend)<br><br>
        
        <b>SYSTEM ACTIONS:</b><br>
        • <b>A Button (BTN_SOUTH):</b> Confirm ARM / DISARM motors<br>
        • <b>B Button (BTN_EAST):</b> Emergency Stop / Reset IMU<br>
        • <b>D-Pad (Arrows):</b> Incremental Camera Tilt Step
        """
        mapping_area.setHtml(mapping_text)
        layout.addWidget(mapping_area)

        # Close Button
        close_btn = QPushButton("CLOSE")
        close_btn.setStyleSheet("""
            QPushButton {
                background-color: #2c3e50; 
                color: white; 
                font-weight: bold; 
                order: 1px solid #34495e;
                padding: 10px; 
                border-radius: 5px;
            }
            QPushButton:hover { background-color: #34495e; }
        """)
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn)

        self.setLayout(layout)



class Ui_MainWindow(QObject):

    def __init__(self) -> None:
        super().__init__()
        self.image_signals = ImageSignals()
        self.controller_status_signal = ControllerStatusSignal()

    def open_joy_config(self) -> None:
        self.joy_config_dialog.exec()

    def setupUi(self, MainWindow: QtWidgets.QMainWindow) -> None:
        """
        Function to setup the UI of the main window.
        """

        self.logs = {"Camera": "",
                    "Barometer": "",
                    "IMU": "",
                    "Thrusters": "",
                    "Diagnostic MicroROS": ""}

        self.top_image_path = "./images/upPNG_white.png"
        self.side_image_path = "./images/sidePNG_white.png"

        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1093, 780)
        MainWindow.setMinimumSize(QtCore.QSize(900, 700))
        MainWindow.setMaximumSize(QtCore.QSize(16777208, 16777215))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        MainWindow.setFont(font)
        MainWindow.setStyleSheet("background-color: #343a40; color: #f8f9fa;")
        MainWindow.setTabShape(QtWidgets.QTabWidget.TabShape.Rounded)

        # Central widget
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        # Grid layout
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")

        # Left side
        self.left_side = QtWidgets.QVBoxLayout()
        self.left_side.setObjectName("left_side")

    ######################################################################
    # HEADER
    ######################################################################
        # Frame 
        self.frame = QtWidgets.QFrame(parent=self.centralwidget)
        self.frame.setStyleSheet("background-color: #212529;")
        self.frame.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame.setLineWidth(5)
        self.frame.setObjectName("frame")

        # Top bar
        self.top_bar = QtWidgets.QHBoxLayout(self.frame)
        self.top_bar.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetMaximumSize)
        self.top_bar.setObjectName("top_bar")

        # Politocean logo and label
        self.politocean_logo = QtWidgets.QLabel(parent=self.frame)
        self.politocean_logo.setMinimumSize(QtCore.QSize(80, 88))
        self.politocean_logo.setMaximumSize(QtCore.QSize(80, 88))
        self.politocean_logo.setText("")
        self.politocean_logo.setPixmap(QtGui.QPixmap("images/logo_new.png"))
        self.politocean_logo.setScaledContents(True)
        self.politocean_logo.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeading|QtCore.Qt.AlignmentFlag.AlignLeft|QtCore.Qt.AlignmentFlag.AlignVCenter)
        self.politocean_logo.setObjectName("politocean_logo")
        self.top_bar.addWidget(self.politocean_logo)
        self.politocean_label = QtWidgets.QLabel(parent=self.frame)
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(40)
        font.setBold(True)
        self.politocean_label.setFont(font)
        self.politocean_label.setObjectName("politocean_label")
        self.top_bar.addWidget(self.politocean_label)

        self.top_bar.addStretch()

        # Status icons
        self.controller_status = QtWidgets.QLabel(parent=self.frame)
        self.controller_status.setMinimumSize(QtCore.QSize(50, 50))
        self.controller_status.setMaximumSize(QtCore.QSize(60, 60))
        self.controller_status.setText("")
        self.controller_status.setPixmap(QtGui.QPixmap("images/red_controller.png"))
        self.controller_status.setScaledContents(True)
        self.controller_status.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.controller_status.setObjectName("controller_status")
        self.top_bar.addWidget(self.controller_status)

        self.power_status = QtWidgets.QLabel(parent=self.frame)
        self.power_status.setMinimumSize(QtCore.QSize(40, 40))
        self.power_status.setMaximumSize(QtCore.QSize(60, 60))
        self.power_status.setText("")
        self.power_status.setPixmap(QtGui.QPixmap("./images/red_power.png"))
        self.power_status.setScaledContents(True)
        self.power_status.setObjectName("power_status")
        self.power_status.setMargin(10)
        self.top_bar.addWidget(self.power_status)

        self.armed_status = QtWidgets.QLabel(parent=self.frame)
        self.armed_status.setMinimumSize(QtCore.QSize(40, 40))
        self.armed_status.setMaximumSize(QtCore.QSize(60, 60))
        self.armed_status.setText("")
        self.armed_status.setPixmap(QtGui.QPixmap("./images/red_shield.png"))
        self.armed_status.setScaledContents(True)
        self.armed_status.setObjectName("armed_status")
        self.armed_status.setMargin(6)
        self.top_bar.addWidget(self.armed_status)
        

    ######################################################################
    # LEFT PART
    ######################################################################

        # Camera frame
        self.frame1 = QtWidgets.QFrame(parent=self.centralwidget)
        self.frame1.setStyleSheet("background-color: #212529;")
        self.frame1.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.frame1.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame1.setLineWidth(5)
        self.frame1.setMidLineWidth(0)
        self.frame1.setObjectName("frame1")
        self.camera_frame = QtWidgets.QVBoxLayout(self.frame1)
        self.camera_frame.setObjectName("camera_frame")
        spacerItem = QtWidgets.QSpacerItem(30, 0, QtWidgets.QSizePolicy.Policy.Expanding, QtWidgets.QSizePolicy.Policy.Minimum)
        self.camera_frame.addItem(spacerItem)

        # Main camera image
        self.main_camera_image = QtWidgets.QLabel(parent=self.frame1)
        self.main_camera_image.setMinimumSize(QtCore.QSize(900, 700))
        self.main_camera_image.setMaximumSize(QtCore.QSize(1920, 1080))
        self.main_camera_image.setStyleSheet("")
        self.main_camera_image.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.main_camera_image.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.main_camera_image.setLineWidth(5)
        self.main_camera_image.setText("")
        self.main_camera_image.setPixmap(QtGui.QPixmap("images/360_F_294161078_5nTGVd3p8753SFo7GyWtqRFk3YNlmrRh.jpg"))
        self.main_camera_image.setScaledContents(True)
        self.main_camera_image.setObjectName("main_camera_image")
        self.camera_frame.addWidget(self.main_camera_image)

        # Main camera label
        self.main_camera_label = QtWidgets.QLabel(parent=self.frame1)
        self.main_camera_label.setMinimumSize(QtCore.QSize(0, 50))
        self.main_camera_label.setMaximumSize(QtCore.QSize(16777215, 80))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(16)
        font.setBold(True)
        self.main_camera_label.setFont(font)
        self.main_camera_label.setStyleSheet("")
        self.main_camera_label.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.main_camera_label.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.main_camera_label.setLineWidth(3)
        self.main_camera_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.main_camera_label.setObjectName("main_camera_label")
        self.camera_frame.addWidget(self.main_camera_label)
        self.left_side.addWidget(self.frame1)
        self.gridLayout_2.addLayout(self.left_side, 1, 0)

    ######################################################################
    # RIGHT PART
    ######################################################################
        self.right_side = QtWidgets.QVBoxLayout()
        #self.right_side.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetMaximumSize)
        self.right_side.setObjectName("right_side")

        self.right_side.addStretch()

        # Sensors frame
        self.sensors_frame_2 = QtWidgets.QFrame(parent=self.centralwidget)
        self.sensors_frame_2.setStyleSheet("background-color: #212529;")
        self.sensors_frame_2.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.sensors_frame_2.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.sensors_frame_2.setLineWidth(5)
        self.sensors_frame_2.setObjectName("sensors_frame_2")
        self.sensors_frame_2.setMaximumWidth(300)

        # Sensors widget
        self.sensors_widget = QtWidgets.QVBoxLayout(self.sensors_frame_2)
        self.sensors_widget.setObjectName("sensors_widget")
        self.frame2 = QtWidgets.QFrame(parent=self.sensors_frame_2)
        self.frame2.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.frame2.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame2.setLineWidth(5)
        self.frame2.setObjectName("frame2")

        # IMU info
        self.imu_info = QtWidgets.QVBoxLayout(self.frame2)
        self.imu_info.setObjectName("imu_info")

        # IMU images
        self.rov_imgs_box = QtWidgets.QHBoxLayout()
        self.rov_imgs_box.setObjectName("rov_imgs_box")

        self.top_image = QtWidgets.QLabel(parent=self.frame2)
        self.top_image.setMaximumSize(QtCore.QSize(100, 100))
        self.top_image.setStyleSheet("")
        self.top_image.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.top_image.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.top_image.setLineWidth(5)
        self.top_image.setText("")
        self.top_image.setPixmap(QtGui.QPixmap(self.top_image_path))
        self.top_image.setScaledContents(True)
        self.top_image.setObjectName("top_image")
        self.rov_imgs_box.addWidget(self.top_image)

        self.side_image = QtWidgets.QLabel(parent=self.frame2)
        self.side_image.setMaximumSize(QtCore.QSize(100, 100))
        self.side_image.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.side_image.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.side_image.setLineWidth(5)
        self.side_image.setText("")
        self.side_image.setPixmap(QtGui.QPixmap(self.side_image_path))
        self.side_image.setScaledContents(True)
        self.side_image.setObjectName("side_image")
        self.rov_imgs_box.addWidget(self.side_image)
        self.imu_info.addLayout(self.rov_imgs_box)

        # Roll, pitch, yaw
        self.roll_box = QtWidgets.QHBoxLayout()
        self.roll_box.setObjectName("roll_box")

        self.roll_text = QtWidgets.QLabel(parent=self.frame2)
        self.roll_text.setMinimumSize(QtCore.QSize(0, 40))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(12)
        font.setBold(True)

        self.roll_text.setFont(font)
        self.roll_text.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.roll_text.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.roll_text.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.roll_text.setObjectName("roll_text")

        self.roll_box.addWidget(self.roll_text)

        self.roll_value = QtWidgets.QLabel(parent=self.frame2)
        self.roll_value.setMinimumSize(QtCore.QSize(0, 40))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        self.roll_value.setFont(font)
        self.roll_value.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.roll_value.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.roll_value.setLineWidth(2)
        self.roll_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.roll_value.setObjectName("roll_value")
        self.roll_value.setStyleSheet("background-color: black; color: #00FF00; font-family: 'Courier New'; font-weight: bold; border: 1px solid #444;")

        self.roll_box.addWidget(self.roll_value)
        self.imu_info.addLayout(self.roll_box)

        self.pitch_box = QtWidgets.QHBoxLayout()
        self.pitch_box.setObjectName("pitch_box")

        self.pitch_text = QtWidgets.QLabel(parent=self.frame2)
        self.pitch_text.setMinimumSize(QtCore.QSize(0, 40))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(12)
        font.setBold(True)
        self.pitch_text.setFont(font)
        self.pitch_text.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.pitch_text.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.pitch_text.setLineWidth(2)
        self.pitch_text.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.pitch_text.setObjectName("pitch_text")

        self.pitch_box.addWidget(self.pitch_text)

        self.pitch_value = QtWidgets.QLabel(parent=self.frame2)
        self.pitch_value.setMinimumSize(QtCore.QSize(0, 40))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        self.pitch_value.setFont(font)
        self.pitch_value.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.pitch_value.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.pitch_value.setLineWidth(2)
        self.pitch_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.pitch_value.setObjectName("pitch_value")
        self.pitch_value.setStyleSheet("background-color: black; color: #00FF00; font-family: 'Courier New'; font-weight: bold; border: 1px solid #444;")

        self.pitch_box.addWidget(self.pitch_value)
        self.imu_info.addLayout(self.pitch_box)

        self.yaw_box = QtWidgets.QHBoxLayout()
        self.yaw_box.setObjectName("yaw_box")

        self.yaw_text = QtWidgets.QLabel(parent=self.frame2)
        self.yaw_text.setMinimumSize(QtCore.QSize(0, 40))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(12)
        font.setBold(True)
        self.yaw_text.setFont(font)
        self.yaw_text.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.yaw_text.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.yaw_text.setLineWidth(2)
        self.yaw_text.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.yaw_text.setObjectName("yaw_text")

        self.yaw_box.addWidget(self.yaw_text)

        self.yaw_value = QtWidgets.QLabel(parent=self.frame2)
        self.yaw_value.setMinimumSize(QtCore.QSize(0, 40))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        self.yaw_value.setFont(font)
        self.yaw_value.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.yaw_value.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.yaw_value.setLineWidth(2)
        self.yaw_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.yaw_value.setObjectName("yaw_value")
        self.yaw_value.setStyleSheet("background-color: black; color: #00FF00; font-family: 'Courier New'; font-weight: bold; border: 1px solid #444;")

        self.yaw_box.addWidget(self.yaw_value)
        self.imu_info.addLayout(self.yaw_box)
        self.sensors_widget.addWidget(self.frame2)
        
        # Barometer info
        self.barometer_info = QtWidgets.QVBoxLayout()
        self.barometer_info.setObjectName("barometer_info")

        self.depth_img_box = QtWidgets.QHBoxLayout()
        self.depth_img_box.setObjectName("depth_img_box")
        self.depth_image = QtWidgets.QLabel(parent=self.sensors_frame_2)
        self.depth_image.setMaximumSize(QtCore.QSize(70, 70))
        self.depth_image.setText("")
        self.depth_image.setPixmap(QtGui.QPixmap("./images/depth2_white.png"))
        self.depth_image.setScaledContents(True)
        self.depth_image.setObjectName("depth_image")
        self.depth_img_box.addWidget(self.depth_image)
        self.barometer_info.addLayout(self.depth_img_box)
        self.barometer_box = QtWidgets.QFrame(parent=self.sensors_frame_2)
        self.barometer_box.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.barometer_box.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.barometer_box.setLineWidth(5)
        self.barometer_box.setObjectName("barometer_box")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.barometer_box)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.depth_text = QtWidgets.QLabel(parent=self.barometer_box)
        self.depth_text.setMinimumSize(QtCore.QSize(80, 40))
        self.depth_text.setMaximumSize(QtCore.QSize(100, 60))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(12)
        font.setBold(True)
        self.depth_text.setFont(font)
        self.depth_text.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.depth_text.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.depth_text.setLineWidth(2)
        self.depth_text.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.depth_text.setObjectName("depth_text")
        self.horizontalLayout.addWidget(self.depth_text)
        self.dept_value = QtWidgets.QLabel(parent=self.barometer_box)
        self.dept_value.setMinimumSize(QtCore.QSize(80, 60))
        self.dept_value.setMaximumSize(QtCore.QSize(80, 60))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        self.dept_value.setFont(font)
        self.dept_value.setFrameShape(QtWidgets.QFrame.Shape.WinPanel)
        self.dept_value.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.dept_value.setLineWidth(2)
        self.dept_value.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.dept_value.setObjectName("dept_value")
        self.dept_value.setStyleSheet("background-color: black; color: #00FFFF; font-size: 18px; font-weight: bold; border: 2px solid #00FFFF;")
        
        self.horizontalLayout.addWidget(self.dept_value)
        self.barometer_info.addWidget(self.barometer_box)
        self.sensors_widget.addLayout(self.barometer_info)
        self.right_side.addWidget(self.sensors_frame_2)

        self.right_side.addStretch()

        self.gridLayout_2.addLayout(self.right_side, 1, 1)

        self.joy_config_dialog = JoystickConfigDialog()

        self.joy_config_button = QtWidgets.QPushButton("JOYSTICK CONFIG")
        self.joy_config_button.setMinimumSize(QtCore.QSize(200, 40))
        self.joy_config_button.clicked.connect(self.open_joy_config)
        self.joy_config_button.setStyleSheet("background-color: #2c3e50; color: white; border-radius: 5px;")
        self.right_side.addWidget(self.joy_config_button)

        self.joy_config_button.setStyleSheet("""
            QPushButton {
                background-color: #2c3e50;
                color: #ecf0f1;
                border: 1px solid #34495e;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #34495e;
            }
        """)

        self.control_panel_box = QtWidgets.QHBoxLayout()
        self.control_panel_box.setSizeConstraint(QtWidgets.QLayout.SizeConstraint.SetMaximumSize)
        self.control_panel_box.setObjectName("control_panel_box")
        self.control_panel = QtWidgets.QPushButton(parent=self.centralwidget)
        self.control_panel.clicked.connect(self.open_control_panel)
        self.control_panel.setMinimumSize(QtCore.QSize(200, 60))
        self.control_panel.setMaximumSize(QtCore.QSize(250, 60))
        font = QtGui.QFont()
        font.setFamily("FreeSans")
        font.setPointSize(14)
        font.setBold(True)
        self.control_panel.setFont(font)
        #self.control_panel.setStyleSheet("background-color: #e5dcdc; color: #212529; box-shadow: rgba(0, 0, 0, 0.19) 0px 10px 20px, rgba(0, 0, 0, 0.23) 0px 6px 6px; border: 2px outset black")
        self.control_panel.setObjectName("control_panel")
        self.control_panel_box.addWidget(self.control_panel)
        self.right_side.addLayout(self.control_panel_box)
        
        self.control_panel.setStyleSheet("""
            QPushButton {
                background-color: #495057;
                color: white;
                border: 2px solid #6c757d;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #6c757d;
            }
        """)

        self.gridLayout_2.setColumnStretch(0, 4) 
        self.gridLayout_2.setColumnStretch(1, 1) 

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(parent=MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1093, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(parent=MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        # TOFIX: Calling control panel stucks the GUI
        self.control_panel_dialog = ControlPanelDialog(self.logs)

        self.gridLayout_2.addWidget(self.frame, 0, 0, 1, 2)

        # declare shortcuts
        self.getShortcuts(MainWindow)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.image_signals.image_signal.connect(lambda image: self.update_camera_frame(image, self.main_camera_image))

    @QtCore.pyqtSlot(bool)
    def update_controller_status(self, status: bool) -> None:
        if status:
            self.controller_status.setPixmap(QtGui.QPixmap("./images/green_controller.png"))
        else:
            self.controller_status.setPixmap(QtGui.QPixmap("./images/red_controller.png"))

    @QtCore.pyqtSlot(QtGui.QImage)
    def update_camera_frame(self, qt_image: QtGui.QImage, camera_frame: QtWidgets.QLabel) -> None:
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(camera_frame.size(), QtCore.Qt.AspectRatioMode.KeepAspectRatio)
            camera_frame.setPixmap(scaled_pixmap)


    def open_control_panel(self) -> None:
        self.control_panel_dialog.exec()
        self.check_armed()

    def check_armed(self) -> None:
        if self.control_panel_dialog.arm_disarm_dialog.arm_status:
            self.armed_status.setPixmap(QtGui.QPixmap("./images/green_shield.png"))
        else:
            self.armed_status.setPixmap(QtGui.QPixmap("./images/red_shield.png"))


    def retranslateUi(self, MainWindow: QtWidgets.QMainWindow) -> None:
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.politocean_label.setText(_translate("MainWindow", "PoliTOcean"))
        self.main_camera_label.setText(_translate("MainWindow", "MAIN CAMERA"))
        self.control_panel.setText(_translate("MainWindow", "CONTROL PANEL"))
        #self.camera1_label.setText(_translate("MainWindow", "CAMERA 1"))
        #self.camera2_label.setText(_translate("MainWindow", "CAMERA 2"))
        self.roll_text.setText(_translate("MainWindow", "ROLL"))
        self.roll_value.setText(_translate("MainWindow", "0°"))
        self.pitch_text.setText(_translate("MainWindow", "PITCH"))
        self.pitch_value.setText(_translate("MainWindow", "0°"))
        self.yaw_text.setText(_translate("MainWindow", "YAW"))
        self.yaw_value.setText(_translate("MainWindow", "0°"))
        self.depth_text.setText(_translate("MainWindow", "DEPTH"))
        self.dept_value.setText(_translate("MainWindow", "23 m"))


    def getShortcuts(self, MainWindow: QtWidgets.QMainWindow) -> None:
        # Create a new shortcut for switching to the main camera
        self.main_camera0_shortcut = QShortcut(QKeySequence("Ctrl+1"), MainWindow)
        self.main_camera0_shortcut.activated.connect(lambda: self.switch_camera(0))
        # Create a new shortcut for switching to the camera 1
        #self.main_camera1_shortcut = QShortcut(QKeySequence("Ctrl+2"), MainWindow)
        #self.main_camera1_shortcut.activated.connect(lambda: self.switch_camera(1))
        # Create a new shortcut for switching to the camera 2
        #self.main_camera2_shortcut = QShortcut(QKeySequence("Ctrl+3"), MainWindow)
        #self.main_camera2_shortcut.activated.connect(lambda: self.switch_camera(2))
        # Shortcut for the control panel
        self.control_panel_shortcut = QShortcut(QKeySequence("Ctrl+C"), MainWindow)
        self.control_panel_shortcut.activated.connect(self.open_control_panel)

    """
    def switch_camera(self, camera: int) -> None:
        if camera == 0:
            self.main_camera_image.setPixmap(QtGui.QPixmap("images/360_F_294161078_5nTGVd3p8753SFo7GyWtqRFk3YNlmrRh.jpg"))
            self.camera1_image.setPixmap(QtGui.QPixmap("images/prev1.jpg"))
            self.camera2_image.setPixmap(QtGui.QPixmap("images/prev2.jpg"))
            self.main_camera_label.setText("MAIN CAMERA")
            self.camera1_label.setText("CAMERA 1")
            self.camera2_label.setText("CAMERA 2")
        elif camera == 1:
            self.main_camera_image.setPixmap(QtGui.QPixmap("images/prev1.jpg"))
            self.camera1_image.setPixmap(QtGui.QPixmap("images/360_F_294161078_5nTGVd3p8753SFo7GyWtqRFk3YNlmrRh.jpg"))
            self.camera2_image.setPixmap(QtGui.QPixmap("images/prev2.jpg"))
            self.main_camera_label.setText("CAMERA 1")
            self.camera1_label.setText("MAIN CAMERA")
            self.camera2_label.setText("CAMERA 2")
        elif camera == 2:
            self.main_camera_image.setPixmap(QtGui.QPixmap("images/prev2.jpg"))
            self.camera1_image.setPixmap(QtGui.QPixmap("images/prev1.jpg"))
            self.camera2_image.setPixmap(QtGui.QPixmap("images/360_F_294161078_5nTGVd3p8753SFo7GyWtqRFk3YNlmrRh.jpg"))
            self.main_camera_label.setText("CAMERA 2")
            self.camera1_label.setText("CAMERA 1")
            self.camera2_label.setText("MAIN CAMERA")
    """
