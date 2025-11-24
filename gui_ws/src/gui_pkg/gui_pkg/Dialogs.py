from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtWidgets import (
    QPushButton, 
    QTableWidget, 
    QTableWidgetItem, 
    QHeaderView, 
    QVBoxLayout, 
    QDialog,
    QLabel,
    QTextEdit,
    QDialogButtonBox
    )
from PyQt6.QtGui import QShortcut, QKeySequence
from PyQt6.QtCore import Qt, QObject

from . import Services

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