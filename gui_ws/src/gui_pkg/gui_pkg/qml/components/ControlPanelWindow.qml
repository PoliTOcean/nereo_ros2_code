import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: root
    property bool rovArmed: false
    signal armRequested(bool armed)

    width: 560
    height: 420
    title: "Control Panel"
    visible: false
    color: "#1f2328"
    flags: Qt.Dialog

    ListModel {
        id: peripheralModel
        ListElement { name: "Camera"; status: "OK"; icon: "📹" }
        ListElement { name: "Barometer"; status: "OK"; icon: "🌡" }
        ListElement { name: "IMU"; status: "OK"; icon: "🧭" }
        ListElement { name: "Thrusters"; status: "OK"; icon: "🚀" }
        ListElement { name: "Diagnostic MicroROS"; status: "OK"; icon: "📋" }
    }

    Rectangle {
        anchors.fill: parent
        color: "#1f2328"
        border.color: "#2c3238"
        border.width: 1
        radius: 6

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 12
            spacing: 10

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 36
                color: "#262b31"
                border.color: "#2f363d"
                border.width: 1
                radius: 4
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 8
                    Text { text: "CONTROL PANEL"; color: "#d8e2ea"; font.pixelSize: 12; font.bold: true; Layout.fillWidth: true }
                    Text { text: root.rovArmed ? "ARMED" : "DISARMED"; color: root.rovArmed ? "#ffcc00" : "#9fb3c3"; font.pixelSize: 12; font.bold: true }
                }
            }

            Button {
                Layout.fillWidth: true
                Layout.preferredHeight: 42
                text: root.rovArmed ? "DISARM" : "ARM"
                font.pixelSize: 14
                font.bold: true
                onClicked: root.armRequested(!root.rovArmed)
                background: Rectangle {
                    color: parent.down ? "#1f4b6e" : "#255a82"
                    border.color: "#7bb8e3"
                    radius: 4
                }
                contentItem: Text {
                    text: parent.text
                    color: "white"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font: parent.font
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#23272d"
                border.color: "#2f363d"
                border.width: 1
                radius: 4

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 6
                    spacing: 6

                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 26
                        color: "#2b3138"
                        RowLayout {
                            anchors.fill: parent
                            anchors.margins: 6
                            spacing: 6
                            Text { text: "Peripheral"; color: "#cbd5df"; font.pixelSize: 12; Layout.fillWidth: true }
                            Text { text: "Status"; color: "#cbd5df"; font.pixelSize: 12; width: 100; horizontalAlignment: Text.AlignHCenter }
                        }
                    }

                    Repeater {
                        model: peripheralModel
                        delegate: Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 28
                            color: index % 2 === 0 ? "#23272d" : "#1f2328"
                            RowLayout {
                                anchors.fill: parent
                                anchors.margins: 6
                                spacing: 6
                                Text { text: icon; color: "#cfefff"; font.pixelSize: 12 }
                                Text { text: (index + 1) + " " + name; color: "#e6edf3"; font.pixelSize: 12; Layout.fillWidth: true }
                                Rectangle {
                                    width: 70
                                    height: 18
                                    radius: 9
                                    color: status === "OK" ? "#1b4d6b" : "#7a2b2b"
                                    border.color: status === "OK" ? "#4cc6ff" : "#e06666"
                                    border.width: 1
                                    Text {
                                        anchors.centerIn: parent
                                        text: status
                                        color: "#e6edf3"
                                        font.pixelSize: 11
                                        font.bold: true
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
