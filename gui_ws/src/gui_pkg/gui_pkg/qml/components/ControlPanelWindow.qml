import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: root
    property bool rovArmed: false
    property var vehicleStatus: ({"groups": []})
    signal armRequested(bool armed)

    width: 600
    height: 760
    title: "Control Panel"
    visible: false
    color: "#1f2328"
    flags: Qt.Dialog

    // Render-only status row: value, unit, badge pill, age. Branches only
    // on the pre-computed badge strings and the alert flag handed down
    // from gui_node — no clock, no arithmetic (PANEL-13).
    component StatusRow: Rectangle {
        property var row
        Layout.fillWidth: true
        Layout.preferredHeight: 26
        color: "#23272d"

        RowLayout {
            anchors.fill: parent
            anchors.margins: 6
            spacing: 6

            Text {
                text: row.label
                color: "#cbd5df"
                font.pixelSize: 12
                Layout.fillWidth: true
            }
            Text {
                text: row.unit !== ""
                      ? row.text + " " + row.unit
                      : row.text
                color: row.alert ? "#ffcc00" : "#e6edf3"
                font.pixelSize: 13
            }
            Rectangle {
                width: 74
                height: 18
                radius: 9
                color: row.badge === "OK" ? "#1b4d6b"
                     : row.badge === "STALE" ? "#7a6a1b" : "#7a2b2b"
                border.color: row.badge === "OK" ? "#4cc6ff"
                            : row.badge === "STALE" ? "#e0c050" : "#e06666"
                border.width: 1
                Text {
                    anchors.centerIn: parent
                    text: row.badge
                    color: "#e6edf3"
                    font.pixelSize: 10
                    font.bold: true
                }
            }
            Text {
                text: row.age
                color: "#8aa1b3"
                font.pixelSize: 11
                width: 40
                horizontalAlignment: Text.AlignRight
            }
        }
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

                ScrollView {
                    anchors.fill: parent
                    anchors.margins: 6
                    clip: true

                    ColumnLayout {
                        width: parent.width
                        spacing: 10

                        Repeater {
                            model: root.vehicleStatus.groups
                            delegate: ColumnLayout {
                                property var group: modelData
                                Layout.fillWidth: true
                                spacing: 2

                                RowLayout {
                                    Layout.fillWidth: true
                                    Text {
                                        text: group.name
                                        color: "#cfefff"
                                        font.pixelSize: 12
                                        font.bold: true
                                        Layout.fillWidth: true
                                    }
                                    Rectangle {
                                        visible: group.badge !== ""
                                        width: 74
                                        height: 18
                                        radius: 9
                                        color: "#7a2b2b"
                                        border.color: "#e06666"
                                        border.width: 1
                                        Text {
                                            anchors.centerIn: parent
                                            text: group.badge
                                            color: "#e6edf3"
                                            font.pixelSize: 10
                                            font.bold: true
                                        }
                                    }
                                    Text {
                                        visible: group.badge !== ""
                                        text: group.note
                                        color: "#8aa1b3"
                                        font.pixelSize: 11
                                    }
                                }

                                Repeater {
                                    model: group.rows
                                    delegate: StatusRow { row: modelData }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
