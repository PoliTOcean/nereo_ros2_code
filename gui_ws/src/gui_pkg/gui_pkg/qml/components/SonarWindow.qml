import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: root
    property real sonarAltimeter: 0.0
    property int sonarConfidence: 0

    width: 700
    height: 420
    title: "Sonar Viewer"
    visible: false
    color: "#0f1115"
    flags: Qt.Dialog

    Rectangle {
        anchors.fill: parent
        color: "#0f1115"
        border.color: "#333"
        border.width: 1

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 16
            spacing: 14

            Text {
                text: "SONAR"
                color: "#00b7ff"
                font.pixelSize: 20
                font.bold: true
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 12

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 70
                    color: "#111"
                    border.color: "#333"
                    border.width: 1
                    radius: 4
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 4
                        Text { text: "ALTEZZA"; color: "#aaa"; font.pixelSize: 12 }
                        Text { text: root.sonarAltimeter.toFixed(2) + " m"; color: "#fff"; font.pixelSize: 22; font.bold: true }
                    }
                }

                Rectangle {
                    Layout.preferredWidth: 200
                    Layout.preferredHeight: 70
                    color: "#111"
                    border.color: "#333"
                    border.width: 1
                    radius: 4
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 4
                        Text { text: "CONFIDENCE"; color: "#aaa"; font.pixelSize: 12 }
                        Text { text: root.sonarConfidence + " %"; color: "#00b7ff"; font.pixelSize: 20; font.bold: true }
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 10

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: "#0a0f14"
                    border.color: "#222"
                    border.width: 1
                    radius: 4
                    Text {
                        anchors.centerIn: parent
                        text: "WATERFALL (placeholder)"
                        color: "#555"
                        font.pixelSize: 14
                    }
                }

                Rectangle {
                    Layout.preferredWidth: 160
                    Layout.fillHeight: true
                    color: "#0a0f14"
                    border.color: "#222"
                    border.width: 1
                    radius: 4
                    Text {
                        anchors.centerIn: parent
                        text: "RETURN PLOT (placeholder)"
                        color: "#555"
                        font.pixelSize: 12
                    }
                }
            }
        }
    }
}
