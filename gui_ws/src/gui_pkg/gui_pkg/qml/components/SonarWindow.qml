import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: root

    width: 780
    height: 520
    title: "Sonar Viewer"
    visible: false
    color: "#0f1115"
    flags: Qt.Dialog

    // ── sonar data bindings ───────────────────────────────────────────────
    property real sonarDistance:   SonarBridge.sonarDistance
    property int  sonarConfidence: SonarBridge.sonarConfidence

    property int  confThreshold: SonarBridge.confidenceThreshold
    property bool belowThreshold: sonarConfidence < confThreshold

    // frame counter drives image reload when SonarBridge.frame_ready fires
    property int frameCounter: 0
    Connections {
        target: SonarBridge
        function onFrame_ready() { root.frameCounter++ }
    }

    // ── layout ────────────────────────────────────────────────────────────
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 12

        // header row: distance | confidence | threshold control
        RowLayout {
            Layout.fillWidth: true
            spacing: 12

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 68
                color: "#111"
                border.color: root.belowThreshold ? "#ff4444" : "#00b7ff"
                border.width: 1
                radius: 4

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 2
                    Text { text: "DISTANCE"; color: "#8aa1b3"; font.pixelSize: 11; font.bold: true }
                    Text {
                        text: root.belowThreshold
                              ? root.sonarDistance.toFixed(2) + " m *"
                              : root.sonarDistance.toFixed(2) + " m"
                        color: root.belowThreshold ? "#ff4444" : "#ffffff"
                        font.pixelSize: 26
                        font.bold: true
                    }
                }
            }

            Rectangle {
                Layout.preferredWidth: 180
                Layout.preferredHeight: 68
                color: "#111"
                border.color: root.belowThreshold ? "#ff4444" : "#00b7ff"
                border.width: 1
                radius: 4

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 2
                    Text { text: "CONFIDENCE"; color: "#8aa1b3"; font.pixelSize: 11; font.bold: true }
                    Text {
                        text: root.sonarConfidence + " %"
                        color: root.belowThreshold ? "#ff4444" : "#00b7ff"
                        font.pixelSize: 24
                        font.bold: true
                    }
                }
            }

            Rectangle {
                Layout.preferredWidth: 210
                Layout.preferredHeight: 68
                color: "#111"
                border.color: "#333"
                border.width: 1
                radius: 4

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 4

                    Text { text: "CONFIDENCE THRESHOLD"; color: "#8aa1b3"; font.pixelSize: 10; font.bold: true }

                    RowLayout {
                        spacing: 8
                        Text { text: thresholdSlider.value + " %"; color: "#cfefff"; font.pixelSize: 14; font.bold: true; Layout.preferredWidth: 40 }
                        Slider {
                            id: thresholdSlider
                            Layout.fillWidth: true
                            from: 0; to: 100; stepSize: 5
                            value: root.confThreshold
                            onValueChanged: SonarBridge.confidenceThreshold = value
                        }
                    }
                }
            }
        }

        // plots row: waterfall | A-scan
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 8

            // waterfall
            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#000010"
                border.color: "#1b2832"
                border.width: 1
                radius: 4
                clip: true

                Image {
                    anchors.fill: parent
                    anchors.margins: 2
                    fillMode: Image.Stretch
                    smooth: false
                    cache: false
                    source: "image://sonar/waterfall?" + root.frameCounter
                }

                Text {
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 6
                    text: "WATERFALL"
                    color: "#ffffff"
                    font.pixelSize: 10
                    font.bold: true
                }
            }

            // A-scan
            Rectangle {
                Layout.preferredWidth: 150
                Layout.fillHeight: true
                color: "#0055a4"
                border.color: "#1b2832"
                border.width: 1
                radius: 4
                clip: true

                Image {
                    anchors.fill: parent
                    anchors.margins: 2
                    fillMode: Image.Stretch
                    smooth: false
                    cache: false
                    source: "image://sonar/ascan?" + root.frameCounter
                }

                Text {
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.margins: 6
                    text: "A-SCAN"
                    color: "#ffffff"
                    font.pixelSize: 10
                    font.bold: true
                }
            }
        }
    }
}
