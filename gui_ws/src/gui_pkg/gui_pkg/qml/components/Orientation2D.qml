import QtQuick 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    property real yaw: 0
    property real pitch: 0
    property real roll: 0

    RowLayout {
        anchors.fill: parent
        spacing: 10

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#0b1116"
            border.color: "#263441"
            border.width: 1
            radius: 6
            clip: true

            Image {
                anchors.top: parent.top
                anchors.bottom: yawBottomBar.top
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: 2 
                source: "../assets/icons/yaw_icon.svg"
                fillMode: Image.PreserveAspectFit
                smooth: true
                mipmap: true
                rotation: root.yaw
                opacity: 0.9
            }

            Rectangle {
                id: yawBottomBar
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                height: 24
                color: "#0f1720"
                border.color: "#20303a"
                border.width: 1
                radius: 4

                Text {
                    anchors.centerIn: parent
                    text: "YAW  " + root.yaw.toFixed(1) + "°"
                    color: "#d7e6f2"
                    font.pixelSize: 10
                    font.bold: true
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "#0b1116"
            border.color: "#263441"
            border.width: 1
            radius: 6
            clip: true

            Image {
                anchors.top: parent.top
                anchors.bottom: rollBottomBar.top
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: 2
                source: "../assets/icons/roll_icon.svg"
                fillMode: Image.PreserveAspectFit
                smooth: true
                mipmap: true
                rotation: root.roll
                opacity: 0.9
            }

            Rectangle {
                id: rollBottomBar
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                height: 24
                color: "#0f1720"
                border.color: "#20303a"
                border.width: 1
                radius: 4

                Text {
                    anchors.centerIn: parent
                    text: "ROLL  " + root.roll.toFixed(1) + "°"
                    color: "#d7e6f2"
                    font.pixelSize: 10
                    font.bold: true
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "transparent"
            border.color: "#263441"
            border.width: 1
            radius: 6
            clip: true

            Image {
                anchors.top: parent.top
                anchors.bottom: pitchBottomBar.top
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.margins: 2
                source: "../assets/icons/pitch_icon.svg"
                fillMode: Image.PreserveAspectFit
                smooth: true
                mipmap: true
                rotation: root.pitch
                opacity: 0.9
            }

            Rectangle {
                id: pitchBottomBar
                anchors.left: parent.left
                anchors.right: parent.right
                anchors.bottom: parent.bottom
                height: 24
                color: "#0f1720"
                border.color: "#20303a"
                border.width: 1
                radius: 4

                Text {
                    anchors.centerIn: parent
                    text: "PITCH  " + root.pitch.toFixed(1) + "°"
                    color: "#d7e6f2"
                    font.pixelSize: 10
                    font.bold: true
                }
            }
        }

    }
}