import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Window 2.15
import QtQuick.Layouts 1.15
import "components"

ApplicationWindow {
    id: window
    visible: true
    width: 1200
    height: 750
    title: "PoliTOcean - Nereo Dashboard"
    color: "#0f1115"

    property bool rovConnected: RosBridge.rovConnected
    property bool joystickConnected: RosBridge.joystickConnected
    property bool rovArmed: RosBridge.rovArmed
    property bool controlActive: RosBridge.controlActive

    property real rovRoll: RosBridge.rovRoll
    property real rovPitch: RosBridge.rovPitch
    property real rovYaw: RosBridge.rovYaw
    property real rovDepth: RosBridge.rovDepth


    property string mainFeedSource: "MAIN CAMERA"
    property string mainFeedId: "main_cam"          
    property string mainFeedColor: "#1a252c"

    property string topFeedSource: "CAMERA 1"
    property string topFeedId: "cam_1"              
    property string topFeedColor: "#1d2b36"

    property string bottomFeedSource: "CAMERA 2"
    property string bottomFeedId: "cam_2"           
    property string bottomFeedColor: "#172228"

    function swapWithMain(clickedFeed, clickedId, isTop) {
        if (isTop && !camera1Box.isOn) return;
        if (!isTop && !camera2Box.isOn) return;

        let tempSource = mainFeedSource;
        let tempId = mainFeedId;

        mainFeedSource = clickedFeed;
        mainFeedId = clickedId;

        if (isTop) {
            topFeedSource = tempSource;
            topFeedId = tempId;
            mainCamera.isOn = true;
            camera1Box.isOn = true;
        } else {
            bottomFeedSource = tempSource;
            bottomFeedId = tempId;
            mainCamera.isOn = true;
            camera2Box.isOn = true;
        }
    }

    function sendArmCommand(armed) {
        console.log("QML invia richiesta cambio stato ARM:", armed ? "ARM" : "DISARM")
        RosBridge.sendArmCommand(armed) 
    }
    
    HeaderBar {
        id: header
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        rovConnected: window.rovConnected
        joystickConnected: window.joystickConnected
        rovArmed: window.rovArmed
        controlActive: window.controlActive
    }

    SonarWindow {
        id: sonarWindow
    }

    ControlPanelWindow {
        id: controlPanelWindow
        rovArmed: window.rovArmed
        onArmRequested: (armed) => {
            sendArmCommand(armed)
        }
    }

    ColumnLayout {
        anchors.top: header.bottom
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.margins: 15
        spacing: 15

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 15

            VideoBox {
                id: mainCamera
                camId: mainFeedId 
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumWidth: 600
                baseColor: mainFeedColor
                title: mainFeedSource
                isClickableToSwap: false
            }

            ColumnLayout {
                id: sideColumn
                Layout.preferredWidth: 450
                Layout.maximumWidth: 450
                Layout.fillHeight: true
                spacing: 10

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    Button {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 50
                        text: "CONTROL PANEL"
                        font.pixelSize: 16
                        font.bold: true
                        onClicked: controlPanelWindow.visible = true
                        background: Rectangle {
                            color: parent.down ? "#009999" : "#005555"
                            border.color: "#00e6e6"
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

                    Button {
                        Layout.preferredWidth: 150
                        Layout.preferredHeight: 50
                        text: "SONAR"
                        font.pixelSize: 16
                        font.bold: true
                        onClicked: sonarWindow.visible = true
                        background: Rectangle {
                            color: parent.down ? "#1b3b4a" : "#0f2b36"
                            border.color: "#00b7ff"
                            radius: 4
                        }
                        contentItem: Text {
                            text: parent.text
                            color: "#cfefff"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font: parent.font
                        }
                    }
                }

                VideoBox {
                    id: camera1Box
                    camId: topFeedId 
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    baseColor: topFeedColor
                    title: topFeedSource
                    isClickableToSwap: true
                    isTopCam: true
                    onSwapRequested: swapWithMain(topFeedSource, topFeedId, true) 
                }

                VideoBox {
                    id: camera2Box
                    camId: bottomFeedId 
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    baseColor: bottomFeedColor
                    title: bottomFeedSource
                    isClickableToSwap: true
                    isTopCam: false
                    onSwapRequested: swapWithMain(bottomFeedSource, bottomFeedId, false) 
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 200
                    color: "#111"
                    border.color: "#333"
                    border.width: 1
                    radius: 4
                    
                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 10

                        RowLayout {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 140

                            Orientation2D {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                yaw: rovYaw                            
                                pitch: rovPitch
                                roll: rovRoll                                
                            }
                        }

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 36
                            color: "#0c141b"
                            border.color: "#1b2832"
                            border.width: 1
                            radius: 4
                            
                            RowLayout {
                                anchors.fill: parent
                                anchors.margins: 8
                                spacing: 8
                                Text { 
                                    text: "DEPTH"
                                    color: "#8aa1b3"
                                    font.pixelSize: 12
                                    font.bold: true 
                                }
                                Text { 
                                    text: rovDepth.toFixed(2) + " m"
                                    color: "#00e6e6"
                                    font.pixelSize: 16
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