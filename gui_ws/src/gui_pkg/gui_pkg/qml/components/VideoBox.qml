import QtQuick 2.15

Rectangle {
    id: root
    property string title: "CAMERA"
    property string camId: "main_cam"
    property bool isOn: true 
    property bool isClickableToSwap: false
    property bool isTopCam: false
    property color baseColor: "#1a252c"

    signal swapRequested()

    color: isOn ? baseColor : "#050505"
    border.color: "#00e6e6"
    border.width: isOn ? 1 : 0
    radius: 4

    Image {
        id: videoDisplay
        anchors.fill: parent
        anchors.margins: 2
        visible: root.isOn
        fillMode: Image.PreserveAspectFit

        property int frameCounter: 0

        source: "image://videostream/" + root.camId + "?" + frameCounter

        Timer {
            interval: 33
            running: root.isOn
            repeat: true
            onTriggered: videoDisplay.frameCounter++
        }
    }

    Rectangle {
        id: textBackground
        color: "#cc1a252c"
        radius: 4
        visible: root.isOn
        anchors.centerIn: labelText
        width: labelText.implicitWidth + 8
        height: labelText.implicitHeight + 8
    }

    Text {
        id: labelText
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 18
        text: root.isOn ? root.title : "OFFLINE"
        color: root.isOn ? "#00e6e6" : "#444"
        font.pixelSize: 20
        font.bold: true
    }

    Rectangle {
        id: powerButtonContainer
        width: 30
        height: 30
        anchors.top: parent.top
        anchors.right: parent.right
        anchors.margins: 10
        color: "#cc1a252c"
        border.color: root.isOn ? "#ff3333" : "#33cc33"
        radius: 15
        z: 3 

        Text {
            anchors.centerIn: parent
            text: "⏻"
            color: powerButtonContainer.border.color
            font.pixelSize: 18
        }

        MouseArea {
            anchors.fill: parent
            onClicked: {
                root.isOn = !root.isOn 
            }
        }
    }

    MouseArea {
        anchors.fill: parent
        enabled: root.isClickableToSwap && root.isOn
        z: 1
        onClicked: {
            root.swapRequested()
        }
    }
}