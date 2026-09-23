import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: splashRoot
    width: 900
    height: 420
    visible: false
    color: "#0f1115"
    flags: Qt.SplashScreen | Qt.FramelessWindowHint | Qt.WindowTransparentForInput
    x: (Screen.width  - width)  / 2
    y: (Screen.height - height) / 2

    signal closed()

    Image {
        anchors.fill: parent
        source: "../assets/logo/Logotype-Colored.svg"
        fillMode: Image.PreserveAspectFit
        sourceSize.width: splashRoot.width
        sourceSize.height: splashRoot.height
        smooth: true
    }

    Timer {
        id: closeTimer
        interval: 2000
        repeat: false
        onTriggered: { splashRoot.close(); splashRoot.closed() }
    }

    onVisibleChanged: if (visible) closeTimer.start()
}
