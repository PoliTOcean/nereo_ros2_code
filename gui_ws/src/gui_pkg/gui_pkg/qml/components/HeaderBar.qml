import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    property bool rovConnected: false
    property bool joystickConnected: false
    property bool rovArmed: false
    property bool controlActive: false
    property string logoSource: "../assets/logo/Logotype-Colored.svg"
    // Overall vehicle status level, pre-computed in gui_node
    // (freshness.summary_level) — one of "ok", "stale", "down" (D-03,
    // PANEL-13). This component never derives it from badge strings or
    // a clock; it only renders the level it is handed.
    property string statusSummary: "down"
    signal statusClicked()

    height: 70
    color: "#12141a"
    border.color: "#333"
    border.width: 1
    z: 10

    RowLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 20

        Image {
            source: root.logoSource
            fillMode: Image.PreserveAspectFit
            smooth: true
            mipmap: true
            sourceSize.height: 40
            Layout.preferredHeight: 40
            Layout.preferredWidth: 220
            Layout.fillWidth: false
        }

        RowLayout {
            spacing: 15
            Layout.alignment: Qt.AlignRight

            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#1a1a1a"
                border.color: root.statusSummary === "ok" ? "#2ecc71"
                            : root.statusSummary === "stale" ? "#f1c40f"
                            : "#ff3333"
                border.width: 2
                Text {
                    text: root.statusSummary === "ok" ? "✓" : "!"
                    color: parent.border.color
                    font.pixelSize: 18
                    font.bold: true
                    anchors.centerIn: parent
                }
                ToolTip.visible: statusMouseArea.containsMouse
                ToolTip.text: root.statusSummary === "ok"
                              ? "Stato veicolo: OK"
                              : root.statusSummary === "stale"
                                ? "Stato veicolo: dati non aggiornati"
                                : "Stato veicolo: dati mancanti o link giù"
                MouseArea {
                    id: statusMouseArea
                    anchors.fill: parent
                    hoverEnabled: true
                    cursorShape: Qt.PointingHandCursor
                    onClicked: root.statusClicked()
                }
            }

            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#1a1a1a"
                border.color: root.rovConnected ? "#00b7ff" : "#ff3333"
                border.width: 2
                Text { text: "⛓"; color: parent.border.color; font.pixelSize: 18; font.bold: true; anchors.centerIn: parent }
                ToolTip.visible: rovConnMouseArea.containsMouse
                ToolTip.text: root.rovConnected ? "ROV Connesso" : "ROV Disconnesso"
                MouseArea { id: rovConnMouseArea; anchors.fill: parent; hoverEnabled: true }
            }

            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#1a1a1a"
                border.color: root.joystickConnected ? "#00b7ff" : "#ff3333"
                border.width: 2
                Text { text: "🎮"; color: parent.border.color; font.pixelSize: 18; font.bold: true; anchors.centerIn: parent }
                ToolTip.visible: joyConnMouseArea.containsMouse
                ToolTip.text: root.joystickConnected ? "Joystick Connesso" : "Joystick Disconnesso"
                MouseArea { id: joyConnMouseArea; anchors.fill: parent; hoverEnabled: true }
            }

            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#1a1a1a"
                border.color: root.rovArmed ? "#00b7ff" : "#ff3333"
                border.width: 2
                Text { text: root.rovArmed ? "🔒" : "🔓"; color: parent.border.color; font.pixelSize: 18; font.bold: true; anchors.centerIn: parent }
                ToolTip.visible: armStatusMouseArea.containsMouse
                ToolTip.text: root.rovArmed ? "ROV ARMATO" : "ROV DISARMATO"
                MouseArea { id: armStatusMouseArea; anchors.fill: parent; hoverEnabled: true }
            }

            Rectangle {
                width: 40
                height: 40
                radius: 20
                color: "#1a1a1a"
                border.color: root.controlActive ? "#00b7ff" : "#ff3333"
                border.width: 2
                Text { text: "👮🏻"; color: parent.border.color; font.pixelSize: 18; font.bold: true; anchors.centerIn: parent }
                ToolTip.visible: ctrlMouseArea.containsMouse
                ToolTip.delay: 500
                ToolTip.text: root.controlActive ? "Modalità CONTROLLORE → /nereo_cmd_vel_no_fb" : "Modalità DIRETTA → /nereo_cmd_vel"
                MouseArea { id: ctrlMouseArea; anchors.fill: parent; hoverEnabled: true }
            }
        }
    }
}
