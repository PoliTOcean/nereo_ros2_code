import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Window 2.15

Window {
    id: root

    width: 980
    height: 720
    title: "Nereo Controller Tuner"
    visible: false
    color: "#0f1115"
    flags: Qt.Dialog

    // ── bindings ──────────────────────────────────────────────────────────
    property bool   controllerConnected: ControllerBridge.controllerConnected
    property string statusMessage: "In attesa…"
    property string statusSeverity: "info"

    // Editable model: kept in sync from ControllerBridge.params_loaded
    property int controlMode: 0

    property var kp: [0.0, 0.0, 0.0, 0.0]
    property var ki: [0.0, 0.0, 0.0, 0.0]
    property var kd: [0.0, 0.0, 0.0, 0.0]

    property bool manualDepth: false
    property bool manualRoll:  false
    property bool manualPitch: false
    property bool manualYaw:   false

    // Stored in display units: depth in metres, angles in degrees.
    // Conversion to controller units (Pa, rad) happens in onParams_loaded / applyAll.
    property real setpointDepth: 0.0
    property real setpointRoll:  0.0
    property real setpointPitch: 0.0
    property real setpointYaw:   0.0

    // Unit conversion: GUI uses metres and degrees, controller uses Pa and radians.
    // depthRho matches the density used by barometer_depth_salt topic in the GUI.
    readonly property real depthRho: 1025.0
    readonly property real gravity: 9.81
    function rad2deg(r) { return r * 180.0 / Math.PI }
    function deg2rad(d) { return d * Math.PI / 180.0 }
    function pa2m(pa)   { return pa / (root.depthRho * root.gravity) }
    function m2pa(m)    { return m * root.depthRho * root.gravity }

    property var csKx0: [0.0, 0.0]
    property var csKx1: [0.0, 0.0]
    property var csKx2: [0.0, 0.0]
    property real csKi0: 0.0
    property real csKi1: 0.0
    property real csKi2: 0.0
    property real csHeaveMin: 0.0
    property real csHeaveMax: 0.0
    property real csAngleMin: 0.0
    property real csAngleMax: 0.0

    // Live telemetry
    property var liveSetpoints: []
    property var liveErrors: []
    property var livePidTerms: []

    Connections {
        target: ControllerBridge
        function onStatus_changed(msg, sev) {
            root.statusMessage = msg
            root.statusSeverity = sev
        }
        function onParams_loaded(p) {
            if (p.control_mode !== undefined) root.controlMode = p.control_mode
            if (p.kp) root.kp = p.kp
            if (p.ki) root.ki = p.ki
            if (p.kd) root.kd = p.kd
            if (p.manual_setpoint_depth !== undefined) root.manualDepth = p.manual_setpoint_depth
            if (p.manual_setpoint_roll  !== undefined) root.manualRoll  = p.manual_setpoint_roll
            if (p.manual_setpoint_pitch !== undefined) root.manualPitch = p.manual_setpoint_pitch
            if (p.manual_setpoint_yaw   !== undefined) root.manualYaw   = p.manual_setpoint_yaw
            if (p.setpoint_depth !== undefined) root.setpointDepth = root.pa2m(p.setpoint_depth)
            if (p.setpoint_roll  !== undefined) root.setpointRoll  = root.rad2deg(p.setpoint_roll)
            if (p.setpoint_pitch !== undefined) root.setpointPitch = root.rad2deg(p.setpoint_pitch)
            if (p.setpoint_yaw   !== undefined) root.setpointYaw   = root.rad2deg(p.setpoint_yaw)
            if (p.cs_kx0) root.csKx0 = p.cs_kx0
            if (p.cs_kx1) root.csKx1 = p.cs_kx1
            if (p.cs_kx2) root.csKx2 = p.cs_kx2
            if (p.cs_ki0 !== undefined) root.csKi0 = p.cs_ki0
            if (p.cs_ki1 !== undefined) root.csKi1 = p.cs_ki1
            if (p.cs_ki2 !== undefined) root.csKi2 = p.cs_ki2
            if (p.cs_heave_min !== undefined) root.csHeaveMin = p.cs_heave_min
            if (p.cs_heave_max !== undefined) root.csHeaveMax = p.cs_heave_max
            if (p.cs_angle_min !== undefined) root.csAngleMin = p.cs_angle_min
            if (p.cs_angle_max !== undefined) root.csAngleMax = p.cs_angle_max
        }
        function onSetpoints_changed(v) { root.liveSetpoints = v }
        function onErrors_changed(v)    { root.liveErrors = v }
        function onPid_terms_changed(v) { root.livePidTerms = v }
    }

    function severityColor(sev) {
        if (sev === "ok")   return "#00e676"
        if (sev === "warn") return "#ffb300"
        if (sev === "err")  return "#ff5252"
        return "#8aa1b3"
    }

    function applyAll() {
        var payload = {
            control_mode: parseInt(controlMode),
            kp: [parseFloat(kp[0]), parseFloat(kp[1]), parseFloat(kp[2]), parseFloat(kp[3])],
            ki: [parseFloat(ki[0]), parseFloat(ki[1]), parseFloat(ki[2]), parseFloat(ki[3])],
            kd: [parseFloat(kd[0]), parseFloat(kd[1]), parseFloat(kd[2]), parseFloat(kd[3])],
            manual_setpoint_depth: manualDepth,
            manual_setpoint_roll:  manualRoll,
            manual_setpoint_pitch: manualPitch,
            manual_setpoint_yaw:   manualYaw,
            setpoint_depth: root.m2pa(parseFloat(setpointDepth)),
            setpoint_roll:  root.deg2rad(parseFloat(setpointRoll)),
            setpoint_pitch: root.deg2rad(parseFloat(setpointPitch)),
            setpoint_yaw:   root.deg2rad(parseFloat(setpointYaw)),
            cs_kx0: [parseFloat(csKx0[0]), parseFloat(csKx0[1])],
            cs_kx1: [parseFloat(csKx1[0]), parseFloat(csKx1[1])],
            cs_kx2: [parseFloat(csKx2[0]), parseFloat(csKx2[1])],
            cs_ki0: parseFloat(csKi0),
            cs_ki1: parseFloat(csKi1),
            cs_ki2: parseFloat(csKi2),
            cs_heave_min: parseFloat(csHeaveMin),
            cs_heave_max: parseFloat(csHeaveMax),
            cs_angle_min: parseFloat(csAngleMin),
            cs_angle_max: parseFloat(csAngleMax)
        }
        ControllerBridge.applyParameters(payload)
    }

    // ── reusable styled components ────────────────────────────────────────
    component NumField: TextField {
        id: numFieldRoot
        property real value: 0.0
        signal committed(real newValue)

        // Fixed size so the field never grows/shrinks with its text content.
        implicitWidth: 140
        implicitHeight: 28

        function _format(v) {
            return Number(v).toFixed(6).replace(/\.?0+$/, "")
        }
        text: _format(value)
        // re-sync text when external value changes, but only if not focused (avoids overwriting user typing)
        onValueChanged: if (!activeFocus) text = _format(value)
        onEditingFinished: {
            var v = parseFloat(text.replace(",", "."))
            if (!isNaN(v) && v !== value) committed(v)
            text = _format(isNaN(v) ? value : v)
        }
        color: "#ffffff"
        font.family: "Courier New"
        font.pixelSize: 13
        selectByMouse: true
        background: Rectangle {
            color: "#1c2436"
            border.color: numFieldRoot.activeFocus ? "#00c8ff" : "#1e3a5f"
            radius: 3
        }
    }

    component SectionFrame: Rectangle {
        property string title: ""
        default property alias body: bodyContainer.data
        color: "#111"
        border.color: "#1b2832"
        border.width: 1
        radius: 4

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 8

            Text {
                text: title
                color: "#00c8ff"
                font.pixelSize: 12
                font.bold: true
            }

            Rectangle { Layout.fillWidth: true; height: 1; color: "#1b2832" }

            Item {
                id: bodyContainer
                Layout.fillWidth: true
                Layout.fillHeight: true
                implicitHeight: childrenRect.height
            }
        }
    }

    // ── layout ────────────────────────────────────────────────────────────
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 10

        // header row: status + actions
        RowLayout {
            Layout.fillWidth: true
            spacing: 10

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 56
                color: "#111"
                border.color: root.controllerConnected ? "#00b7ff" : "#ff5252"
                border.width: 1
                radius: 4

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 10
                    Text {
                        text: "●"
                        color: severityColor(root.statusSeverity)
                        font.pixelSize: 16
                    }
                    Text {
                        text: root.controllerConnected
                              ? "CONNESSO  /nereo_controller_node"
                              : "DISCONNESSO"
                        color: "#cfefff"
                        font.pixelSize: 12
                        font.bold: true
                    }
                    Text {
                        text: root.statusMessage
                        color: "#8aa1b3"
                        font.pixelSize: 11
                        Layout.fillWidth: true
                        elide: Text.ElideRight
                    }
                }
            }

            Button {
                text: "RELOAD"
                Layout.preferredHeight: 56
                Layout.preferredWidth: 110
                font.bold: true
                onClicked: ControllerBridge.loadParameters()
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

            Button {
                text: "APPLY"
                Layout.preferredHeight: 56
                Layout.preferredWidth: 110
                font.bold: true
                enabled: root.controllerConnected
                onClicked: root.applyAll()
                background: Rectangle {
                    color: parent.enabled ? (parent.down ? "#009999" : "#005555") : "#222"
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
        }

        // scrollable content
        ScrollView {
            Layout.fillWidth: true
            Layout.fillHeight: true
            clip: true

            ColumnLayout {
                width: root.width - 28
                spacing: 10

                // ── Control Mode ────────────────────────────────────────────
                SectionFrame {
                    title: "CONTROL MODE"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 70

                    RowLayout {
                        anchors.fill: parent
                        spacing: 12
                        ComboBox {
                            id: modeCombo
                            model: ["0 — passthrough", "1 — PID", "2 — PID anti-windup", "3 — CS"]
                            currentIndex: root.controlMode
                            onActivated: root.controlMode = currentIndex
                            Layout.preferredWidth: 220
                        }
                        Text {
                            text: "0: passthrough  │  1: PID  │  2: PID anti-windup  │  3: CS"
                            color: "#8aa1b3"
                            font.pixelSize: 11
                            Layout.fillWidth: true
                        }
                    }
                }

                // ── PID Gains ───────────────────────────────────────────────
                SectionFrame {
                    title: "PID GAINS — depth · roll · pitch · yaw"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 200

                    GridLayout {
                        anchors.fill: parent
                        columns: 4
                        columnSpacing: 8
                        rowSpacing: 4

                        Text { text: "AXIS"; color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }
                        Text { text: "Kp";   color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }
                        Text { text: "Ki";   color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }
                        Text { text: "Kd";   color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }

                        Repeater {
                            model: ["Depth", "Roll", "Pitch", "Yaw"]
                            delegate: RowLayout {
                                Layout.column: 0
                                Layout.row: index + 1
                                Layout.columnSpan: 4
                                Layout.fillWidth: true
                                spacing: 8

                                Text {
                                    text: modelData
                                    color: "#e0eaf8"
                                    Layout.preferredWidth: 80
                                    font.pixelSize: 12
                                }
                                NumField {
                                    Layout.fillWidth: true
                                    value: root.kp[index]
                                    onCommitted: (newValue) => {
                                        var a = root.kp.slice(); a[index] = newValue; root.kp = a
                                    }
                                }
                                NumField {
                                    Layout.fillWidth: true
                                    value: root.ki[index]
                                    onCommitted: (newValue) => {
                                        var a = root.ki.slice(); a[index] = newValue; root.ki = a
                                    }
                                }
                                NumField {
                                    Layout.fillWidth: true
                                    value: root.kd[index]
                                    onCommitted: (newValue) => {
                                        var a = root.kd.slice(); a[index] = newValue; root.kd = a
                                    }
                                }
                            }
                        }
                    }
                }

                // ── Manual Setpoints ────────────────────────────────────────
                SectionFrame {
                    title: "MANUAL SETPOINTS"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 200

                    GridLayout {
                        anchors.fill: parent
                        columns: 3
                        columnSpacing: 10
                        rowSpacing: 6

                        Text { text: "AXIS";     color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }
                        Text { text: "MANUAL";   color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }
                        Text { text: "SETPOINT"; color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }

                        Text { text: "Depth [m]"; color: "#e0eaf8"; font.pixelSize: 12 }
                        CheckBox { checked: root.manualDepth; onToggled: root.manualDepth = checked }
                        NumField { value: root.setpointDepth; onCommitted: (newValue) => root.setpointDepth = newValue }

                        Text { text: "Roll [°]"; color: "#e0eaf8"; font.pixelSize: 12 }
                        CheckBox { checked: root.manualRoll; onToggled: root.manualRoll = checked }
                        NumField { value: root.setpointRoll; onCommitted: (newValue) => root.setpointRoll = newValue }

                        Text { text: "Pitch [°]"; color: "#e0eaf8"; font.pixelSize: 12 }
                        CheckBox { checked: root.manualPitch; onToggled: root.manualPitch = checked }
                        NumField { value: root.setpointPitch; onCommitted: (newValue) => root.setpointPitch = newValue }

                        Text { text: "Yaw [°]"; color: "#e0eaf8"; font.pixelSize: 12 }
                        CheckBox { checked: root.manualYaw; onToggled: root.manualYaw = checked }
                        NumField { value: root.setpointYaw; onCommitted: (newValue) => root.setpointYaw = newValue }
                    }
                }

                // ── CS Controller ───────────────────────────────────────────
                SectionFrame {
                    title: "CS CONTROLLER"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 360

                    GridLayout {
                        anchors.fill: parent
                        columns: 3
                        columnSpacing: 10
                        rowSpacing: 6

                        Text { text: "PARAM";   color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }
                        Text { text: "VALUE 1"; color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }
                        Text { text: "VALUE 2"; color: "#a0c4e0"; font.bold: true; font.pixelSize: 11 }

                        Text { text: "cs_kx0 (heave)"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csKx0[0]; onCommitted: (newValue) => { var a=root.csKx0.slice(); a[0]=newValue; root.csKx0=a } }
                        NumField { Layout.fillWidth: true; value: root.csKx0[1]; onCommitted: (newValue) => { var a=root.csKx0.slice(); a[1]=newValue; root.csKx0=a } }

                        Text { text: "cs_kx1 (roll)"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csKx1[0]; onCommitted: (newValue) => { var a=root.csKx1.slice(); a[0]=newValue; root.csKx1=a } }
                        NumField { Layout.fillWidth: true; value: root.csKx1[1]; onCommitted: (newValue) => { var a=root.csKx1.slice(); a[1]=newValue; root.csKx1=a } }

                        Text { text: "cs_kx2 (pitch)"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csKx2[0]; onCommitted: (newValue) => { var a=root.csKx2.slice(); a[0]=newValue; root.csKx2=a } }
                        NumField { Layout.fillWidth: true; value: root.csKx2[1]; onCommitted: (newValue) => { var a=root.csKx2.slice(); a[1]=newValue; root.csKx2=a } }

                        Text { text: "cs_ki0"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csKi0; onCommitted: (newValue) => root.csKi0 = newValue }
                        Item { Layout.fillWidth: true }

                        Text { text: "cs_ki1"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csKi1; onCommitted: (newValue) => root.csKi1 = newValue }
                        Item { Layout.fillWidth: true }

                        Text { text: "cs_ki2"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csKi2; onCommitted: (newValue) => root.csKi2 = newValue }
                        Item { Layout.fillWidth: true }

                        Text { text: "cs_heave [min, max]"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csHeaveMin; onCommitted: (newValue) => root.csHeaveMin = newValue }
                        NumField { Layout.fillWidth: true; value: root.csHeaveMax; onCommitted: (newValue) => root.csHeaveMax = newValue }

                        Text { text: "cs_angle [min, max]"; color: "#e0eaf8"; font.pixelSize: 12 }
                        NumField { Layout.fillWidth: true; value: root.csAngleMin; onCommitted: (newValue) => root.csAngleMin = newValue }
                        NumField { Layout.fillWidth: true; value: root.csAngleMax; onCommitted: (newValue) => root.csAngleMax = newValue }
                    }
                }

                // ── Live telemetry ──────────────────────────────────────────
                SectionFrame {
                    title: "LIVE TELEMETRY  (/controller/setpoints · /errors · /pid_terms)"
                    Layout.fillWidth: true
                    Layout.preferredHeight: 130

                    ColumnLayout {
                        anchors.fill: parent
                        spacing: 4

                        Text {
                            text: "setpoints: " + JSON.stringify(root.liveSetpoints.map(function(v){ return Number(v).toFixed(3) }))
                            color: "#cfefff"
                            font.family: "Courier New"
                            font.pixelSize: 11
                            elide: Text.ElideRight
                            Layout.fillWidth: true
                        }
                        Text {
                            text: "errors:    " + JSON.stringify(root.liveErrors.map(function(v){ return Number(v).toFixed(3) }))
                            color: "#cfefff"
                            font.family: "Courier New"
                            font.pixelSize: 11
                            elide: Text.ElideRight
                            Layout.fillWidth: true
                        }
                        Text {
                            text: "pid_terms: " + JSON.stringify(root.livePidTerms.map(function(v){ return Number(v).toFixed(3) }))
                            color: "#cfefff"
                            font.family: "Courier New"
                            font.pixelSize: 11
                            elide: Text.ElideRight
                            Layout.fillWidth: true
                        }
                    }
                }
            }
        }
    }
}
