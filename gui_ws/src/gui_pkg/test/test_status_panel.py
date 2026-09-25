"""rclpy round trip and offscreen QML load for the Control Panel.

Exercises the real StatusBridge/ROSQmlBridge wiring against a live
rclpy graph, and loads the actual ControlPanelWindow.qml offscreen to
catch binding errors QML would otherwise hide silently.

Author: Davide Colabella
"""
import os
import subprocess
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from PyQt6.QtCore import QCoreApplication, QUrl, qInstallMessageHandler
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtQml import QQmlComponent, QQmlEngine

from gui_pkg import freshness
from gui_pkg import PoliciesUtils
from gui_pkg.gui_node import (
    ControllerBridge, ROSQmlBridge, SonarBridge, SonarRenderer, StatusBridge)

# One QGuiApplication for the whole module -- QML/Quick objects require
# one to exist, and PyQt6 refuses to construct a second.
_app = QGuiApplication.instance() or QGuiApplication([])


def _group(model, name):
    for group in model['groups']:
        if group['name'] == name:
            return group
    raise AssertionError(f'no group named {name!r} in {model!r}')


@pytest.fixture
def rclpy_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_depth_roundtrip(rclpy_context):
    status_node = Node('status_test_gui')
    pub_node = Node('status_test_publisher')
    bridge = ROSQmlBridge(status_node)
    sonar_bridge = SonarBridge(status_node, SonarRenderer())
    controller_bridge = ControllerBridge(status_node)
    status_bridge = StatusBridge(
        status_node, bridge, sonar_bridge, controller_bridge)

    depth_pub = pub_node.create_publisher(
        Float32, 'barometer_depth', PoliciesUtils.sensor_qos)

    try:
        deadline = time.monotonic() + 5.0
        depth_row = None
        while time.monotonic() < deadline:
            msg = Float32()
            msg.data = 1.23
            depth_pub.publish(msg)

            rclpy.spin_once(status_node, timeout_sec=0.05)
            rclpy.spin_once(pub_node, timeout_sec=0.0)
            status_bridge.refresh()

            model = status_bridge.vehicleStatus
            pi_sensors = _group(model, 'PI SENSORS')
            depth_row = next(
                r for r in pi_sensors['rows'] if r['label'] == 'Depth')
            if (depth_row['badge'] == freshness.BADGE_OK
                    and depth_row['text'] == '1.23'):
                break

        assert depth_row is not None
        assert depth_row['badge'] == freshness.BADGE_OK
        assert depth_row['text'] == '1.23'
        assert depth_row['unit'] == 'm'
        # This test never calls bridge.sendArmCommand -- the command
        # path is out of scope for a read-only status test.
    finally:
        status_node.destroy_node()
        pub_node.destroy_node()


def test_pi_health_live(rclpy_context):
    """Run the real pi_health_pub binary into the real StatusBridge.

    Skipped when PI_HEALTH_PUB is unset -- a skip here means the node's
    real output never reached the GUI model, which is exactly what
    T-06-12 (contract drift between piHealthPub.cpp's KeyValue keys and
    freshness.py's PI rows) exists to catch.
    """
    pi_health_pub = os.environ.get('PI_HEALTH_PUB')
    if not pi_health_pub:
        pytest.skip('PI_HEALTH_PUB not set: no built pi_health_pub binary')

    status_node = Node('status_test_pi_health')
    bridge = ROSQmlBridge(status_node)
    sonar_bridge = SonarBridge(status_node, SonarRenderer())
    controller_bridge = ControllerBridge(status_node)
    status_bridge = StatusBridge(
        status_node, bridge, sonar_bridge, controller_bridge)

    proc = subprocess.Popen([pi_health_pub])
    try:
        deadline = time.monotonic() + 10.0
        cpu_row = None
        while time.monotonic() < deadline:
            rclpy.spin_once(status_node, timeout_sec=0.1)
            status_bridge.refresh()

            model = status_bridge.vehicleStatus
            pi_group = _group(model, 'PI')
            cpu_row = next(
                r for r in pi_group['rows'] if r['label'] == 'CPU load')
            if cpu_row['badge'] == freshness.BADGE_OK:
                try:
                    float(cpu_row['text'])
                    break
                except ValueError:
                    pass

        assert cpu_row is not None
        assert cpu_row['badge'] == freshness.BADGE_OK
        float(cpu_row['text'])  # a real number, not the raw text
        assert cpu_row['unit'] == '%'
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        status_node.destroy_node()


def test_refresh_emits_once(rclpy_context):
    node = Node('status_test_refresh_once')
    bridge = ROSQmlBridge(node)
    sonar_bridge = SonarBridge(node, SonarRenderer())
    controller_bridge = ControllerBridge(node)
    status_bridge = StatusBridge(node, bridge, sonar_bridge, controller_bridge)

    status_bridge.record('depth', 1.0)

    calls = []
    status_bridge.status_changed.connect(lambda: calls.append(1))

    now = time.monotonic()
    status_bridge.refresh(now)
    status_bridge.refresh(now)

    try:
        assert len(calls) == 1
    finally:
        node.destroy_node()


def test_record_coalesces(rclpy_context):
    node = Node('status_test_coalesce')
    bridge = ROSQmlBridge(node)
    sonar_bridge = SonarBridge(node, SonarRenderer())
    controller_bridge = ControllerBridge(node)
    status_bridge = StatusBridge(node, bridge, sonar_bridge, controller_bridge)

    now = time.monotonic()
    status_bridge.record('depth', 1.0)
    status_bridge.record('depth', 2.0)
    status_bridge.refresh(now)

    try:
        pi_sensors = _group(status_bridge.vehicleStatus, 'PI SENSORS')
        depth_row = next(
            r for r in pi_sensors['rows'] if r['label'] == 'Depth')
        assert depth_row['text'] == '2.00'
    finally:
        node.destroy_node()


def test_control_panel_loads():
    messages = []
    qInstallMessageHandler(
        lambda mode, ctx, msg: messages.append(msg))

    engine = QQmlEngine()
    qml_path = (Path(__file__).parent
                / '..' / 'gui_pkg' / 'qml' / 'components'
                / 'ControlPanelWindow.qml').resolve()
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(qml_path)))

    obj = component.create()
    QCoreApplication.processEvents()

    assert component.errors() == [], [e.toString() for e in
                                       component.errors()]
    assert obj is not None

    obj.setProperty('vehicleStatus', freshness.build_status({}, 0.0))
    QCoreApplication.processEvents()

    now = time.monotonic()
    populated = freshness.build_status({'depth': (1.23, now - 0.1)}, now)
    obj.setProperty('vehicleStatus', populated)
    QCoreApplication.processEvents()

    banned = ('ReferenceError', 'TypeError', 'Unable to assign',
              'is not defined', 'Cannot read property')
    for message in messages:
        for token in banned:
            assert token not in message, message


def test_header_bar_loads():
    messages = []
    qInstallMessageHandler(
        lambda mode, ctx, msg: messages.append(msg))

    engine = QQmlEngine()
    qml_path = (Path(__file__).parent
                / '..' / 'gui_pkg' / 'qml' / 'components'
                / 'HeaderBar.qml').resolve()
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(qml_path)))

    obj = component.create()
    QCoreApplication.processEvents()

    assert component.errors() == [], [e.toString() for e in
                                       component.errors()]
    assert obj is not None

    for level in ('ok', 'stale', 'down'):
        obj.setProperty('statusSummary', level)
        QCoreApplication.processEvents()

    banned = ('ReferenceError', 'TypeError', 'Unable to assign',
              'is not defined', 'Cannot read property')
    for message in messages:
        for token in banned:
            assert token not in message, message
