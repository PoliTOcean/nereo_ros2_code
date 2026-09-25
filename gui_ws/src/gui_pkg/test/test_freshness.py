"""Pure-function tests for gui_pkg.freshness.

No rclpy, no Qt — every assertion here is against hand-computed reference
values so this file runs under plain pytest.

Author: Davide Colabella
"""
from gui_pkg import freshness


# ── classify_freshness boundaries ──────────────────────────────────────

def test_depth_age_exactly_stale_threshold_is_ok():
    stale_s, nodata_s = freshness.THRESHOLDS_S['depth']
    badge = freshness.classify_freshness(0.0, stale_s, stale_s, nodata_s)
    assert badge == freshness.BADGE_OK


def test_depth_age_just_past_stale_threshold_is_stale():
    stale_s, nodata_s = freshness.THRESHOLDS_S['depth']
    now = stale_s + 1e-6
    badge = freshness.classify_freshness(0.0, now, stale_s, nodata_s)
    assert badge == freshness.BADGE_STALE


def test_depth_age_exactly_nodata_threshold_is_stale():
    stale_s, nodata_s = freshness.THRESHOLDS_S['depth']
    badge = freshness.classify_freshness(0.0, nodata_s, stale_s, nodata_s)
    assert badge == freshness.BADGE_STALE


def test_depth_age_just_past_nodata_threshold_is_no_data():
    stale_s, nodata_s = freshness.THRESHOLDS_S['depth']
    now = nodata_s + 1e-6
    badge = freshness.classify_freshness(0.0, now, stale_s, nodata_s)
    assert badge == freshness.BADGE_NO_DATA


def test_never_received_is_no_data():
    stale_s, nodata_s = freshness.THRESHOLDS_S['depth']
    badge = freshness.classify_freshness(None, 0.0, stale_s, nodata_s)
    assert badge == freshness.BADGE_NO_DATA


# ── format_age ──────────────────────────────────────────────────────────

def test_format_age_none_is_dash():
    assert freshness.format_age(None, 0.0) == freshness.NO_VALUE


def test_format_age_below_ten_seconds_one_decimal():
    assert freshness.format_age(0.0, 0.04) == '0.0 s'
    assert freshness.format_age(0.0, 9.94) == '9.9 s'


def test_format_age_ten_and_above_whole_seconds():
    assert freshness.format_age(0.0, 12.7) == '12 s'


def test_unrounded_age_drives_badge_not_the_display_string():
    stale_s, nodata_s = freshness.THRESHOLDS_S['depth']
    # 0.94 is STALE against a 0.9 threshold, but format_age rounds the
    # same age to '0.9 s' -- the badge must use the unrounded value.
    badge = freshness.classify_freshness(0.0, 0.94, stale_s, nodata_s)
    display = freshness.format_age(0.0, 0.94)
    assert badge == freshness.BADGE_STALE
    assert display == '0.9 s'


# ── WR-02: "received" comes from t_received, not `value is None` ────────
# StatusBridge.record() always pairs a non-None value with a timestamp
# today, so these two signals normally agree. These tests decouple them
# directly (a value present but t_received None) to prove the row
# builders trust the timestamp, not the value, for "never received".

def test_decoupled_value_with_no_timestamp_reads_no_value_not_stale_text():
    now = 0.05
    raw = {'depth': (1.234, None)}
    model = freshness.build_status(raw, now)
    depth_row = _row(_group(model, 'PI SENSORS'), 'Depth')
    assert depth_row['text'] == freshness.NO_VALUE
    assert depth_row['badge'] == freshness.BADGE_NO_DATA


def test_decoupled_thrusters_with_no_timestamp_read_no_value():
    now = 0.05
    raw = {'armed': (True, 0.0), 'thrusters': ([1500] * 8, None)}
    model = freshness.build_status(raw, now)
    firmware = _group(model, 'FIRMWARE')
    for row in firmware['rows']:
        assert row['text'] == freshness.NO_VALUE
        assert row['badge'] == freshness.BADGE_NO_DATA


def test_decoupled_pi_health_with_no_timestamp_reads_no_value():
    now = 0.05
    raw = {'pi_health': ({'cpu_load_percent': '42.0'}, None)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'CPU load')
    assert row['text'] == freshness.NO_VALUE
    assert row['badge'] == freshness.BADGE_NO_DATA


# ── build_status ──────────────────────────────────────────────────────

def _group(model, name):
    for group in model['groups']:
        if group['name'] == name:
            return group
    raise AssertionError(f'no group named {name!r} in {model!r}')


def _row(group, label):
    for row in group['rows']:
        if row['label'] == label:
            return row
    raise AssertionError(f'no row labelled {label!r} in {group!r}')


def test_build_status_empty_raw_gives_no_data_depth_row():
    model = freshness.build_status({}, 0.0)
    depth_row = _row(_group(model, 'PI SENSORS'), 'Depth')
    assert depth_row['text'] == freshness.NO_VALUE
    assert depth_row['unit'] == 'm'
    assert depth_row['badge'] == freshness.BADGE_NO_DATA
    assert depth_row['age'] == freshness.NO_VALUE


def test_build_status_fresh_depth_reads_ok():
    now = 10.0
    raw = {'depth': (1.234, now - 0.1)}
    model = freshness.build_status(raw, now)
    depth_row = _row(_group(model, 'PI SENSORS'), 'Depth')
    assert depth_row['text'] == '1.23'
    assert depth_row['badge'] == freshness.BADGE_OK


def test_every_model_value_is_str_or_bool():
    now = 10.0
    raw = {'depth': (1.234, now - 0.1)}
    model = freshness.build_status(raw, now)

    def walk(node):
        if isinstance(node, dict):
            for value in node.values():
                walk(value)
        elif isinstance(node, list):
            for item in node:
                walk(item)
        else:
            assert isinstance(node, (str, bool)), (
                f'unexpected type {type(node)!r} in model: {node!r}')

    walk(model)


# ── build_status: FIRMWARE / armed / LINK DOWN (Task 2) ────────────────

def test_armed_never_received_is_link_down():
    model = freshness.build_status({}, 0.0)
    assert model['armed']['text'] == freshness.NO_VALUE
    assert model['armed']['badge'] == freshness.BADGE_LINK_DOWN
    firmware = _group(model, 'FIRMWARE')
    assert firmware['badge'] == freshness.BADGE_LINK_DOWN
    assert firmware['rows'] == []


def test_armed_age_exactly_nodata_threshold_is_stale_not_link_down():
    stale_s, nodata_s = freshness.THRESHOLDS_S['armed']
    raw = {'armed': (True, 0.0)}
    model = freshness.build_status(raw, nodata_s)
    assert model['armed']['badge'] == freshness.BADGE_STALE
    firmware = _group(model, 'FIRMWARE')
    assert firmware['badge'] == ''
    assert len(firmware['rows']) == 8


def test_armed_age_just_past_nodata_threshold_is_link_down():
    stale_s, nodata_s = freshness.THRESHOLDS_S['armed']
    now = nodata_s + 1e-6
    raw = {
        # armed silent since t=0 -- link down at `now`
        'armed': (True, 0.0),
        # thrusters received an instant ago -- fresh on their own
        'thrusters': ([1500] * 8, now - 0.01),
    }
    model = freshness.build_status(raw, now)
    assert model['armed']['badge'] == freshness.BADGE_LINK_DOWN
    firmware = _group(model, 'FIRMWARE')
    assert firmware['badge'] == freshness.BADGE_LINK_DOWN
    # thruster rows suppressed even though /thruster_status is fresh
    assert firmware['rows'] == []


def test_armed_true_reads_armed_text():
    now = 0.05
    raw = {'armed': (True, 0.0)}
    model = freshness.build_status(raw, now)
    assert model['armed']['text'] == 'ARMED'


def test_armed_false_reads_disarmed_text():
    now = 0.05
    raw = {'armed': (False, 0.0)}
    model = freshness.build_status(raw, now)
    assert model['armed']['text'] == 'DISARMED'


# ── build_status: thruster PWM, pinned cue (Task 2) ─────────────────────

def test_pwm_at_limits_is_pinned():
    assert freshness.is_pinned(freshness.PWM_MIN_US) is True
    assert freshness.is_pinned(freshness.PWM_MAX_US) is True


def test_pwm_one_us_inside_limits_is_not_pinned():
    assert freshness.is_pinned(freshness.PWM_MIN_US + 1) is False
    assert freshness.is_pinned(freshness.PWM_MAX_US - 1) is False


def test_pwm_neutral_is_not_pinned():
    assert freshness.is_pinned(1500) is False


def test_thruster_rows_never_received_read_dash_with_unit():
    now = 0.05
    raw = {'armed': (True, 0.0)}
    model = freshness.build_status(raw, now)
    firmware = _group(model, 'FIRMWARE')
    assert len(firmware['rows']) == 8
    for row in firmware['rows']:
        assert row['text'] == freshness.NO_VALUE
        assert row['unit'] == 'µs'
        assert row['badge'] == freshness.BADGE_NO_DATA


def test_thruster_row_pinned_channel_carries_alert():
    now = 0.05
    pwms = [1500] * 8
    pwms[3] = freshness.PWM_MIN_US
    raw = {'armed': (True, 0.0), 'thrusters': (pwms, 0.0)}
    model = freshness.build_status(raw, now)
    firmware = _group(model, 'FIRMWARE')
    assert firmware['rows'][3]['alert'] is True
    assert firmware['rows'][3]['text'] == str(freshness.PWM_MIN_US)
    assert firmware['rows'][0]['alert'] is False


# ── build_status: PI SENSORS ordering and units (Task 2) ────────────────

def test_pi_sensors_label_order_is_fixed():
    now = 0.05
    # Fill the raw dict in a deliberately different order than the
    # required label order.
    raw = {
        'sonar_confidence': (80, 0.0),
        'sonar_range': (1.5, 0.0),
        'water_temp': (21.0, 0.0),
        'pressure': (101325.0, 0.0),
        'depth': (1.0, 0.0),
        'yaw': (10.0, 0.0),
        'pitch': (5.0, 0.0),
        'roll': (1.0, 0.0),
    }
    model = freshness.build_status(raw, now)
    labels = [row['label'] for row in _group(model, 'PI SENSORS')['rows']]
    assert labels == [
        'Roll', 'Pitch', 'Yaw', 'Depth', 'Pressure',
        'Water temperature', 'Sonar range', 'Sonar confidence',
    ]


def test_sonar_rows_are_always_last_two_in_order():
    now = 0.05
    raw = {'sonar_confidence': (80, 0.0), 'sonar_range': (1.5, 0.0)}
    model = freshness.build_status(raw, now)
    rows = _group(model, 'PI SENSORS')['rows']
    assert rows[-2]['label'] == 'Sonar range'
    assert rows[-1]['label'] == 'Sonar confidence'


def test_pressure_converts_pa_to_hpa():
    now = 0.05
    raw = {'pressure': (101325.0, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI SENSORS'), 'Pressure')
    assert row['text'] == '1013.2'
    assert row['unit'] == 'hPa'


def test_water_temperature_unit_and_format():
    now = 0.05
    raw = {'water_temp': (21.456, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI SENSORS'), 'Water temperature')
    assert row['text'] == '21.5'
    assert row['unit'] == '°C'


def test_sonar_range_unit_and_format():
    now = 0.05
    raw = {'sonar_range': (1.5, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI SENSORS'), 'Sonar range')
    assert row['text'] == '1.50'
    assert row['unit'] == 'm'


def test_sonar_confidence_unit_and_format():
    now = 0.05
    raw = {'sonar_confidence': (80.0, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI SENSORS'), 'Sonar confidence')
    assert row['text'] == '80'
    assert row['unit'] == '%'


def test_sonar_range_age_exactly_stale_threshold_is_ok():
    stale_s, nodata_s = freshness.THRESHOLDS_S['sonar_range']
    raw = {'sonar_range': (1.5, 0.0)}
    model = freshness.build_status(raw, stale_s)
    row = _row(_group(model, 'PI SENSORS'), 'Sonar range')
    assert row['badge'] == freshness.BADGE_OK


def test_sonar_range_age_one_step_past_stale_is_stale():
    stale_s, nodata_s = freshness.THRESHOLDS_S['sonar_range']
    raw = {'sonar_range': (1.5, 0.0)}
    model = freshness.build_status(raw, stale_s + 1e-6)
    row = _row(_group(model, 'PI SENSORS'), 'Sonar range')
    assert row['badge'] == freshness.BADGE_STALE


def test_sonar_range_never_received_reads_dash_not_zero():
    model = freshness.build_status({}, 0.0)
    row = _row(_group(model, 'PI SENSORS'), 'Sonar range')
    assert row['text'] == freshness.NO_VALUE
    assert row['unit'] == 'm'
    assert row['badge'] == freshness.BADGE_NO_DATA


def test_every_numeric_row_text_has_a_unit_including_dash_rows():
    model = freshness.build_status({}, 0.0)
    for group in model['groups']:
        for row in group['rows']:
            try:
                float(row['text'])
                is_numeric_text = True
            except ValueError:
                is_numeric_text = (row['text'] == freshness.NO_VALUE
                                    and row['label'] != 'ROV')
            if is_numeric_text:
                assert row['unit'] != '', row


def test_every_badge_in_model_is_one_of_the_four_literals():
    now = 10.0
    raw = {
        'armed': (True, now - 0.1),
        'thrusters': ([1500] * 8, now - 0.1),
        'roll': (1.0, now - 10.0),
        'depth': (1.0, None),
    }
    model = freshness.build_status(raw, now)
    valid = {freshness.BADGE_OK, freshness.BADGE_STALE,
             freshness.BADGE_NO_DATA, freshness.BADGE_LINK_DOWN}
    assert model['armed']['badge'] in valid
    for group in model['groups']:
        if group['badge'] != '':
            assert group['badge'] in valid
        for row in group['rows']:
            assert row['badge'] in valid


# ── badge_transitions (Task 2) ───────────────────────────────────────────

# ── summary_level (header indicator, Task 1) ────────────────────────────

def test_summary_level_all_ok_is_ok():
    status = {
        'armed': {'badge': freshness.BADGE_OK},
        'groups': [{'badge': '', 'rows': [{'badge': freshness.BADGE_OK}]}],
    }
    assert freshness.summary_level(status) == 'ok'


def test_summary_level_one_stale_row_is_stale():
    status = {
        'armed': {'badge': freshness.BADGE_OK},
        'groups': [{'badge': '',
                     'rows': [{'badge': freshness.BADGE_STALE}]}],
    }
    assert freshness.summary_level(status) == 'stale'


def test_summary_level_no_data_row_is_down():
    status = {
        'armed': {'badge': freshness.BADGE_OK},
        'groups': [{'badge': '',
                     'rows': [{'badge': freshness.BADGE_NO_DATA}]}],
    }
    assert freshness.summary_level(status) == 'down'


def test_summary_level_link_down_group_is_down():
    status = {
        'armed': {'badge': freshness.BADGE_OK},
        'groups': [{'badge': freshness.BADGE_LINK_DOWN, 'rows': []}],
    }
    assert freshness.summary_level(status) == 'down'


def test_build_status_empty_raw_summary_is_down():
    model = freshness.build_status({}, 0.0)
    assert model['summary'] == 'down'


# ── node_alive / classify_event (Task 2) ─────────────────────────────────

def test_node_alive_within_window_is_true():
    now = freshness.NODE_ALIVE_S
    assert freshness.node_alive(
        'bar_publisher', {'bar_publisher': 0.0}, now) is True


def test_node_alive_never_seen_is_false():
    assert freshness.node_alive('bar_publisher', {}, 0.0) is False
    assert freshness.node_alive('bar_publisher', None, 0.0) is False


def test_node_alive_past_window_is_false():
    now = freshness.NODE_ALIVE_S + 1e-6
    assert freshness.node_alive(
        'bar_publisher', {'bar_publisher': 0.0}, now) is False


def test_classify_event_never_received_is_no_data():
    assert freshness.classify_event(None, True) == freshness.BADGE_NO_DATA


def test_classify_event_received_and_alive_is_ok():
    assert freshness.classify_event(0.0, True) == freshness.BADGE_OK


def test_classify_event_received_and_dead_is_stale():
    assert freshness.classify_event(0.0, False) == freshness.BADGE_STALE


# ── DEFAULT_EXPECTED_NODES (D-11 research correction, Task 2) ───────────

def test_default_expected_nodes_uses_real_joystick_node_name():
    assert 'joy_to_cmd_vel' in freshness.DEFAULT_EXPECTED_NODES
    assert 'joy_to_cmdvel' not in freshness.DEFAULT_EXPECTED_NODES
    assert 'pi_health_publisher' in freshness.DEFAULT_EXPECTED_NODES


# ── ROS NODES group (Task 2, PANEL-09) ───────────────────────────────────

def test_ros_nodes_rows_follow_expected_order_and_liveness():
    now = 10.0
    expected = ['bar_publisher', 'imu_publisher', 'sonar_node']
    node_last_seen = {'bar_publisher': now - 0.1, 'sonar_node': now - 100.0}
    model = freshness.build_status(
        {}, now, expected_nodes=expected, node_last_seen=node_last_seen)
    rows = _group(model, 'ROS NODES')['rows']
    assert [r['label'] for r in rows] == expected
    assert rows[0]['text'] == 'running'
    assert rows[0]['badge'] == freshness.BADGE_OK
    assert rows[1]['text'] == 'not running'
    assert rows[1]['badge'] == freshness.BADGE_NO_DATA
    assert rows[1]['age'] == freshness.NO_VALUE
    assert rows[2]['text'] == 'not running'


# ── control mode rows (Task 2, PANEL-02, D-12) ───────────────────────────

def test_control_mode_direct_when_inactive():
    now = 1.0
    raw = {'control_active': (False, 0.0)}
    node_last_seen = {freshness.JOY_NODE: now - 0.1}
    model = freshness.build_status(raw, now, node_last_seen=node_last_seen)
    assert len(model['control']) == 1
    assert model['control'][0]['label'] == 'Control mode'
    assert model['control'][0]['text'] == 'DIRECT'


def test_control_mode_never_received_reads_dash():
    model = freshness.build_status({}, 0.0)
    assert model['control'][0]['text'] == freshness.NO_VALUE
    assert model['control'][0]['badge'] == freshness.BADGE_NO_DATA


def test_control_mode_controller_adds_controller_row():
    now = 1.0
    raw = {'control_active': (True, 0.0), 'control_mode': (2, 0.0)}
    node_last_seen = {freshness.JOY_NODE: now - 0.1,
                       freshness.CONTROLLER_NODE: now - 0.1}
    model = freshness.build_status(raw, now, node_last_seen=node_last_seen)
    assert len(model['control']) == 2
    assert model['control'][0]['text'] == 'CONTROLLER'
    assert model['control'][1]['label'] == 'Controller'
    assert model['control'][1]['text'] == 'PID ANTI-WINDUP'
    assert model['control'][1]['badge'] == freshness.BADGE_OK


def test_control_mode_unknown_value_reads_unknown():
    now = 1.0
    raw = {'control_active': (True, 0.0), 'control_mode': (7, 0.0)}
    node_last_seen = {freshness.JOY_NODE: now - 0.1,
                       freshness.CONTROLLER_NODE: now - 0.1}
    model = freshness.build_status(raw, now, node_last_seen=node_last_seen)
    assert model['control'][1]['text'] == 'UNKNOWN (7)'


def test_control_row_goes_stale_when_joy_node_leaves_graph():
    now = 1.0
    raw = {'control_active': (False, 0.0)}
    # joy_to_cmd_vel absent from node_last_seen -- gone from the graph.
    # A last-known mode must never read OK once its publisher died.
    model = freshness.build_status(raw, now, node_last_seen={})
    assert model['control'][0]['badge'] == freshness.BADGE_STALE


def test_controller_row_goes_stale_when_controller_node_leaves_graph():
    now = 1.0
    raw = {'control_active': (True, 0.0), 'control_mode': (1, 0.0)}
    node_last_seen = {freshness.JOY_NODE: now - 0.1}
    model = freshness.build_status(raw, now, node_last_seen=node_last_seen)
    assert model['control'][1]['badge'] == freshness.BADGE_STALE


def test_summary_counts_control_and_node_rows():
    now = 1.0
    raw = {'control_active': (True, 0.0), 'control_mode': (1, 0.0)}
    model = freshness.build_status(
        raw, now, expected_nodes=['bar_publisher'], node_last_seen={})
    assert model['summary'] == 'down'


# ── PI group: SoC temperature and disk (Task 2, PANEL-10) ───────────────

def test_soc_temperature_row_formats_and_unit():
    now = 0.05
    raw = {'pi_health': ({'soc_temp_c': '48.3'}, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'SoC temperature')
    assert row['text'] == '48.3'
    assert row['unit'] == '°C'
    assert row['badge'] == freshness.BADGE_OK


def test_disk_used_row_formats_and_unit():
    now = 0.05
    raw = {'pi_health': ({'disk_used_percent': '63.2'}, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'Disk used (/)')
    assert row['text'] == '63.2'
    assert row['unit'] == '%'
    assert row['badge'] == freshness.BADGE_OK


def test_soc_temperature_unavailable_value_reads_unavailable():
    now = 0.05
    raw = {'pi_health': (
        {'soc_temp_c': freshness.PI_HEALTH_UNAVAILABLE_VALUE}, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'SoC temperature')
    assert row['text'] == 'unavailable'
    assert row['unit'] == ''
    assert row['badge'] == freshness.BADGE_NO_DATA


# ── PI group: Throttling (Task 2) ───────────────────────────────────────

def test_throttle_flags_wire_order_and_labels():
    assert freshness.THROTTLE_FLAGS == (
        ('under_voltage_now', 'under-voltage now'),
        ('freq_capped_now', 'frequency capped now'),
        ('throttled_now', 'throttled now'),
        ('soft_temp_limit_now', 'soft temperature limit now'),
        ('under_voltage_since_boot', 'under-voltage since boot'),
        ('freq_capped_since_boot', 'frequency capped since boot'),
        ('throttled_since_boot', 'throttled since boot'),
        ('soft_temp_limit_since_boot', 'soft temperature limit since boot'),
    )


def _all_flags_false():
    return {key: 'false' for key, _ in freshness.THROTTLE_FLAGS}


def test_throttling_all_false_reads_none():
    now = 0.05
    raw = {'pi_health': (_all_flags_false(), 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'Throttling')
    assert row['text'] == 'none'
    assert row['unit'] == ''
    assert row['alert'] is False


def test_throttling_two_active_flags_join_in_wire_order():
    now = 0.05
    flags = _all_flags_false()
    flags['under_voltage_now'] = 'true'
    flags['throttled_since_boot'] = 'true'
    raw = {'pi_health': (flags, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'Throttling')
    assert row['text'] == 'under-voltage now; throttled since boot'
    assert row['alert'] is True


def test_throttling_since_boot_only_carries_no_alert():
    now = 0.05
    flags = _all_flags_false()
    flags['throttled_since_boot'] = 'true'
    raw = {'pi_health': (flags, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'Throttling')
    assert row['text'] == 'throttled since boot'
    assert row['alert'] is False


def test_throttling_missing_flag_reads_unavailable():
    now = 0.05
    flags = _all_flags_false()
    del flags['soft_temp_limit_since_boot']
    raw = {'pi_health': (flags, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'Throttling')
    assert row['text'] == 'unavailable'
    assert row['unit'] == ''
    assert row['badge'] == freshness.BADGE_NO_DATA


def test_throttling_never_received_reads_unavailable_no_data():
    model = freshness.build_status({}, 0.0)
    row = _row(_group(model, 'PI'), 'Throttling')
    assert row['text'] == 'unavailable'
    assert row['badge'] == freshness.BADGE_NO_DATA


# ── PI group: micro-ROS agent (Task 2, D-10) ─────────────────────────────

def test_agent_running_true_reads_running():
    now = 0.05
    raw = {'pi_health': ({'agent_running': 'true'}, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'micro-ROS agent')
    assert row['text'] == 'running'
    assert row['alert'] is False


def test_agent_running_false_reads_not_running_with_alert():
    now = 0.05
    raw = {'pi_health': ({'agent_running': 'false'}, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'micro-ROS agent')
    assert row['text'] == 'not running'
    assert row['alert'] is True


def test_agent_running_missing_key_reads_unavailable():
    now = 0.05
    raw = {'pi_health': ({}, 0.0)}
    model = freshness.build_status(raw, now)
    row = _row(_group(model, 'PI'), 'micro-ROS agent')
    assert row['text'] == 'unavailable'
    assert row['badge'] == freshness.BADGE_NO_DATA


# ── link_down_cause (Task 2, D-10) ───────────────────────────────────────

def test_link_down_cause_agent_not_running():
    cause = freshness.link_down_cause(
        freshness.BADGE_OK, {'agent_running': 'false'})
    assert cause == freshness.CAUSE_AGENT_DOWN


def test_link_down_cause_agent_running_firmware_silent():
    cause = freshness.link_down_cause(
        freshness.BADGE_OK, {'agent_running': 'true'})
    assert cause == freshness.CAUSE_FIRMWARE_SILENT


def test_link_down_cause_pi_health_not_ok_is_unknown():
    cause = freshness.link_down_cause(
        freshness.BADGE_STALE, {'agent_running': 'true'})
    assert cause == freshness.CAUSE_UNKNOWN


def test_link_down_cause_missing_agent_key_is_unknown():
    cause = freshness.link_down_cause(freshness.BADGE_OK, {})
    assert cause == freshness.CAUSE_UNKNOWN


def test_link_down_cause_no_health_at_all_is_unknown():
    assert freshness.link_down_cause(
        freshness.BADGE_NO_DATA, None) == freshness.CAUSE_UNKNOWN


def test_firmware_link_down_note_carries_cause():
    now = 10.0
    raw = {
        # armed silent -- link down
        'armed': (True, 0.0),
        'pi_health': ({'agent_running': 'false'}, now - 0.1),
    }
    model = freshness.build_status(raw, now)
    firmware = _group(model, 'FIRMWARE')
    assert firmware['badge'] == freshness.BADGE_LINK_DOWN
    assert firmware['note'] == freshness.CAUSE_AGENT_DOWN


def test_firmware_link_down_note_unknown_when_pi_health_also_silent():
    now = 10.0
    raw = {'armed': (True, 0.0)}
    model = freshness.build_status(raw, now)
    firmware = _group(model, 'FIRMWARE')
    assert firmware['note'] == freshness.CAUSE_UNKNOWN


def test_badge_transitions_reports_depth_and_firmware_changes():
    old_model = freshness.build_status(
        {'armed': (True, 0.0), 'depth': (1.0, 0.0)}, 0.05)
    # 1.5 s: past armed's 1.0 s NO DATA bound (-> LINK DOWN) but only
    # past depth's 0.9 s STALE bound, still under its 3.0 s NO DATA
    # bound (-> STALE, not NO DATA).
    new_model = freshness.build_status(
        {'armed': (True, 0.0), 'depth': (1.0, 0.0)}, 1.5)
    transitions = freshness.badge_transitions(old_model, new_model)
    assert 'PI SENSORS/Depth: OK -> STALE' in transitions
    assert 'FIRMWARE:  -> LINK DOWN' in transitions
