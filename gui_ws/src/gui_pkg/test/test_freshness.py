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
