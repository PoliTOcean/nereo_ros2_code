"""Pure status-model helpers for the Control Panel's live vehicle status.

Standard library only: no rclpy, no PyQt6. Every function here is a plain
transform from raw (value, receipt-time) pairs to a display-ready status
model, so it can be exercised under plain pytest with no ROS or Qt runtime.

Author: Davide Colabella
"""

# Freshness thresholds in seconds, keyed by field name. Each value is a
# pair: (stale_after_s, no_data_after_s). An age past stale_after_s but at
# or below no_data_after_s reads STALE; past no_data_after_s reads NO DATA.
# The barometer publishes every 300 ms, so STALE follows three missed
# periods.
THRESHOLDS_S = {
    'depth': (0.9, 3.0),
    # ponytail: firmware publishes /rov_armed and /thruster_status every
    # 25 ms control cycle, so 0.5/1.0 s sits well above gui_node's
    # worst-case ~0.2 s callback service cycle. If healthy ages read
    # above 0.3 s on the bench, drain the executor per tick; do not
    # raise this threshold to compensate.
    'armed': (0.5, 1.0),
    'thrusters': (0.5, 1.0),
    'roll': (0.6, 2.0),
    'pitch': (0.6, 2.0),
    'yaw': (0.6, 2.0),
    'pressure': (0.9, 3.0),
    'water_temp': (0.9, 3.0),
    'sonar_range': (1.0, 3.0),
    'sonar_confidence': (1.0, 3.0),
}

# (raw key, label, unit, format function) for each PI SENSORS row, in the
# fixed display order (PANEL-11 ordering) -- independent of whatever
# order the raw dict happens to be filled in.
_PI_SENSORS_FIELDS = (
    ('roll', 'Roll', '°', lambda v: f'{v:.1f}'),
    ('pitch', 'Pitch', '°', lambda v: f'{v:.1f}'),
    ('yaw', 'Yaw', '°', lambda v: f'{v:.1f}'),
    ('depth', 'Depth', 'm', lambda v: f'{v:.2f}'),
    ('pressure', 'Pressure', 'hPa', lambda v: f'{v / 100:.1f}'),
    ('water_temp', 'Water temperature', '°C', lambda v: f'{v:.1f}'),
    ('sonar_range', 'Sonar range', 'm', lambda v: f'{v:.2f}'),
    ('sonar_confidence', 'Sonar confidence', '%', lambda v: f'{int(v)}'),
)

# Push cadence for the model reaching QML, in milliseconds (PANEL-14).
STATUS_PUSH_MS = 250

# Placeholder text for a value that has never been received.
NO_VALUE = '—'

BADGE_OK = 'OK'
BADGE_STALE = 'STALE'
BADGE_NO_DATA = 'NO DATA'
BADGE_LINK_DOWN = 'LINK DOWN'

# Post-clamp, pre-offset PWM limits, mirroring
# nereo_FC_firmware/Core/Inc/navigation/navigation.h lines 28-29.
PWM_MIN_US = 1330
PWM_MAX_US = 1670


def classify_freshness(t_received, now, stale_s, nodata_s):
    """Classify a value's freshness from its receipt time.

    Args:
        t_received: monotonic time the value was last received, or None
            if it has never been received.
        now: current monotonic time.
        stale_s: age in seconds beyond which the value reads STALE.
        nodata_s: age in seconds beyond which the value reads NO DATA.

    Returns:
        One of BADGE_OK, BADGE_STALE or BADGE_NO_DATA. An age exactly
        equal to a threshold keeps the milder badge.
    """
    if t_received is None:
        return BADGE_NO_DATA
    age = now - t_received
    if age > nodata_s:
        return BADGE_NO_DATA
    if age > stale_s:
        return BADGE_STALE
    return BADGE_OK


def format_age(t_received, now):
    """Format a value's age for display only.

    Args:
        t_received: monotonic time the value was last received, or None.
        now: current monotonic time.

    Returns:
        NO_VALUE when t_received is None, one decimal place below 10 s
        (e.g. '0.1 s'), whole seconds from 10 s up (e.g. '12 s'). Badges
        are always computed from the unrounded age, never this string.
    """
    if t_received is None:
        return NO_VALUE
    age = now - t_received
    if age < 10:
        return f'{age:.1f} s'
    return f'{int(age)} s'


def make_row(label, text, unit, badge, age, alert=False):
    """Build one status-model row.

    Args:
        label: row label, e.g. 'Depth'.
        text: pre-formatted value text, or NO_VALUE.
        unit: unit label, or '' when the row has none.
        badge: one of BADGE_OK, BADGE_STALE, BADGE_NO_DATA, BADGE_LINK_DOWN.
        age: pre-formatted age text, from format_age.
        alert: True when the row should be visually flagged (e.g. a
            thruster channel pinned at its PWM limit).

    Returns:
        A dict with exactly the keys label, text, unit, badge, age, alert.
    """
    return {
        'label': label,
        'text': text,
        'unit': unit,
        'badge': badge,
        'age': age,
        'alert': alert,
    }


def is_pinned(pwm_us):
    """Return True when a thruster PWM value sits at or beyond its limit.

    Args:
        pwm_us: PWM pulse width in microseconds.

    Returns:
        True when pwm_us is at or beyond PWM_MIN_US or PWM_MAX_US.
    """
    return pwm_us <= PWM_MIN_US or pwm_us >= PWM_MAX_US


def _classify_field(raw, now, key):
    """Classify one raw field's freshness, badge and display age.

    Args:
        raw: the build_status raw dict.
        now: current monotonic time.
        key: the field's key in raw and in THRESHOLDS_S.

    Returns:
        A (value, badge, age_text) tuple. value is None when the field
        was never received.
    """
    value, t_received = raw.get(key, (None, None))
    stale_s, nodata_s = THRESHOLDS_S[key]
    badge = classify_freshness(t_received, now, stale_s, nodata_s)
    age = format_age(t_received, now)
    return value, badge, age


def _pi_sensors_rows(raw, now):
    """Build the PI SENSORS group's rows, in fixed display order."""
    rows = []
    for key, label, unit, fmt in _PI_SENSORS_FIELDS:
        value, badge, age = _classify_field(raw, now, key)
        text = NO_VALUE if value is None else fmt(value)
        rows.append(make_row(label, text, unit, badge, age))
    return rows


def _armed_row_and_link_down(raw, now):
    """Build the armed ('ROV') row and whether the firmware link is down.

    Link down (PANEL-08, D-10) means /rov_armed has never arrived or has
    been silent past its NO DATA threshold.
    """
    value, badge, age = _classify_field(raw, now, 'armed')
    link_down = badge == BADGE_NO_DATA
    if link_down:
        return make_row('ROV', NO_VALUE, '', BADGE_LINK_DOWN, age), True
    text = 'ARMED' if value else 'DISARMED'
    return make_row('ROV', text, '', badge, age), False


def _thruster_rows(raw, now):
    """Build the eight Thruster N rows, sharing /thruster_status's age."""
    value, badge, age = _classify_field(raw, now, 'thrusters')
    pwms = value if value is not None else [None] * 8
    rows = []
    for i, pwm in enumerate(pwms):
        text = NO_VALUE if pwm is None else str(pwm)
        alert = pwm is not None and is_pinned(pwm)
        rows.append(
            make_row(f'Thruster {i + 1}', text, 'µs', badge, age,
                      alert=alert))
    return rows


def build_status(raw, now):
    """Build the full status model from raw field readings.

    Args:
        raw: dict mapping a field key to (value, t_received_monotonic).
            A field never received is simply absent from raw.
        now: current monotonic time, used for every freshness/age
            computation in the returned model.

    Returns:
        A dict with 'armed' (the ROV row) and 'groups', a list of group
        dicts (a list, not a dict, so QML's QVariantMap key-sorting
        cannot reorder groups). Each group is
        {'name', 'badge', 'note', 'rows'}.
    """
    armed_row, link_down = _armed_row_and_link_down(raw, now)

    if link_down:
        firmware_group = {
            'name': 'FIRMWARE',
            'badge': BADGE_LINK_DOWN,
            'note': '',
            'rows': [],
        }
    else:
        firmware_group = {
            'name': 'FIRMWARE',
            'badge': '',
            'note': '',
            'rows': _thruster_rows(raw, now),
        }

    pi_sensors_group = {
        'name': 'PI SENSORS',
        'badge': '',
        'note': '',
        'rows': _pi_sensors_rows(raw, now),
    }

    status = {
        'armed': armed_row,
        'groups': [firmware_group, pi_sensors_group],
    }
    status['summary'] = summary_level(status)
    return status


def summary_level(status):
    """Compute the header indicator's overall level from a status model.

    Collects the armed row's badge, every control row's badge (D-12,
    absent before Task 2 adds status['control']), and every group's own
    badge plus its rows' badges. QML never derives this itself (PANEL-13)
    -- it only renders the level this function computed.

    Args:
        status: a build_status return value (or a plain dict carrying
            the same 'armed'/'control'/'groups' shape, for tests).

    Returns:
        'down' if any collected badge is NO DATA or LINK DOWN, 'stale'
        if any is STALE, 'ok' otherwise.
    """
    badges = [status['armed']['badge']]
    badges += [row['badge'] for row in status.get('control', [])]
    for group in status.get('groups', []):
        if group['badge']:
            badges.append(group['badge'])
        badges += [row['badge'] for row in group.get('rows', [])]

    if any(b in (BADGE_NO_DATA, BADGE_LINK_DOWN) for b in badges):
        return 'down'
    if any(b == BADGE_STALE for b in badges):
        return 'stale'
    return 'ok'


def badge_transitions(old, new):
    """Diff two status models to a list of human-readable transitions.

    Args:
        old: a previous return value of build_status (or {} / None).
        new: the current return value of build_status.

    Returns:
        A list of strings, one per badge that changed between old and
        new: 'ROV: OLD -> NEW' for the armed row, 'GROUP: OLD -> NEW' for
        a group badge change, and 'GROUP/label: OLD -> NEW' for a row
        present in both models whose badge changed.
    """
    old = old or {}
    transitions = []

    old_armed = (old.get('armed') or {}).get('badge')
    new_armed = (new.get('armed') or {}).get('badge')
    if new_armed is not None and old_armed != new_armed:
        transitions.append(f'ROV: {old_armed} -> {new_armed}')

    old_groups = {g['name']: g for g in old.get('groups', [])}
    new_groups = {g['name']: g for g in new.get('groups', [])}

    for name, new_group in new_groups.items():
        old_group = old_groups.get(name)
        old_group_badge = old_group['badge'] if old_group else None
        if old_group_badge != new_group['badge']:
            transitions.append(
                f'{name}: {old_group_badge} -> {new_group["badge"]}')

        old_rows = {r['label']: r for r in (old_group or {}).get(
            'rows', [])} if old_group else {}
        for row in new_group.get('rows', []):
            old_row = old_rows.get(row['label'])
            if old_row is not None and old_row['badge'] != row['badge']:
                transitions.append(
                    f'{name}/{row["label"]}: '
                    f'{old_row["badge"]} -> {row["badge"]}')

    return transitions
