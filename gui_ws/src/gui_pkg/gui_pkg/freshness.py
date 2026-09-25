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
}

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


def build_status(raw, now):
    """Build the full status model from raw field readings.

    Args:
        raw: dict mapping a field key to (value, t_received_monotonic).
            A field never received is simply absent from raw.
        now: current monotonic time, used for every freshness/age
            computation in the returned model.

    Returns:
        A dict with one key, 'groups', a list of group dicts (a list, not
        a dict, so QML's QVariantMap key-sorting cannot reorder groups).
        Each group is {'name', 'badge', 'note', 'rows'}.
    """
    value, t_received = raw.get('depth', (None, None))
    stale_s, nodata_s = THRESHOLDS_S['depth']
    badge = classify_freshness(t_received, now, stale_s, nodata_s)
    age = format_age(t_received, now)
    text = NO_VALUE if value is None else f'{value:.2f}'
    depth_row = make_row('Depth', text, 'm', badge, age)

    pi_sensors_group = {
        'name': 'PI SENSORS',
        'badge': '',
        'note': '',
        'rows': [depth_row],
    }

    return {'groups': [pi_sensors_group]}


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
