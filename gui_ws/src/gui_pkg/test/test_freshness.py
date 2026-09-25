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

def test_build_status_empty_raw_gives_no_data_depth_row():
    model = freshness.build_status({}, 0.0)
    depth_row = model['groups'][0]['rows'][0]
    assert depth_row['label'] == 'Depth'
    assert depth_row['text'] == freshness.NO_VALUE
    assert depth_row['unit'] == 'm'
    assert depth_row['badge'] == freshness.BADGE_NO_DATA
    assert depth_row['age'] == freshness.NO_VALUE


def test_build_status_fresh_depth_reads_ok():
    now = 10.0
    raw = {'depth': (1.234, now - 0.1)}
    model = freshness.build_status(raw, now)
    depth_row = model['groups'][0]['rows'][0]
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
