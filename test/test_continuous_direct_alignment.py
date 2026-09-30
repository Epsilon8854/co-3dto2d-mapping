from types import SimpleNamespace

from co_3dto2d_mapping.inter_robot_place_alignment import (
    InterRobotPlaceAlignment,
)


def test_direct_mode_creates_keyframe_for_every_new_map_stamp():
    previous = SimpleNamespace(
        stamp_ns=100,
        created_ns=1_000,
        pose=(0.0, 0.0, 0.0),
    )
    node = SimpleNamespace(
        direct_latest_pair=True,
        keyframes=([previous], []),
        keyframe_min_interval_sec=100.0,
        keyframe_translation_m=1.0,
        keyframe_rotation_rad=1.0,
        alignment_message=object(),
        stationary_keyframe_period_sec=100.0,
    )

    assert InterRobotPlaceAlignment._should_create_keyframe(
        node,
        robot_id=0,
        stamp_ns=101,
        pose=(0.0, 0.0, 0.0),
        now_ns=1_001,
    )


def test_direct_mode_registers_only_latest_pair_once_per_processing_tick():
    matches = []
    created = [object(), object()]
    now = SimpleNamespace(nanoseconds=123, to_msg=lambda: object())
    node = SimpleNamespace(
        alignment_message=None,
        direct_latest_pair=True,
        stop_processing_after_lock=False,
        lock_after_consensus=False,
        get_clock=lambda: SimpleNamespace(now=lambda: now),
        _inputs_ready=lambda _now_ns: True,
        _create_keyframe=lambda robot_id, _now_ns: created[robot_id],
        _match_new_keyframe=lambda keyframe, _now_ns: matches.append(keyframe),
        _update_consensus=lambda _now_ns: None,
        _publish_status=lambda _now_ns: None,
    )

    InterRobotPlaceAlignment._processing_timer(node)

    assert matches == [created[1]]
