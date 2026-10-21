from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
MONITOR = REPOSITORY_ROOT / "scripts" / "monitor_experiment_timing.py"


def _load_monitor_module():
    spec = importlib.util.spec_from_file_location("timing_monitor_under_test", MONITOR)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_stream_stats_counts_header_timestamp_gap_over_threshold():
    monitor = _load_monitor_module()
    stats = monitor.StreamStats(gap_threshold_ns=20_000_000)

    stats.observe(header_stamp_ns=1_000_000_000, received_ns=10_000_000_000)
    stats.observe(header_stamp_ns=1_005_000_000, received_ns=10_005_000_000)
    event = stats.observe(header_stamp_ns=1_510_000_000, received_ns=10_510_000_000)

    assert event.header_gap is True
    assert stats.message_count == 3
    assert stats.header_gap_count == 1
    assert stats.max_header_delta_ns == 505_000_000
    assert stats.max_receive_delta_ns == 505_000_000


def test_stream_gap_thresholds_keep_imu_and_ten_hz_streams_separate():
    monitor = _load_monitor_module()

    assert monitor.stream_gap_thresholds_ns(
        imu_gap_threshold_ms=20.0,
        lidar_gap_threshold_ms=250.0,
        odom_gap_threshold_ms=500.0,
    ) == {
        "imu_raw": 20_000_000,
        "imu_filtered": 20_000_000,
        "lidar": 250_000_000,
        "odom": 500_000_000,
    }


def test_spin_until_stopped_exits_after_a_signal_flag_is_set():
    monitor = _load_monitor_module()
    calls = []
    stopped = [False]

    def spin_once(timeout_sec):
        calls.append(timeout_sec)
        stopped[0] = True

    monitor.spin_until_stopped(
        spin_once=spin_once,
        ros_is_ok=lambda: True,
        stop_requested=lambda: stopped[0],
    )

    assert calls == [0.25]


def test_capture_recorder_starts_timing_monitor_and_records_its_outputs():
    recorder = (REPOSITORY_ROOT / "scripts" / "record_experiment_bag.sh").read_text(
        encoding="utf-8"
    )

    assert "monitor_experiment_timing.py" in recorder
    assert "timing_monitor.csv" in recorder
    assert "timing_monitor_summary.yaml" in recorder
    assert 'filtered_imu_topic="${filtered_imu_topic:-/r${robot_id}/mapping/imu_filtered}"' in recorder
    assert 'odom_topic="${odom_topic:-/r${robot_id}/odom}"' in recorder
