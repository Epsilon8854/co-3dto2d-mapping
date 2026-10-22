import importlib.util
from pathlib import Path

import pytest

spec=importlib.util.spec_from_file_location('exp04_monitor',Path(__file__).resolve().parents[1]/'scripts/monitor_experiment_04_pipeline.py')
monitor=importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def test_monitor_reports_idle_and_receipt_rate_without_ros():
    stats=monitor.StreamStats()
    assert stats.summary(10.)['receipt_age_sec'] is None
    stats.add(10.,size=12,stamp=1_000_000_000,ros_now=1_200_000_000)
    stats.add(10.5,size=20,stamp=1_500_000_000,ros_now=1_700_000_000)
    result=stats.summary(11.)
    assert result['receipt_hz']==pytest.approx(2.)
    assert result['serialized_bytes']==32
    assert result['max_gap_sec']==pytest.approx(.5)
    assert result['header_age_median_sec']==pytest.approx(.2)
    assert result['first_header_stamp_ns']==1_000_000_000
    assert result['receipt_age_sec']==pytest.approx(.5)


def test_monitor_distinguishes_duplicate_and_backward_source_stamps():
    stats=monitor.StreamStats()
    for now,stamp in ((1.,20),(2.,20),(3.,10)):
        stats.add(now,stamp=stamp)
    result=stats.summary(4.)
    assert result['duplicate_stamps']==1
    assert result['backward_stamps']==1
    assert result['header_age_median_sec'] is None
