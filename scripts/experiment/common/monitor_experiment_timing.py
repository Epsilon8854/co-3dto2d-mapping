#!/usr/bin/env python3
"""Record timestamp continuity for the inputs and odometry of one capture."""

from __future__ import annotations

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))

import argparse
import csv
from dataclasses import asdict, dataclass
from functools import partial
from pathlib import Path
import signal
import time
from typing import Optional

import yaml


@dataclass
class TimingEvent:
    header_delta_ns: Optional[int]
    receive_delta_ns: Optional[int]
    header_gap: bool
    receive_gap: bool


@dataclass
class StreamStats:
    gap_threshold_ns: int
    message_count: int = 0
    header_gap_count: int = 0
    receive_gap_count: int = 0
    header_time_reversal_count: int = 0
    max_header_delta_ns: int = 0
    max_receive_delta_ns: int = 0
    _previous_header_stamp_ns: Optional[int] = None
    _previous_receive_ns: Optional[int] = None

    def observe(self, *, header_stamp_ns: int, received_ns: int) -> TimingEvent:
        header_delta_ns = None
        receive_delta_ns = None
        header_gap = False
        receive_gap = False
        if self._previous_header_stamp_ns is not None:
            header_delta_ns = header_stamp_ns - self._previous_header_stamp_ns
            if header_delta_ns < 0:
                self.header_time_reversal_count += 1
            else:
                self.max_header_delta_ns = max(self.max_header_delta_ns, header_delta_ns)
                header_gap = header_delta_ns > self.gap_threshold_ns
                self.header_gap_count += int(header_gap)
        if self._previous_receive_ns is not None:
            receive_delta_ns = received_ns - self._previous_receive_ns
            self.max_receive_delta_ns = max(self.max_receive_delta_ns, receive_delta_ns)
            receive_gap = receive_delta_ns > self.gap_threshold_ns
            self.receive_gap_count += int(receive_gap)

        self.message_count += 1
        self._previous_header_stamp_ns = header_stamp_ns
        self._previous_receive_ns = received_ns
        return TimingEvent(
            header_delta_ns=header_delta_ns,
            receive_delta_ns=receive_delta_ns,
            header_gap=header_gap,
            receive_gap=receive_gap,
        )

    def summary(self) -> dict[str, int]:
        values = asdict(self)
        return {
            key: value
            for key, value in values.items()
            if not key.startswith("_")
        }


def header_stamp_ns(message: object) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def spin_until_stopped(*, spin_once, ros_is_ok, stop_requested) -> None:
    while ros_is_ok() and not stop_requested():
        spin_once(timeout_sec=0.25)


def stream_gap_thresholds_ns(
    *,
    imu_gap_threshold_ms: float,
    lidar_gap_threshold_ms: float,
    odom_gap_threshold_ms: float,
) -> dict[str, int]:
    return {
        "imu_raw": int(imu_gap_threshold_ms * 1_000_000),
        "imu_filtered": int(imu_gap_threshold_ms * 1_000_000),
        "lidar": int(lidar_gap_threshold_ms * 1_000_000),
        "odom": int(odom_gap_threshold_ms * 1_000_000),
    }


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Write per-message timing continuity for one experiment capture."
    )
    parser.add_argument("--output-csv", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--raw-imu-topic", required=True)
    parser.add_argument("--filtered-imu-topic", required=True)
    parser.add_argument("--lidar-topic", required=True)
    parser.add_argument("--odom-topic", required=True)
    parser.add_argument("--imu-gap-threshold-ms", type=float, default=20.0)
    parser.add_argument("--lidar-gap-threshold-ms", type=float, default=250.0)
    parser.add_argument("--odom-gap-threshold-ms", type=float, default=500.0)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if any(
        value <= 0.0
        for value in (
            args.imu_gap_threshold_ms,
            args.lidar_gap_threshold_ms,
            args.odom_gap_threshold_ms,
        )
    ):
        raise SystemExit("all gap thresholds must be positive")

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2

    class TimingMonitor(Node):
        def __init__(self) -> None:
            super().__init__("experiment_timing_monitor")
            self.started_at_unix_ns = time.time_ns()
            threshold_by_stream = stream_gap_thresholds_ns(
                imu_gap_threshold_ms=args.imu_gap_threshold_ms,
                lidar_gap_threshold_ms=args.lidar_gap_threshold_ms,
                odom_gap_threshold_ms=args.odom_gap_threshold_ms,
            )
            self.streams = {
                "imu_raw": (
                    args.raw_imu_topic,
                    StreamStats(threshold_by_stream["imu_raw"]),
                ),
                "imu_filtered": (
                    args.filtered_imu_topic,
                    StreamStats(threshold_by_stream["imu_filtered"]),
                ),
                "lidar": (
                    args.lidar_topic,
                    StreamStats(threshold_by_stream["lidar"]),
                ),
                "odom": (
                    args.odom_topic,
                    StreamStats(threshold_by_stream["odom"]),
                ),
            }
            args.output_csv.parent.mkdir(parents=True, exist_ok=True)
            self._csv_file = args.output_csv.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(
                self._csv_file,
                fieldnames=(
                    "received_unix_ns",
                    "stream",
                    "topic",
                    "header_stamp_ns",
                    "header_delta_ns",
                    "receive_delta_ns",
                    "header_gap",
                    "receive_gap",
                ),
            )
            self._writer.writeheader()
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=100,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                Imu, args.raw_imu_topic, partial(self._on_message, "imu_raw"), qos
            )
            self.create_subscription(
                Imu,
                args.filtered_imu_topic,
                partial(self._on_message, "imu_filtered"),
                qos,
            )
            self.create_subscription(
                PointCloud2, args.lidar_topic, partial(self._on_message, "lidar"), qos
            )
            self.create_subscription(
                Odometry, args.odom_topic, partial(self._on_message, "odom"), qos
            )

        def _on_message(self, stream: str, message: object) -> None:
            received_monotonic_ns = time.monotonic_ns()
            received_unix_ns = time.time_ns()
            topic, stats = self.streams[stream]
            stamp_ns = header_stamp_ns(message)
            event = stats.observe(
                header_stamp_ns=stamp_ns, received_ns=received_monotonic_ns
            )
            self._writer.writerow(
                {
                    "received_unix_ns": received_unix_ns,
                    "stream": stream,
                    "topic": topic,
                    "header_stamp_ns": stamp_ns,
                    "header_delta_ns": event.header_delta_ns,
                    "receive_delta_ns": event.receive_delta_ns,
                    "header_gap": str(event.header_gap).lower(),
                    "receive_gap": str(event.receive_gap).lower(),
                }
            )
            if event.header_gap:
                self.get_logger().warn(
                    "%s header timestamp gap %.3f ms on %s",
                    stream,
                    event.header_delta_ns / 1_000_000.0,
                    topic,
                )

        def close(self) -> None:
            self._csv_file.flush()
            self._csv_file.close()
            args.summary.parent.mkdir(parents=True, exist_ok=True)
            summary = {
                "started_at_unix_ns": self.started_at_unix_ns,
                "ended_at_unix_ns": time.time_ns(),
                "gap_threshold_ms": {
                    "imu_raw": args.imu_gap_threshold_ms,
                    "imu_filtered": args.imu_gap_threshold_ms,
                    "lidar": args.lidar_gap_threshold_ms,
                    "odom": args.odom_gap_threshold_ms,
                },
                "streams": {
                    name: {"topic": topic, **stats.summary()}
                    for name, (topic, stats) in self.streams.items()
                },
            }
            args.summary.write_text(
                yaml.safe_dump(summary, sort_keys=False), encoding="utf-8"
            )

    rclpy.init()
    monitor = TimingMonitor()

    stop_requested = [False]

    def shutdown_handler(_signal_number: int, _frame: object) -> None:
        stop_requested[0] = True

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)
    try:
        spin_until_stopped(
            spin_once=lambda timeout_sec: rclpy.spin_once(
                monitor, timeout_sec=timeout_sec
            ),
            ros_is_ok=rclpy.ok,
            stop_requested=lambda: stop_requested[0],
        )
    finally:
        monitor.close()
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
