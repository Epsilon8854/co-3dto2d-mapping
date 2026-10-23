#!/usr/bin/env python3
"""Summarize Experiment 01 pre-ICP cloud-input replay outputs."""

from __future__ import annotations

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))

import argparse
import csv
import html
import json
import math
from pathlib import Path
from statistics import mean

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else math.nan


def format_value(value: float, digits: int = 3) -> str:
    return "—" if not math.isfinite(value) else f"{value:.{digits}f}"


def run_value(run_dir: Path, key: str) -> str:
    for line in (run_dir / "run.yaml").read_text(encoding="utf-8").splitlines():
        if line.startswith(key + ":"):
            return line.split(":", 1)[1].strip().strip('"')
    return "—"


def read_run(run_dir: Path) -> dict:
    derived = run_dir / "derived"
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(derived), storage_id="sqlite3"),
        ConverterOptions("cdr", "cdr"),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    message_types = {name: get_message(type_name) for name, type_name in topic_types.items()}
    clouds: dict[str, list[int]] = {}
    odom: list[tuple[int, float, float, float]] = []
    info: list[dict] = []
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        message = deserialize_message(serialized, message_types[topic])
        if topic.startswith("/r0/mapping/lidar_") or topic == "/r0/livox/lidar":
            clouds.setdefault(topic, []).append(int(message.width) * int(message.height))
        elif topic == "/r0/odom":
            position = message.pose.pose.position
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            odom.append((stamp, float(position.x), float(position.y), float(position.z)))
        elif topic == "/r0/odom_info_lite":
            info.append(
                {
                    "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec),
                    "lost": bool(message.lost),
                    "ratio": float(message.icp_inliers_ratio),
                    "translation": float(message.icp_translation),
                    "rotation": float(message.icp_rotation),
                    "correspondences": int(message.icp_correspondences),
                }
            )

    odom_array = np.asarray(odom, dtype=float)
    if len(odom_array) >= 2:
        deltas = np.diff(odom_array[:, 1:4], axis=0)
        step_3d = np.linalg.norm(deltas, axis=1)
        step_xy = np.linalg.norm(deltas[:, :2], axis=1)
        endpoint_3d = float(np.linalg.norm(odom_array[-1, 1:4] - odom_array[0, 1:4]))
        endpoint_xy = float(np.linalg.norm(odom_array[-1, 1:3] - odom_array[0, 1:3]))
        path_length_3d = float(np.sum(step_3d))
        duration_s = (odom_array[-1, 0] - odom_array[0, 0]) / 1e9
    else:
        step_3d = np.asarray([])
        step_xy = np.asarray([])
        endpoint_3d = endpoint_xy = path_length_3d = duration_s = math.nan

    filter_stats = []
    stats_path = run_dir / "pre_icp_filter_stats.csv"
    if stats_path.exists():
        filter_stats = list(csv.DictReader(stats_path.open(encoding="utf-8")))
    published_stats = [row for row in filter_stats if row["state"] == "published"]
    height_counts = clouds.get("/r0/mapping/lidar_icp_height_filtered", [])
    rear_counts = clouds.get("/r0/mapping/lidar_rear_filtered", [])
    raw_counts = clouds.get("/r0/livox/lidar", [])
    ratios = [float(row["retention_ratio"]) for row in published_stats]
    lost_entries = [item for item in info if item["lost"]]
    first_lost_s = math.nan
    if lost_entries and info:
        first_lost_s = (lost_entries[0]["stamp_ns"] - info[0]["stamp_ns"]) / 1e9

    base_condition = (
        "rear + gravity-height"
        if height_counts
        else "rear only"
        if rear_counts
        else "raw only"
    )
    return {
        "run_dir": str(run_dir),
        "condition": base_condition,
        "deskewing": run_value(run_dir, "deskewing"),
        "bundle_adjustment": run_value(run_dir, "bundle_adjustment"),
        "odom_messages": len(odom),
        "duration_s": duration_s,
        "endpoint_3d_m": endpoint_3d,
        "endpoint_xy_m": endpoint_xy,
        "path_length_3d_m": path_length_3d,
        "max_step_3d_m": float(np.max(step_3d)) if len(step_3d) else math.nan,
        "p95_step_3d_m": percentile(step_3d.tolist(), 95),
        "steps_over_0_2m": int(np.count_nonzero(step_3d > 0.2)),
        "rear_cloud_messages": len(rear_counts),
        "rear_cloud_mean_points": mean(rear_counts) if rear_counts else math.nan,
        "raw_cloud_messages": len(raw_counts),
        "raw_cloud_mean_points": mean(raw_counts) if raw_counts else math.nan,
        "height_cloud_messages": len(height_counts),
        "height_cloud_mean_points": mean(height_counts) if height_counts else math.nan,
        "height_published": len(published_stats),
        "height_gravity_unavailable": sum(
            row["state"] == "gravity_unavailable" for row in filter_stats
        ),
        "height_retention_mean": mean(ratios) if ratios else math.nan,
        "icp_lost": sum(item["lost"] for item in info),
        "icp_lost_rate": (
            sum(item["lost"] for item in info) / len(info) if info else math.nan
        ),
        "icp_first_lost_s": first_lost_s,
        "icp_zero_ratio": sum(item["ratio"] == 0.0 for item in info),
        "icp_ratio_mean": mean(item["ratio"] for item in info) if info else math.nan,
        "icp_translation_p95_m": percentile([item["translation"] for item in info], 95),
        "icp_translation_max_m": max((item["translation"] for item in info), default=math.nan),
        "icp_rotation_p95_rad": percentile([item["rotation"] for item in info], 95),
        "icp_correspondences_zero": sum(
            item["correspondences"] == 0 for item in info
        ),
    }


def row_cells(summary: dict) -> str:
    values = (
        summary["condition"],
        summary["deskewing"],
        summary["bundle_adjustment"],
        summary["odom_messages"],
        format_value(summary["endpoint_3d_m"]),
        format_value(summary["path_length_3d_m"]),
        format_value(summary["max_step_3d_m"]),
        summary["steps_over_0_2m"],
        format_value(summary["icp_ratio_mean"]),
        summary["icp_lost"],
        format_value(summary["icp_lost_rate"] * 100.0, 1) + "%",
        format_value(summary["icp_first_lost_s"], 1),
        summary["icp_zero_ratio"],
        format_value(summary["icp_translation_p95_m"]),
        summary["raw_cloud_messages"] or "—",
        format_value(summary["raw_cloud_mean_points"], 0),
        summary["rear_cloud_messages"],
        format_value(summary["rear_cloud_mean_points"], 0),
        summary["height_cloud_messages"] or "—",
        format_value(summary["height_cloud_mean_points"], 0),
        format_value(summary["height_retention_mean"] * 100.0, 1) + "%"
        if math.isfinite(summary["height_retention_mean"])
        else "—",
    )
    return "".join(f"<td>{html.escape(str(value))}</td>" for value in values)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("runs", nargs="+", type=Path)
    args = parser.parse_args()
    summaries = [read_run(path.resolve()) for path in args.runs]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(
        json.dumps(summaries, indent=2, ensure_ascii=False), encoding="utf-8"
    )

    static_rows = [summary for summary in summaries if "static_" in summary["run_dir"]]
    dynamic_rows = [summary for summary in summaries if "dynamic_" in summary["run_dir"]]
    table_head = """
      <tr><th>ICP input</th><th>Deskew</th><th>Bundle adjustment</th><th>Odom msgs</th><th>Endpoint 3D m</th><th>Path length 3D m</th>
      <th>Max step m</th><th>Steps &gt;0.2 m</th><th>Mean ICP ratio</th>
      <th>Lost updates</th><th>Lost rate</th><th>First lost s</th>
      <th>Zero-ratio updates</th><th>ICP translation p95 m</th>
      <th>Raw clouds</th><th>Raw mean pts</th>
      <th>Rear clouds</th><th>Rear mean pts</th><th>Height clouds</th>
      <th>Height mean pts</th><th>Height retention</th></tr>"""
    document = f"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>Experiment 01 ICP input ablation</title>
<style>
body {{ font-family: system-ui, sans-serif; margin: 32px; color: #17212b; }}
h1 {{ margin-bottom: 4px; }} p {{ line-height: 1.55; }}
table {{ border-collapse: collapse; margin: 12px 0 28px; font-size: 14px; }}
th, td {{ border: 1px solid #cbd5e1; padding: 7px 9px; text-align: right; }}
th {{ background: #eaf2ff; }} td:first-child, th:first-child {{ text-align: left; }}
.bad {{ color: #b42318; font-weight: 650; }} .note {{ background: #fff6df; padding: 12px; }}
code {{ background: #eef2f7; padding: 2px 4px; }}
</style></head><body>
<h1>Experiment 01: ICP input ablation</h1>
<p>Same raw rosbag, same pose-off and BundleAdjustment setting. The table records the deskewing setting for each run; only compare rows with the same setting when evaluating ICP cloud input.</p>
<div class="note"><b>Interpretation:</b> the static bag is the false-motion test. The dynamic bag contains real robot motion, so its endpoint displacement is not an absolute drift error.</div>
<h2>Static bag — false-motion test</h2><table>{table_head}{''.join('<tr>'+row_cells(row)+'</tr>' for row in static_rows)}</table>
<h2>Dynamic bag — robustness comparison</h2><table>{table_head}{''.join('<tr>'+row_cells(row)+'</tr>' for row in dynamic_rows)}</table>
<h2>Recorded artifacts</h2><ul>{''.join('<li><code>'+html.escape(row['run_dir'])+'</code></li>' for row in summaries)}</ul>
</body></html>"""
    args.output.write_text(document, encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
