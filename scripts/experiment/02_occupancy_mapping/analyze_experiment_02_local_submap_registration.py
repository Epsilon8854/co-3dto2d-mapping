#!/usr/bin/env python3
"""Summarise one Experiment 2 local-submap-registration diagnostics CSV."""

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))

import argparse
import csv
import html
import math
from collections import Counter
from pathlib import Path
from statistics import fmean


def number(row, name):
    return float(row[name])


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def angle_degrees(radians):
    return math.degrees((radians + math.pi) % (2.0 * math.pi) - math.pi)


def displacement(first, last, prefix):
    dx = number(last, f"{prefix}_x") - number(first, f"{prefix}_x")
    dy = number(last, f"{prefix}_y") - number(first, f"{prefix}_y")
    dyaw = angle_degrees(number(last, f"{prefix}_yaw") - number(first, f"{prefix}_yaw"))
    return math.hypot(dx, dy), dyaw


def path_length(rows, prefix):
    return sum(
        math.hypot(
            number(current, f"{prefix}_x") - number(previous, f"{prefix}_x"),
            number(current, f"{prefix}_y") - number(previous, f"{prefix}_y"),
        )
        for previous, current in zip(rows, rows[1:])
    )


def runtime_summary(rows, field):
    values = [number(row, field) for row in rows if number(row, field) > 0.0]
    if not values:
        return (0, 0.0, 0.0, 0.0)
    return len(values), fmean(values), percentile(values, 0.95), max(values)


def render_table(rows):
    return "\n".join(
        "<tr>" + "".join(f"<td>{html.escape(str(value))}</td>" for value in row) + "</tr>"
        for row in rows
    )


def build_report(rows, source_path):
    if not rows:
        raise ValueError("diagnostics CSV has no data rows")
    first, last = rows[0], rows[-1]
    raw_closure, raw_yaw = displacement(first, last, "raw")
    corrected_closure, corrected_yaw = displacement(first, last, "corrected")
    attempts = [row for row in rows if row.get("reason") != "rate_limited"]
    accepted = [row for row in attempts if row.get("accepted") == "1"]
    reasons = Counter(row.get("reason") or "missing" for row in attempts)
    corrections = [
        math.hypot(number(row, "corrected_x") - number(row, "raw_x"),
                   number(row, "corrected_y") - number(row, "raw_y"))
        for row in rows
    ]
    runtime_rows = []
    for field, label in [
        ("target_extraction_ms", "Target extraction"),
        ("coarse_search_ms", "Coarse search"),
        ("refinement_ms", "ICP refinement"),
        ("total_ms", "Total registration"),
    ]:
        count, mean, p95, maximum = runtime_summary(attempts, field)
        runtime_rows.append((label, count, f"{mean:.2f}", f"{p95:.2f}", f"{maximum:.2f}"))
    reason_rows = [(reason, count) for reason, count in sorted(reasons.items())]
    acceptance = f"{len(accepted)} / {len(attempts)}" if attempts else "0 / 0"
    return f"""<!doctype html>
<html lang=\"en\"><head><meta charset=\"utf-8\"><title>Experiment 2 local-submap registration</title>
<style>body{{font-family:system-ui,sans-serif;max-width:1000px;margin:32px auto;color:#18222d}} table{{border-collapse:collapse;margin:10px 0 26px;min-width:560px}}th,td{{padding:8px 10px;border:1px solid #cbd5e1;text-align:right}}th{{background:#e2e8f0}}th:first-child,td:first-child{{text-align:left}}.note{{color:#475569}}</style></head>
<body><h1>Experiment 2: local-submap registration</h1>
<p class=\"note\">Source: {html.escape(str(source_path))}. Closure is start-to-end odometry displacement; without external ground truth it is a repeatability indicator, not absolute accuracy.</p>
<h2>Closure displacement</h2><table><tr><th>Trajectory</th><th>Start-to-end XY (m)</th><th>Start-to-end yaw (deg)</th><th>Path length (m)</th></tr>
{render_table([("Raw RTAB-Map prior", f"{raw_closure:.3f}", f"{raw_yaw:.3f}", f"{path_length(rows, 'raw'):.3f}"), ("Local-submap corrected", f"{corrected_closure:.3f}", f"{corrected_yaw:.3f}", f"{path_length(rows, 'corrected'):.3f}")])}</table>
<h2>Acceptance</h2><table><tr><th>Accepted registrations</th><th>Max cumulative planar correction (m)</th><th>p95 cumulative planar correction (m)</th></tr>
{render_table([(acceptance, f"{max(corrections):.3f}", f"{percentile(corrections, 0.95):.3f}")])}</table>
<h3>Registration reasons</h3><table><tr><th>Reason</th><th>Count</th></tr>{render_table(reason_rows)}</table>
<h2>Registration runtime</h2><table><tr><th>Stage</th><th>Samples</th><th>Mean (ms)</th><th>p95 (ms)</th><th>Max (ms)</th></tr>{render_table(runtime_rows)}</table>
</body></html>"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    args = parser.parse_args()
    with args.csv.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / "local_submap_registration_report.html"
    output.write_text(build_report(rows, args.csv))
    print(output)


if __name__ == "__main__":
    main()
