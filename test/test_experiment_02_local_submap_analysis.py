import csv
import subprocess
import sys
from pathlib import Path


def test_local_submap_report_includes_closure_and_runtime(tmp_path):
    csv_path = tmp_path / "diagnostics.csv"
    fields = [
        "stamp_ns", "raw_x", "raw_y", "raw_yaw", "corrected_x", "corrected_y",
        "corrected_yaw", "accepted", "reason", "source_points", "target_points",
        "correspondences", "overlap", "initial_rmse", "rmse", "target_extraction_ms",
        "coarse_search_ms", "refinement_ms", "total_ms",
    ]
    rows = [
        ["1", "0", "0", "0", "0", "0", "0", "0", "rate_limited", "20", "0", "0", "0", "0", "0", "0", "0", "0", "0"],
        ["2", "1", "0", "0", "1.1", "0", "0.01", "1", "accepted", "20", "40", "30", "0.8", "0.2", "0.1", "1", "2", "3", "6"],
        ["3", "0.2", "0", "0", "0.1", "0", "0.00", "1", "accepted", "20", "40", "30", "0.8", "0.2", "0.1", "2", "4", "6", "12"],
    ]
    with csv_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(fields)
        writer.writerows(rows)
    output_dir = tmp_path / "report"
    subprocess.run(
        [sys.executable, "scripts/analyze_experiment_02_local_submap_registration.py",
         "--csv", str(csv_path), "--output-dir", str(output_dir)],
        check=True,
    )
    report = (output_dir / "local_submap_registration_report.html").read_text()
    assert "Closure displacement" in report
    assert "Registration runtime" in report
    assert "p95" in report
    assert "2 / 2" in report
