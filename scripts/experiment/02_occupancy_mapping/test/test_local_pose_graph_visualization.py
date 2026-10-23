import json
import subprocess
import sys
from pathlib import Path


def test_pose_graph_visualization_embeds_snapshots_and_alignment_controls(tmp_path):
    snapshot_dir = tmp_path / "snapshots"
    snapshot_dir.mkdir()
    (snapshot_dir / "snapshots.jsonl").write_text(json.dumps({
        "frame": 0, "stamp_ns": 1, "raw": [0, 0, 0],
        "optimized": [0, 0, 0], "keyframe": True, "accepted": 0,
        "constraints": 0, "resolution": 0.05,
        "occupancy": [[0, 0], [0.05, 0]], "points": [[0, 0], [1, 0]],
        "lines": [[0, 0, 1, 0]],
        "map": {
            "origin": [-1, -1], "resolution": 0.05, "width": 40, "height": 40,
            "free_runs": [[20, 0, 10]], "occupied_runs": [[20, 10, 1]],
        },
        "map_lines": [[0, 0, 1, 0]],
        "pht_map_lines": [[0, 0, 1, 0]],
    }) + "\n")
    (snapshot_dir / "events.jsonl").write_text(json.dumps({
        "frame": 0, "candidates": [{
            "target_frame": 0, "accepted": True, "reason": "accepted",
            "initial_rmse": 0.08, "rmse": 0.01, "pairs": [[0, 0]],
            "raw_relative": [0, 0, 0], "matched_relative": [0, 0, 0],
        }],
    }) + "\n")
    output = tmp_path / "pose_graph_process.html"
    subprocess.run([
        sys.executable, "scripts/experiment/02_occupancy_mapping/analyze_experiment_02_local_pose_graph_visualization.py",
        "--snapshot-dir", str(snapshot_dir), "--output", str(output),
    ], check=True)
    text = output.read_text()
    assert "Cumulative PHT occupancy-line matching inspection" in text
    assert "snapshots" in text
    assert "Cumulative PHT target submap" in text
    assert "Current frame 2D occupancy" in text
    assert "PHT wall segments" in text
    assert "pht_map_lines" in text
    assert "Matched PHT pairs" in text
