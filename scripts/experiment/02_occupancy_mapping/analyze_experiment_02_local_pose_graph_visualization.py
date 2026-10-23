#!/usr/bin/env python3
"""Create a self-contained PHT cumulative-submap matching inspection view."""

from pathlib import Path as _WorkspacePath
import sys as _workspace_sys
_workspace_root = next(p for p in _WorkspacePath(__file__).resolve().parents if (p/"package.xml").is_file() and (p/"CMakeLists.txt").is_file())
_workspace_sys.path.insert(0, str(_workspace_root))
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


PHT_MIN_LENGTH_M = 1.50
PHT_MAX_GAP_M = 0.10
PHT_MAX_CONTIGUOUS_GAP_M = 0.15
PHT_MIN_SUPPORT_RATIO = 0.70
PHT_HISTORY_KEYFRAMES = 10


def read_jsonl(path):
    if not path.exists():
        return []
    values = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        try:
            values.append(json.loads(line.replace(":inf", ":null").replace(":-inf", ":null")))
        except json.JSONDecodeError:
            # A forced replay shutdown can leave one incomplete final record.
            continue
    return values


def occupied_world_cells(snapshot):
    grid = snapshot.get("map")
    if not grid:
        return set()
    resolution = grid["resolution"]
    origin_x, origin_y = grid["origin"]
    cells = set()
    for row, start, length in grid.get("occupied_runs", []):
        for column in range(start, start + length):
            x = origin_x + (column + 0.5) * resolution
            y = origin_y + (row + 0.5) * resolution
            cells.add((math.floor(x / resolution), math.floor(y / resolution)))
    return cells


def encode_runs(cells, width, height):
    rows = {}
    for column, row in cells:
        rows.setdefault(row, []).append(column)
    runs = []
    for row, columns in rows.items():
        columns.sort()
        start = previous = columns[0]
        for column in columns[1:]:
            if column == previous + 1:
                previous = column
                continue
            runs.append([row, start, previous - start + 1])
            start = previous = column
        runs.append([row, start, previous - start + 1])
    return runs


def angle_difference(first, second):
    delta = (first - second + math.pi / 2.0) % math.pi - math.pi / 2.0
    return abs(delta)


def segment_angle(segment):
    return math.atan2(segment[3] - segment[1], segment[2] - segment[0])


def segment_center(segment):
    return ((segment[0] + segment[2]) * 0.5, (segment[1] + segment[3]) * 0.5)


def segment_length(segment):
    return math.hypot(segment[2] - segment[0], segment[3] - segment[1])


def normal_distance(segment, point):
    angle = segment_angle(segment)
    nx, ny = -math.sin(angle), math.cos(angle)
    return abs(nx * (point[0] - segment[0]) + ny * (point[1] - segment[1]))


def overlap_length(first, second):
    angle = segment_angle(second)
    tx, ty = math.cos(angle), math.sin(angle)
    center = segment_center(second)

    def projection(point):
        return (point[0] - center[0]) * tx + (point[1] - center[1]) * ty

    first_min = min(projection((first[0], first[1])), projection((first[2], first[3])))
    first_max = max(projection((first[0], first[1])), projection((first[2], first[3])))
    second_min = min(projection((second[0], second[1])), projection((second[2], second[3])))
    second_max = max(projection((second[0], second[1])), projection((second[2], second[3])))
    return max(0.0, min(first_max, second_max) - max(first_min, second_min))


def has_support(mask, column, row):
    height, width = mask.shape
    for row_offset in (-1, 0, 1):
        for column_offset in (-1, 0, 1):
            test_column, test_row = column + column_offset, row + row_offset
            if 0 <= test_column < width and 0 <= test_row < height and mask[test_row, test_column]:
                return True
    return False


def pht_segments(mask, origin_x, origin_y, resolution):
    min_length_cells = max(1, math.ceil(PHT_MIN_LENGTH_M / resolution))
    max_gap_cells = max(0, math.floor(PHT_MAX_GAP_M / resolution))
    candidates = cv2.HoughLinesP(
        mask, 1.0, math.pi / 180.0, threshold=12,
        minLineLength=min_length_cells, maxLineGap=max_gap_cells,
    )
    if candidates is None:
        return []
    accepted = []
    for x1, y1, x2, y2 in candidates[:, 0, :]:
        count = max(abs(int(x2) - int(x1)), abs(int(y2) - int(y1))) + 1
        supported = largest_gap = gap = 0
        for index in range(count):
            fraction = index / max(1, count - 1)
            column = round(x1 + (x2 - x1) * fraction)
            row = round(y1 + (y2 - y1) * fraction)
            if has_support(mask, column, row):
                supported += 1
                gap = 0
            else:
                gap += 1
                largest_gap = max(largest_gap, gap)
        if supported / count < PHT_MIN_SUPPORT_RATIO:
            continue
        if largest_gap * resolution > PHT_MAX_CONTIGUOUS_GAP_M:
            continue
        segment = [
            origin_x + (x1 + 0.5) * resolution,
            origin_y + (y1 + 0.5) * resolution,
            origin_x + (x2 + 0.5) * resolution,
            origin_y + (y2 + 0.5) * resolution,
        ]
        if segment_length(segment) < PHT_MIN_LENGTH_M:
            continue
        duplicate = False
        for existing in accepted:
            if (angle_difference(segment_angle(segment), segment_angle(existing)) <= math.radians(5.0)
                    and normal_distance(existing, segment_center(segment)) <= 0.10
                    and overlap_length(segment, existing) >= 0.5 * min(segment_length(segment), segment_length(existing))):
                duplicate = True
                break
        if not duplicate:
            accepted.append(segment)
    return sorted(accepted, key=segment_length, reverse=True)[:12]


def pht_pairs(source_lines, target_lines):
    pairs, used_targets = [], set()
    for source_index, source in enumerate(source_lines):
        best = None
        for target_index, target in enumerate(target_lines):
            if target_index in used_targets:
                continue
            angle = angle_difference(segment_angle(source), segment_angle(target))
            normal = normal_distance(target, segment_center(source))
            overlap = overlap_length(source, target)
            if angle > math.radians(10.0) or normal > 0.35 or overlap < 0.50:
                continue
            score = normal + 0.25 * angle - 0.10 * min(overlap, 2.0)
            candidate = (score, target_index, angle, normal, overlap)
            if best is None or candidate < best:
                best = candidate
        if best is not None:
            _, target_index, angle, normal, overlap = best
            used_targets.add(target_index)
            pairs.append({
                "source": source_index, "target": target_index,
                "angle_deg": math.degrees(angle), "normal_m": normal,
                "overlap_m": overlap,
            })
    return pairs


def build_cumulative_pht_submaps(snapshots, history_keyframes=PHT_HISTORY_KEYFRAMES):
    """Attach a prior-only target submap and its PHT correspondences to each frame."""
    for index, snapshot in enumerate(snapshots):
        history = snapshots[max(0, index - history_keyframes):index]
        history = [candidate for candidate in history if candidate.get("map")]
        if not history or not snapshot.get("map"):
            snapshot["pht_submap"] = {"map": None, "lines": [], "pairs": [], "keyframes": []}
            continue
        resolution = snapshot["map"]["resolution"]
        cells = set()
        for candidate in history:
            cells.update(occupied_world_cells(candidate))
        if not cells:
            snapshot["pht_submap"] = {"map": None, "lines": [], "pairs": [], "keyframes": []}
            continue
        columns = [cell[0] for cell in cells]
        rows = [cell[1] for cell in cells]
        min_column, max_column = min(columns), max(columns)
        min_row, max_row = min(rows), max(rows)
        width, height = max_column - min_column + 1, max_row - min_row + 1
        mask = np.zeros((height, width), dtype=np.uint8)
        local_cells = set()
        for column, row in cells:
            local_column, local_row = column - min_column, row - min_row
            mask[local_row, local_column] = 255
            local_cells.add((local_column, local_row))
        origin_x, origin_y = min_column * resolution, min_row * resolution
        lines = pht_segments(mask, origin_x, origin_y, resolution)
        source_lines = snapshot.get("pht_map_lines", [])
        snapshot["pht_submap"] = {
            "map": {
                "origin": [origin_x, origin_y], "resolution": resolution,
                "width": width, "height": height,
                "occupied_runs": encode_runs(local_cells, width, height),
            },
            "lines": lines,
            "pairs": pht_pairs(source_lines, lines),
            "keyframes": [candidate["frame"] for candidate in history],
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot-dir", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    snapshots = read_jsonl(args.snapshot_dir / "snapshots.jsonl")
    if not snapshots:
        raise SystemExit("snapshots.jsonl is empty")
    build_cumulative_pht_submaps(snapshots)

    payload = json.dumps({"snapshots": snapshots}, separators=(",", ":"))
    template = r'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<title>Cumulative PHT occupancy-line matching inspection</title>
<style>
body{font-family:system-ui,sans-serif;margin:22px;background:#111827;color:#e5e7eb}h1{margin:0 0 6px}.note{color:#cbd5e1;max-width:1120px}.controls{display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin:16px 0}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:16px}canvas{background:#18222d;border:1px solid #475569;max-width:100%;height:auto}label{display:flex;gap:8px;align-items:center}input,button{accent-color:#22d3ee}button{padding:5px 9px;background:#334155;color:#e5e7eb;border:1px solid #64748b;border-radius:4px}#summary,#pairs{white-space:pre-wrap;color:#cbd5e1;margin:8px 0}h2{font-size:17px;margin:0 0 8px}.legend{font-size:13px;color:#cbd5e1;min-height:42px}
</style></head><body>
<h1>Cumulative PHT occupancy-line matching inspection</h1>
<p class="note">The target contains only prior corrected keyframes, accumulated as a 2D occupancy submap. Magenta lines are target PHT wall segments; cyan lines are current-frame PHT segments. Thick segments and yellow connectors show the actual PHT pairs used for this inspection. This view does not add a pose-graph factor.</p>
<div class="controls"><button id="previous" type="button">Previous PHT matching case</button><button id="next" type="button">Next PHT matching case</button><label>Frame <input id="frame" type="range" min="0" value="0"></label><span id="frameLabel"></span></div>
<div id="summary"></div><div class="grid"><section><h2>Cumulative PHT target submap</h2><canvas id="targetMap" width="620" height="500" aria-label="Cumulative target occupancy submap and PHT lines"></canvas><div class="legend">Dark: unknown · White: prior occupied · Magenta: target PHT wall segment T#</div></section><section><h2>Current frame 2D occupancy map</h2><canvas id="sourceMap" width="620" height="500" aria-label="Current occupancy map and PHT lines"></canvas><div class="legend">Dark: unknown · Gray: raycast free · White: occupied · Cyan: current PHT wall segment S#</div></section><section><h2>Matched PHT pairs</h2><canvas id="matchingMap" width="620" height="500" aria-label="Cumulative submap and current PHT correspondences"></canvas><div class="legend">Cyan: current PHT · Magenta: accumulated target PHT · Yellow connector: accepted S# ↔ T# candidate</div></section></div><div id="pairs"></div>
<script>
const data=__PAYLOAD__,snapshots=data.snapshots;const slider=document.getElementById('frame'),label=document.getElementById('frameLabel');slider.max=Math.max(0,snapshots.length-1);
const matchingIndexes=snapshots.map((s,i)=>s.pht_submap&&s.pht_submap.pairs&&s.pht_submap.pairs.length?i:null).filter(i=>i!==null);
function clear(canvas){const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);return ctx}
function linePoints(lines){const out=[];for(const line of(lines||[]))out.push([line[0],line[1]],[line[2],line[3]]);return out}
function bounds(points){if(!points.length)return[-1,1,-1,1];const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);let loX=Math.min(...xs),hiX=Math.max(...xs),loY=Math.min(...ys),hiY=Math.max(...ys);const margin=Math.max(.5,(hiX-loX)*.12,(hiY-loY)*.12);return[loX-margin,hiX+margin,loY-margin,hiY+margin]}
function project(p,b,w,h){return[(p[0]-b[0])/(b[1]-b[0])*w,h-(p[1]-b[2])/(b[3]-b[2])*h]}
function mapRunPoints(map){if(!map)return[];const points=[];for(const runs of [map.free_runs||[],map.occupied_runs||[]])for(const [row,start,length] of runs){points.push([map.origin[0]+start*map.resolution,map.origin[1]+row*map.resolution],[map.origin[0]+(start+length)*map.resolution,map.origin[1]+(row+1)*map.resolution])}return points}
function drawRuns(ctx,map,b,runs,color,alpha=1){if(!map)return;ctx.globalAlpha=alpha;ctx.fillStyle=color;for(const [row,start,length] of runs||[]){const x=map.origin[0]+start*map.resolution,y=map.origin[1]+row*map.resolution;const a=project([x,y],b,ctx.canvas.width,ctx.canvas.height),z=project([x+length*map.resolution,y+map.resolution],b,ctx.canvas.width,ctx.canvas.height);ctx.fillRect(a[0],Math.min(a[1],z[1]),Math.max(1,Math.abs(z[0]-a[0])),Math.max(1,Math.abs(z[1]-a[1])))}ctx.globalAlpha=1}
function drawLines(ctx,lines,b,color,prefix,thick){ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=thick?5:2;(lines||[]).forEach((line,index)=>{const a=project([line[0],line[1]],b,ctx.canvas.width,ctx.canvas.height),z=project([line[2],line[3]],b,ctx.canvas.width,ctx.canvas.height);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...z);ctx.stroke();if(!thick){ctx.fillText(`${prefix}${index+1}`,(a[0]+z[0])/2+4,(a[1]+z[1])/2-4)}})}
function center(line){return[(line[0]+line[2])/2,(line[1]+line[3])/2]}
function drawTarget(snapshot,pairTargets){const sub=snapshot.pht_submap||{},map=sub.map,lines=sub.lines||[],ctx=clear(document.getElementById('targetMap'));if(!map)return;const b=bounds([...mapRunPoints(map),...linePoints(lines)]);drawRuns(ctx,map,b,map.occupied_runs,'#e2e8f0',.95);drawLines(ctx,lines,b,'#e879f9','T');drawLines(ctx,[...pairTargets].map(i=>lines[i]).filter(Boolean),b,'#e879f9','T',true)}
function drawSource(snapshot,pairSources){const map=snapshot.map,lines=snapshot.pht_map_lines||[],ctx=clear(document.getElementById('sourceMap'));if(!map)return;const b=bounds([...mapRunPoints(map),...linePoints(lines)]);drawRuns(ctx,map,b,map.free_runs,'#64748b',.72);drawRuns(ctx,map,b,map.occupied_runs,'#e2e8f0',.95);drawLines(ctx,lines,b,'#22d3ee','S');drawLines(ctx,[...pairSources].map(i=>lines[i]).filter(Boolean),b,'#22d3ee','S',true)}
function drawMatching(snapshot,pairs){const sub=snapshot.pht_submap||{},target=sub.lines||[],source=snapshot.pht_map_lines||[],ctx=clear(document.getElementById('matchingMap'));const b=bounds([...linePoints(target),...linePoints(source)]);drawLines(ctx,target,b,'#e879f9','T');drawLines(ctx,source,b,'#22d3ee','S');for(const pair of pairs){const s=source[pair.source],t=target[pair.target];if(!s||!t)continue;drawLines(ctx,[s],b,'#22d3ee','S',true);drawLines(ctx,[t],b,'#e879f9','T',true);const a=project(center(s),b,ctx.canvas.width,ctx.canvas.height),z=project(center(t),b,ctx.canvas.width,ctx.canvas.height);ctx.strokeStyle='#fbbf24';ctx.lineWidth=1.5;ctx.setLineDash([3,3]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...z);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#fbbf24';ctx.fillText(`S${pair.source+1}↔T${pair.target+1}`,(a[0]+z[0])/2+4,(a[1]+z[1])/2-4)}}
function render(){const snapshot=snapshots[Number(slider.value)],sub=snapshot.pht_submap||{},pairs=sub.pairs||[];label.textContent=`${snapshot.frame} / ${snapshots.length-1}`;const targetIds=new Set(pairs.map(p=>p.target)),sourceIds=new Set(pairs.map(p=>p.source));drawTarget(snapshot,targetIds);drawSource(snapshot,sourceIds);drawMatching(snapshot,pairs);const ids=(sub.keyframes||[]);const history=ids.length?`${ids[0]}…${ids[ids.length-1]} (${ids.length} keyframes)`:'none';document.getElementById('summary').textContent=`current frame ${snapshot.frame}\nprior-only cumulative target: ${history}\nPHT lines: current ${(snapshot.pht_map_lines||[]).length} · target ${(sub.lines||[]).length} · matched ${pairs.length}`;document.getElementById('pairs').textContent=pairs.length?`Matched PHT pairs: ${pairs.map(p=>`S${p.source+1} ↔ T${p.target+1}  |  angle ${p.angle_deg.toFixed(1)}° · normal ${p.normal_m.toFixed(3)} m · overlap ${p.overlap_m.toFixed(2)} m`).join('\n')}`:'Matched PHT pairs: none'}
function moveMatching(step){if(!matchingIndexes.length)return;const current=Number(slider.value);let position=matchingIndexes.findIndex(i=>i===current);if(position<0){position=step>0?matchingIndexes.findIndex(i=>i>current):matchingIndexes.findLastIndex(i=>i<current);if(position<0)position=step>0?0:matchingIndexes.length-1}else position=(position+step+matchingIndexes.length)%matchingIndexes.length;slider.value=matchingIndexes[position];render()}
slider.addEventListener('input',render);document.getElementById('previous').addEventListener('click',()=>moveMatching(-1));document.getElementById('next').addEventListener('click',()=>moveMatching(1));if(matchingIndexes.length)slider.value=matchingIndexes[0];render();
</script></body></html>'''
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(template.replace("__PAYLOAD__", payload))
    print(args.output)


if __name__ == "__main__":
    main()
