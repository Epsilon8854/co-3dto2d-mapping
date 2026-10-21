#!/usr/bin/env python3
"""Render online cumulative-PHT submap correspondences recorded by Experiment 2."""
import argparse
import json
from pathlib import Path


def read_jsonl(path):
    result = []
    if not path.exists():
        return result
    for line in path.read_text().splitlines():
        try:
            result.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    snapshots = read_jsonl(args.snapshot_dir / "snapshots.jsonl")
    events = {item["frame"]: item for item in read_jsonl(args.snapshot_dir / "events.jsonl")}
    records = [{**snapshot, "event": events.get(snapshot["frame"], {})} for snapshot in snapshots]
    if not records:
        raise SystemExit("No PHT-submap snapshots were written.")
    payload = json.dumps(records, separators=(",", ":"))
    html = r"""<!doctype html><html lang="en"><meta charset="utf-8">
<title>PHT cumulative submap inspection</title>
<style>
body{background:#111827;color:#e5e7eb;font-family:system-ui,sans-serif;margin:20px} .controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap} button{background:#334155;color:#e5e7eb;border:1px solid #64748b;border-radius:4px;padding:5px 8px}canvas{background:#18222d;border:1px solid #475569;max-width:100%;height:auto}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(350px,1fr));gap:15px}#summary,#pairs{white-space:pre-wrap;color:#cbd5e1}.legend{font-size:13px;color:#cbd5e1}
</style><body>
<h1>PHT cumulative-submap matching inspection</h1>
<p>Left: prior-only cumulative occupancy and target PHT segments. Center: current occupancy and PHT segments. Right: actual online matched pairs. Cyan is current, magenta is target, yellow connects matched S# ↔ T#. The heading states whether a factor was applied.</p>
<div class="controls"><button id="prev">Previous matched frame</button><button id="next">Next matched frame</button><label>Keyframe <input id="slider" type="range" min="0" value="0"></label><span id="label"></span></div>
<div id="summary"></div><div class="grid"><section><h2>Prior cumulative 2D occupancy</h2><canvas id="target" width="600" height="500"></canvas><div class="legend">White: prior occupied · Magenta: target PHT line T#</div></section><section><h2>Current 2D occupancy</h2><canvas id="source" width="600" height="500"></canvas><div class="legend">White: current occupied · Cyan: current PHT line S#</div></section><section><h2>Online PHT line matching</h2><canvas id="match" width="600" height="500"></canvas><div class="legend">Thick lines and yellow connectors identify the accepted online pairs.</div></section></div><div id="pairs"></div>
<script>
const records=__PAYLOAD__,slider=document.querySelector('#slider'),label=document.querySelector('#label');slider.max=records.length-1;
const matched=records.map((r,i)=>r.event&&r.event.pairs&&r.event.pairs.length?i:null).filter(x=>x!==null);
function pts(lines){return(lines||[]).flatMap(l=>[[l[0],l[1]],[l[2],l[3]]])}
function bnd(points){if(!points.length)return[-1,1,-1,1];let xs=points.map(p=>p[0]),ys=points.map(p=>p[1]),m=Math.max(.5,(Math.max(...xs)-Math.min(...xs))*.12,(Math.max(...ys)-Math.min(...ys))*.12);return[Math.min(...xs)-m,Math.max(...xs)+m,Math.min(...ys)-m,Math.max(...ys)+m]}
function pr(p,b,c){return[(p[0]-b[0])/(b[1]-b[0])*c.width,c.height-(p[1]-b[2])/(b[3]-b[2])*c.height]}
function clear(c){let x=c.getContext('2d');x.clearRect(0,0,c.width,c.height);return x}
function cells(x,p,b){x.fillStyle='#e2e8f0';for(const q of p||[]){let a=pr(q,b,x.canvas);x.fillRect(a[0]-1,a[1]-1,2,2)}}
function lines(x,L,b,color,prefix,focus=[]){x.strokeStyle=color;x.fillStyle=color;x.lineWidth=2;(L||[]).forEach((l,i)=>{let a=pr([l[0],l[1]],b,x.canvas),z=pr([l[2],l[3]],b,x.canvas);x.lineWidth=focus.includes(i)?5:2;x.beginPath();x.moveTo(...a);x.lineTo(...z);x.stroke();if(!focus.includes(i))x.fillText(prefix+(i+1),(a[0]+z[0])/2+3,(a[1]+z[1])/2-3)})}
function draw(){let r=records[+slider.value],e=r.event||{},sl=r.pht_map_lines||[],tl=e.target_pht_lines||[],pairs=e.pairs||[];label.textContent=r.frame+' / '+records[records.length-1].frame;let B=bnd([...r.occupancy||[],...e.target_occupancy||[],...pts(sl),...pts(tl)]),si=new Set(pairs.map(p=>p.source)),ti=new Set(pairs.map(p=>p.target));let a=clear(document.querySelector('#target'));cells(a,e.target_occupancy,B);lines(a,tl,B,'#e879f9','T',[...ti]);a=clear(document.querySelector('#source'));cells(a,r.occupancy,B);lines(a,sl,B,'#22d3ee','S',[...si]);a=clear(document.querySelector('#match'));lines(a,tl,B,'#e879f9','T',[...ti]);lines(a,sl,B,'#22d3ee','S',[...si]);for(const p of pairs){let s=sl[p.source],t=tl[p.target];if(!s||!t)continue;let sc=[(s[0]+s[2])/2,(s[1]+s[3])/2],tc=[(t[0]+t[2])/2,(t[1]+t[3])/2],u=pr(sc,B,a.canvas),v=pr(tc,B,a.canvas);a.strokeStyle='#fbbf24';a.setLineDash([3,3]);a.beginPath();a.moveTo(...u);a.lineTo(...v);a.stroke();a.setLineDash([]);a.fillStyle='#fbbf24';a.fillText('S'+(p.source+1)+'↔T'+(p.target+1),(u[0]+v[0])/2+3,(u[1]+v[1])/2-3)}document.querySelector('#summary').textContent='frame '+r.frame+' · mode '+(e.mode||'')+' · factor applied '+(e.applied?'yes':'no')+' · '+(e.reason||'')+'\nsource PHT '+sl.length+' · target PHT '+tl.length+' · accepted pairs '+pairs.length;document.querySelector('#pairs').textContent=pairs.length?pairs.map(p=>'S'+(p.source+1)+' ↔ T'+(p.target+1)+' | angle '+p.angle_deg.toFixed(1)+'° · normal '+p.normal_m.toFixed(3)+' m · overlap '+p.overlap_m.toFixed(2)+' m').join('\n'):'No accepted pair'}
function move(d){if(!matched.length)return;let x=matched.indexOf(+slider.value);if(x<0)x=d>0?0:matched.length-1;else x=(x+d+matched.length)%matched.length;slider.value=matched[x];draw()}slider.oninput=draw;document.querySelector('#prev').onclick=()=>move(-1);document.querySelector('#next').onclick=()=>move(1);if(matched.length)slider.value=matched[0];draw()
</script></body></html>"""
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(html.replace("__PAYLOAD__", payload))
    print(args.output)


if __name__ == "__main__":
    main()
