#!/usr/bin/env python3
"""Render saved central-capture ablations; never fabricates ground truth."""
import argparse
import base64
import html
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.ndimage import binary_erosion


def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path)
    p.add_argument('--periodic-case',default='periodic_final_4s')
    args=p.parse_args();root=args.directory.resolve();periodic_case=args.periodic_case
    cases=[('baseline_count30','Fixed alignment / free=30 / range=6m'),
           (periodic_case,'Periodic alignment / free=30 / range=6m'),
           ('dynamic_filteroff','Dynamic filtering OFF'),('dynamic_count15','Fixed alignment / free=15 / range=6m'),
           ('dynamic_count10','Fixed alignment / free=10 / range=6m'),('dynamic_range10','Fixed alignment / free=30 / range=10m')]
    loaded={};summaries={}
    for name,label in cases:
        with np.load(root/name/'merged_global_map_data.npz') as z:
            d=z['data'];res=float(z['resolution']);o=z['origin']
        loaded[name]=(d,res,o)
        summaries[name]=json.loads((root/name/'summary.json').read_text())
    bounds=[(o[0],o[0]+d.shape[1]*r,o[1],o[1]+d.shape[0]*r) for d,r,o in loaded.values()]
    xlim=(min(v[0] for v in bounds),max(v[1] for v in bounds));ylim=(min(v[2] for v in bounds),max(v[3] for v in bounds))
    def draw(ax,name,label):
        d,r,o=loaded[name];rgb=np.where(d<0,.72,np.where(d>50,0.,1.))
        ax.imshow(rgb,cmap='gray',vmin=0,vmax=1,origin='lower',extent=[o[0],o[0]+d.shape[1]*r,o[1],o[1]+d.shape[0]*r],interpolation='nearest')
        ax.set(xlim=xlim,ylim=ylim,xlabel='map x [m]',ylabel='map y [m]',title=label);ax.set_aspect('equal');ax.grid(alpha=.15)
    fig,axs=plt.subplots(1,2,figsize=(13,7),layout='constrained')
    for ax,(name,label) in zip(axs,cases[:2]):draw(ax,name,label)
    fig.savefig(root/'periodic_comparison.png',dpi=180);plt.close(fig)
    fig,axs=plt.subplots(2,3,figsize=(15,12),layout='constrained')
    for ax,(name,label) in zip(axs.ravel(),cases):
        draw(ax,name,label);ax.text(.02,.02,f"occupied cells = {summaries[name]['final_occupied_cells']:,}",transform=ax.transAxes,
            bbox=dict(facecolor='white',alpha=.9,edgecolor='none'),fontsize=9)
    fig.savefig(root/'dynamic_ablation.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(13,7),layout='constrained')
    for ax,(name,label) in zip(axs,cases[:2]):
        draw(ax,name,label)
        for rid,color in ((0,'#f08634'),(1,'#00a8c4')):
            with np.load(root/name/f'r{rid}_layer_data.npz') as z:d=z['data'];r=float(z['resolution']);o=z['origin']
            b=(d>50)&~binary_erosion(d>50);y,x=np.nonzero(b)
            ax.scatter(o[0]+(x+.5)*r,o[1]+(y+.5)*r,s=1,c=color,label=f'r{rid} boundary',alpha=.6)
        ax.legend(fontsize=9)
    fig.savefig(root/'agent_boundary_overlay.png',dpi=180);plt.close(fig)
    probes=json.loads((root/periodic_case/'probes.json').read_text());t=np.array([r['elapsed_sec'] for r in probes])
    fig,axs=plt.subplots(3,1,figsize=(11,9),sharex=True,layout='constrained')
    for ax,key,scale,label in zip(axs,('rmse_m','overlap','free_conflict'),(100,100,100),('Inlier boundary RMSE [cm]','Boundary overlap [%]','Free / occupied conflict [%]')):
        ax.plot(t,[v['before'][key]*scale for v in probes],label='Fixed initial transform',color='#d48425')
        ax.plot(t,[v['current_transform_metrics_on_fixed_support'][key]*scale for v in probes],label='Periodic transform',color='#168baa')
        ax.set_ylabel(label);ax.grid(alpha=.25);ax.legend()
    axs[-1].set_xlabel('Source time after initial lock [s]');fig.savefig(root/'alignment_metrics.png',dpi=170);plt.close(fig)
    stats=np.load(root/'baseline_count30/retained_cell_diagnostics.npz');xy=stats['xy']
    # A repeat-occupied support proxy, not a semantic wall/obstacle GT label.
    stable=stats['occupied_observations']>=50
    changes=[]
    for name,label in cases:
        d,r,o=loaded[name];ij=np.floor((xy-o[:2])/r+1e-6).astype(int);x,y=ij.T
        valid=(x>=0)&(y>=0)&(x<d.shape[1])&(y<d.shape[0]);v=np.full(len(x),-1,np.int8);v[valid]=d[y[valid],x[valid]]
        changes.append(dict(case=name,occupied_cells=summaries[name]['final_occupied_cells'],
            baseline_occupied_now_free=int(np.count_nonzero(v==0)),repeated_occupied_proxy_cells=int(np.count_nonzero(stable)),
            repeated_occupied_proxy_now_free=int(np.count_nonzero(stable&(v==0)))))
    (root/'comparison_metrics.json').write_text(json.dumps(dict(cases=changes,
        late_window_count=10,late_metrics={k:dict(fixed=float(np.mean([v['before'][k] for v in probes[-10:]])),
        periodic=float(np.mean([v['current_transform_metrics_on_fixed_support'][k] for v in probes[-10:]])))
        for k in ('rmse_m','overlap','free_conflict')}),indent=2))
    data={key:stats[key].tolist() for key in ('rows','cols','final_free_streak','max_free_streak','occupied_observations',
        'free_observations','range_blocked','new_bin_free_votes','occupied_streak_resets','timeout_streak_resets','same_bin_occupied_blocks',
        'r0_occupied_observations','r1_occupied_observations','r0_free_observations','r1_free_observations','cleared_transitions','reoccupied_after_clear')}
    d,r,o=loaded['baseline_count30'];data.update(width=d.shape[1],height=d.shape[0],resolution=r,origin=o.tolist())
    image=base64.b64encode((root/'baseline_count30/merged_global_map.png').read_bytes()).decode()
    rows=''.join(f'<tr><td>{html.escape(label)}</td><td>{summaries[name]["final_occupied_cells"]:,}</td><td>{summaries[name]["worker_p95_ms"]:.1f}</td></tr>' for name,label in cases)
    document=r'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>실험 4 · 주기적 정합 / 동적 객체 비교</title><style>
body{font:16px/1.6 system-ui,sans-serif;max-width:1250px;margin:24px auto;padding:0 20px;background:#f7f8fa;color:#1b2633}
h1,h2{line-height:1.3}img{width:100%;height:auto}table{border-collapse:collapse;width:100%}td,th{text-align:left;padding:10px;border-bottom:1px solid #cbd1d9}
canvas{width:min(100%,780px);height:auto;image-rendering:pixelated;border:1px solid #ccd3dc}pre{white-space:pre-wrap}a{color:#00689c}
.inspect{display:flex;gap:24px;align-items:flex-start;flex-wrap:wrap}.inspect>div{flex:1;min-width:250px}li{margin:5px 0}
</style><h1>실험 4 · 주기적 정합과 동적 객체 필터</h1>
<ul><li>약 112초 녹화 중 초기 정합 이후 97.44초, local map 972건을 같은 source timestamp 순서로 처리.</li>
<li>ICP·단일 agent odometry는 고정. 초기 정합 C도 모든 조건에서 동일. 실시간 수신 순서까지 똑같이 복제한 결과는 아님.</li>
<li>map 원점·r0 지도 유지. r1 최근 3초 관측과 r0 과거 지도 일부를 2초마다 비교. 채택 시 전체 원본 관측을 새 C로 재투영.</li>
<li>ALIGNMENT_SUMMARY 큰 개선으로 보기 어려움. GT 없이 지도 간 일관성을 비교.</li></ul>
<h2>1. 전체 지도 · 같은 축 범위</h2><img src="periodic_comparison.png" alt="주기적 정합 전후 전체 지도">
<img src="agent_boundary_overlay.png" alt="r0와 r1 경계를 각기 다른 색으로 표시한 전체 지도">
<h2>2. 시간별 지도 간 일관성</h2><img src="alignment_metrics.png" alt="고정된 관측 지지 영역에서 측정한 경계 오차·겹침·충돌 지표">
<ul><li>LATE_METRICS</li>
<li>RUNTIME_SUMMARY 장시간 데이터에서는 재투영 비용이 늘어남.</li>
<li>단일 rigid C 갱신으로 각 agent 내부에 이미 휘어진 지도까지 바로잡을 수 없음.</li></ul>
<h2>3. 동적 객체 필터 파라미터 비교</h2><img src="dynamic_ablation.png" alt="동일 입력에서 필터 off·30회·15회·10회·10m 거리 제한 비교">
<table><thead><tr><th>조건</th><th>최종 occupied 셀</th><th>지도 worker p95 [ms]</th></tr></thead><tbody>TABLE_ROWS</tbody></table>
<ul><li>occupied 셀 감소는 정확도 상승을 뜻하지 않음. 실제 정적 물체까지 지워졌는지 함께 확인해야 함.</li>
<li>local map은 약 5Hz. 한 agent만 같은 셀을 관측하면 30회는 최선의 경우 약 6초, 15회 약 3초, 10회 약 2초가 필요함.</li>
<li>occupied 재관측 시 누적 초기화, 5초 관측 공백 시 누적 초기화, 6m 밖 free의 기존 occupied 삭제 제한이 실제로 동작.</li>
<li>권장: 거리 제한 6m와 occupied 초기화 유지 → 15회부터 시각 검토. 바로 10m로 확대하면 정적 구조 보존 위험도 함께 커짐.</li></ul>
<h3>실내 수평 흔적 후보 ROI: map x=3~6m, y=0.7~1.5m</h3>
<ul><li>기존 occupied 213셀 중 178셀은 실제 clear 후 occupied로 다시 바뀜. 마지막 occupied 관측의 중앙값은 종료 약 0.54초 전.</li>
<li>이 영역은 거리 제한에 막힌 free가 0건. r0·r1 모두 occupied를 공급함. free 갱신 누락이나 r1만의 문제로 보기 어려움.</li>
<li>남은 기준 셀: 30회 213 → 15회 170 → 10회 142. 거리만 10m로 확대하면 213으로 동일.</li>
<li>실제 빈 공간이었는지는 현장 확인 필요. 빈 공간이라면 투영된 occupied endpoint와 양쪽 odometry 오차를 먼저 점검하고, 강한 삭제보다 짧은 occupied 재확인/점유 신뢰도 정책을 별도 비교하는 것이 적절함.</li></ul>
<h2>4. 남은 occupied 셀의 실제 관측 기록</h2>
<p>기존 설정의 검은 셀을 클릭하면 해당 셀의 free 누적·거리 제한·occupied 재관측 횟수를 확인합니다. 벽도 포함된 통계이며 동적 객체 정답 표시는 아닙니다.</p>
<div class="inspect"><canvas id="cells" aria-label="남은 occupied 셀 관측 이력" role="img"></canvas><div><strong id="coord">셀을 선택하세요.</strong><pre id="details"></pre></div></div>
<h2>검증 범위와 다음 판단</h2><ul><li>원본 bag·다른 브랜치는 수정하지 않음. 기본 라이브 설정은 초기 정합 유지, 주기적 정합은 별도 alignment_periodic.yaml로 활성화.</li>
<li>같은 입력의 기존 지도 반복 결과 일치. 새 변환으로 전체 이력을 다시 처리한 지도와 temporal 상태 일치. r0 layer 불변 확인.</li>
<li>원본 timestamp 순서로 중앙 입력을 보낸 1배속 ROS 노드에서도 3회 보정, 최종 변환과 지도 셀 일치. live 네트워크의 지연·유실까지 재현했다는 의미는 아님.</li>
<li>새 목표 pose는 map 기준으로 유지되지만, 보정 후 지도에서 목표 셀의 주행 가능성은 탐사·주행 측에서 다시 확인해야 함.</li></ul>
<p><a href="comparison_metrics.json">정량 비교</a> · <a href="invariant_checks.json">불변 조건 확인</a> · <a href="bag_audit.json">bag 상세 검사</a></p>
<script>const data=INSPECT_DATA,cv=document.getElementById('cells'),ctx=cv.getContext('2d'),im=new Image();
cv.width=data.width;cv.height=data.height;const lookup=new Map(data.rows.map((r,i)=>[r*data.width+data.cols[i],i]));
im.onload=()=>ctx.drawImage(im,0,0);im.src='data:image/png;base64,IMAGE_DATA';
cv.addEventListener('click',e=>{const b=cv.getBoundingClientRect(),x=Math.floor((e.clientX-b.left)*cv.width/b.width),y=Math.floor((e.clientY-b.top)*cv.height/b.height),row=data.height-1-y;
let idx=lookup.get(row*data.width+x);if(idx===undefined){document.getElementById('details').textContent='남은 occupied 셀을 선택해주세요.';return}
ctx.drawImage(im,0,0);ctx.strokeStyle='#e03030';ctx.lineWidth=1;ctx.strokeRect(x-2,y-2,5,5);
document.getElementById('coord').textContent=`map x=${(data.origin[0]+(x+.5)*data.resolution).toFixed(2)}m, y=${(data.origin[1]+(row+.5)*data.resolution).toFixed(2)}m`;
const labels={final_free_streak:'마지막 free 누적',max_free_streak:'기간 중 최대 free 누적',occupied_observations:'occupied 관측',free_observations:'free 관측(거리 제한 포함)',range_blocked:'거리 제한으로 삭제 증거 제외',new_bin_free_votes:'새 시간 bin의 유효 free 증거',occupied_streak_resets:'occupied 재관측으로 누적 초기화',timeout_streak_resets:'5초 공백으로 누적 초기화',same_bin_occupied_blocks:'같은 bin의 occupied 우선 처리',r0_occupied_observations:'r0 occupied 관측',r1_occupied_observations:'r1 occupied 관측',r0_free_observations:'r0 free 관측',r1_free_observations:'r1 free 관측',cleared_transitions:'occupied → free 전환',reoccupied_after_clear:'clear 이후 occupied 복귀'};
document.getElementById('details').textContent=Object.entries(labels).map(([k,l])=>l+': '+data[k][idx]).join('\n');});
</script></html>'''
    per=summaries[periodic_case];reports=json.loads((root/periodic_case/'registration.json').read_text())
    accepted=[v for v in reports if v['accepted']];regtime=np.array([v['runtime_ms'] for v in reports])
    late={k:[float(np.mean([v[m][k] for v in probes[-10:]])) for m in ('before','current_transform_metrics_on_fixed_support')]
          for k in ('rmse_m','overlap','free_conflict')}
    alignment_text=f"정합 {per['registration_attempts']}회 중 {per['registration_accepted']}회 채택, 상대 yaw {np.degrees(per['initial_alignment'][2]):.3f}° → {np.degrees(per['final_alignment'][2]):.3f}°."
    late_text=f"후반 10개 관측 창: 경계 RMSE {late['rmse_m'][0]*100:.2f} → {late['rmse_m'][1]*100:.2f}cm, 겹침 {late['overlap'][0]*100:.1f} → {late['overlap'][1]*100:.1f}%, free/occupied 충돌 {late['free_conflict'][0]*100:.2f} → {late['free_conflict'][1]*100:.2f}%."
    runtime_text=f"주기적 검색 p95 {np.percentile(regtime,95):.1f}ms. 전체 과거 지도 재투영 " + (
        f"{min(v['reprojection_ms'] for v in accepted)/1000:.2f}~{max(v['reprojection_ms'] for v in accepted)/1000:.2f}초." if accepted else '채택 없음.')
    document=document.replace('ALIGNMENT_SUMMARY',alignment_text).replace('LATE_METRICS',late_text).replace('RUNTIME_SUMMARY',runtime_text)
    document=document.replace('TABLE_ROWS',rows).replace('INSPECT_DATA',json.dumps(data,separators=(',',':'))).replace('IMAGE_DATA',image)
    (root/'comparison.html').write_text(document)
    print(root/'comparison.html')

if __name__=='__main__':main()
