# Experiment 4: recorded-input periodic alignment and temporal-filter study

## Scope

- Reuse each agent's recorded native mapping odometry and occupancy observations
  - No ICP recomputation or live sensor is required
- Compare fixed initial alignment against periodic **r1 → r0** alignment
  - The fixed `map` anchor and r0 poses are preserved
- Compare dynamic-filter parameters separately with the initial transform fixed
  - Fewer occupied cells alone does not prove a better map
- The reference branch and original bags are not modified
  - Result files remain under `outputs/` and are not committed

## Registration and reprojection

1. Keep startup full-circle registration unchanged
2. On absolute original-source-time boundaries every 2 seconds, take r1 observations from the preceding 3 seconds
    - Keep an r0 historical-map snapshot ending at that same boundary
3. Crop the r0 historical map within 8 m of the predicted r1 position
    - This is r0-only history, including previously explored rooms; no r1 contribution is used as its own reference
4. Match occupied boundaries in mutually observed support
    - Freeze the support mask before fitting
    - Search near the previous transform using three yaw starts, then trimmed SE(2) ICP on **2D map boundaries**, not raw LiDAR ICP
5. Require correspondence/overlap/RMSE/free-conflict/observability gates, a bounded correction, and at least 2% loss reduction for the full fit
    - Apply half the fitted correction; this damped step must still reduce loss and pass geometry gates
6. Rebuild a separate fusion state by replaying the disk journal of original native grids and timestamps under the proposed transform
    - This rebuilds cross-agent free streaks as well as occupancy
7. Commit the transform and rebuilt map only when the replay succeeds
    - `alignment_revision` changes; `coordinate_epoch` and the world anchor do not

- For fixed `A = map ← r0/mapping_odom`, updated `C = r0/mapping_odom ← r1/mapping_odom`, and native poses `P_i`:

```text
global_pose_r0 = A P_0
global_pose_r1 = A C P_1
```

- The independent pose/TF process consumes the new C
  - Mapping/search/IO never run inside that process
- A single C cannot correct a non-rigid distortion already present inside either agent's map
- ROS topic delivery is not a transactional multi-topic update
  - `alignment_revision` identifies a completed map rebuild; consumers should allow normal message-delivery latency
- The existing paths are reprojected under the committed C
  - A completed path job using an older revision is discarded

## Live configuration

- Operational default `config/multi_agent.yaml`: `central.periodic.enabled: false` (startup-only alignment)
- Opt in on **r0 only**:

```bash
cd ~/co-3dto2d-mapping
bash scripts/run_multi_agent.sh --role coordinator --robot-id 0 \
  --alignment-config "$PWD/config/experiment/experiment_04_multi_agent_mapping/alignment_periodic.yaml"
```

- r1 uses its existing launcher
  - No new remote raw-cloud subscriptions are introduced
- Pull and build after installing new Python modules/configs on another machine:

```bash
colcon build --symlink-install --packages-select co_3dto2d_mapping
source install/local_setup.bash
```

### Important limits

- `periodic.max_reprojection_sec: 4.0`: abandon an over-budget rebuild and keep the previous complete map/transform
  - Wait 10 registration intervals before retrying a failed rebuild; incremental mapping continues
- `periodic.journal_max_mb: 2048.0`: stop proposing further corrections if the source journal fills
  - Continue mapping under the last committed transform and expose the reason in status
  - Never silently truncate history
- Journal storage and complete-history replay grow with run length
  - This is a correctness-first experimental implementation, not evidence of bounded long-duration replay cost
  - Long missions need submap/checkpoint-based reprojection before this is made the default
- Online input loss, CPU time-budget rejection, or source-clock offsets can still change results
  - Fixed source windows remove worker-phase dependence, not those causes
- `map` goal coordinates stay fixed, but the navigation/exploration planner must revalidate a goal if corrected map geometry changes its traversability

## Offline comparison commands

- Use the capture directory containing `bag/`
- Always choose a new output directory; analysis refuses to overwrite an existing run

```bash
source /opt/ros/humble/setup.bash
python3 scripts/experiment/04_multi_agent_mapping/analyze_experiment_04_capture.py CAPTURE \
  --output outputs/experiment_04_multi_agent_mapping/analysis/NAME/baseline --probe
python3 scripts/experiment/04_multi_agent_mapping/analyze_experiment_04_capture.py CAPTURE \
  --output outputs/experiment_04_multi_agent_mapping/analysis/NAME/periodic --periodic --probe
python3 scripts/experiment/04_multi_agent_mapping/analyze_experiment_04_capture.py CAPTURE \
  --output outputs/experiment_04_multi_agent_mapping/analysis/NAME/free15 --free-count 15
```

- All comparisons reuse the recorded initial C, the same latest pre-lock global seeds, and the same native local maps ordered by original header timestamp and robot ID
- Duplicate/older local stamps do not count as new evidence
  - Unknown cells are not free observations
- `--probe` measures initial and current transform consistency on the same fixed support
  - It does not apply the probe candidate
- The replay is a deterministic offline counterfactual
  - It does not reproduce the exact network-arrival order and callback grouping of the live capture
- This comparison uses the recorded local maps (~5 Hz)
  - To study a different local-map generation cadence, regenerate maps from the filtered cloud/pose inputs instead

## Verify the actual ROS central node

```bash
source /opt/ros/humble/setup.bash
python3 scripts/experiment/04_multi_agent_mapping/replay_experiment_04_mapping_inputs.py CAPTURE \
  --output outputs/experiment_04_multi_agent_mapping/analysis/NAME/ros_periodic \
  --periodic --rate 0.5
```

- Runs a central node and source-ordered input publisher on localhost, default domain 174
  - Does not start drivers or contact live robots
- Publishes actual captured native poses/sessions, the fixed pre-lock seeds and native local grids with their original timestamps, plus one replay clock
- Uses recorded C rather than redoing startup search, so startup-candidate selection is not a confound
- Replays only the central-mapping stage
  - It does **not** provide the complete cloud/TF displays of the live RViz setup
- Playback publishers offer Reliable QoS and session identities are Transient Local
  - Recording QoS overrides are not suitable for playback into Reliable map/session subscriptions

## Dynamic evidence diagnostics

```bash
python3 scripts/experiment/04_multi_agent_mapping/diagnose_experiment_04_dynamic.py CAPTURE --baseline BASELINE_OUTPUT
```

- `retained_cell_diagnostics.npz` stores per-cell free/occupied observations, range-blocked evidence, streak resets, maximum and final free streak for cells still occupied in the baseline
- Categories overlap and include real walls
  - They are explanations of the filter state, not semantic dynamic-object ground truth
- At ~5 Hz local-map publication, 30 eligible observations from one agent require about 6 seconds at best; 15 about 3 seconds; 10 about 2 seconds
  - Unknown gaps, occupied reobservations and range restriction extend this
- Default central settings remain free=30, timeout=5 s, reliable clear range=6 m
  - Compare 15 before 10 or expanding the range; keep static-object preservation visible

## Artifacts and verification

- `config.json`: effective parameters, source hashes, bootstrap and ordering policy
- `summary.json`, `registration.json`, `probes.json`, `series.json`: timing, candidates/rejection reasons, committed transforms and consistency metrics
- `observations/`: compressed original grids and source timestamps; allows independent all-history reconstruction
- `temporal_evidence.npz`, maps, timeline PNGs and JSON: map and evidence state
- Central status exposes `alignment_revision`, `periodic_registration_enabled`, `geometry_reprojection_applied`, journal usage, runtime and failure reason
- Live run stores both initial and latest alignment files, plus each periodic decision
  - `save_status.json` carries the final C and temporal diagnostics
- Correctness checks: exact baseline repeat, independent full-journal replay equals the committed map **and** temporal arrays, unchanged r0 layer, actual ROS-node replay

## Study captured 2026-10-23

- Capture: `periodic_alignment_dynamic_objects_20261023_165603_706961` (~112 s, 28 topics)
- Complete minimum topic/type/static-TF audit
  - No source timestamp reversal
  - r0 local grids include 19 duplicate timestamps; r0 cloud maximum interval is 1.6 s
- Results and detailed interpretation are in the local `outputs/experiment_04_multi_agent_mapping/analysis/20261023_periodic_dynamic/comparison.html`
- Exploratory callback-phase results are retained separately; the final comparison uses fixed source windows
  - Do not use exploratory PNGs/numbers as the final result
- These recordings have no external pose/object GT; wall consistency and evidence statistics do not establish absolute localization accuracy or navigation safety
- Final fixed-window run: 47 attempts, 3 committed corrections, C yaw −8.176665° → −8.039954°
  - Late-window inlier RMSE 14.440 → 14.416 cm: improvement is small
- A 1x ROS central replay used the same recorded C/seeds and source-ordered inputs
  - Its committed C, final map cells and accepted frame counts matched the offline result (r0 485, r1 484)
  - This verifies the central implementation, not network robustness or the full ICP pipeline
- Interior trace candidate (map x=3–6 m, y=0.7–1.5 m): 213 remaining cells; 178 actually cleared then became occupied again
  - Both agents supplied occupied observations
  - No range-blocked free in this ROI
  - Actual emptiness at recording end still needs field confirmation
- Remaining baseline ROI cells: count30=213, count15=170, count10=142, clear range10m=213
  - In the whole map, lowering count to 15 also removed 67 cells with at least 50 occupied observations; that is a static-structure risk proxy, not a GT false-removal count
- Next dynamic work should inspect the recorded 3D occupied endpoints and their agent/pose/time first
  - A 2D free ray does not establish that the entire robot-height column is clear; ray-height aggregation and occupied confidence deserve separate evaluation before changing immediate occupied protection
