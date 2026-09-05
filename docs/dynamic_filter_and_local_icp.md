# Temporal dynamic-object filtering and local-window ICP

This mapper now applies two corrections before a scan is committed to the global
occupancy grid:

1. planar scan-to-submap ICP refines the raw odometry pose against a local
   submap built from the most recent corrected scans;
2. temporal occupancy evidence prevents a one-frame obstacle from becoming a
   permanent wall and clears an occupied cell after repeated free-space rays.

Both features are opt-in in `config/occupancy.yaml` and are applied
independently per mapper instance. With local-window ICP disabled, each mapper
uses the incoming odometry pose directly and does not retain a scan submap.

## Temporal occupancy evidence

Free-space and occupied evidence is counted at most once per grid cell per
mapping frame. This is important because a dense LiDAR scan can send hundreds
of rays through the same cell; those rays must still count as one temporal
observation rather than clearing a cell immediately. Stable evidence uses a
compact 8-byte record per cell, while frame-local observations are reset only
for cells touched by that frame so large global maps do not require a full-grid
clear on every scan.

The state transition uses hysteresis:

- an unknown cell becomes free on the first valid free-space observation;
- a free or unknown cell becomes occupied after
  `dynamic_occupied_confirm_count` occupied frames;
- an established occupied cell becomes free after
  `dynamic_free_clear_count` free frames;
- an opposite observation reduces pending evidence by
  `dynamic_counter_decay`;
- evidence is reset after `dynamic_evidence_timeout_frames` frames without an
  observation, so stale history does not dominate a much later revisit.

Default values are four free frames and three occupied frames. At 10 Hz this
usually removes a departed pedestrian or cart after about 0.4 seconds while
rejecting isolated returns. Increase `dynamic_free_clear_count` if static walls
are erased by pose noise; decrease it to remove trails faster.

`raycast_clear_occupied` still controls the legacy behavior when
`dynamic_filter_enabled` is false. With the temporal filter enabled, clearing
is governed by the evidence counters instead of immediate ray clearing.

## Local-window ICP odometry refinement

When enabled, the current filtered 2D scan is voxel-downsampled and matched to
a submap made from up to `icp_window_size` recent corrected scans. It is disabled
by default, so raw odometry is used directly without maintaining the local scan
window. When enabled, ICP only adds a bounded correction to the odometry
prediction.

The matcher uses:

- a small correlative search around the odometry prediction;
- trimmed point-to-point ICP to reduce the influence of moving objects and
  outliers;
- overlap, correspondence count, RMSE, correction-size, and RMSE-improvement
  gates;
- interpolation through `icp_correction_gain` to avoid pose jitter;
- automatic window reset after tracking failure, non-monotonic timestamps, or
  a large raw-odometry jump.

When any quality gate fails, the mapper uses the odometry-predicted pose for
that frame. Mapping therefore continues without accepting an unsafe ICP jump.
The pose actually used for mapping is also published as
`toy/corrected_odometry` (`nav_msgs/msg/Odometry`) by default. The topic can be
renamed or disabled without publishing a second TF tree. Its twist and
covariance are copied from the synchronized input odometry; the corrected topic
is therefore intended primarily for map-pose inspection unless downstream
fusion explicitly models the ICP correction uncertainty.

### Main tuning parameters

| Parameter | Default | Effect |
| --- | ---: | --- |
| `enable_local_window_icp` | `false` | Enables scan-to-submap pose refinement. |
| `icp_window_size` | `10` | Number of corrected scans retained in the submap. |
| `icp_voxel_size_m` | `0.12` | 2D scan/submap downsampling resolution. |
| `icp_max_correspondence_distance_m` | `0.45` | Maximum nearest-neighbour distance. |
| `icp_trim_ratio` | `0.75` | Best correspondence fraction retained each iteration. |
| `icp_max_rmse_m` | `0.20` | Rejects a geometrically poor alignment. |
| `icp_max_correction_translation_m` | `0.35` | Rejects a large per-frame translational correction. |
| `icp_max_correction_yaw_deg` | `8.0` | Rejects a large per-frame yaw correction. |
| `icp_correction_gain` | `0.70` | Fraction of an accepted correction applied. |

For a mostly stationary startup, first tune `icp_max_correspondence_distance_m`
and `icp_max_rmse_m`. In a sparse corridor, a larger correspondence distance
may be necessary, but the correction bounds should remain conservative. If CPU
usage is high, raise `icp_voxel_size_m`, lower `icp_max_source_points`, or
reduce the coarse-search ranges.

## Runtime diagnostics

The mapper prints throttled messages for accepted/rejected ICP updates. An
accepted message includes source/submap sizes, correspondence count, overlap,
RMSE, and correction magnitude. Rejection messages show the same quality data
and confirm that the odometry prediction was used.

The temporal filter logs the number of occupied cells cleared in the current
frame and the number of transient occupied cells whose insertion was delayed.

## Suggested validation

1. Keep both robots fixed for at least 10 seconds after startup. Verify that ICP
   accepts small corrections or safely rejects them, and that the global map
   does not visibly walk with raw odometry noise.
2. Move a person or cart through an already observed free corridor. Verify that
   the object does not become occupied immediately and that any committed trail
   is cleared after roughly four free observations.
3. Place a new static obstacle in free space. It should appear after three
   consecutive occupied frames, demonstrating that the filter still admits
   persistent scene changes.
4. Force an odometry tracking failure or replay a bag from an earlier timestamp.
   Verify that the local ICP window resets instead of matching against stale
   scans.
