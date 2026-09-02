# CPU-only inter-robot 2D place recognition and map alignment

## Purpose

The two robots continue to build their own occupancy maps in independent
`r0/odom` and `r1/odom` frames.  The fusion host estimates one planar transform

```text
T_map0_map1 = (x, y, yaw)
```

that maps robot 1's odometry/map frame into robot 0's frame.  No pose graph is
created and neither robot's historical trajectory is optimized.

The public executable used by the existing two-live launch remains
`initial_xy_icp_alignment.py`; CMake now installs the place-recognition node
under that name.  Therefore `scripts/run_two_mid360_2d_mapping.sh` starts the
new method without changing the operator workflow.  The previous heading-
constrained whole-map ICP remains installed as
`legacy_initial_xy_icp_alignment.py` for comparison.

## Runtime data flow

```text
/r0/toy/global_occupancy -----+       +--> polar occupancy descriptor
/r0/toy/corrected_odometry ---+--> r0 keyframes
                                      \
                                       +--> descriptor candidates
                                      /     --> 2D geometric verification
/r1/toy/global_occupancy -----+--> r1 keyframes
/r1/toy/corrected_odometry ---+       +--> robust SE(2) consensus
                                                   |
                                                   +--> /toy/initial_xy_alignment
                                                   +--> /toy/inter_robot_relative_transform
                                                   +--> /toy/inter_robot_alignment_status
```

`record_republisher.py` already subscribes to `/toy/initial_xy_alignment` and
uses it to publish the common TF tree and merged occupancy map.  The place
recognition node therefore does not modify either local mapper and never feeds
the merged map back into local SLAM.

## Algorithm

### 1. Frozen local occupancy keyframes

A keyframe is generated when corrected odometry moves by either the configured
translation or rotation threshold.  A circular local patch is resampled from
the current global occupancy grid around that pose.  The patch is expressed in
the keyframe/body frame, not in the robot's map frame.

Unknown cells remain unknown.  Occupied boundary cells, known-free cells, and a
known-space mask are stored separately.  A candidate is deferred when the
known-space ratio or occupied-boundary count is too small.

Default keyframe settings:

```text
translation                 1.0 m
rotation                    10 deg
submap radius               15 m
submap resolution           0.10 m
minimum known ratio         0.12
minimum boundary points     120
```

### 2. Two-channel polar occupancy descriptor

The local patch is divided into 20 radial rings and 60 angular sectors.  Each
bin stores two values:

```text
occupied channel = normalized log(1 + occupied-boundary count)
free channel     = known-free cells / known cells
```

A validity mask excludes bins dominated by unknown space.  Sector averaging
produces a rotation-invariant ring key.  Ring-key distance selects a small
candidate set, after which the full descriptor is circularly shifted through
all sectors.  The best shift directly supplies an initial source-to-target yaw.

This is the 2D occupancy analogue of Scan Context: the ring key is used for
fast retrieval, while circular sector matching estimates orientation.  Unlike
a binary occupied-only descriptor, the second channel penalizes geometrically
similar walls whose observed free-space topology is incompatible.

### 3. Coarse-to-fine 2D geometric verification

A descriptor match is only a candidate.  It is verified using the frozen local
submaps:

1. Compute a Euclidean distance transform of the target occupied-boundary map.
2. Search translation and yaw around the descriptor yaw on a coarse grid.
3. Search again around the best coarse pose on a finer grid.
4. Refine the result with trimmed point-to-point ICP.
5. Evaluate both source-to-target and target-to-source directions.

The correlative-search cost contains boundary distance, an unknown-space
penalty, an occupied-to-known-free contradiction penalty, and an overlap
reward.  Free-space conflict is only counted beyond a clearance band around a
wall so normal rasterization error is not mistaken for a contradiction.

The final match must satisfy all of the following:

```text
minimum symmetric overlap       0.35
maximum symmetric RMSE          0.20 m
maximum free-space conflict     0.10
minimum symmetric correspondences 60
```

### 4. Map-frame transform from a place pair

The registration result maps robot 1 keyframe coordinates into robot 0
keyframe coordinates:

```text
T_K0_K1
```

With each keyframe pose in its own robot map, the map transform is

```text
T_M0_M1 = T_M0_K0 * T_K0_K1 * inverse(T_M1_K1)
```

The code uses one explicit `Pose2` convention everywhere: `T_parent_child`
maps child-frame coordinates into the parent frame.  Unit tests verify this
composition direction.

### 5. PGO-free robust SE(2) consensus

Every verified place pair measures the same single unknown `T_M0_M1`.  The
measurements are clustered in `(x, y, yaw)` using translation and angular
thresholds.  The best cluster is averaged with quality weights:

```text
weight = descriptor_similarity
       * symmetric_overlap^2
       * exp(-(RMSE / 0.20 m)^2)
       * (1 - free_space_conflict)
```

Translation uses a weighted mean and yaw uses a weighted circular mean.  A
cluster is accepted only when it contains the configured number of distinct
keyframes from each robot; repeatedly evaluating the same keyframe pair does
not create artificial support.

The default is two consistent measurements involving two distinct keyframes
per robot.  After lock, the transform is held fixed by default so the merged
map and coordination frame do not jump.  Later verified measurements monitor
the lock.  Repeated inconsistent measurements change the status to `DEGRADED`
without silently moving the map.

### 6. Relative robot coordination pose

Once `T_M0_M1` is locked, the live relative base pose is computed as

```text
T_B0_B1(t) = inverse(T_M0_B0(t)) * T_M0_M1 * T_M1_B1(t)
```

and published as `/toy/inter_robot_relative_transform`.  The message parent is
`r0/base_link` and the child is `r1/base_link`.  It is a topic message, not a
second TF broadcaster, so it does not conflict with the existing TF tree.

## Build and run

```bash
cd <workspace>
colcon build --packages-select co_3dto2d_mapping --symlink-install
source install/setup.bash
```

Run the same two-robot commands as before.  Exactly one laptop should be the
fusion host:

```bash
# physical robot 1
scripts/run_two_mid360_2d_mapping.sh --robot-number 1

# physical robot 2 and fusion host
scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host
```

The fusion host waits for both occupancy maps and corrected odometry streams,
then waits the existing `alignment_startup_delay_sec` before creating place
keyframes.

Monitor the state and outputs:

```bash
ros2 topic echo /toy/inter_robot_alignment_status
ros2 topic echo /toy/initial_xy_alignment
ros2 topic echo /toy/inter_robot_relative_transform
ros2 topic echo /toy_record/merged_global_occupancy
```

The status state progresses through `SEARCHING`, `TENTATIVE`, and `LOCKED`.
`DEGRADED` means later verified measurements disagree with the retained lock.

## Tuning from the existing runner

The existing launch arguments still control startup and consensus:

```bash
scripts/run_two_mid360_2d_mapping.sh --robot-number 2 --mapping-host \
  --launch-arg alignment_startup_delay_sec:=5.0 \
  --launch-arg alignment_required_consistent_results:=3 \
  --launch-arg alignment_max_consistency_translation_m:=0.35 \
  --launch-arg alignment_max_consistency_rotation_rad:=0.06981317
```

Frequently tuned place-recognition defaults can also be changed before running
the script:

```bash
export CO3DTO2D_PLACE_KEYFRAME_TRANSLATION_M=0.8
export CO3DTO2D_PLACE_KEYFRAME_ROTATION_DEG=8
export CO3DTO2D_PLACE_SUBMAP_RADIUS_M=15
export CO3DTO2D_PLACE_SUBMAP_RESOLUTION_M=0.10
export CO3DTO2D_PLACE_MIN_DESCRIPTOR_SIMILARITY=0.45
export CO3DTO2D_PLACE_MIN_SYMMETRIC_OVERLAP=0.35
export CO3DTO2D_PLACE_MAX_SYMMETRIC_RMSE_M=0.20
export CO3DTO2D_PLACE_MIN_SUPPORTS=2
export CO3DTO2D_PLACE_MIN_DISTINCT_KEYFRAMES_PER_ROBOT=2
```

The existing heading-prior environment variables remain supported:

```bash
export CO3DTO2D_ENFORCE_HEADING_PRIOR=true
export CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG=0
export CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG=90
```

Disable the heading prior when the two robots may initialize with arbitrary
relative headings and the environment has enough non-symmetric structure.

## Scope and limitation

This design removes the global PGO cost, but it does not correct deformation
inside either robot's historical map.  It assumes each local map is reasonably
rigid and that the cross-map relationship is well represented by one planar
transform.  Local-window ICP and corrected odometry reduce drift before
keyframe creation.  On very long trajectories with significant non-rigid map
drift, separate frozen submap layers or a pose graph are still required to make
all distant overlaps simultaneously exact.
