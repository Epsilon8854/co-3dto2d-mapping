# CPU-only 2-D inter-robot place recognition

The live two-robot launch uses a centralized place-recognition front-end and
keeps each robot's local mapper independent. No pose graph is created.

```text
r0 global occupancy + planar odometry -- keyframes --\
                                                     polar descriptor retrieval
r1 global occupancy + planar odometry -- keyframes --/
                         -> symmetric correlative search
                         -> trimmed 2-D ICP
                         -> free/occupied contradiction check
                         -> repeated SE(2) consensus
                         -> /toy/initial_xy_alignment (map <- r1/odom)
                         -> existing world odometry and merged-map compositor
```

## Frame contract

Every keyframe patch is local to the robot pose at which it was frozen. A
verified keyframe match estimates `r0_keyframe <- r1_keyframe`. The node then
composes the two stored odometry poses:

```text
r0_odom_T_r1_odom =
    r0_odom_T_r0_keyframe
    * r0_keyframe_T_r1_keyframe
    * inverse(r1_odom_T_r1_keyframe)
```

With the default launch, `map` coincides with `r0/odom`; therefore the published
transform is `map <- r1/odom`. The existing record republisher uses the same
transform for `/toy_record/r1/odom`, TF and merged occupancy.

## Inputs and outputs

Inputs:

- `/r0/toy/global_occupancy`
- `/r1/toy/global_occupancy`
- `/r0/toy/planar_odometry`
- `/r1/toy/planar_odometry`

Outputs:

- `/toy/initial_xy_alignment` (`geometry_msgs/TransformStamped`)
- `/toy/place_recognition/status` (`std_msgs/String`, JSON)

The status state progresses through `SEARCHING`, `TENTATIVE`, and `LOCKED`.
The default requires three verified pair measurements supported by at least two
distinct keyframes from each robot. Once locked, the transform is held constant
so the merged map cannot blink because of a later noisy match.

## Descriptor

Each frozen submap is divided into polar rings and sectors. Every bin stores:

1. occupied-boundary density,
2. known-free ratio,
3. observed-area ratio.

The sector-averaged ring key provides rotation-invariant candidate retrieval.
Circular descriptor shifts provide one yaw hypothesis. A second yaw hypothesis
comes from the two keyframe odometry headings plus `expected_map_yaw_rad`; this
is especially useful when the robots start side-by-side, where the default map
yaw is zero.

## Geometric verification

Candidate verification is entirely CPU based:

1. symmetric distance-transform correlative search,
2. coarse-to-fine x/y/yaw refinement,
3. trimmed point-to-point ICP on occupied boundaries,
4. forward and reverse overlap/RMSE checks,
5. occupied-to-free and interior-free-to-occupied contradiction checks.

Only geometrically verified candidates enter consensus.

## Tuning

Parameters are in `config/place_recognition.yaml`.

For robots farther than 2.5 m apart at startup, increase
`registration_coarse_translation_range_m`. For an unknown initial relative
heading, increase `registration_coarse_yaw_range_rad` or enable
`try_opposite_descriptor_yaw`; both increase CPU use. In repeated corridors,
keep the free-space conflict check enabled and raise
`consensus_min_measurements` rather than loosening every verification threshold.

The default `descriptor_ratio_test` is 1.0 (disabled) because several stationary
startup keyframes are intentionally near-duplicates. Geometric verification and
multi-keyframe consensus provide the ambiguity rejection instead.

## Runtime checks

```bash
ros2 topic echo /toy/place_recognition/status
ros2 topic echo --once /toy/initial_xy_alignment
ros2 run tf2_ros tf2_echo map r1/odom
ros2 topic echo --once /toy_record/r1/odom --field header
```

After lock, `map -> r1/odom` should remain fixed while `map -> r1/base_link`
changes with robot odometry.
