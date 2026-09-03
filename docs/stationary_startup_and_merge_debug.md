# Stationary startup and merged-map diagnostics

## Fixed failure modes

### 1. Stationary robots produced changing transforms

The previous policy needed spatially distinct keyframes from both robots before
locking.  With two robots parked side-by-side, legitimate motion keyframes do
not exist; only odometry random walk could create the extra evidence.

The public alignment executable now wraps the same CPU-only occupancy place
recognizer with a stationary startup policy:

1. Wait for both global maps, corrected odometry, and raw `/rN/odom`.
2. Require a stable two-second window using pose span and reported twist.
3. Freeze one corrected pose for each robot.
4. Register newly updated map snapshots around those frozen poses, at most once
   per second.
5. Keep every result private while tentative.
6. Publish `/toy/initial_xy_alignment` only after three snapshot estimates agree
   within 0.20 m and 3 degrees.
7. Keep the first accepted transform fixed; later place matches monitor it.

The side-by-side deployment assumes the robots start with similar headings.  The
stationary search therefore defaults to an expected map yaw of zero with a
35-degree limit and a 3 m translation limit.  Relevant overrides are:

```bash
export CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG=0
export CO3DTO2D_PLACE_INITIAL_MAX_YAW_DEVIATION_DEG=35
export CO3DTO2D_PLACE_INITIAL_MAX_TRANSLATION_M=3.0
export CO3DTO2D_PLACE_INITIAL_ESTIMATES=3
```

### 2. One robot could disappear from the merged map

An empty first global map could previously be marked as seeded.  Also, if local
observations arrived first, the robot could be marked live and never receive a
useful global bootstrap.  The result depended on callback order.

The new compositor evaluates every growing global-map message, but copies only
cells that are still unknown in the common fusion grid.  Therefore newly
explored regions are added continuously, while an old persistent occupied cell
cannot overwrite a cell that live local observations have already cleared.

### 3. Single-r1 operation was disconnected from `map`

The old compositor always published `map -> r0/odom` identity.  With only r1
running, r1 data had no path to the RViz fixed frame.

Reference selection is now:

```text
verified two-robot alignment -> map is r0/odom
only r0 usable              -> map is r0/odom
only r1 usable for 2 s      -> map is r1/odom temporarily
no usable robot             -> no phantom common-frame transform
```

When a verified alignment later arrives, temporal fusion is reset once and
rebuilt in canonical r0/map coordinates.

## Visualization

The supplied `rviz/two_robot_mapping.rviz` enables:

- merged occupancy;
- raw r0 and r1 global maps;
- blue r0 and orange r1 fusion-contribution points;
- blue/orange place boundaries after geometric registration;
- tentative/locked alignment arrow and text;
- r0 and r1 odometry expressed in the active common map frame;
- the complete TF tree.

Diagnostic topics:

```text
/toy/inter_robot_alignment_status
/toy/place_alignment/markers
/toy_record/fusion_status
/toy_record/fusion_markers
/toy_record/r0/odom_in_map
/toy_record/r1/odom_in_map
/toy_record/merged_global_occupancy
```

Expected startup state sequence:

```text
WAITING_INPUTS -> SETTLING -> WAITING_MAP_GROWTH
-> WAITING_STATIONARY -> STATIONARY_SEARCH -> TENTATIVE -> LOCKED
```

Useful checks:

```bash
ros2 topic echo /toy/inter_robot_alignment_status
ros2 topic echo /toy_record/fusion_status
ros2 topic hz /r0/toy/global_occupancy
ros2 topic hz /r1/toy/global_occupancy
ros2 topic hz /r0/toy/corrected_odometry
ros2 topic hz /r1/toy/corrected_odometry
ros2 topic echo /tf --once
```

`fusion_status.maps_received` distinguishes a missing DDS/map input from a map
that arrived but contributed no new cells.  `global_seed_added_cells` shows how
many newly explored cells each robot has actually added to the common grid.
