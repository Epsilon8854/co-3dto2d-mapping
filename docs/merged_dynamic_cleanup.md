# Dynamic cleanup in `run_two_mid360_2d_mapping.sh`

## Why the object remained in the default RViz view

The two-live script opens RViz with `/toy_record/merged_global_occupancy`
enabled. The old merger rebuilt that topic from the two persistent per-robot
global maps and resolved every free/occupied conflict as occupied. Consequently,
a cell that remained occupied in either robot's old global map could never be
cleared by fresh free-space observations from the other robot.

The per-robot temporal filter and the merged-map filter solve different
problems:

- `/rN/toy/global_occupancy` removes trails observed again by robot `N`;
- `/toy_record/merged_global_occupancy` must also accept fresh observations from
  either robot without re-inserting the other robot's stale global cell.

## New merged-map behavior

The executable name remains `record_republisher.py`, so existing launch files
and the run script do not change. Its installed implementation now:

1. uses each per-robot global map at most once as startup bootstrap data;
2. consumes `/r0/toy/local_occupancy` and `/r1/toy/local_occupancy` as
   current-frame observation grids;
3. ignores duplicate local-grid publications with the same sensor timestamp;
4. transforms robot 1 observations through the current inter-robot alignment;
5. counts free and occupied evidence in the common `map` frame;
6. clears an occupied merged cell after four free observation frames;
7. requires three occupied observation frames before inserting a new obstacle;
8. expands free observations by one 5 cm Manhattan cell by default so small
   pose/grid quantization changes do not prevent all four observations from
   reaching the same trail cell;
9. gives occupied priority only when free and occupied are both present in the
   same new observation frame, not forever because of an old global map.

A significant inter-robot alignment change resets and re-seeds the temporal
merged map because old robot-1 cells would otherwise be expressed in the wrong
common-frame coordinates.

## Required rebuild

`run_two_mid360_2d_mapping.sh` launches files and executables from the
workspace's `install/` directory. Pulling or switching the source branch alone
does not update the running mapper. Rebuild the package on both robot laptops:

```bash
cd /path/to/co-3dto2d-mapping
git switch feature/dynamic-filter-local-window-icp
git pull
rm -rf build/co_3dto2d_mapping install/co_3dto2d_mapping
colcon build --packages-select co_3dto2d_mapping --symlink-install
source install/local_setup.bash
```

On the mapping-host laptop, startup must include a line similar to:

```text
Temporal merged occupancy enabled=true source=local_observation free_clear=4 occupied_confirm=3 free_inflation=0.050m
```

Each robot's mapper must also print:

```text
dynamic_filter=true free_clear=4 occupied_confirm=3
```

If either line is absent, an older `install/` tree is still being executed.

During operation, the mapping host periodically reports the number of current
free/occupied observations and the number of merged occupied cells actually
cleared:

```text
Temporal merged occupancy update r0: observed=... free=... occupied=... cleared=... (... total)
```

For diagnosis, enable `R0 Global Occupancy` and `R1 Global Occupancy` in RViz.
This separates per-robot ray coverage from common-frame fusion behavior.
