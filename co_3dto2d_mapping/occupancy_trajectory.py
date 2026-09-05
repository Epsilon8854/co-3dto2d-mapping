"""Accumulate and transform trajectories for occupancy PNG overlays."""

from __future__ import annotations

import math
from typing import Final, List, NamedTuple, Optional, Tuple

from co_3dto2d_mapping.occupancy_png import (
    OccupancyGridEncodingError,
    OccupancyGridGeometry,
    TrajectoryOverlay,
    trajectory_pixels,
)


PlanarTransform = Tuple[float, float, float]
RgbColor = Tuple[int, int, int]
WorldPoint = Tuple[float, float]


class MapOutput(NamedTuple):
    """One occupancy-grid input and its base and trajectory image outputs."""

    parameter_name: str
    key: str
    filename: str
    trajectory_filename: str
    trajectory_sources: Tuple[Tuple[str, RgbColor], ...]


MAP_OUTPUTS: Final[Tuple[MapOutput, ...]] = (
    MapOutput(
        "robot0_map_topic",
        "r0",
        "r0_global_occupancy.png",
        "r0_global_occupancy_trajectory.png",
        (("r0", (255, 64, 64)),),
    ),
    MapOutput(
        "robot1_map_topic",
        "r1",
        "r1_global_occupancy.png",
        "r1_global_occupancy_trajectory.png",
        (("r1", (64, 128, 255)),),
    ),
    MapOutput(
        "merged_map_topic",
        "merged",
        "merged_global_occupancy.png",
        "merged_global_occupancy_trajectories.png",
        (("r0_merged", (255, 64, 64)), ("r1_merged", (64, 128, 255))),
    ),
)


class TrajectoryStore:
    """Mutable odometry history bounded by a fixed number of route points."""

    def __init__(self, max_points: int) -> None:
        if max_points < 2:
            raise OccupancyGridEncodingError(
                reason="max_trajectory_points must be at least two"
            )
        self._max_points = max_points
        self._robot0: List[WorldPoint] = []
        self._robot1: List[WorldPoint] = []
        self._robot0_merged: List[WorldPoint] = []
        self._robot1_merged: List[WorldPoint] = []
        self._robot1_alignment: Optional[PlanarTransform] = None

    def add_robot0(self, point: WorldPoint) -> bool:
        """Append one r0 point in both its local and merged map coordinates."""
        local_changed = self._append(self._robot0, point)
        merged_changed = self._append(self._robot0_merged, point)
        return local_changed or merged_changed

    def add_robot1(self, point: WorldPoint) -> bool:
        """Append r1 locally and in the merged frame after startup ICP is known."""
        local_changed = self._append(self._robot1, point)
        if self._robot1_alignment is None:
            return local_changed
        merged_changed = self._append(
            self._robot1_merged,
            self._apply_alignment(point, self._robot1_alignment),
        )
        return local_changed or merged_changed

    def set_robot1_alignment(self, alignment: PlanarTransform) -> None:
        """Rebuild r1's merged path in the fixed startup-ICP map frame."""
        self._robot1_alignment = alignment
        self._robot1_merged = [
            self._apply_alignment(point, alignment) for point in self._robot1
        ]

    def overlays_for(
        self, output: MapOutput, geometry: OccupancyGridGeometry
    ) -> Tuple[TrajectoryOverlay, ...]:
        """Return every colored path that belongs on one occupancy-grid output."""
        paths = {
            "r0": self._robot0,
            "r1": self._robot1,
            "r0_merged": self._robot0_merged,
            "r1_merged": self._robot1_merged,
        }
        return tuple(
            TrajectoryOverlay(
                points=trajectory_pixels(geometry=geometry, points=paths[source]),
                color=color,
            )
            for source, color in output.trajectory_sources
        )

    def _append(self, trajectory: List[WorldPoint], point: WorldPoint) -> bool:
        if trajectory and trajectory[-1] == point:
            return False
        trajectory.append(point)
        if len(trajectory) > self._max_points:
            del trajectory[: len(trajectory) - self._max_points]
        return True

    @staticmethod
    def _apply_alignment(
        point: WorldPoint, alignment: PlanarTransform
    ) -> WorldPoint:
        translation_x, translation_y, yaw = alignment
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        return (
            translation_x + cosine * point[0] - sine * point[1],
            translation_y + sine * point[0] + cosine * point[1],
        )
