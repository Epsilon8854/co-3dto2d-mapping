#!/usr/bin/env python3
"""Initial XY ICP with a physical relative-heading prior.

The underlying mapper can produce nearly symmetric occupancy submaps. Pure
geometric ICP may then score a 180-degree solution as highly as the physically
correct one. This wrapper keeps the existing alignment node API, but starts
ICP near the expected robot-to-robot yaw, constrains every iteration, and
rejects the final transform when it points the source robot in the opposite
hemisphere.
"""

from __future__ import annotations

import math
import os
from typing import Iterable, List, Optional, Tuple

import numpy as np
import rclpy
from scipy.spatial import cKDTree

from co_3dto2d_mapping.heading_constraint import (
    angular_distance,
    initial_yaw_candidates,
    registration_rank,
    yaw_within_prior,
)
from co_3dto2d_mapping.initial_xy_icp_alignment import InitialXyIcpAlignment


def _env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError("%s must be a boolean value" % name)


def _env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    return default if value is None else float(value)


def _env_degree_offsets(name: str, defaults: Iterable[float]) -> List[float]:
    value = os.environ.get(name)
    degree_values = list(defaults) if value is None else [
        float(item.strip()) for item in value.split(",") if item.strip()
    ]
    return [math.radians(item) for item in degree_values]


class HeadingConstrainedInitialXyIcpAlignment(InitialXyIcpAlignment):
    def __init__(self) -> None:
        super().__init__()

        # Environment-backed defaults make these tunable through the existing
        # shell runners without requiring a new launch argument surface.
        self.declare_parameter(
            "enforce_heading_prior",
            _env_bool("CO3DTO2D_ENFORCE_HEADING_PRIOR", True),
        )
        self.declare_parameter(
            "expected_yaw_rad",
            math.radians(
                _env_float("CO3DTO2D_EXPECTED_RELATIVE_YAW_DEG", 0.0)
            ),
        )
        self.declare_parameter(
            "max_yaw_deviation_rad",
            math.radians(
                _env_float("CO3DTO2D_MAX_RELATIVE_YAW_DEVIATION_DEG", 90.0)
            ),
        )
        self.declare_parameter(
            "initial_yaw_offsets_rad",
            _env_degree_offsets(
                "CO3DTO2D_INITIAL_YAW_OFFSETS_DEG",
                (0.0, -30.0, 30.0),
            ),
        )
        self.declare_parameter(
            "max_iteration_rotation_rad",
            math.radians(
                _env_float("CO3DTO2D_MAX_ICP_ROTATION_STEP_DEG", 60.0)
            ),
        )
        self.declare_parameter(
            "heading_prior_weight",
            _env_float("CO3DTO2D_HEADING_PRIOR_WEIGHT", 0.05),
        )

        self.enforce_heading_prior = bool(
            self.get_parameter("enforce_heading_prior").value
        )
        self.expected_yaw_rad = float(self.get_parameter("expected_yaw_rad").value)
        self.max_yaw_deviation_rad = float(
            self.get_parameter("max_yaw_deviation_rad").value
        )
        self.initial_yaw_offsets_rad = [
            float(value)
            for value in self.get_parameter("initial_yaw_offsets_rad").value
        ]
        self.max_iteration_rotation_rad = float(
            self.get_parameter("max_iteration_rotation_rad").value
        )
        self.heading_prior_weight = float(
            self.get_parameter("heading_prior_weight").value
        )

        if self.max_yaw_deviation_rad < 0.0 or self.max_yaw_deviation_rad > math.pi:
            raise ValueError("max_yaw_deviation_rad must be between 0 and pi")
        if self.max_iteration_rotation_rad < 0.0 or self.max_iteration_rotation_rad > math.pi:
            raise ValueError("max_iteration_rotation_rad must be between 0 and pi")
        if self.heading_prior_weight < 0.0:
            raise ValueError("heading_prior_weight must be non-negative")

        self._last_heading_rejection: Optional[str] = None
        self.get_logger().info(
            "Initial XY ICP heading prior: enabled=%s expected=%.1fdeg "
            "max_deviation=%.1fdeg offsets=%s max_step=%.1fdeg weight=%.3f"
            % (
                "true" if self.enforce_heading_prior else "false",
                math.degrees(self.expected_yaw_rad),
                math.degrees(self.max_yaw_deviation_rad),
                ",".join(
                    "%.1f" % math.degrees(value)
                    for value in self.initial_yaw_offsets_rad
                ),
                math.degrees(self.max_iteration_rotation_rad),
                self.heading_prior_weight,
            )
        )

    @staticmethod
    def _rotation_matrix(yaw: float) -> np.ndarray:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return np.asarray(
            [[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]],
            dtype=np.float64,
        )

    def _published_yaw(self, rotation: np.ndarray) -> float:
        raw_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        return -raw_yaw if self.invert_result else raw_yaw

    def _heading_allowed(self, rotation: np.ndarray) -> bool:
        return (
            not self.enforce_heading_prior
            or yaw_within_prior(
                self._published_yaw(rotation),
                self.expected_yaw_rad,
                self.max_yaw_deviation_rad,
            )
        )

    def _remember_heading_rejection(self, yaw: float, reason: str) -> None:
        self._last_heading_rejection = (
            "%s: candidate=%.1fdeg expected=%.1fdeg deviation=%.1fdeg limit=%.1fdeg"
            % (
                reason,
                math.degrees(yaw),
                math.degrees(self.expected_yaw_rad),
                math.degrees(angular_distance(yaw, self.expected_yaw_rad)),
                math.degrees(self.max_yaw_deviation_rad),
            )
        )

    def _run_constrained_icp_once(
        self,
        target: np.ndarray,
        source: np.ndarray,
        initial_rotation: np.ndarray,
        initial_translation: np.ndarray,
        initialization_label: str,
    ):
        tree = cKDTree(target)
        total_rotation = initial_rotation.astype(np.float64, copy=True)
        total_translation = initial_translation.astype(np.float64, copy=True)
        last_rmse = None

        if not self._heading_allowed(total_rotation):
            self._remember_heading_rejection(
                self._published_yaw(total_rotation),
                "initial yaw rejected",
            )
            return None

        for _ in range(self.max_iterations):
            transformed = source @ total_rotation.T + total_translation
            distances, indices = tree.query(transformed, k=1)
            mask = distances <= self.max_correspondence_distance
            correspondences = int(np.count_nonzero(mask))
            if correspondences < self.min_correspondences:
                return None

            matched_source = transformed[mask]
            matched_target = target[indices[mask]]
            delta_rotation, delta_translation = self._estimate_rigid_2d(
                matched_source, matched_target
            )
            yaw_delta = math.atan2(delta_rotation[1, 0], delta_rotation[0, 0])
            if (
                self.max_iteration_rotation_rad > 0.0
                and abs(yaw_delta) > self.max_iteration_rotation_rad
            ):
                self._remember_heading_rejection(
                    self._published_yaw(delta_rotation @ total_rotation),
                    "single ICP rotation step rejected",
                )
                return None

            proposed_rotation = delta_rotation @ total_rotation
            proposed_translation = (
                delta_rotation @ total_translation + delta_translation
            )
            if not self._heading_allowed(proposed_rotation):
                self._remember_heading_rejection(
                    self._published_yaw(proposed_rotation),
                    "ICP left the allowed heading hemisphere",
                )
                return None

            total_rotation = proposed_rotation
            total_translation = proposed_translation
            rmse = float(np.sqrt(np.mean(np.square(distances[mask]))))
            if (
                last_rmse is not None
                and abs(last_rmse - rmse) < 1e-5
                and np.linalg.norm(delta_translation)
                < self.convergence_translation_m
                and abs(yaw_delta) < self.convergence_rotation_rad
            ):
                break
            last_rmse = rmse

        transformed = source @ total_rotation.T + total_translation
        distances, _ = tree.query(transformed, k=1)
        mask = distances <= self.max_correspondence_distance
        correspondences = int(np.count_nonzero(mask))
        if correspondences < self.min_correspondences:
            return None
        rmse = float(np.sqrt(np.mean(np.square(distances[mask]))))
        fitness = float(correspondences) / float(len(source))
        return (
            total_rotation,
            total_translation,
            rmse,
            fitness,
            correspondences,
            initialization_label,
        )

    def _run_icp(self, target: np.ndarray, source: np.ndarray):
        self._last_heading_rejection = None
        if len(target) < self.min_correspondences or len(source) < self.min_correspondences:
            return None

        published_initial_yaws = initial_yaw_candidates(
            self.expected_yaw_rad,
            self.max_yaw_deviation_rad,
            self.initial_yaw_offsets_rad,
        )
        results: List[Tuple[tuple, tuple]] = []
        target_centroid = np.mean(target, axis=0)
        source_centroid = np.mean(source, axis=0)

        for published_initial_yaw in published_initial_yaws:
            raw_initial_yaw = (
                -published_initial_yaw
                if self.invert_result
                else published_initial_yaw
            )
            initial_rotation = self._rotation_matrix(raw_initial_yaw)
            translations = [("origin", np.zeros(2, dtype=np.float64))]
            if self.initialize_from_centroids:
                rotated_source_centroid = source_centroid @ initial_rotation.T
                centroid_translation = target_centroid - rotated_source_centroid
                if np.linalg.norm(centroid_translation) > 1e-9:
                    translations.append(("centroid", centroid_translation))

            for translation_label, initial_translation in translations:
                label = "yaw=%.1fdeg/%s" % (
                    math.degrees(published_initial_yaw),
                    translation_label,
                )
                result = self._run_constrained_icp_once(
                    target,
                    source,
                    initial_rotation,
                    initial_translation,
                    label,
                )
                if result is None:
                    continue
                rank = registration_rank(
                    fitness=result[3],
                    rmse=result[2],
                    correspondences=result[4],
                    yaw=self._published_yaw(result[0]),
                    expected_yaw=self.expected_yaw_rad,
                    max_deviation=self.max_yaw_deviation_rad,
                    heading_prior_weight=self.heading_prior_weight,
                    enforce_prior=self.enforce_heading_prior,
                )
                if rank is not None:
                    results.append((result, rank))

        if not results:
            return None
        return max(results, key=lambda item: item[1])[0]

    def _set_alignment_from_result(self, result, stamp, label: str) -> str:
        if result is None and self._last_heading_rejection is not None:
            self.pending_candidates.clear()
            self.get_logger().warn(
                "%s XY ICP rejected by heading prior; %s. It will be retried."
                % (label, self._last_heading_rejection)
            )
            return "rejected"

        if result is not None and not self._heading_allowed(result[0]):
            self.pending_candidates.clear()
            yaw = self._published_yaw(result[0])
            self._remember_heading_rejection(yaw, "final safety check rejected")
            self.get_logger().warn(
                "%s XY ICP rejected by heading prior; %s. It will be retried."
                % (label, self._last_heading_rejection)
            )
            return "rejected"

        return super()._set_alignment_from_result(result, stamp, label)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadingConstrainedInitialXyIcpAlignment()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
