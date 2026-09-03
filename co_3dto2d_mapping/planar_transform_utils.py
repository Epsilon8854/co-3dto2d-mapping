"""Small, dependency-free helpers for composing planar frame transforms.

A transform named ``a_from_b`` maps coordinates expressed in frame ``b`` into
frame ``a``.  ``compose_planar(a_from_b, b_from_c)`` therefore returns
``a_from_c``.
"""

from __future__ import annotations

import math
from typing import Tuple


PlanarTransform = Tuple[float, float, float]


def normalize_angle(angle: float) -> float:
    """Normalize an angle to ``[-pi, pi]`` using a numerically stable form."""

    return math.atan2(math.sin(angle), math.cos(angle))


def compose_planar(
    parent_from_middle: PlanarTransform,
    middle_from_child: PlanarTransform,
) -> PlanarTransform:
    """Compose two SE(2) transforms.

    ``parent_from_middle`` is applied after ``middle_from_child``.
    """

    ax, ay, ayaw = parent_from_middle
    bx, by, byaw = middle_from_child
    cos_yaw = math.cos(ayaw)
    sin_yaw = math.sin(ayaw)
    return (
        ax + cos_yaw * bx - sin_yaw * by,
        ay + sin_yaw * bx + cos_yaw * by,
        normalize_angle(ayaw + byaw),
    )


def invert_planar(parent_from_child: PlanarTransform) -> PlanarTransform:
    """Return the inverse SE(2) transform."""

    x, y, yaw = parent_from_child
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -cos_yaw * x - sin_yaw * y,
        sin_yaw * x - cos_yaw * y,
        normalize_angle(-yaw),
    )


def world_from_source_odom(
    target_odom_from_target_base: PlanarTransform,
    target_base_from_source_base: PlanarTransform,
    source_odom_from_source_base: PlanarTransform,
) -> PlanarTransform:
    """Convert a base-to-base registration into an odom-to-odom transform.

    The common/world frame is chosen to coincide with the target robot's odom
    frame.  The returned transform maps the source robot's odom coordinates into
    that common frame::

        target_odom_T_source_odom =
            target_odom_T_target_base
            * target_base_T_source_base
            * inverse(source_odom_T_source_base)

    This is the step that prevents startup ICP from implicitly treating both
    odometry poses as identity.
    """

    return compose_planar(
        target_odom_from_target_base,
        compose_planar(
            target_base_from_source_base,
            invert_planar(source_odom_from_source_base),
        ),
    )


def transform_pose(
    parent_from_middle: PlanarTransform,
    middle_from_child: PlanarTransform,
) -> PlanarTransform:
    """Alias for composing a frame transform with a planar pose."""

    return compose_planar(parent_from_middle, middle_from_child)
