"""Rigid transform helpers for fixed-world mapping and continuous local odom."""
import numpy as np
from scipy.spatial.transform import Rotation


def rigid_matrix(position, quaternion):
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if position.shape != (3,) or quaternion.shape != (4,) or not np.all(np.isfinite(np.r_[position, quaternion])):
        raise ValueError('invalid rigid pose')
    if np.linalg.norm(quaternion) < 1e-8:
        raise ValueError('zero quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = position
    return matrix


def rigid_components(matrix):
    matrix = np.asarray(matrix, dtype=float)
    return matrix[:3, 3].tolist(), Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()


def world_from_odom(world_from_base, odom_from_base):
    """Same timestamp, same physical base: W_T_O = W_T_B * inverse(O_T_B)."""
    inverse = np.eye(4)
    inverse[:3, :3] = odom_from_base[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ odom_from_base[:3, 3]
    return world_from_base @ inverse
