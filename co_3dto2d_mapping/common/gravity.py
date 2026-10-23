"""IMU gravity and fixed-height pose mathematics; no floor estimator."""
from __future__ import annotations
import math
from typing import Optional,Sequence,Tuple
import numpy as np
_EPS=1e-12

def normalize_vector(vector: Sequence[float]) -> Optional[np.ndarray]:
    result = np.asarray(vector, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(result)):
        return None
    length = float(np.linalg.norm(result))
    if length <= _EPS:
        return None
    return result / length

def vector_angle(left: Sequence[float], right: Sequence[float]) -> float:
    left_unit = normalize_vector(left)
    right_unit = normalize_vector(right)
    if left_unit is None or right_unit is None:
        return math.inf
    cosine = float(np.clip(np.dot(left_unit, right_unit), -1.0, 1.0))
    return math.acos(cosine)

def imu_timestamp_is_usable(cloud_stamp_ns: int, imu_stamp_ns: int, timeout_sec: float) -> bool:
    """Reject only IMU samples that are older than the cloud beyond timeout."""
    if timeout_sec <= 0.0 or cloud_stamp_ns <= 0 or imu_stamp_ns <= 0:
        return True
    return cloud_stamp_ns - imu_stamp_ns <= int(timeout_sec * 1000000000.0)

def normalize_quaternion(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    q = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm <= _EPS:
        raise ValueError('quaternion must be finite and non-zero')
    return q / norm

def quaternion_rotate(quaternion_xyzw: Sequence[float], vector: Sequence[float]) -> np.ndarray:
    """Rotate ``vector`` by an ``(x, y, z, w)`` quaternion."""
    (x, y, z, w) = normalize_quaternion(quaternion_xyzw)
    (vx, vy, vz) = np.asarray(vector, dtype=np.float64).reshape(3)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.asarray([vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)], dtype=np.float64)

def rotation_matrix_from_quaternion(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Return a 3x3 active rotation matrix from an ``(x, y, z, w)`` quaternion."""
    (x, y, z, w) = normalize_quaternion(quaternion_xyzw)
    return np.asarray([[1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)], [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)], [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)]], dtype=np.float64)

def up_from_world_orientation(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Return world +Z expressed in the oriented sensor frame.

    ROS REP-145 defines IMU orientation as the sensor frame orientation with
    respect to the world frame. Therefore world up in sensor coordinates is
    obtained by applying the inverse (conjugate) orientation.
    """
    q = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    inverse = np.asarray([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)
    up = normalize_vector(quaternion_rotate(inverse, (0.0, 0.0, 1.0)))
    if up is None:
        raise ValueError('could not derive up vector from orientation')
    return up

def roll_pitch_from_up(up_vector: Sequence[float]) -> Tuple[float, float]:
    """Recover ZYX roll and pitch from world-up expressed in body coordinates."""
    up = normalize_vector(up_vector)
    if up is None:
        raise ValueError('up vector must be finite and non-zero')
    roll = math.atan2(float(up[1]), float(up[2]))
    pitch = math.atan2(-float(up[0]), math.hypot(float(up[1]), float(up[2])))
    return (roll, pitch)

def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return a ROS-order ``(x, y, z, w)`` quaternion for ZYX Euler angles."""
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    (cr, sr) = (math.cos(half_roll), math.sin(half_roll))
    (cp, sp) = (math.cos(half_pitch), math.sin(half_pitch))
    (cy, sy) = (math.cos(half_yaw), math.sin(half_yaw))
    return np.asarray([sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy], dtype=np.float64)

def fixed_z_gravity_pose(*, fixed_base_link_height_m: float, up_vector: Sequence[float], icp_yaw_rad: float) -> Tuple[float, np.ndarray]:
    """Build a fixed-height pose from IMU world-up and an ICP yaw estimate."""
    if not math.isfinite(fixed_base_link_height_m):
        raise ValueError('fixed_base_link_height_m must be finite')
    if not math.isfinite(icp_yaw_rad):
        raise ValueError('icp_yaw_rad must be finite')
    (roll, pitch) = roll_pitch_from_up(up_vector)
    return (float(fixed_base_link_height_m), quaternion_from_rpy(roll, pitch, icp_yaw_rad))

def yaw_from_quaternion(quaternion_xyzw: Sequence[float]) -> float:
    (x, y, z, w) = normalize_quaternion(quaternion_xyzw)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def blend_unit_vectors(previous: Sequence[float], current: Sequence[float], gain: float) -> np.ndarray:
    gain = float(np.clip(gain, 0.0, 1.0))
    previous_unit = normalize_vector(previous)
    current_unit = normalize_vector(current)
    if previous_unit is None or current_unit is None:
        raise ValueError('vectors must be finite and non-zero')
    if float(np.dot(previous_unit, current_unit)) < 0.0:
        current_unit = -current_unit
    blended = normalize_vector((1.0 - gain) * previous_unit + gain * current_unit)
    if blended is None:
        return current_unit
    return blended
