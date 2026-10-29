#!/usr/bin/env python3
"""Synchronize clouds/odometry, filter by mount height, fix z and IMU attitude."""
from __future__ import annotations
from collections import deque
from copy import deepcopy
import math
import threading
from typing import Optional,Tuple
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry,Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu,PointCloud2,PointField
from tf2_ros import Buffer,TransformListener
from co_3dto2d_mapping.common.gravity import (
    blend_unit_vectors,normalize_vector,quaternion_rotate,imu_timestamp_is_usable,
    up_from_world_orientation,rotation_matrix_from_quaternion,yaw_from_quaternion,fixed_z_gravity_pose,
)
from co_3dto2d_mapping.common.height_filter import filter_points_by_fixed_sensor_height
from co_3dto2d_mapping.common.planar_transform_utils import invert_planar


class GravityHeightFilter(Node):
    def __init__(self):
        super().__init__('gravity_height_filter')
        defaults=dict(pose_correction_enabled=True,height_filter_enabled=True,
            fixed_base_link_height_above_floor_m=.30,lidar_mount_height_above_floor_m=.30,
            input_cloud_topic='mapping/lidar_rear_filtered',imu_topic='mapping/imu_filtered',
            input_odometry_topic='odom',output_odometry_topic='mapping/floor_based_odometry',
            output_cloud_topic='mapping/plane_height_filtered',local_frame_id='base_link',output_frame_id='',
            anchor_output_to_initial_pose=False,
            gravity_source='orientation_then_acceleration',imu_timeout_sec=.30,
            acceleration_min_mps2=6.,acceleration_max_mps2=13.,acceleration_blend=.15,
            gravity_filter_gain=.25,gravity_hold_timeout_sec=.75,
            gravity_sync_queue_size=30,gravity_sync_slop_sec=.08,
            height_filter_min_m=.30,height_filter_max_m=1.20,filtered_cloud_min_points=20,
            orientation_stddev_rad=.03,debug_paths_enabled=True,debug_path_max_poses=10000,
            debug_path_publish_period_ms=250,debug_raw_path_topic='mapping/debug/raw_odometry_path',
            debug_fused_path_topic='mapping/debug/floor_based_odometry_path')
        for key,value in defaults.items():self.declare_parameter(key,value)
        p={key:self.get_parameter(key).value for key in defaults}
        self.enabled=p['pose_correction_enabled'];self.height_filter_enabled=p['height_filter_enabled']
        self.fixed_base_link_height_m=p['fixed_base_link_height_above_floor_m']
        self.lidar_mount_height_m=p['lidar_mount_height_above_floor_m']
        self.local_frame_id=p['local_frame_id'];self.output_frame_id=p['output_frame_id']
        self.anchor_output=p['anchor_output_to_initial_pose'];self._world_anchor=None
        for key in ('gravity_source','imu_timeout_sec','acceleration_min_mps2','acceleration_max_mps2','acceleration_blend',
                    'filtered_cloud_min_points','orientation_stddev_rad','debug_path_publish_period_ms'):
            setattr(self,key,p[key])
        self.filter_gain=p['gravity_filter_gain'];self.hold_timeout_sec=p['gravity_hold_timeout_sec']
        self.filter_min_height_m=p['height_filter_min_m'];self.filter_max_height_m=p['height_filter_max_m']
        if self.gravity_source not in ('orientation','acceleration','orientation_then_acceleration','blend'):
            raise ValueError('invalid gravity_source')
        if not 0<self.filter_gain<=1 or self.hold_timeout_sec<0 or not 0<=self.acceleration_blend<=1:
            raise ValueError('invalid gravity smoothing')
        if self.filter_min_height_m<0 or self.filter_max_height_m<=self.filter_min_height_m or self.lidar_mount_height_m<=0:
            raise ValueError('invalid fixed-height filter configuration')
        self._publisher=self.create_publisher(Odometry,p['output_odometry_topic'],10)
        self._height_cloud_publisher=self.create_publisher(PointCloud2,p['output_cloud_topic'],qos_profile_sensor_data)
        self._imu_lock=threading.Lock();self._latest_imu=None;self._warned_frames=set()
        self._imu_callback_group=MutuallyExclusiveCallbackGroup()
        self._imu_subscription=self.create_subscription(Imu,p['imu_topic'],self._imu_callback,qos_profile_sensor_data,
            callback_group=self._imu_callback_group)
        self._tf_buffer=Buffer();self._tf_listener=TransformListener(self._tf_buffer,self)
        self._cloud_subscriber=message_filters.Subscriber(self,PointCloud2,p['input_cloud_topic'],qos_profile=qos_profile_sensor_data)
        self._odom_subscriber=message_filters.Subscriber(self,Odometry,p['input_odometry_topic'],
            qos_profile=QoSProfile(depth=max(10,p['gravity_sync_queue_size']),reliability=ReliabilityPolicy.RELIABLE))
        self._synchronizer=message_filters.ApproximateTimeSynchronizer([self._cloud_subscriber,self._odom_subscriber],
            queue_size=p['gravity_sync_queue_size'],slop=p['gravity_sync_slop_sec'],allow_headerless=False)
        self._synchronizer.registerCallback(self._synchronized_callback)
        self._filtered_normal=None;self._last_gravity_stamp_ns=None
        self._raw_path_publisher=self.create_publisher(Path,p['debug_raw_path_topic'],10) if p['debug_paths_enabled'] else None
        self._fused_path_publisher=self.create_publisher(Path,p['debug_fused_path_topic'],10) if p['debug_paths_enabled'] else None
        self._raw_path_poses=deque(maxlen=p['debug_path_max_poses']);self._fused_path_poses=deque(maxlen=p['debug_path_max_poses'])
        self._last_debug_path_stamp_ns=None;self._last_debug_path_publish_ns=0
        self.get_logger().info('Fixed-height gravity mapping: height=[%.2f, %.2f]m, sensor=%.2fm, base=%.2fm' % (
            self.filter_min_height_m,self.filter_max_height_m,self.lidar_mount_height_m,self.fixed_base_link_height_m))

    def _clear_gravity_state(self):
        self._filtered_normal=None;self._last_gravity_stamp_ns=None

    def _synchronized_callback(self,cloud,odometry):
        up=self._up_from_imu(cloud.header.stamp)
        points=self._cloud_to_local(cloud)
        ns=self._stamp_ns(cloud.header.stamp)
        normal,_=self._accept_or_hold_gravity(up,ns)
        if points is not None and len(points):
            selected=points
            if self.height_filter_enabled:
                selected=filter_points_by_fixed_sensor_height(points,normal,self.lidar_mount_height_m,
                    self.filter_min_height_m,self.filter_max_height_m).points if normal is not None else None
            if selected is not None and (not self.height_filter_enabled or len(selected)>=self.filtered_cloud_min_points):
                self._height_cloud_publisher.publish(self._xyz_cloud(selected,cloud.header.stamp,self.local_frame_id))
        pose=odometry.pose.pose;q=pose.orientation
        values=[pose.position.x,pose.position.y,pose.position.z,q.x,q.y,q.z,q.w]
        try:yaw=yaw_from_quaternion((q.x,q.y,q.z,q.w))
        except ValueError:return
        if not np.all(np.isfinite(values)):return
        if self.anchor_output and self._world_anchor is None:
            if odometry.pose.covariance[0]>1000.0:return
            self._world_anchor=invert_planar((pose.position.x,pose.position.y,yaw))
            self.get_logger().info(f'Fixed {self.output_frame_id} anchor from initial odometry: {self._world_anchor}')
        output=deepcopy(odometry);output.header.stamp=cloud.header.stamp
        if self.output_frame_id:output.header.frame_id=self.output_frame_id
        if self.enabled:
            output.pose.pose.position.z=self.fixed_base_link_height_m
            if normal is not None:
                _,quat=fixed_z_gravity_pose(fixed_base_link_height_m=self.fixed_base_link_height_m,up_vector=normal,icp_yaw_rad=yaw)
                q=output.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,quat)
                output.pose.covariance[14]=0.
                output.pose.covariance[21]=output.pose.covariance[28]=self.orientation_stddev_rad**2
        raw_debug=odometry
        if self.anchor_output:
            output=self._to_world(output)
            raw_debug=self._to_world(odometry)
        self._publisher.publish(output);self._publish_debug_paths(raw_debug,output,ns)

    def _to_world(self, msg):
        """Apply the fixed initial XY/yaw anchor to coordinates and covariance."""
        out=deepcopy(msg);out.header.frame_id=self.output_frame_id
        x,y,yaw=self._world_anchor;c,s=math.cos(yaw),math.sin(yaw)
        p=out.pose.pose.position;px,py=p.x,p.y
        p.x=x+c*px-s*py;p.y=y+s*px+c*py
        q=out.pose.pose.orientation;qx,qy,qz,qw=q.x,q.y,q.z,q.w
        ch,sh=math.cos(yaw/2),math.sin(yaw/2)
        q.x=ch*qx-sh*qy;q.y=sh*qx+ch*qy
        q.z=ch*qz+sh*qw;q.w=ch*qw-sh*qz
        rotation=np.eye(6);rotation[:2,:2]=[[c,-s],[s,c]];rotation[3:5,3:5]=[[c,-s],[s,c]]
        out.pose.covariance=(rotation@np.asarray(msg.pose.covariance).reshape(6,6)@rotation.T).ravel().tolist()
        return out


    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1000000000 + int(stamp.nanosec)

    def _imu_callback(self, msg: Imu) -> None:
        with self._imu_lock:
            self._latest_imu = msg

    def _transform_direction(self, vector: np.ndarray, source_frame: str) -> Optional[np.ndarray]:
        direction = normalize_vector(vector)
        if direction is None:
            return None
        if not source_frame or source_frame == self.local_frame_id:
            return direction
        try:
            transform = self._tf_buffer.lookup_transform(self.local_frame_id, source_frame, Time())
        except Exception as exc:
            key = (self.local_frame_id, source_frame)
            if key not in self._warned_frames:
                self._warned_frames.add(key)
                self.get_logger().warn('Waiting for transform %s <- %s for gravity alignment: %s' % (self.local_frame_id, source_frame, exc))
            return None
        rotation = transform.transform.rotation
        transformed = quaternion_rotate((rotation.x, rotation.y, rotation.z, rotation.w), direction)
        return normalize_vector(transformed)

    def _up_from_imu(self, cloud_stamp) -> Optional[np.ndarray]:
        with self._imu_lock:
            imu = self._latest_imu
        if imu is None:
            return None
        cloud_stamp_ns = self._stamp_ns(cloud_stamp)
        imu_stamp_ns = self._stamp_ns(imu.header.stamp)
        if not imu_timestamp_is_usable(cloud_stamp_ns, imu_stamp_ns, self.imu_timeout_sec):
            return None
        orientation_up = None
        if imu.orientation_covariance[0] >= 0.0:
            try:
                orientation_up = up_from_world_orientation((imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w))
                orientation_up = self._transform_direction(orientation_up, imu.header.frame_id)
            except ValueError:
                orientation_up = None
        acceleration_up = None
        if imu.linear_acceleration_covariance[0] >= 0.0:
            acceleration = np.asarray([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z], dtype=np.float64)
            magnitude = float(np.linalg.norm(acceleration))
            if self.acceleration_min_mps2 <= magnitude <= self.acceleration_max_mps2:
                acceleration_up = self._transform_direction(acceleration, imu.header.frame_id)
        if self.gravity_source == 'orientation':
            return orientation_up
        if self.gravity_source == 'acceleration':
            return acceleration_up
        if self.gravity_source == 'orientation_then_acceleration':
            return orientation_up if orientation_up is not None else acceleration_up
        if orientation_up is None:
            return acceleration_up
        if acceleration_up is None:
            return orientation_up
        return blend_unit_vectors(orientation_up, acceleration_up, self.acceleration_blend)

    @staticmethod
    def _point_field(msg: PointCloud2, name: str) -> PointField:
        for field in msg.fields:
            if field.name == name:
                return field
        raise ValueError('PointCloud2 is missing field %r' % name)

    @staticmethod
    def _field_format(field: PointField, big_endian: bool) -> str:
        byte_order = '>' if big_endian else '<'
        if field.datatype == PointField.FLOAT32:
            return byte_order + 'f4'
        if field.datatype == PointField.FLOAT64:
            return byte_order + 'f8'
        raise ValueError('PointCloud2 xyz fields must be FLOAT32 or FLOAT64')

    def _cloud_xyz(self, msg: PointCloud2) -> np.ndarray:
        x_field = self._point_field(msg, 'x')
        y_field = self._point_field(msg, 'y')
        z_field = self._point_field(msg, 'z')
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [self._field_format(x_field, msg.is_bigendian), self._field_format(y_field, msg.is_bigendian), self._field_format(z_field, msg.is_bigendian)], 'offsets': [x_field.offset, y_field.offset, z_field.offset], 'itemsize': int(msg.point_step)})
        rows = []
        data = memoryview(msg.data)
        width = int(msg.width)
        height = int(msg.height)
        for row in range(height):
            offset = row * int(msg.row_step)
            rows.append(np.frombuffer(data, dtype=dtype, count=width, offset=offset))
        if not rows:
            return np.empty((0, 3), dtype=np.float64)
        array = np.concatenate(rows) if len(rows) > 1 else rows[0]
        points = np.column_stack((array['x'], array['y'], array['z']))
        points = np.asarray(points, dtype=np.float64)
        return points[np.all(np.isfinite(points), axis=1)]

    def _cloud_to_local(self, msg: PointCloud2) -> Optional[np.ndarray]:
        try:
            points = self._cloud_xyz(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return None
        if not msg.header.frame_id or msg.header.frame_id == self.local_frame_id:
            return points
        try:
            transform = self._tf_buffer.lookup_transform(self.local_frame_id, msg.header.frame_id, Time())
        except Exception as exc:
            key = (self.local_frame_id, msg.header.frame_id)
            if key not in self._warned_frames:
                self._warned_frames.add(key)
                self.get_logger().warn('Waiting for transform %s <- %s for height filtering: %s' % (self.local_frame_id, msg.header.frame_id, exc))
            return None
        rotation = transform.transform.rotation
        matrix = rotation_matrix_from_quaternion((rotation.x, rotation.y, rotation.z, rotation.w))
        translation = np.asarray([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z], dtype=np.float64)
        return points @ matrix.T + translation

    def _accept_or_hold_gravity(self, up: Optional[np.ndarray], stamp_ns: int) -> Tuple[Optional[np.ndarray], str]:
        """Smooth IMU world-up without creating a cloud-plane observation."""
        if self._last_gravity_stamp_ns is not None and stamp_ns > 0 and (stamp_ns < self._last_gravity_stamp_ns):
            self._clear_gravity_state()
        if up is not None:
            if self._filtered_normal is None:
                self._filtered_normal = np.asarray(up, dtype=np.float64)
                state = 'gravity_initialized'
            else:
                self._filtered_normal = blend_unit_vectors(self._filtered_normal, up, self.filter_gain)
                state = 'gravity_accepted'
            self._last_gravity_stamp_ns = stamp_ns
            return (self._filtered_normal, state)
        if self._filtered_normal is not None and self._last_gravity_stamp_ns is not None and (stamp_ns - self._last_gravity_stamp_ns <= int(self.hold_timeout_sec * 1000000000.0)):
            return (self._filtered_normal, 'gravity_held')
        return (None, 'gravity_unavailable')

    @staticmethod
    def _xyz_cloud(points_xyz: np.ndarray, stamp, frame_id: str) -> PointCloud2:
        points = np.ascontiguousarray(points_xyz, dtype=np.float32)
        message = PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height = 1
        message.width = int(len(points))
        message.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1), PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1), PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.data = points.tobytes(order='C')
        message.is_dense = True
        return message

    def _append_path_pose(self, history, odometry: Odometry) -> None:
        pose = PoseStamped()
        pose.header = deepcopy(odometry.header)
        pose.pose = deepcopy(odometry.pose.pose)
        history.append(pose)

    def _publish_debug_paths(self, raw_odometry: Odometry, fused_odometry: Odometry, stamp_ns: int) -> None:
        if self._last_debug_path_stamp_ns is not None and stamp_ns < self._last_debug_path_stamp_ns:
            self._raw_path_poses.clear()
            self._fused_path_poses.clear()
            self._last_debug_path_publish_ns = 0
        self._last_debug_path_stamp_ns = stamp_ns
        if self._raw_path_publisher is None or self._fused_path_publisher is None:
            return
        self._append_path_pose(self._raw_path_poses, raw_odometry)
        self._append_path_pose(self._fused_path_poses, fused_odometry)
        if stamp_ns - self._last_debug_path_publish_ns < self.debug_path_publish_period_ms * 1000000 and self._last_debug_path_publish_ns != 0:
            return
        raw_path = Path()
        raw_path.header = deepcopy(raw_odometry.header)
        raw_path.poses = list(self._raw_path_poses)
        fused_path = Path()
        fused_path.header = deepcopy(fused_odometry.header)
        fused_path.poses = list(self._fused_path_poses)
        self._raw_path_publisher.publish(raw_path)
        self._fused_path_publisher.publish(fused_path)
        self._last_debug_path_publish_ns = stamp_ns


def main(args=None):
    rclpy.init(args=args);node=GravityHeightFilter();executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
