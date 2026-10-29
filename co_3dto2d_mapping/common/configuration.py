"""Resolve one shared mapping profile for a logical robot and execution mode."""
from pathlib import Path
import yaml


def multi_agent_role(robot_id, role='auto'):
    """Resolve public role aliases and choose the central host from robot_id."""
    if robot_id not in (0, 1):
        raise ValueError('robot_id must be 0 or 1')
    role = {'agent': 'robot', 'coordinator': 'all'}.get(role, role)
    if role == 'auto':
        role = 'all' if robot_id == 0 else 'robot'
    if role not in ('robot', 'fusion', 'all'):
        raise ValueError('invalid multi-agent role')
    if robot_id != 0 and role in ('fusion', 'all'):
        raise ValueError('coordinator/fusion role requires robot-id 0')
    return role


def load_parameters(path):
    value=yaml.safe_load(Path(path).read_text())
    return dict(value.get('central',value.get('/**',{}).get('ros__parameters',{})))


def robot_parameters(profile,robot_id,multi_agent=False,multi_config=None):
    values=load_parameters(profile)
    if multi_agent and multi_config:
        values.update(yaml.safe_load(Path(multi_config).read_text()).get('agent_overrides',{}))
    prefix=f'r{robot_id}'
    frame=f'{prefix}/mapping_odom' if multi_agent else 'map'
    values.update(scan_cloud_topic=f'/{prefix}/mapping/plane_height_filtered',
        odom_topic=f'/{prefix}/mapping/floor_based_odometry',local_frame_id=f'{prefix}/base_link',
        global_frame_id=frame,output_frame_id=frame,use_odom_header_frame=False,
        input_cloud_topic=f'/{prefix}/mapping/lidar_rear_filtered',imu_topic=f'/{prefix}/mapping/imu_filtered',
        input_odometry_topic='odom',output_odometry_topic='mapping/floor_based_odometry',
        output_cloud_topic='mapping/plane_height_filtered',
        anchor_output_to_initial_pose=not multi_agent)
    return values
