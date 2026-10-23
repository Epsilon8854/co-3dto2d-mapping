"""Resolve one shared mapping profile for a logical robot and execution mode."""
from pathlib import Path
import yaml


def load_parameters(path):
    value=yaml.safe_load(Path(path).read_text())
    return dict(value.get('central',value.get('/**',{}).get('ros__parameters',{})))


def robot_parameters(profile,robot_id,multi_agent=False,multi_config=None):
    values=load_parameters(profile)
    if multi_agent and multi_config:
        values.update(yaml.safe_load(Path(multi_config).read_text()).get('agent_overrides',{}))
    prefix=f'r{robot_id}'
    frame=f'{prefix}/mapping_odom' if multi_agent else f'{prefix}/odom'
    values.update(scan_cloud_topic=f'/{prefix}/mapping/plane_height_filtered',
        odom_topic=f'/{prefix}/mapping/floor_based_odometry',local_frame_id=f'{prefix}/base_link',
        global_frame_id=frame,output_frame_id=frame,use_odom_header_frame=False,
        input_cloud_topic=f'/{prefix}/mapping/lidar_rear_filtered',imu_topic=f'/{prefix}/mapping/imu_filtered',
        input_odometry_topic='odom',output_odometry_topic='mapping/floor_based_odometry',
        output_cloud_topic='mapping/plane_height_filtered')
    return values
