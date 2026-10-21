"""Experiment 4 role launcher. Existing single launch remains the estimator."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value=lambda key:LaunchConfiguration(key).perform(context)
    share=Path(get_package_share_directory('co_3dto2d_mapping'))
    role=value('role');rid=int(value('robot_id'));sim=value('use_sim_time').lower()=='true'
    if role not in ('robot','fusion','all') or rid not in (0,1):raise ValueError('invalid role or robot_id')
    output=Path(value('output_directory')).expanduser().resolve()
    # A critical child's exit must stop its owned launch, so the runtime can
    # detect failure and clean up the complete process group.
    actions=[RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='Experiment 4 child process exited'))]))]
    if role in ('robot','all'):
        config=value('robot_config_file') or str(share/'config/experiments/experiment_04_multi_agent_mapping'/f'robot{rid}.yaml')
        params=yaml.safe_load(Path(config).read_text())['/**']['ros__parameters']
        prefix=f'/r{rid}'
        if params['local_pht_submap_pose_graph_mode']!='full_se2':raise ValueError('Experiment 4 requires single PHT full_se2')
        args=dict(robot_id=str(rid),use_bag='false',use_sim_time=str(sim).lower(),
            publish_tf_odom='true',publish_sensor_static_tf=value('publish_sensor_static_tf'),
            enable_rear_lidar_filter='true',icp_topic_queue_size='1',
            sensor_parent_frame=f'r{rid}/base_link',sensor_child_frame=f'r{rid}/livox_frame',
            local_frame_id=f'r{rid}/base_link',global_frame_id=f'r{rid}/odom',use_odom_header_frame='true',
            bag_lidar_topic=prefix+'/livox/lidar',icp_scan_cloud_topic=prefix+'/livox/lidar',
            scan_cloud_topic=prefix+'/mapping/lidar_rear_filtered',
            imu_raw_topic=prefix+'/livox/imu',imu_filtered_topic=prefix+'/mapping/imu_filtered',
            plane_filtered_cloud_topic=prefix+'/mapping/plane_height_filtered',
            mapping_odometry_topic=prefix+'/mapping/floor_based_odometry',
            corrected_odometry_topic=prefix+'/mapping/occupancy_mapper_odometry',
            occupancy_config_file=config,odometry_config_file=value('odometry_config_file'),
            alignment_required='false',local_pht_submap_pose_graph_mode='full_se2',
            center_box_filter_half_extent_m=str(params.get('center_box_filter_half_extent_m',2.5)))
        for axis in ('x','y','z','roll','pitch','yaw'):
            args['sensor_tf_'+axis]=str(params.get('experiment_sensor_tf_'+axis,0.))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/single_bag_mapping.launch.py')),launch_arguments=args.items()))
        actions.append(Node(package='co_3dto2d_mapping',executable='multi_agent_session.py',
            name=f'robot{rid}_session',parameters=[dict(robot_id=rid,config_file=config,use_sim_time=sim)]))
    if role in ('fusion','all'):
        actions.append(Node(package='co_3dto2d_mapping',executable='multi_agent_mapping.py',
            parameters=[value('alignment_config_file'),dict(output_directory=str(output),use_sim_time=sim)],output='screen'))
    if value('rviz').lower()=='true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'rviz/experiment_04_multi_agent_mapping.rviz')],parameters=[dict(use_sim_time=sim)]))
    return actions


def generate_launch_description():
    share=get_package_share_directory('co_3dto2d_mapping')
    defaults=dict(role='robot',robot_id='0',use_sim_time='false',publish_sensor_static_tf='true',
        robot_config_file='',alignment_config_file=os.path.join(share,'config/experiments/experiment_04_multi_agent_mapping/alignment.yaml'),
        odometry_config_file=os.path.join(share,'config/lidar_odometry.yaml'),
        output_directory='outputs/experiment_04_multi_agent_mapping/runs/direct',rviz='false')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
