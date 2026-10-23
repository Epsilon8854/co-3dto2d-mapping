"""Shared single-agent sensor, raw ICP, gravity-height and occupancy pipeline."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from co_3dto2d_mapping.common.configuration import load_parameters,robot_parameters


def setup(context):
    value=lambda name:LaunchConfiguration(name).perform(context)
    rid=int(value('robot_id'));ns=f'/r{rid}';base=f'r{rid}/base_link';sensor=f'r{rid}/livox_frame'
    share=Path(get_package_share_directory('co_3dto2d_mapping'))
    sim=value('use_sim_time').lower()=='true'
    # A resolved run snapshot already has robot-specific names; rendering it
    # again is harmless and supports direct ros2 launch with the shared profile.
    params=robot_parameters(value('config_file'),rid,value('multi_agent').lower()=='true',value('multi_config_file'))
    params['use_sim_time']=sim
    hardware=yaml.safe_load(Path(value('sensor_config_file') or share/f'config/sensors/r{rid}.yaml').read_text())
    rear=hardware['rear_filter'];tf=hardware['sensor_tf']
    actions=[RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason='single-agent child exited'))]))]
    actions.append(Node(package='imu_filter_madgwick',executable='imu_filter_madgwick_node',name='imu_filter_madgwick',namespace=ns,
        parameters=[dict(use_mag=False,publish_tf=False,reverse_tf=False,world_frame='enu',remove_gravity_vector=False,use_sim_time=sim)],
        remappings=[('imu/data_raw',ns+'/livox/imu'),('imu/data',ns+'/mapping/imu_filtered_raw_frame')]))
    actions.append(Node(package='co_3dto2d_mapping',executable='imu_frame_republisher.py',namespace=ns,name='imu_frame_republisher',
        parameters=[dict(input_topic=ns+'/mapping/imu_filtered_raw_frame',output_topic=ns+'/mapping/imu_filtered',output_frame_id=sensor,use_sim_time=sim)]))
    actions.append(Node(package='co_3dto2d_mapping',executable='pointcloud_rear_sector_filter.py',namespace=ns,name='pointcloud_rear_sector_filter',
        parameters=[dict(input_topic=ns+'/livox/lidar',output_topic=ns+'/mapping/lidar_rear_filtered',enabled=rear['enabled'],
            rear_filter_angle_deg=rear['angle_deg'],rear_axis=rear['axis'],min_xy_range_m=rear['min_xy_range_m'],output_frame_id=sensor,use_sim_time=sim)]))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/internal/icp_odometry.launch.py')),
        launch_arguments=dict(namespace=ns,frame_id=base,odom_topic='odom',odom_frame_id=f'r{rid}/odom',scan_cloud_topic=ns+'/livox/lidar',
            imu_topic=ns+'/mapping/imu_filtered',wait_imu_to_init='true',expected_update_rate='10.0',topic_queue_size='1',
            odometry_config_file=value('odometry_config_file'),publish_tf='true',use_sim_time=str(sim).lower()).items()))
    actions.append(Node(package='co_3dto2d_mapping',executable='gravity_height_filter.py',namespace=ns,name='gravity_height_filter',parameters=[params],output='screen'))
    actions.append(Node(package='co_3dto2d_mapping',executable='occupancy_mapper',namespace=ns,name='occupancy_mapper',parameters=[params],output='screen'))
    if value('output_directory') and value('multi_agent').lower()!='true':
        actions.append(Node(package='co_3dto2d_mapping',executable='single_agent_map_saver.py',
            parameters=[dict(robot_id=rid,output_directory=value('output_directory'),use_sim_time=sim)]))
    if value('publish_sensor_static_tf').lower()=='true':
        actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name=f'r{rid}_sensor_static_tf',
            arguments=[str(tf[k]) for k in ('x','y','z','yaw','pitch','roll')]+[base,sensor],parameters=[dict(use_sim_time=sim)]))
    return actions


def generate_launch_description():
    share=Path(get_package_share_directory('co_3dto2d_mapping'))
    defaults=dict(robot_id='0',config_file=str(share/'config/single_agent.yaml'),sensor_config_file='',
        multi_agent='false',multi_config_file=str(share/'config/multi_agent.yaml'),odometry_config_file=str(share/'config/lidar_odometry.yaml'),
        publish_sensor_static_tf='true',use_sim_time='false',output_directory='')
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=setup)])
