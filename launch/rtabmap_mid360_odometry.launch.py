from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    namespace = LaunchConfiguration("namespace").perform(context)
    use_sim_time = (
        LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    )
    startup_delay_sec = float(
        LaunchConfiguration("startup_delay_sec").perform(context)
    )
    if startup_delay_sec < 0.0:
        raise RuntimeError("startup_delay_sec must be non-negative")

    parameters = [{
        "frame_id": LaunchConfiguration("frame_id").perform(context),
        "odom_frame_id": LaunchConfiguration("odom_topic").perform(context),
        "publish_tf": LaunchConfiguration("publish_tf").perform(context).lower() == "true",
        "use_sim_time": use_sim_time,
        "wait_for_transform": 0.2,
        "expected_update_rate": float(
            LaunchConfiguration("expected_update_rate").perform(context)
        ),
        "wait_imu_to_init": LaunchConfiguration("wait_imu_to_init").perform(
            context
        ).lower() == "true",
        "qos": int(LaunchConfiguration("qos").perform(context)),
        "qos_imu": int(LaunchConfiguration("qos_imu").perform(context)),
        "Icp/PointToPlane": "true",
        "Icp/Iterations": "10",
        "Icp/VoxelSize": "0.1",
        "Icp/Epsilon": "0.001",
        "Icp/PointToPlaneK": "20",
        "Icp/MaxTranslation": "2",
        "Icp/MaxCorrespondenceDistance": "1",
        "Icp/Strategy": "1",
        "Icp/OutlierRatio": "0.7",
        "Icp/CorrespondenceRatio": "0.01",
        "Odom/ScanKeyFrameThr": "0.4",
        "OdomF2M/ScanSubtractRadius": "0.1",
        "OdomF2M/ScanMaxSize": "15000",
        "OdomF2M/BundleAdjustment": "false",
    }]

    odometry_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        output="screen",
        name="mid360_icp_odometry",
        namespace=namespace,
        parameters=parameters,
        remappings=[
            ("scan_cloud", LaunchConfiguration("scan_cloud_topic").perform(context)),
            ("imu", LaunchConfiguration("imu_topic").perform(context)),
            ("odom", LaunchConfiguration("odom_topic").perform(context)),
        ],
    )
    if startup_delay_sec > 0.0:
        return [TimerAction(period=startup_delay_sec, actions=[odometry_node])]
    return [odometry_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="/r0"),
            DeclareLaunchArgument("frame_id", default_value="base_link"),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("scan_cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("wait_imu_to_init", default_value="true"),
            DeclareLaunchArgument("expected_update_rate", default_value="10.0"),
            DeclareLaunchArgument("startup_delay_sec", default_value="0.0"),
            DeclareLaunchArgument("qos", default_value="0"),
            DeclareLaunchArgument("qos_imu", default_value="0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
