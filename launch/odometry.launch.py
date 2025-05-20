from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mhsboat_ctrl")
    ekf_config = os.path.join(pkg_share, "config", "ekf.yaml")
    
    dlio_pkg_share = get_package_share_directory("direct_lidar_inertial_odometry")
    dlio_yaml_path = os.path.join(dlio_pkg_share, "config", "dlio.yaml")
    dlio_params_yaml_path = os.path.join(dlio_pkg_share, "config", "dlio_params.yaml")

    POINTCLOUD_TOPIC = '/velodyne_points'
    IMU_TOPIC = '/imu/data'

    return LaunchDescription(
        [
            # DLIO Odometry Node
            Node(
                package='direct_lidar_inertial_odometry',
                executable='dlio_odom_node',
                output='screen',
                parameters=[dlio_yaml_path, dlio_params_yaml_path],
                remappings=[
                    ('pointcloud', POINTCLOUD_TOPIC),
                    ('imu', IMU_TOPIC),
                    ('odom', 'dlio/odom_node/odom'),
                    ('pose', 'dlio/odom_node/pose'),
                    ('path', 'dlio/odom_node/path'),
                    ('kf_pose', 'dlio/odom_node/keyframes'),
                    ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
                    ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
                ],
            ),
            # DLIO Mapping Node
            Node(
                package='direct_lidar_inertial_odometry',
                executable='dlio_map_node',
                output='screen',
                parameters=[dlio_yaml_path, dlio_params_yaml_path],
                remappings=[
                    ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
                ],
            ),
            # Launch IMU Filter Node
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="imu_filter_madgwick",
                output="screen",
                parameters=[{"use_mag": False}],
                remappings=[("imu/data_raw", "/mavros/imu/data_raw")],
            ),
            # Launch Robot Localization Node
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_localization",
                output="screen",
                parameters=[ekf_config],
            )
        ]
    )
