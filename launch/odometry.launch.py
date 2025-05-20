from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mhsboat_ctrl")
    ekf_config = os.path.join(pkg_share, "config", "ekf.yaml")

    return LaunchDescription(
        [
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
