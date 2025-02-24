from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("mhsboat_ctrl")

    return LaunchDescription(
        [
            # Launch Clustering Node
            Node(
                package="mhsboat_ctrl",
                executable="center_of_clusters",
                name="center_of_clusters",
                output="screen",
            ),
            # Launch Computer Vision Node
            Node(
                package="mhsboat_ctrl",
                executable="buoy_recognition",
                name="buoy_recognition",
                output="screen",
            ),
            # Launch Sensor Processing Node
            Node(
                package="mhsboat_ctrl",
                executable="sensors",
                name="sensors",
                output="screen",
            ),
        ]
    )
