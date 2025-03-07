from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("mhsboat_ctrl")

    return LaunchDescription(
        [
            Node(
                package="mhsboat_ctrl",
                executable="vision_mhsboat_ctrl",
                name="vision_mhsboat_ctrl",
            ),
            Node(
                package="mhsboat_ctrl",
                executable="vision",
                name="vision",
            ),
        ]
    )