from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("mhsboat_ctrl")

    headless_mode_arg = DeclareLaunchArgument(
        'headless_mode',
        default_value='True',
        description='Run computer vision node in headless mode'
    )

    return LaunchDescription(
        [
            headless_mode_arg,
            # Launch Clustering Node
            Node(
                package="mhsboat_ctrl",
                executable="center_of_clusters",
                name="center_of_clusters",
                output="screen",
            ),
            # Launch Computer Vision Node with headless_mode parameter
            # Node(
            #     package="mhsboat_ctrl",
            #     executable="buoy_recognition",
            #     name="buoy_recognition",
            #     output="screen",
            #     parameters=[{"headless_mode": LaunchConfiguration("headless_mode")}],
            # ),
            # Launch Sensor Processing Node
            Node(
                package="mhsboat_ctrl",
                executable="sensors",
                name="sensors",
                output="screen",
            ),
        ]
    )