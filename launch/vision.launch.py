from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory("mhsboat_ctrl")

    return LaunchDescription(
        [
            """
            - Imu dead reckoning
            - Buoy detections (depth map) 
            - mhsboat_ctrl (mavros)
            - PID to drive straight
            """

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
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="realsense2_camera",
                output="screen",
                parameters=[
                    {
                        "enable_color": True,
                        "rgb_camera.color_profile": "1920x1080x30",
                        "enable_depth": True,
                        "depth_module.profile": "1280x720x30",
                    }
                ],
            ),
            Node(
                package="mhsboat_ctrl",
                executable="buoy_recognition",
                name="buoy_recognition",
                output="screen",
                parameters=[{"headless_mode": LaunchConfiguration("headless_mode")}],
            ),
        ]
    )