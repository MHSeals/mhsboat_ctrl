import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    mhsboat_ctrl_dir = get_package_share_directory('mhsboat_ctrl')

    # Paths to your custom config files within mhsboat_ctrl
    common_config_path = os.path.join(mhsboat_ctrl_dir, 'config', 'common_stereo.yaml')
    zed2i_config_path = os.path.join(mhsboat_ctrl_dir, 'config', 'zed2i.yaml')
    ffmpeg_config_path = os.path.join(mhsboat_ctrl_dir, 'config', 'ffmpeg.yaml')
    object_detection_config_path = os.path.join(mhsboat_ctrl_dir, 'config', 'object_detection.yaml')
    custom_object_detection_config_path = os.path.join(mhsboat_ctrl_dir, 'config', 'custom_object_detection.yaml')

    return LaunchDescription([
        # Declare launch arguments for the ZED camera (optional, but good practice)
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2i',
            description='The model of the ZED camera.'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='The name of the ZED camera node.'
        ),
        # You can declare more arguments from the ZED launch file as needed

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={
                'camera_model': LaunchConfiguration('camera_model'),
                'camera_name': LaunchConfiguration('camera_name'),
                # Pass your custom config paths here
                'common_config_path': common_config_path,
                'zed_id_path': zed2i_config_path,
                'ffmpeg_config_path': ffmpeg_config_path,
                'object_detection_config_path': object_detection_config_path,
                'custom_object_detection_config_path': custom_object_detection_config_path,
            }.items()
        )
    ])