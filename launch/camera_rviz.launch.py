# Copyright 2024 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')

    publish_svo_clock = LaunchConfiguration('publish_svo_clock')

    # RVIZ2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('zed_display_rviz2'),
        'rviz2',
        'zed_stereo.rviz'
    )

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        namespace='zed',
        executable='rviz2',
        name='zed_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
        parameters=[{'use_sim_time': publish_svo_clock}]
    )

    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('mhsboat_ctrl'),
            '/launch/camera.launch.py'
        ]),
        condition=IfCondition(start_zed_node)
    )

    return [
        rviz2_node,
        zed_wrapper_launch
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_zed_node',
                default_value='True',
                description='Set to `False` to start only RVIZ2 if a ZED node is already running.'),
            DeclareLaunchArgument(
                'publish_svo_clock',
                default_value='false',
                description='If set to `true` the node will act as a clock server publishing the SVO timestamp. This is useful for node synchronization'),
            OpaqueFunction(function=launch_setup)
        ]
    )
