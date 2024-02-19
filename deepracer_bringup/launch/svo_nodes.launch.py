#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    deepracer_bringup_dir = get_package_share_directory('deepracer_bringup')
    camera_dir = get_package_share_directory("camera_pkg")
    camera_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=camera_dir
            + "/launch/camera_pkg_launch.py"
        ),
    )
    imu_dir = get_package_share_directory("imu_pkg")
    imu_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=imu_dir
            + "/imu_pkg_launch.py"
        ),
    )

    return LaunchDescription([
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.136966', '0', '0.143272', '-1.5707963267948966',
                       '0', '-1.5707963267948966', 'base_link', 'camera_link_left'],
            parameters=[
                deepracer_bringup_dir + '/config/static_tf.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.06', '0', '0', '0',
                       '0', '0', 'camera_link_left', 'camera_link_right'],
            parameters=[
                deepracer_bringup_dir + '/config/static_tf.yaml'
            ]
        ),
        camera_launch,
        imu_launch,
    ])
