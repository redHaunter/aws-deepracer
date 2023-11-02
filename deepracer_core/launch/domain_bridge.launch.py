from launch import LaunchDescription
import launch_ros.actions
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    domain_bridge_dir = get_package_share_directory("deepracer_core")
    camera_service = ExecuteProcess(
        cmd=[
            [
                'ROS_DOMAIN_ID=5 ros2 service call /camera_pkg/media_state deepracer_interfaces_pkg/srv/VideoStateSrv "{activate_video: 1}"'
            ]
        ],
        shell=True,
    )
    return LaunchDescription(
        [
            camera_service,
            launch_ros.actions.Node(
                package="domain_bridge",
                executable="domain_bridge",
                output="screen",
                arguments=[
                    domain_bridge_dir + "/config/bridge_config.yaml",
                ],
            ),
            launch_ros.actions.Node(
                package="decompressor",
                executable="decompression_node",
                output="screen",
            ),
        ]
    )
