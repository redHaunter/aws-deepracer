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
    # image_transport_1 = ExecuteProcess(
    #     cmd=[
    #         [
    #             'ROS_DOMAIN_ID=5 ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera_pkg/right/compressed --remap out:=/camera_pkg/right/image_raw'
    #         ]
    #     ],
    #     shell=True,
    # )
    # image_transport_2 = ExecuteProcess(
    #     cmd=[
    #         [
    #             'ROS_DOMAIN_ID=5 ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera_pkg/left/compressed --remap out:=/camera_pkg/left/image_raw'
    #         ]
    #     ],
    #     shell=True,
    # )
    decompressor_parameters = [
        {
            "publish_rgb": False,
        }
    ]
    return LaunchDescription(
        [
            camera_service,
            # image_transport_1,
            # image_transport_2,
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
                parameters=decompressor_parameters,
                output="screen",
            ),
            # launch_ros.actions.Node(
            #     package="decompressor",
            #     executable="resize_decompression_node",
            #     output="screen",
            # ),
        ]
    )
