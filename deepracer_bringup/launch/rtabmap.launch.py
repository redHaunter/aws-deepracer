from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    parameters = [
        {
            "use_sim_time": True,
            "subscribe_stereo": True,
            "subscribe_depth": False,
            "frame_id": "camera_link",
            "approx_sync": True,
            "queue_size": 30,
            # "icp_odometry": True,
            # "visual_odometry": True,
            # "Grid/FromDepth": True,
        }
    ]
    remmapings = [
        ("right/image_rect", "/zed_camera/right/image_rect_color"),
        ("left/image_rect", "/zed_camera/left/image_rect_color"),
        ("right/camera_info", "/zed_camera/right/camera_info"),
        ("left/camera_info", "/zed_camera/left/camera_info"),
    ]
    stereo_image_proc_dir = get_package_share_directory("stereo_image_proc")
    stereo_image_proc_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=stereo_image_proc_dir
            + "/launch/stereo_image_proc.launch.py"
        ),
        launch_arguments={
            "approximate_sync": "True",
            "left_namespace": "/zed_camera/left",
            "right_namespace": "/zed_camera/right",
        }.items(),
    )

    return LaunchDescription(
        [
            stereo_image_proc_launcher,
            launch_ros.actions.Node(
                package="rtabmap_odom",
                executable="stereo_odometry",
                output="screen",
                parameters=parameters,
                remappings=remmapings,
            ),
            launch_ros.actions.Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                arguments=["-d"],
                parameters=parameters,
                remappings=remmapings,
            ),
            launch_ros.actions.Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=parameters,
                remappings=remmapings,
            ),
        ]
    )
