from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    slam_parameters = [
        {
            "use_sim_time": False,
            "frame_id": "base_link",
            "subscribe_depth": True,
            "subscribe_rgbd": False,
            "approx_sync": False,
            # "subscribe_scan": True,
            # "RGBD/AngularUpdate": "0.01",
            # "RGBD/LinearUpdate": "0.01",
            # "RGBD/OptimizeFromGraphEnd": "false",
            # "Grid/FlatObstacleDetected": "false",
            # "Grid/FootprintHeight": "0.1",
            # "Grid/MapFrameProjection": "true",
            # "Grid/MaxGroundAngle": "6", # is this even working?
            # "GridGlobal/FullUpdate": "false",
            # "Icp/Iterations": "150",
        }
    ]
    slam_remmapings = [
        ("rgb/image", "/image_raw"),
        ("rgb/camera_info", "/camera_info"),
        ("depth/image", "/depth/image_raw"),
    ]
    odom_params = [
        {
            "approx_sync": False,
            "subscribe_rgbd": False,
            "frame_id": "base_link",
        }
    ]
    odom_remmapings = [
        ("rgb/image", "/image_raw"),
        ("depth/image", "/depth/image_raw"),
        ("rgb/camera_info", "/camera_info"),
    ]

    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "0",
                    "0",
                    "0",
                    "1.5707963267948966",
                    "3.14159265359",
                    "1.5707963267948966",
                    "base_link",
                    "kinect_rgb",
                ],
            ),
            launch_ros.actions.Node(
                package="kinect_ros2",
                executable="kinect_ros2_node",
                name="kinect_ros2_node",
                output="screen",
            ),
            launch_ros.actions.Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                output="screen",
                parameters=odom_params,
                remappings=odom_remmapings,
            ),
            launch_ros.actions.Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                arguments=[
                    "-d",
                ],
                parameters=slam_parameters,
                remappings=slam_remmapings,
            ),
            launch_ros.actions.Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=slam_parameters,
                remappings=slam_remmapings,
            ),
        ]
    )
