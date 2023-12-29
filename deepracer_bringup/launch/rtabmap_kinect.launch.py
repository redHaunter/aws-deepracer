from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch.actions import ExecuteProcess

def generate_launch_description():
    slam_parameters = [
        {
            "use_sim_time": False,
            "frame_id": "base_link",
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            "approx_sync": False,
            # "subscribe_scan": True,
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            "RGBD/OptimizeFromGraphEnd": "false",
            # "Grid/FlatObstacleDetected": "false",
            # "Grid/FootprintHeight": "0.1",
            # "Grid/MapFrameProjection": "true",
            # "Grid/MaxGroundAngle": "6", # is this even working?
            # "GridGlobal/FullUpdate": "false",
            # "Icp/Iterations": "150",
        }
    ]
    rgbd_sync_params = [
        {
            "approx_sync": True,
        }
    ]
    odom_params = [
        {
            "subscribe_rgbd": True,
            "frame_id": "base_link",
        }
    ]

    rgbd_sync_remmapings = [
        ("rgb/image", "/image_raw"),
        ("depth/image", "/depth/image_raw"),
        ("rgb/camera_info", "/camera_info"),
    ]
    odom_remmapings = [
        ("rgbd_image", "/rgbd_image"),
    ]
    slam_remmapings = [
        ("rgbd_image", "/rgbd_image"),
    ]
    kinect_save = ExecuteProcess(
        cmd=[
            [
                'ros2 bag record -o kinect_record_1 /camera_info /depth/camera_info /depth/image_raw /image_raw'
            ]
        ],
        shell=True,
    )
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
                    "-1.5707963267948966",
                    "0",
                    "-1.5707963267948966",
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
            kinect_save,
            launch_ros.actions.Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                output="screen",
                parameters=rgbd_sync_params,
                remappings=rgbd_sync_remmapings,
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
                    "--delete_db_on_start",
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
