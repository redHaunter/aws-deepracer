from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions


def generate_launch_description():
    slam_parameters = [
        {
            "use_sim_time": True,
            "frame_id": "base_link",
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            # "subscribe_scan": True,
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            "RGBD/OptimizeFromGraphEnd": "false",
            "Grid/FlatObstacleDetected": "false",
            # "Grid/FootprintHeight": "0.1",
            # "Grid/MapFrameProjection": "true", 
            # "Grid/MaxGroundAngle": "6", # is this even working?
            "GridGlobal/FullUpdate": "false",
            "Icp/Iterations": "150",

        }
    ]
    depth_params = [
        {

            "fixed_frame_id": "odom",
        }
    ]
    odom_params = [
        {
            "use_sim_time": True,
            "subscribe_rgbd": True,
            "subscribe_depth": False,
            "approx_sync": True,
        }
    ]
    rgbd_sync_remmapings = [
        ("rgb/image", "/zed_camera/left/image_rect_color"),
        ("depth/image", "/image_raw"),
        ("rgb/camera_info", "/zed_camera/left/camera_info"),
    ]
    depth_remmapings = [
        ("cloud", "/points2"),
        ("camera_info", "/zed_camera/left/camera_info"),
    ]
    odom_remmapings = [
        ("rgb/image", "/zed_camera/left/image_rect_color"),
        ("depth/image", "/image_raw"),
        ("rgb/camera_info", "/zed_camera/left/camera_info"),
    ]
    
    stereo_image_proc_dir = get_package_share_directory("stereo_image_proc")
    stereo_image_proc_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=stereo_image_proc_dir
            + "/launch/stereo_image_proc.launch.py"
        ),
        launch_arguments={
            "left_namespace": "/zed_camera/left",
            "right_namespace": "/zed_camera/right",
            "target_frame_id": "/camera_link",
            # "disparity_range": "32",
            # "speckle_size": "1000",
        }.items(),
    )

    return LaunchDescription(
        [
            stereo_image_proc_launcher,
            launch_ros.actions.Node(
                package="rtabmap_util",
                executable="pointcloud_to_depthimage",
                output="screen",
                parameters=depth_params,
                remappings=depth_remmapings,
            ),
            launch_ros.actions.Node(
                package="rtabmap_sync",
                executable="rgbd_sync",
                output="screen",
                parameters=[{"approx_sync": True}],
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
            ),
            launch_ros.actions.Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=slam_parameters,
            ),
        ]
    )
