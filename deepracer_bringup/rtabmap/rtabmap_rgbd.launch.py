from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch.actions import ExecuteProcess


def generate_launch_description():
    deepracer_bringup_dir = get_package_share_directory("deepracer_bringup")
    slam_parameters = [
        {
            "use_sim_time": False,
            "frame_id": "base_link",
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            "approx_sync": True,
            # "subscribe_scan": True,
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            "RGBD/OptimizeFromGraphEnd": "false",
            "Grid/FlatObstacleDetected": "false",
            "Grid/FootprintHeight": "0.1",
            "Grid/MapFrameProjection": "true",
            "Grid/MaxGroundAngle": "6", # is this even working?
            "GridGlobal/FullUpdate": "false",
            "Icp/Iterations": "150",
        }
    ]
    slam_remmapings = [
        # ("rgb/image", "/camera_pkg/left/image_raw"),
        # ("depth/image", "/image_raw"),
        # ("rgb/camera_info", "/camera_pkg/left/camera_info"),
    ]
    depth_params = [
        {
            "use_sim_time": False,
            "fixed_frame_id": "odom",
        }
    ]
    depth_remmapings = [
        ("cloud", "/points2"),
        ("camera_info", "/camera_pkg/left/camera_info"),
    ]
    odom_params = [
        {
            "use_sim_time": False,
            "subscribe_rgbd": True,
            # "subscribe_depth": True,
            "approx_sync": True,
        }
    ]
    odom_remmapings = [
        ("rgbd_image", "/rgbd_image"),
        ("right/image_rect", "/camera_pkg/right/image_raw"),
        ("left/image_rect", "/camera_pkg/left/image_raw"),
        ("right/camera_info", "/camera_pkg/right/camera_info"),
        ("left/camera_info", "/camera_pkg/left/camera_info"),
        # ("rgb/image", "/camera_pkg/left/image_raw"),
        # ("depth/image", "/image_raw"),
        # ("rgb/camera_info", "/camera_pkg/left/camera_info"),
    ]
    disparity_parameters = [
        {
            "approximate_sync": True,
            "use_sim_time": False,
            # "correlation_window_size": 5,
            # "speckle_range": 2,
            # "speckle_size": 1000,
            # "stereo_algorithm": 1,
            # "uniqueness_ratio": 15.0,
            # "disparity_range": 64,
            "correlation_window_size": 7,
            "speckle_range": 31,
            "speckle_size": 500,
            "stereo_algorithm": 0,
            "texture_threshold": 500,
            "uniqueness_ratio": 0.0,
            "disparity_range": 80,
        }
    ]
    pointcloud_parameters = [
        {
            "approximate_sync": True,
            "use_sim_time": False,
            # "correlation_window_size": "11",
            # "speckle_range": "15",
            # "speckle_size": "250",
            # "texture_threshold": "2000",
            # "uniqueness_ratio": "5.0",
        }
    ]
    pointcloud_remmapings = [
        ("left/image_rect_color", "/camera_pkg/left/image_raw"),
        ("right/camera_info", "/camera_pkg/right/camera_info"),
        ("left/camera_info", "/camera_pkg/left/camera_info"),
    ]
    stereo_parameters = [
        {
            "use_sim_time": True,
            "approx_sync": False,
        }
    ]
    stereo_remmapings = [
        ("right/image_rect", "/camera_pkg/right/image_raw"),
        ("left/image_rect", "/camera_pkg/left/image_raw"),
        ("right/camera_info", "/camera_pkg/right/camera_info"),
        ("left/camera_info", "/camera_pkg/left/camera_info"),
    ]
    rgbd_sync_remmapings = [
        ("rgb/image", "/camera_pkg/left/image_rect_color"),
        ("depth/image", "/image_raw"),
        ("rgb/camera_info", "/camera_pkg/left/camera_info"),
    ]
    laser_remmapings = [
        ("depth", "/image_raw"),
        ("depth_camera_info", "/camera_pkg/left/camera_info"),
        ("scan", "/test/scan"),
    ]
    domain_bridge_dir = get_package_share_directory("deepracer_core")
    image_view_params = [
        ("image", "/disparity"),
    ]
    # stereo_image_proc_dir = get_package_share_directory("stereo_image_proc")
    # stereo_image_proc_launcher = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         launch_file_path=stereo_image_proc_dir
    #         + "/launch/stereo_image_proc.launch.py"
    #     ),
    #     launch_arguments={
    #         # "use_sim_time": "True",
    #         # "approximate_sync": "True",
    #         "queue_size": "100",
    #         "left_namespace": "/camera_pkg/left",
    #         "right_namespace": "/camera_pkg/right",
    #         "target_frame_id": "/camera_link_left",
    #         # "correlation_window_size": "11",
    #         # "speckle_range": "15",
    #         # "speckle_size": "250",
    #         # "texture_threshold": "2000",
    #         # "uniqueness_ratio": "5.0",
    #     }.items(),
    # )
    camera_service = ExecuteProcess(
        cmd=[
            [
                'ROS_DOMAIN_ID=5 ros2 service call /camera_pkg/media_state deepracer_interfaces_pkg/srv/VideoStateSrv "{activate_video: 1}"'
            ]
        ],
        shell=True,
    )
    decompressor_parameters = [
        {
            "publish_rgb": True,
        }
    ]
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[
                    "0.03",
                    "0",
                    "0",
                    "0",
                    "0",
                    "0",
                    "camera_link_left",
                    "camera_link",
                ],
                parameters=[deepracer_bringup_dir + "/config/static_tf.yaml"],
            ),
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
                parameters=decompressor_parameters,
                output="screen",
            ),
            launch_ros.actions.Node(
                package="rtabmap_odom",
                executable="icp_odometry",
                output="screen",
                # parameters=odom_params,
                # remappings=odom_remmapings,
            ),
            launch_ros.actions.Node(
                package="rtabmap_sync",
                executable="stereo_sync",
                output="screen",
                parameters=stereo_parameters,
                remappings=stereo_remmapings,
            ),
            launch_ros.actions.Node(
                package="stereo_image_proc",
                executable="disparity_node",
                output="screen",
                parameters=disparity_parameters,
                remappings=stereo_remmapings,
            ),
            launch_ros.actions.Node(
                package="stereo_image_proc",
                executable="point_cloud_node",
                output="screen",
                parameters=pointcloud_parameters,
                remappings=pointcloud_remmapings,
            ),
            launch_ros.actions.Node(
                package="image_view",
                executable="disparity_view",
                output="screen",
                remappings=image_view_params,
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
