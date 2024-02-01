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
            "approx_sync": False,
            # "subscribe_scan": True,
            "RGBD/AngularUpdate": "0.01",
            "RGBD/LinearUpdate": "0.01",
            # "RGBD/OptimizeFromGraphEnd": "false",
            # "Grid/FlatObstacleDetected": "false",
            # "Grid/FootprintHeight": "0.1",
            # "Grid/MapFrameProjection": "true", 
            # "Grid/MaxGroundAngle": "6", # is this even working?
            # "GridGlobal/FullUpdate": "false",
            # "Icp/Iterations": "150",

        }
    ]
    depth_params = [
        {
            "use_sim_time": True,
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
    stereo_parameters = [
        {
            "use_sim_time": True,
            "approx_sync": True,
        }
    ]
    rgbd_sync_remmapings = [
        ("rgb/image", "/camera_pkg/left/image_rect_color"),
        ("depth/image", "/image_raw"),
        ("rgb/camera_info", "/camera_pkg/left/camera_info"),
    ]
    depth_remmapings = [
        ("cloud", "/points2"),
        ("camera_info", "/camera_pkg/left/camera_info"),
    ]
    laser_remmapings = [
        ("depth", "/image_raw"),
        ("depth_camera_info", "/camera_pkg/left/camera_info"),
        ("scan", "/test/scan"),
    ]
    odom_remmapings = [
        ("rgb/image", "/camera_pkg/left/image_rect_color"),
        ("depth/image", "/image_raw"),
        ("rgb/camera_info", "/camera_pkg/left/camera_info"),
    ]
    stereo_remmapings = [
        ("right/image_rect", "/camera_pkg/right/image_raw"),
        ("left/image_rect", "/camera_pkg/left/image_raw"),
        ("right/camera_info", "/camera_pkg/right/camera_info"),
        ("left/camera_info", "/camera_pkg/left/camera_info"),
    ]
    domain_bridge_dir = get_package_share_directory("domain_bridge")
    domain_bridge_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=domain_bridge_dir
            + "/launch/stereo_image_proc.launch.py"
        ),
        launch_arguments={
            "approximate_sync": "False",
            "left_namespace": "/camera_pkg/left",
            "right_namespace": "/camera_pkg/right",
            "target_frame_id": "/camera_link",
            # "disparity_range": "32",
            "speckle_size": "1000",
        }.items(),
    )
    image_view_params = [
        ("image", "/disparity"),
    ]
    stereo_image_proc_dir = get_package_share_directory("stereo_image_proc")
    stereo_image_proc_launcher = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=stereo_image_proc_dir
            + "/launch/stereo_image_proc.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
            "approximate_sync": "True",
            "left_namespace": "/camera_pkg/left",
            "right_namespace": "/camera_pkg/right",
            "target_frame_id": "/camera_link",
            "correlation_window_size": "9",
            "speckle_range": "31",
            "speckle_size": "500",
            "texture_threshold": "500",
            "uniqueness_ratio": "5.0",
        }.items(),
    )

    return LaunchDescription(
        [
            # launch_ros.actions.Node(
            #     package="domain_bridge",
            #     executable="domain_bridge",
            #     output="screen",
            #     arguments=[
            #         "/home/redha/ros2_ws/src/domain_bridge/examples/example_bridge_config.yaml",
            #     ],
            # ),
            # launch_ros.actions.Node(
            #     package="decompressor",
            #     executable="decompression_node",
            #     output="screen",
            # ),
            # launch_ros.actions.Node(
            #     package="rtabmap_sync",
            #     executable="stereo_sync",
            #     output="screen",
            #     parameters=stereo_parameters,
            #     remappings=stereo_remmapings,
            # ),
            stereo_image_proc_launcher,
            # launch_ros.actions.Node(
            #     package="image_view",
            #     executable="disparity_view",
            #     output="screen",
            #     remappings=image_view_params,
            # ),
            # launch_ros.actions.Node(
            #     package="rtabmap_util",
            #     executable="pointcloud_to_depthimage",
            #     output="screen",
            #     parameters=depth_params,
            #     remappings=depth_remmapings,
            # ),
        #     launch_ros.actions.Node(
        #     package="depthimage_to_laserscan",
        #     executable="depthimage_to_laserscan_node",
        #     output="screen",
        #     parameters=[
        #     {'use_sim_time': True},
        #     {'range_min': 0.15},
        #     {'range_max': 10.0}, #10.0
        #     # {'scan_height': 450},
        #     {'output_frame': 'fakelaser'}
        #     ],
        #     remappings=laser_remmapings,
        # ),
        # launch_ros.actions.Node(
        #         package="tf2_ros",
        #         executable="static_transform_publisher",
        #         output="screen",
        #         arguments=[
        #             "0",
        #             "0",
        #             "0",
        #             "0",
        #             "0",
        #             "0",
        #             "base_link",
        #             "fakelaser",
        #         ],
        #         parameters=[{'use_sim_time': True},]
        #     ),
            # launch_ros.actions.Node(
            #     package="rtabmap_odom",
            #     executable="rgbd_odometry",
            #     output="screen",
            #     parameters=odom_params,
            #     remappings=odom_remmapings,
            # ),
            # launch_ros.actions.Node(
            #     package="rtabmap_slam",
            #     executable="rtabmap",
            #     output="screen",
            #     arguments=[
            #         "--delete_db_on_start",
            #     ],
            #     parameters=slam_parameters,
            # ),
            # launch_ros.actions.Node(
            #     package="rtabmap_viz",
            #     executable="rtabmap_viz",
            #     output="screen",
            #     parameters=slam_parameters,
            # ),
        ]
    )
