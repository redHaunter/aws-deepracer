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
            "subscribe_stereo": False,
            "subscribe_depth": True,
            "frame_id": "camera_link",
            "approx_sync": True,
            "queue_size": 200,
            "publish_tf": True,
            # "subscribe_scan": True,
        }
    ]
    slam_remmapings = [
        # ("right/image_rect", "/zed_camera/right/image_rect"),
        # ("left/image_rect", "/zed_camera/left/image_rect"),
        # ("right/camera_info", "/zed_camera/right/camera_info"),
        # ("left/camera_info", "/zed_camera/left/camera_info"),
        ("rgb/image", "/zed_camera/left/image_rect_color"),
        ("rgb/camera_info", "/zed_camera/left/camera_info"),
        ("depth/image", "/image_raw"),

        
    ]
    depth_params = [
        {
            "fixed_frame_id": "odom",
        }
    ]
    odom_params = [
        {
            "use_sim_time": True,
            "subscribe_stereo": True,
            "subscribe_depth": False,
            "frame_id": "camera_link",
            "odom_frame_id": "odom",
            "approx_sync": True,
            "Vis/MinInliers": "10",
            "OdomF2M/BundleAdjustment": "0",
            "OdomF2M/MaxSize": "1000",
            "GFTT/MinDistance": "10",
            "GFTT/QualityLevel": "0.00001",
        }
    ]
    stereo_sync_remmapings = [
        ("right/image_rect", "/zed_camera/right/image_rect"),
        ("left/image_rect", "/zed_camera/left/image_rect_color"),
        ("right/camera_info", "/zed_camera/right/camera_info"),
        ("left/camera_info", "/zed_camera/left/camera_info"),
    ]
    depth_remmapings = [
        ("cloud", "/points2"),
        ("camera_info", "/zed_camera/left/camera_info"),
    ]
    odometry_remmapings = [
        ("right/image_rect", "/zed_camera/right/image_rect"),
        ("left/image_rect", "/zed_camera/left/image_rect"),
        ("right/camera_info", "/zed_camera/right/camera_info"),
        ("left/camera_info", "/zed_camera/left/camera_info"),
    ]
    # -r right/image_rect:=/zed_camera/right/image_rect -r left/image_rect:=/zed_camera/left/image_rect -r right/camera_info:=/zed_camera/right/camera_info -r left/camera_info:=/zed_camera/left/camera_info

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
            "target_frame_id": "/zed_camera_link_leftcam",
            "avoid_point_cloud_padding": "True",
        }.items(),
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
                    "camera_link",
                ],
            ),
            stereo_image_proc_launcher,
            launch_ros.actions.Node(
                package="rtabmap_util",
                executable="pointcloud_to_depthimage",
                output="screen",
                parameters=depth_params,
                remappings=depth_remmapings,
            ),
            # launch_ros.actions.Node(
            #     package="rtabmap_sync",
            #     executable="stereo_sync",
            #     output="screen",
            #     arguments=["standalone", "rtabmap_sync/stereo_sync"],
            #     remappings=stereo_sync_remmapings,
            # ),
            # launch_ros.actions.Node(
            #     package="rtabmap_odom",
            #     executable="stereo_odometry",
            #     output="screen",
            #     # parameters=odom_params,
            #     arguments=[
            #         "--ros-args",
            #         "-p",
            #         "use_sim_time:=true",
            #         "-p",
            #         "subscribe_stereo:=true",
            #         "-p",
            #         "subscribe_depth:=false",
            #         "-p",
            #         "frame_id:=camera_link",
            #         "-p",
            #         "queue_size:=30",
            #         "-p",
            #         "odom_frame_id:=odom",
            #         "-p",
            #         "approx_sync:=true",
            #         "-r",
            #         "right/image_rect:=/zed_camera/right/image_rect",
            #         "-r",
            #         "left/image_rect:=/zed_camera/left/image_rect",
            #         "-r",
            #         "right/camera_info:=/zed_camera/right/camera_info",
            #         "-r",
            #         "left/camera_info:=/zed_camera/left/camera_info",
            #     ],
            #     # remappings=odometry_remmapings,
            # ),
            launch_ros.actions.Node(
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                arguments=[
                    "-d",
                    "--Vis/CorFlowMaxLevel",
                    "5",
                    "--Stereo/MaxDisparity", 
                    "200",
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
