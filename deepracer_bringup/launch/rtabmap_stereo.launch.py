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
            "subscribe_stereo": True,
            "subscribe_depth": False,
            "frame_id": "base_link",
            "approx_sync": True,
            "queue_size": 30,
            "publish_tf": True,
            "Vis/MinInliers": "12",
        }
    ]
    odom_params = [
        {
            "use_sim_time": True,
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "approx_sync": True,
            "Odom/MinInliers": "12",
            "Odom/RoiRatios": "0.03 0.03 0.04 0.04",
        }
    ]
    viz_parameters = [
        {
            "use_sim_time": True,
            "subscribe_stereo": True,
            "subscribe_odom_info": True,
            "queue_size": 10,
            "frame_id": "base_link",
            "approx_sync": True,
        }
    ]
    stereo_parameters = [
        {
            "use_sim_time": True,
            "approx_sync": True,
            "queue_size": 5,
        }
    ]
    slam_remmapings = [
        ("right/image_rect", "/zed_camera/right/image_rect"),
        ("left/image_rect", "/zed_camera/left/image_rect_color"),
        ("right/camera_info", "/zed_camera/right/camera_info"),
        ("left/camera_info", "/zed_camera/left/camera_info"),
        ("odom", "/stereo_camera/odom"),

    ]
    odometry_remmapings = [
        ("right/image_rect", "/zed_camera/right/image_rect"),
        ("left/image_rect", "/zed_camera/left/image_rect"),
        ("right/camera_info", "/zed_camera/right/camera_info"),
        ("left/camera_info", "/zed_camera/left/camera_info"),
    ]
    viz_remmapings = [
        ("right/image_rect", "/zed_camera/right/image_rect"),
        ("left/image_rect", "/zed_camera/left/image_rect_color"),
        ("right/camera_info", "/zed_camera/right/camera_info"),
        ("left/camera_info", "/zed_camera/left/camera_info"),
        ("odom", "/stereo_camera/odom"),
        ("odom_info", "/stereo_camera/odom_info"),
    ]
    stereo_remmapings = [
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
            "target_frame_id": "/zed_camera_link_leftcam",
            "avoid_point_cloud_padding": "True",
        }.items(),
    )

    return LaunchDescription(
        [
            # launch_ros.actions.Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     output="screen",
            #     arguments=[
            #         "0",
            #         "0",
            #         "0",
            #         "-1.5707963267948966",
            #         "0",
            #         "-1.5707963267948966",
            #         "base_link",
            #         "camera_link",
            #     ],
            # ),
            stereo_image_proc_launcher,
            # launch_ros.actions.Node(
            #     package="rtabmap_util",
            #     executable="pointcloud_to_depthimage",
            #     output="screen",
            #     parameters=depth_params,
            #     remappings=depth_remmapings,
            # ),
            launch_ros.actions.Node(
                package="rtabmap_sync",
                executable="stereo_sync",
                output="screen",
                parameters=stereo_parameters,
                remappings=stereo_remmapings,
            ),
            # launch_ros.actions.Node(
            #     package="rtabmap_odom",
            #     executable="stereo_odometry",
            #     output="screen",
            #     parameters=odom_params,
            #     remappings=odometry_remmapings,
            # ),
            # launch_ros.actions.Node(
            #     package="rtabmap_slam",
            #     executable="rtabmap",
            #     output="screen",
            #     arguments=["--delete_db_on_start",],
            #     parameters=slam_parameters,
            #     remappings=slam_remmapings,
            # ),
            launch_ros.actions.Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                output="screen",
                parameters=viz_parameters,
                remappings=viz_remmapings,
            ),
        ]
    )
