from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    stereo_image_proc_dir = get_package_share_directory('stereo_image_proc')
    stereo_image_proc_launcher = IncludeLaunchDescription(
    launch_description_source=PythonLaunchDescriptionSource(
        launch_file_path=stereo_image_proc_dir + '/launch/stereo_image_proc.launch.py'),
        launch_arguments={'left_namespace': 'zed_camera_left_sensor',
                            'right_namespace': 'zed_camera_right_sensor'}.items())
    
    return LaunchDescription([
        stereo_image_proc_launcher
    ])
