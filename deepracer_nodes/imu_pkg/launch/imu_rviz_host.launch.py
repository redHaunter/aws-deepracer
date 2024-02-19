# Author: Robofoundry
# Date: Apr 2, 2022
# Description: Launch file to start nodes for RPi


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  pkg_share = get_package_share_directory('imu_pkg')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_config_dir = os.path.join(get_package_share_directory('imu_pkg'), 'config')
  robot_name_in_urdf = 'plane'
  default_rviz_config_path = os.path.join(get_package_share_directory('imu_pkg'), 'rviz', 'imu_visual.rviz')
  imu_params_config = os.path.join(get_package_share_directory('imu_pkg'), 'config', 'imu_params.yaml')
  
  urdf_path = os.path.join(get_package_share_directory('imu_pkg'), 'urdf', 'plane.urdf.xacro')
  robot_description = Command(['xacro ', urdf_path])

  
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': robot_description, }],
    arguments=[urdf_path])
    

  
  # Launch RViz
  start_rviz_cmd = Node(
    #condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    #name='rviz2',
    output='screen',
    arguments=['-d', default_rviz_config_path])    


  
  ld = LaunchDescription()
  ld.add_action(start_rviz_cmd)
  ld.add_action(robot_state_publisher)


  return ld
