# Implementing Visual SLAM in Simulation

### This instruction offers guidance on configuring Visual SLAM (VSLAM) for DeepRacer within the simulation environment. 

## Overview

When implementing Visual SLAM, selecting a robust ROS-integrated package for crafting a 3D map is crucial. In this documentation, we have chosen the [rtabmap package](http://wiki.ros.org/rtabmap_ros) for this important task. With the rtabmap package, we can generate a 3D point cloud of the surrounding environment and a 2D occupancy grid map for streamlined navigation.

This package offers versatility by providing various approaches tailored to the available sensors and devices on the robot. Given that we are working with the Deepracer robot, we have access to stereo cameras and LiDAR sensors, which we can effectively leverage to construct our 3D map.

In the subsequent sections of this documentation, we will explore the `Stereo A` approach, as outlined in the [official setup page](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot). By adopting this approach, we can generate depth images and, by carefully employing the appropriate packages, ensure that the `rtabmap` package produces the map data as originally planned.

Before proceeding to the subsequent sections of this documentation, it is important to install the relevant packages mentioned on the linked page.

## Establishing Connections with Vision Processing Nodes
In this section, we leverage the `stereo_image_proc` package to perform critical vision processing tasks, including the correction of distortion and colorization of raw images, all conducted on calibrated cameras. Furthermore, we calculate the disparity image using [OpenCV's block matching](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereobm) algorithms, enabling us to generate a point cloud using this node. Disparity image can be shown using the below command:

	ros2 run image_view disparity_view image:=/disparity

However, before diving into these tasks, it is crucial to modify some of the sensor names defined in the `deepracer_description/models/xacro/sensor/deepracer_gazebo_stereo_cameras.xacro` file. This adjustment is essential to ensure the accurate remapping of raw image and camera information topics.
> `zed_camera_left_sensor` changed to `zed_camera/left`
>  `zed_camera_right_sensor` changed to `zed_camera/right`

  
As the URDF of the mentioned sensors is written in ROS1, we need to make changes to specific tag names in the xacro file mentioned above:

> `cameraName` changed to `camera_name`
> `frameName` changed to `frame_name`
> `hackBaseline` changed to `hack_baseline`

Additionally, the remapping settings need to be adjusted as follows:
```
<ros>
    <remapping>image_raw:=rgb/image_rect_color</remapping>
    <remapping>camera_info:=rgb/camera_info</remapping>
</ros>
```
To address the issue of not being able to view the point cloud in RViz, both `hack_baseline` values in the file should be set to `0.12`, which represents the distance between the two cameras.

By following the steps outlined in the [official page](http://wiki.ros.org/stereo_image_proc), we should be able to publish the relevant topics required for subsequent procedures.

## Generating Depth Images
By employing the `pointcloud_to_depthimage` package found within the `rtabmap_util` package, we can generate a depth image by reprojecting a point cloud into the camera frame.
Detailed examples and step-by-step instructions are readily accessible on their webpage. Simply follow the prescribed steps for remapping the topics, and you should be able to successfully create the depth image.

## Utilizing Visual Tools for Odometry
Similar to [Creating a 3D Map part](https://github.com/redHaunter/aws-deepracer/blob/main/docs/Visual_SLAM_Instructions_Sim.md#creating-a-3d-map) we have utilized the RGB-D images to calculate the robot's odometry using built-in RTAB-Map packages.
Using RGBD images, odometry is computed using visual features extracted from the RGB images with their depth information from the depth images. Using the feature correspondences between the images, a RANSAC approach, Perspective-n-Points (PnP), computes the transformation between the consecutive images.

## Creating a 3D Map
Refering to [rtabmap setup page ](http://wiki.ros.org/rtabmap_ros/Tutorials/SetupOnYourRobot#A2D_laser_only) we have used both stereo and RGB-D SLAM approaches.
The only problem we faced was that the projection matrix of stereo cameras was not properly set

>Projection/camera matrix
$$ P = \begin{bmatrix} fx' & 0 & cx' & Tx \\ 0 & fy' & cy' & Ty \\ 0 & 0 & 1 & 0 \end{bmatrix} $$
	-   P is a 3x4 matrix used in computer vision.
	-   It converts 3D camera coordinates to 2D pixel coordinates.
	-   The left 3x3 part contains camera intrinsic parameters.
	-   For monocular cameras, Tx = Ty = 0.
	-   In stereo pairs, Tx depends on baseline (B) and fx'.
	-   To project a 3D point [X Y Z] to 2D [x y]:
	    -   Compute [u v w] = P * [X Y Z 1]'
	    -   Calculate x = u / w and y = v / w for both stereo images.

And because the cameras was launched through gazebo plugins in a simulator, we didn't have the required access to change the cameras' matrix separately (which was needed to be generated through calibrations of cameras).
> Set_camera_info ROS service was same for both cameras and there was no individual access also it was difficult to do calibration in simulation


> Launch files of different SLAM setups can be found in **[deepracer_bringup](https://github.com/redHaunter/aws-deepracer/tree/main/deepracer_bringup)**
