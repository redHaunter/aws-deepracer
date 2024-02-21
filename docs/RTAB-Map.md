# Visual SLAM document

Visual SLAM relies on cameras or other visual sensors to gather information about the surroundings. In our case the visual sensor is a stereo camera which installed at the top of the robot stack in front part of the robot.

In this document we explain about visual SLAM by using the **RTAB-Map** library. RTAB-Map short for Real-Time Appearance-Based Mapping, is a graph-based SLAM approach. Appearance-based SLAM means that the algorithm uses data collected from vision sensors to localize the robot and map the environment. A process called **loop closures** is used to determine whether the robot has seen a location before. As the robot travels to new areas in its environment, the map is expanded, and the number of images that each new image must be compared to increases. This causes the loop closures to take longer but with complexity increasing linearly. RTAB-Map is optimized for large-scale and long-term SLAM by using multiple strategies to allow for loop closure to be done in real-time. The loop closure is happening fast enough that the result can be obtained before the next camera images are acquired.

## Frontend and Backend of RTAB-Map

### Frontend
The frontend of RTAB-Map focuses on the sensor data used to obtain the constraints that are used for feature optimization approaches (odometry constraints and loop closure constraints are considered here). The odometry constraints can come from wheel encoders, IMU, LiDAR, or visual odometry. Visual odometry can be accomplished using 2D features such as SIFT/SURF.

![Rtabmap front-end](https://miro.medium.com/v2/resize:fit:720/format:webp/1*4hdAzal7eCk7TlkddFlAOA@2x.png)


### Backend
The backend of RTAB-Map includes the graph optimization and an assembly of an occupancy grid from the data of the graph.

![enter image description here](https://miro.medium.com/v2/resize:fit:720/format:webp/1*XcQtXUNeuofOD-InkkpusQ@2x.png)

## Loop Closures

Loop closure is the process of finding a match between the current and previously visited locations in SLAM. There are two types of loop closure detections: local and global.

### Local Loop Closures
The matches are found between a new observation and a limited map region. The size and location of this limited map region are determined by the uncertainty associated with the robot’s position. This type of approach fails if the estimated position is incorrect.

### Global Loop Closures
In this approach, a new location is compared with previously viewed locations. If no match is found, the new location is added to the memory. As the robot moves around and the map grows, the amount of time to check the new locations with ones previously seen increases linearly. If the time it takes to search and compare new images to the one stored in memory becomes larger than the acquisition time, the map becomes ineffective.


> RTAB-Map uses global loop closures along with other techniques to ensure that the loop closure process happens in real-time.


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


# Implementing Visual SLAM on Device

### This instruction provides guidance on configuring Visual SLAM (VSLAM) for DeepRacer and deploying it on the robot.

## Overview
Setting up VSLAM on the robot can be a complex task, but with the assistance of the `RTAB-Map` package and the knowledge gained from our simulation experiences, we can successfully configure it to generate a 3D map.

As the robot plays a crucial role in this process, there is a need to install certain packages on the robot itself. The subsequent steps will clarify the process of installing these packages.

Additionally, as outlined in the device setup, the connection between the robot and your computer is established via a local network using SSH. It is essential that both the robot and the computer are connected to the same network. Furthermore, during the execution of the following steps, ensure that both cameras and the LiDAR are properly connected to the robot.

## publishing camera related topics

To use cameras properly in ROS, it is essential that images and their camera info data are publishing in ROS within the same timestamps.
We have modified the camera_node to only publish the compressed data of each camera's in jpeg format in  `compressed_image` topics seperately.
> The code can be found in [camera_pkg](https://github.com/redHaunter/aws-deepracer/blob/robot/deepracer_nodes/camera_pkg/src/camera_node.cpp)
Then in [decompressor](https://github.com/redHaunter/aws-deepracer/blob/main/deepracer_nodes/decompressor/src/decompressor.cpp) code we decompress the jpeg images to raw data and publish in our ROS-machine in `image_raw` topic. This will prevents the network overload.
- **Camera Info**
We publish our camera info topics calibrated with [Camera Calibration](https://github.com/redHaunter/aws-deepracer/blob/main/docs/Visual_SLAM_Instructions_Device.md#camera-calibration) in the [decompressor](https://github.com/redHaunter/aws-deepracer/blob/main/deepracer_nodes/decompressor/src/decompressor.cpp) in `camera_info` topic using the `camera_calibration_parsers` library.

	> Time stamp of both cameras `image_raw` and `camera_info` needed to be set as same.

- **Network Overload**
The main core of the robot is not proper for image processing tasks so we decided to send data over ROS network and then use it on our own ROS machines.
Robot cameras capture 640x480 resolution images at 30 frames per second, by a simple calculation we can see that the stereo camera has 55MB/s data to publish which compared to our network card with 22.5MB/s bandwidth is a lot of overload. For that, we have done an image compression to `jpeg` format by using `OpenCV` library to lower the size of the image and prevent the network's overload.
	> jpeg quality of 90 has been used which can be change in `camera_pkg` launch file.

- **ROS Domain Bridge**
We also separated the `ROS_DOMAIN_ID`of the environment that is integrated with publishing and receiving camera data from the environment that is integrated with processing images inside our ROS machine by using [ROS-domain-bridge](https://github.com/ros2/domain_bridge) library. By that, we make sure that there are no more network overloads.

	>There is a decompressor node available in our ROS machine to extract raw images and publish them into ROS.


## Camera Calibration
Utilizing camera calibration is a fundamental step for advancing image processing tasks. It serves various critical purposes such as distortion removal and precise reconstruction, particularly for stereo cameras, which capture multiple images of the same scene from slightly different angles. Additionally, camera calibration provides accurate measurements and metric information by establishing a mapping between pixel coordinates in the image and real-world coordinates. This metric data is vital for applications like object size estimation, distance measurement, and, in our specific case, depth estimation.

To perform camera calibration, you can follow the steps outlined in this [link](https://navigation.ros.org/tutorials/docs/camera_calibration.html) while using the `camera_calibration` package and its `cameracalibrator.py` node, and ensure that you have installed [these packages](https://navigation.ros.org/tutorials/docs/camera_calibration.html) on your computer system. This process involves using a printed checkerboard and enables you to acquire new `camera_info` values based on the measurements generated during calibration. These calibrated values significantly improve the accuracy and reliability of subsequent image processing tasks.

## Generating 3D Map

The process is similar to the one outlined in the simulation part, with minor adjustments to the remapping configurations and by employing RTAB-Map, we can generate a 3D map of the environment.

The `image_raw` topics in this case have different names with `camera_pkg` as their namespace, and making the necessary remapping changes should be sufficient to apply the steps we undertook in the simulation part.