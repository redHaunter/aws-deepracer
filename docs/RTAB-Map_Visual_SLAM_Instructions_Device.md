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