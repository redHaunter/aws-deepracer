
# Introduction to the ROS Navigation Stack using AWS DeepRacer Evo


  
  

**AWS DeepRacer components for integration:**

  

* **[deepracer_bringup](https://github.com/redHaunter/aws-deepracer/tree/main/deepracer_bringup):** Launch and configuration parameter files

* **[deepracer_description](https://github.com/redHaunter/aws-deepracer/tree/main/deepracer_description):** URDF files for the AWS DeepRacer device
>
* **[deepracer_gazebo](https://github.com/redHaunter/aws-deepracer/tree/main/deepracer_gazebo):**  `deepracer_drive `plugin required to move the car in simulation

* **[deepracer_nodes](https://github.com/redHaunter/aws-deepracer/tree/main/deepracer_nodes):** Set of nodes that are responsible for making the device compatible with the navigation and SLAM packages



  

In our case, the AWS [DeepRacer Evo](https://aws.amazon.com/deepracer/) robot is an 1/18th scale 4WD with monster truck chassis and Ackermann drive type platform with independent servo and motors to control the wheels. The AWS DeepRacer hardware consists of two independent forward facing 4 MP RGB cameras, a 360 degree planar LiDAR (restricted to view only 300 degrees with reverse orientation), an integrated accelerometer and gyroscope providing the IMU data. More details about the AWS DeepRacer hardware and simulation artifacts can be found [here](https://github.com/aws-deepracer/aws-deepracer).

  

There are two flows for using ROS Nav2 on AWS DeepRacer, the Simulation flow and the Device flow. Both flows have different prerequisites to work with Nav2 stack.

  

## **Coordinate frame standards and state estimation requirements**:

  

The [REP-105](https://www.ros.org/reps/rep-0105.html) (ROS Enhancement Proposal) specifies naming conventions and the semantic meaning for coordinate frames of mobile platforms used with ROS. The coordinate frame commonly called `base_link` is rigidly attached to the mobile robot base. The coordinate frame called `odom` is a world-fixed frame that has to be continuously updated. This frame helps to get the locally accurate localization. The coordinate frame called `map` is a world fixed frame, with its Z-axis pointing upwards. This can have discrete jumps and helps in determining global pose estimates. Along with these, there are few static transformations with the sensor pose data (`base_link → camera_link` and `base_link → laser`) published to indicate the positions of the sensors.

  

According to community standards, there are 2 major transformations involving the coordinate frames that need to be provided. The `map` to `odom` transform is provided by a positioning system (localization, mapping, SLAM) and `odom` to `base_link` by an odometry system. This `map → odom → base_link` frames and their corresponding TF transformations are the core requirements for any robot to work with the navigation stack.




In AWS DeepRacer, the [deepracer_gazebo](https://github.com/aws-deepracer/aws-deepracer/tree/main/deepracer_gazebo) package contains the gazebo plugin responsible for providing the ground truth odometry information along with subscription to the Twist messages published on `/cmd_vel` topic in the simulation flow, and the [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) package provides the odometry information based on the consecutive LiDAR laser scan messages in the device flow. The [deepracer_nodes](https://github.com/aws-deepracer/aws-deepracer/tree/main/deepracer_nodes) contains the required nodes to convert the Twist messages published on `/cmd_vel` topic to servo messages.

