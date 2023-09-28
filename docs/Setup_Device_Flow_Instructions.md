calibration - rplidar node

# Device

### This instruction explains how to setup the AWS DeepRacer robot and use open-source ROS-integrated packages to run Nav2 stack on the device.

## Robot preparation
In this section of the instructions, the robot will be prepared to be controlled manually or autonomously via the installed application on the robot. The potential issues have been mentioned so that the user can be prepared for them.
All the steps mentioned below can be found in the [official instructions](https://d1.awsstatic.com/deepracer/Evo%20and%20Sensor%20Launch%202020/AWS%20DeepRacer_and_Sensor_Guide.pdf) on the [AWS DeepRacer website](https://aws.amazon.com/deepracer/getting-started/), accompanied by detailed explanations and instructions with pictures.
### steps:
 - **Vehicle setup**
Before installing any components on the robot, ensure that both the compute battery and the vehicle battery are fully charged. Due to the limited lifespan of the compute battery during setup or connection testing, the robot can be connected to wall power using the included power cord.

	Charging the batteries typically takes approximately 2 hours, so plan accordingly. The instructions for charging the batteries are provided in the documentation, which you should refer to.
	
	When assembling the car, you'll need to connect several essential components to their respective parts, including the compute battery, vehicle battery, lidar sensor, and cameras. Detailed instructions can be found in the linked guide, but it's important to be aware of potential issues.
	
	When connecting the vehicle battery, make sure to firmly push in the red connector until you hear a clicking sound. If you don't hear the click, the battery might not be properly connected, which could lead to issues. Additionally, upon turning on the vehicle battery, listen for two short beep and a long beep that indicate the power is active. If you don't hear these sounds, it's advisable to check the wiring.
	
	Connecting the stereo cameras to the USB ports might be a bit challenging. Look for small bumps on the sides of the camera connectors; these bumps should not be visible when the cameras are correctly connected.

- **Turning the vehicle on**
To turn on the vehicle, follow these steps:
-Ensure that the board has power by connecting it to the wall power or by turning on the compute battery.
-Use the power button to turn on the vehicle.
-Wait for the booting process to complete. You'll know it's finished when the battery LED turns blue, indicating that the application is running and you can proceed to the next step.

	If the battery LED remains stuck on yellow (note that booting can take up to 2 minutes, so please wait for this period), there may be issues with the board's booting process. In such cases, you might need to update your device manually. Refer to the [relevant documentation](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-ubuntu-update.html) available on the website for instructions on performing a manual update.
	
	If you encounter an issue similar to the one mentioned in this [link](https://repost.aws/questions/QUX4ZvFGsBSqK0wFWFUA-KIw/deepracer-boot-error), please follow the instructions provided in the linked resource. After updating the device, you might encounter an error page before the booting process, but Ubuntu will eventually boot, resolving the problem.

	Please note that when turning off the vehicle, follow these steps:
-Hold the power button until the blue LED light turns off.
-Turn off the vehicle battery.
-Disconnect the compute battery cable from the board. This step is important to prevent the battery from draining.

- **Network connection**
In the setup steps for the robot, one of the crucial stages involves connecting the robot to Wi-Fi. To ensure a seamless connection between the robot and a device, it's important to utilize the same Wi-Fi network for both.

	To establish the robot's connection to the Wi-Fi network, you can use a cable to connect the robot to a computer system. By disabling Wi-Fi on the computer and relying on the cable connection, you can access the `deepracer.aws` website and use the password printed on the robot to configure the robot's Wi-Fi network.

	The procedure described above has been implemented on various operating systems, such as Ubuntu, Kubuntu, Windows, and macOS. However, a consistent and reliable connection was achieved solely using macOS, mainly because macOS recognizes a USB connection as a LAN connection, allowing for straightforward network setup. Since access to a macOS computer might not always be available, an alternative method was sought to connect the robot to a Wi-Fi network.

	Another method for Wi-Fi network configuration, detailed in [this document](https://d1.awsstatic.com/deepracer/AWS-DeepRacer-Getting-Started-Guide.pdf), involves the use of a flash drive containing a specific text file. The file should be named `wifi-creds.txt`, and its content must adhere to the specified format for the network's SSID and password, as outlined in [this source](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-troubleshooting-wifi-connection-first-time.html).

	Following a successful booting process and the robot's status light turning blue, insert the flash drive into the robot. If you've named the file correctly and populated it with the requisite information as described, the robot's Wi-Fi indicator should turn blue, indicating a successful connection to the Wi-Fi network.

	After using the robot for a period, another straightforward method of interacting with the robot's operating system involves connecting it to a monitor. Using a keyboard and mouse, you can establish a Wi-Fi connection through the operating system's settings, thus manually connecting to the desired network.

- **Test drive**
Utilize any device connected to the same access point as the robot. By entering the vehicle's IP address, you can access the installed application on the robot. For the password, input the code located on the underside of your car. In our case, the password has been updated to `@Deepracer1`.

	Confirm the correct connections of the cameras and lidar sensor by checking the green indicators located near the respective components. Additionally, the loaded page will display the vehicle's battery level.
	
	Whether in manual or autonomous mode, you can operate your vehicle without issues. Ensure the vehicle's speed is appropriately set to prevent collisions with objects in the environment.

- **Calibration**
Calibrating your robot is essential if you intend to use it through the provided application by the company. You can follow the steps outlined in [this link](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-calibrate-vehicle.html) to complete this process. Please be aware that, according to the requirements, you must ensure the robot is properly situated on the ground to allow movement or place the chassis on an object to prevent unintended motion.

## Open source connection:
Communication and navigation using Nav2 packages to traverse the generated map can be achieved through the open-source packages offered by the company on their [GitHub page](https://github.com/aws-deepracer). Before proceeding with the steps provided below, ensure that you have a computer system ready to act as a client, and that your system is equipped with ROS2 Foxy, as it will be used in the communication process.
Be sure to visit [the introduction](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md) available on their GitHub page for the comprehensive documentation and detailed instructions. Due to the challenges encountered while following their provided instructions, we have outlined all the specific details below, ensuring you have a comprehensive reference that doesn't solely rely on their documentation.

### steps:

- **Utilizing the Vehicle's Ubuntu Operating System**
The Ubuntu installation on the robot can serve as the server-side for communication, with all map creation and navigation processing taking place on the robot's processor.

	To interact with the OS, you can free up two USB ports by disconnecting the stereo cameras or lidar sensor, allowing you to connect a mouse and keyboard. Furthermore, you can link a monitor using the available HDMI port on the robot's board. Upon logging in, use the username `deepracer` and note that on our device, the password has been updated form default value of `deepracer` to `dp1`. After successfully logging in, you'll have the ability to engage with the operating system and modify the robot's processing functionalities.

- **Clone the related packages and install dependencies**
By utilizing the package available on GitHub, we can clone the open-source package onto our device. Once the robot's Ubuntu system is booted, follow the steps below to establish a ROS workspace and leverage it for utilizing the Nav2 packages.
	```
	mkdir -p ~/deepracer_nav2_ws
	cd ~/deepracer_nav2_ws
	git clone https://github.com/aws-deepracer/aws-deepracer.git
	cd ~/deepracer_nav2_ws/aws-deepracer/
	```
	Utilizing the `install_dependencies.sh` file, you can clone the relevant dependency packages: [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry) and [rplidar_ros](https://github.com/Slamtec/rplidar_ros/tree/ros2).
	```
	cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes && ./install_dependencies.sh
	```
	Using `rosws update` can fetch unreleased dependencies. Please note that due to the absence of the necessary dependencies in the subsequent packages, this step must also be performed within individual package.
	```
	source /opt/ros/foxy/setup.bash 
	cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes && rosws update
	cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes/rf2o_laser_odometry && rosws update
	cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes/rplidar_ros && rosws update
	cd ~/deepracer_nav2_ws/aws-deepracer/deepracer_nodes/aws-deepracer-camera-pkg && rosws update
	```
	> **Important Note**: An issue that has been highlighted on the [AWS DeepRacer GitHub issue page](https://github.com/aws-deepracer/aws-deepracer/issues/9) involves the modification of the package name from `rplidar_ros2` to `rplidar_ros` in the `deepracer.launch.py` file situated within the `deepracer_bringup` package. Prior to building the packages, ensure that you make this adjustment.
	>	Additionally, please note that the name of the `rplidar_node` has been changed from `rplidar_scan_publisher` to `rplidar_node` in recent developments. It is essential to update this name in the launch file to ensure proper functionality.
	
	In order to successfully build the packages, it is necessary to have certain `navigation2` and `nav2_bringup` packages installed. In the provided documentation, the use of `apt` for package installation is recommended, as it is considered the optimal and most reliable solution. However, in our situation, `apt` did not manage to identify the ideal version compatible with the appropriate ROS distribution for package installation. Consequently, we were required to manually clone and build these packages from the source, which can be found on the [navigation2 GitHub repository](https://github.com/ros-planning/navigation2).

	To execute this process, you can either create a new workspace or clone the repository into your existing workspace. Remember to source the installation each time you utilize any launch file from the Deepracer packages.

- **Building the packages**
Execute the following command to build all the packages related to the robot. Please be aware that the documentation omitted the creation of an `src` folder during workspace setup. As a result, when building the packages, the corresponding directories are created in the same directory, leading to a cluttered structure.
	```
	cd ~/deepracer_nav2_ws/aws-deepracer/ && colcon build --packages-select deepracer_interfaces_pkg deepracer_bringup cmdvel_to_servo_pkg enable_deepracer_nav_pkg rf2o_laser_odometry rplidar_ros camera_pkg servo_pkg
	```

- **Communicate via ssh**
To control the robot and issue commands without utilizing its internal Ubuntu system, establishing an SSH connection via a ROS-equipped computer system is necessary. This involves logging into the robot's installed application and enabling SSH connectivity. You can follow the instructions detailed in [this link](https://docs.aws.amazon.com/deepracer/latest/developerguide/deepracer-manage-vehicle-settings.html) to achieve this configuration.
 
	By employing the command `ssh deepracer@host_ip` and entering the password (`dp1`), establishing communication becomes straightforward.

- **Setup ROS_DOMAIN_ID**
In order to open RViz for viewing the generated map and navigate through it, it's essential to establish a network between the vehicle and the computer system. This allows the published topics to be visible and accessible from the client side for use in RViz.

	To configure the network, execute the following commands in each new terminal. It's crucial to run these commands on both the client and server sides. Creating an alias in the `.bashrc` file can streamline access to these commands. Additionally, it's important to disable the firewall to ensure a seamless connection on both ends.
	
	*Server:*
	```
	ufw disable
	export ROS_MASTER_URI=http://"self_ip:11311"
	export ROS_HOSTNAME="self_ip"
	expert ROS_DOMAIN_ID=5
	```
	*Client:*
	```
	ufw disable
	export ROS_MASTER_URI=http://"host_ip:11311"
	export ROS_HOSTNAME="self_ip"
	expert ROS_DOMAIN_ID=5
	```

- **Launch and use slam toolbox to create and save map**
To generate a map, you should execute the following commands. Ensure that all commands executed on the vehicle side through SSH are run as root using `sudo su`. To halt the `deepracer-core.service`, employ `systemctl stop deepracer-core`. Additionally, it's crucial to launch all the `ROS_DOMAIN_ID` related commands in each new terminal on both the vehicle and client sides.

	*Launch the AWS DeepRacer robot related packages (vehicle)*
	```
	source /opt/ros/foxy/setup.bash 
	source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash
	ros2 launch deepracer_bringup deepracer.launch.py
	```
	*Launch the ROS Navigation packages (vehicle)*
	```
	source /opt/ros/foxy/setup.bash
	source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash
	ros2 launch deepracer_bringup deepracer_navigation_dr.launch.py use_sim_time:=False params_file:=/home/deepracer/deepracer_nav2_ws/aws-deepracer/deepracer_bringup/config/nav2_slam_params.yaml
	```
	*Launch the SLAM toolbox (vehicle)*
	```
	source /opt/ros/foxy/setup.bash
	source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash
	ros2 launch deepracer_bringup slam_toolbox.launch.py use_sim_time:=False params_file:=/home/deepracer/deepracer_nav2_ws/deepracer/deepracer_bringup/config/slam_toolbox.yaml
	```
	*Run teleop twist keyboard to move the vehicle (computer)*
	```
	ros2 run teleop_twist_keyboard teleop_twist_keyboard
	```
	*Run rviz2 to view map (computer)*
	```
	ros2 run rviz2 rviz2
	```
	*Save the map (vehicle)*
	```
	ros2 run nav2_map_server map_saver_cli -f ~/map
	```
	If the Navigation2 package was built in a different workspace, it's essential to source that workspace as well. Additionally, you can utilize `scp` by running the command `scp deepracer@host_ip:~/map.yaml ~/` to transfer the generated map to your computer (execute this command on your computer).

- **Launch navigation related packages to view map and visualize in rviz**
To navigate through generated map, you should execute the following commands. Ensure that all commands executed on the vehicle side through SSH are run as root using `sudo su`. To halt the `deepracer-core.service`, employ `systemctl stop deepracer-core`. Additionally, it's crucial to launch all the `ROS_DOMAIN_ID` related commands in each new terminal on both the vehicle and client sides.

	Due to the limitations of Ackermann vehicles in performing pure rotations, and given that our robot employs Ackermann steering, it's crucial to modify two parameters in the `nav2_params_nav_amcl_dr_demo.yaml` file situated in the `deepracer_bringup` package. Specifically, the parameters `use_rotate_to_heading: false` and `allow_reversing: true` need to be adjusted. Once you've made these changes, ensure to rebuild the package to apply the modified values.

	As the robot's speed may not be correctly configured, it's necessary to modify the relevant parameters in the `constants.py` file located in `cmdvel_to_servo_pkg` within the `deepracer_nodes` package. Below are the adjusted values:

	After making these adjustments, remember to rebuild the package to ensure the changes take effect.

	*Launch the AWS DeepRacer robot related packages (vehicle)*
	```
	source /opt/ros/foxy/setup.bash 
	source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash
	ros2 launch deepracer_bringup deepracer.launch.py
	```
	*Launch the ROS Navigation stack (vehicle)*
	```
	source /opt/ros/foxy/setup.bash
	source ~/deepracer_nav2_ws/aws-deepracer/install/setup.bash
	ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/deepracer/map.yaml params_file:=/home/deepracer/deepracer_nav2_ws/aws-deepracer/deepracer_bringup/config/nav2_params_nav_amcl_dr_demo.yaml
	```
	*Visualize the robot in RViz* (computer)*
	```
	ros2 run rviz2 rviz2
	```
	If the Navigation2 package was built in a different workspace, it's essential to source that workspace as well. 
