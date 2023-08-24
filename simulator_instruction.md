# simulator

### This instruction explains how to simulate your aws-deepracer model in gazebo-classic in order to see robot actions and moves in a pre-mapped world.
  

## installing ros2 (foxy distribution)

Because of the heavy system integration ros2 makes, it is recommended to install ros2 in a container, so the environment will be instinct from the main system. To that so we have used [distrobox](https://github.com/89luca89/distrobox)  with the docker image of ros2 foxy
```
distrobox create --image osrf/ros:foxy-desktop --name foxyros
```
>vpn required
```
distrobot enter foxyros
```
## installing gazebo-classic
```
curl -sSL http://get.gazebosim.org | sh
```
  
 
## installing dependencies


Rosdep is a package manager which deep-searches packages inside a ros2 workspace and installing it's dependencies and requierd packages. Unfortunately foxy distribution end-of-life has become and it is no more supported by rosdep, so we had to installing the packages manually with "apt" package manager
```
sudo apt-get install ros-foxy-gazebo-ros2-control
sudo apt-get install ros-foxy-ros2-controllers
sudo apt-get install ros-foxy-ros2-control
sudo apt-get install ros-foxy-rqt*
sudo apt-get install ros-foxy-gazebo-dev
sudo apt-get install ros-foxy-gazebo-ros-pkgs
sudo apt-get install ros-foxy-navigation2
sudo apt-get install ros-foxy-nav2-bringup
sudo apt-get install ros-foxy-xacro
```
>vpn required


## code changes
For loading the robot model in your world, you need to define the transformation between the robot odometry and the world, it can be done by set a static transformation from map to odom, simply add the following code to the end part of deepracer_bringup/launch/deepracer_spawn.launch.py launch file
```
launch_ros.actions.Node(
	package='tf2_ros',
	executable='static_transform_publisher',
	output='screen',
	arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
	parameters=[
	deepracer_bringup_dir  +  '/config/static_tf.yaml'
	]
),
``` 

## world
We do recommend that to address map and world file using absolute address in deepracer_bringup/launch/nav_amcl_demo_sim.launch.py, instead of getting_package_share_directory. We also downloaded a already built and mapped world of a bookstore for our simulator.
```
git clone https://github.com/aws-robotics/aws-robomaker-bookstore-world.git
```
> no need to build the package

The first time world gets load in your gazebo simulator it might take several minutes, note that you have to be in the package directory because of the way world file load models and set models path to gazebo.
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models:/home/"your_username"/ros2_ws/src/aws-robomaker-bookstore-world/models
cd aws-robomaker-bookstore-world
gazebo worlds/bookstore.world
```
>change "your_username"
>you can also add the first line to your ~/.bashrc file
## build
use colcon to build the packages of aws-deepracer repository in your workspace
```
cd "your_workspace_directory"
colcon build
```

## run the simulator
Each launch file of deepracer_bringup package represent different nodes, services, and etc. So we can use our robot in simulated or physical environment, in our case we have used the nav_amcl_demo_sim.launch.py launch file which is a combination of some other launch files and also some more applications, [read more](https://github.com/redHaunter/aws-deepracer/blob/main/deepracer_bringup/README.md).

Now we can run our robot model in our preferred world:
```
source /opt/ros/foxy/setup.bash
source ~/"your_workspace_directory/install/setup.bash
cd aws-robomaker-bookstore-world
```
>last line needed to load the world without errors

**use teleop to control the robot**
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> or any other way to publish Twist message to /cmd_vel topic in ros2

 **use nav2 package of ros2 by setting goal in rviz2**
 ```
 ros2 run rviz2 rviz2
 ```
 open the config file from "deepracer_description/rviz/nav2_default_view.rviz" in the rviz2
 >map should be loaded after openning the config file

and then you can set goal_pose in rviz2 by using the toolbar of rviz2, trajectory will be planned and executed by nav2 package of ros2 both in gazebo and rviz2 simulators.