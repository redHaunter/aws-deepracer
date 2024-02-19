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


