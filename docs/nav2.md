# Nav2 document

Three main components of the Nav2 stack are planners, controllers, and recoveries. Planners and controllers are at the main parts of a navigation task. The task of a planner is to compute a path to complete an objective function. Controllers, also known as local planners in ROS 1, are the way the robot follows the globally computed path or completes a local task. The controller will have access to a local environment representation to attempt to compute feasible control efforts for the robot base to follow. The general task in Nav2 for a controller is to compute a valid control effort to follow the global plan. Recoveries are used to get the robot out of a bad situation or attempt to deal with various forms of issues to make the system fault-tolerant. The goal of recoveries are to deal with unknown or failure conditions of the system and autonomously handle them. To learn more about other components such as behavior trees, navigation servers, and action servers.

  

**Nav2 tools** (from [Nav2 documentation](https://navigation.ros.org/))**:**

  

* **Nav2 Planner**: Plans a path from A to B around obstacles

* **Nav2 Controller:** Controls the robot as it follows the path

* **Nav2 Recoveries:** Compute recovery behaviors in case of failure

* **Nav2 Behavior Trees and BT Navigator:** Builds complicated robot behaviors using behavior trees

* **Nav2 Costmap 2D**: Converts sensor data into a costmap representation of the world

* **AMCL:** Localizes the robot on the map

* **Map Server:** Loads, serves, and store maps

* **Nav2 Waypoint Follower:** Follows sequential waypoints

* **Nav2 Lifecycle Manager**: Manage the lifecycle and watchdog for the servers

* **Nav2 Core:** Plugins to enable your own custom algorithms and behaviors

  
  

The Nav2 stack defines plugin interfaces for you to create your own custom costmap layer, planner, controller, behavior tree or recovery plugins. One of the powerful features of the Nav2 stack of packages is this ability to extend usage by adding more custom plugins that can be configured to be used through parameter files. Each plugin implements a specific behavior and supports a particular set of robot base platforms. These plugins can then be further configured to the platform and sensor specifications of the robot.