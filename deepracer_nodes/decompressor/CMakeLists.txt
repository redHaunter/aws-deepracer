cmake_minimum_required(VERSION 3.5)
project(decompressor)

# Set the C++ standard to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find ROS 2 packages and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_calibration_parsers REQUIRED)

# Add your package dependencies here
# find_package(<dependency> REQUIRED)

# Create an executable for your node
add_executable(decompression_node src/decompressor.cpp)

# Include directories for the executable
target_include_directories(decompression_node
  PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

# Link your executable with ROS 2 libraries
ament_target_dependencies(decompression_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  camera_calibration_parsers
  # Add your dependencies here
  # <dependency>
)

# Install the executable
install(TARGETS
  decompression_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, if any
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

# Install other files, if any
# install(FILES
#   myfile.txt
#   DESTINATION share/${PROJECT_NAME}/
# )

ament_package()
