cmake_minimum_required(VERSION 3.5)
project(paraceptor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DroneCoords.msg"
  DEPENDENCIES sensor_msgs std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(sensor_msgs std_msgs)

# Install launch files (if any)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Finalize
ament_package()

