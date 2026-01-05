cmake_minimum_required(VERSION 3.5)
project(curvature_velocity_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(curvature_velocity_node src/curvature_velocity_node.cpp)

# Link libraries
ament_target_dependencies(curvature_velocity_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install targets
install(TARGETS curvature_velocity_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
