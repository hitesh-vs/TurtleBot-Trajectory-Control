cmake_minimum_required(VERSION 3.8)
project(odom_logger)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings (good practice)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- Find dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ---------------- Build node ----------------
add_executable(odom_logger_node
  src/odom_logger_node.cpp
)

ament_target_dependencies(odom_logger_node
  rclcpp
  nav_msgs
)

install(TARGETS
  odom_logger_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
