cmake_minimum_required(VERSION 3.8)
project(spline_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(spline_planner_node src/spline_planner_node.cpp)

target_include_directories(spline_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(spline_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs)

# Install executable
install(TARGETS spline_planner_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()