cmake_minimum_required(VERSION 3.5)
project(line_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(line_control_node
  src/line_control_node.cpp
  src/line_control.cpp
)

target_include_directories(line_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ament_INCLUDE_DIRS}
)

ament_target_dependencies(line_control_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS line_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
)

ament_package()
