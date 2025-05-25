cmake_minimum_required(VERSION 3.5)
project(lab6_patrol_bot)

# Установка стандарта C++ (используем 14 или новее для ROS2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Поиск зависимостей ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(nav2_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

add_executable(patrol_bot_node src/patrol_bot.cpp)
target_include_directories(patrol_bot_node PRIVATE
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

target_link_libraries(patrol_bot_node
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

install(TARGETS
  patrol_bot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Финальная настройка пакета
ament_package()
