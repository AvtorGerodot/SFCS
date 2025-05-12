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

add_executable(patrol_bot src/patrol_bot.cpp)
target_include_directories(patrol_bot PRIVATE
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

target_link_libraries(patrol_bot
  ${rclcpp_LIBRARIES}
  ${rclcpp_action_LIBRARIES}
  ${nav2_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

install(TARGETS
  patrol_bot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# # Включение директорий с заголовками
# include_directories(
#   # include
#   ${rclcpp_INCLUDE_DIRS}
#   ${nav_msgs_INCLUDE_DIRS}
#   ${geometry_msgs_INCLUDE_DIRS}
#   ${sensor_msgs_INCLUDE_DIRS}
#   ${tf2_ros_INCLUDE_DIRS}
# )

# # Создание исполняемого файла узла
# add_executable(simple_map_node src/simple_map.cpp)

# # Связывание узла с зависимостями
# # target_link_libraries(simple_map_node
# #   simple_map_lib
# # )
# ament_target_dependencies(simple_map_node
#   rclcpp
#   nav_msgs
#   sensor_msgs
#   tf2_ros
#   tf2_geometry_msgs
# )

# # Установка целей для установки
# install(TARGETS
#   simple_map_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# Добавление тестов (если нужно)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Экспорт зависимостей
#ament_export_include_directories(include)
# ament_export_libraries(simple_map_lib)
# ament_export_dependencies(
#   rclcpp
#   nav_msgs
#   sensor_msgs
#   geometry_msgs
#   tf2_ros
#   tf2_geometry_msgs
# )

# Финальная настройка пакета
ament_package()