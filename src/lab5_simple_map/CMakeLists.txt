cmake_minimum_required(VERSION 3.5)
project(lab5_simple_map)

# Установка стандарта C++ (используем 14 или новее для ROS2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Поиск зависимостей ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Включение директорий с заголовками
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# Создание исполняемого файла узла
add_executable(simple_map_node src/simple_map.cpp)

# Связывание узла с зависимостями
# target_link_libraries(simple_map_node
#   simple_map_lib
# )
ament_target_dependencies(simple_map_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Установка целей для установки
install(TARGETS
  simple_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

# Установка заголовочных файлов
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Добавление тестов (если нужно)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Экспорт зависимостей
ament_export_include_directories(include)
# ament_export_libraries(simple_map_lib)
ament_export_dependencies(
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Финальная настройка пакета
ament_package()
