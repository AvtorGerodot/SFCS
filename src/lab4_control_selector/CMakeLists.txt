cmake_minimum_required(VERSION 3.5)
project(lab4_control_selector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Поиск необходимых зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Создание библиотеки с классами управления
add_library(control_classes
  src/dummy.cpp
  src/voyager.cpp
  src/wallFollower.cpp
)
ament_target_dependencies(control_classes
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

# Создание исполняемого файла для узла управления
add_executable(control_selector_node
  src/control_selector_node.cpp
)
target_link_libraries(control_selector_node
  control_classes
)
ament_target_dependencies(control_selector_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

# Создание исполняемого файла для узла выбора режима
add_executable(selector
  src/selector.cpp
)
ament_target_dependencies(selector
  rclcpp
  std_msgs
)

# Установка файлов
install(TARGETS
  control_selector_node
  selector
  control_classes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Установка launch-файлов
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Экспорт зависимостей
ament_export_libraries(control_classes)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

# Добавление макросов для пакета
ament_package()
