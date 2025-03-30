cmake_minimum_required(VERSION 3.8)
project(lab2_robot_control)

# Настройка компилятора
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Добавьте эту строку
find_package(std_msgs REQUIRED)  # Добавьте эту строку
find_package(geometry_msgs REQUIRED)  # Добавьте эту строку
find_package(sensor_msgs REQUIRED)  # Добавьте эту строку
find_package(nav_msgs REQUIRED)  # Добавьте эту строку

# Сборка узла control_node
add_executable(control_node_1 src/robot_control_1.cpp)
ament_target_dependencies(control_node_1 rclcpp geometry_msgs sensor_msgs nav_msgs)  # Укажите зависимости

# Сборка узла control_node
add_executable(control_node_2 src/robot_control_2.cpp)
ament_target_dependencies(control_node_2 rclcpp geometry_msgs sensor_msgs nav_msgs)  # Укажите зависимости

# Установка узлов в папку lib
install(TARGETS
  control_node_1
  control_node_2
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
