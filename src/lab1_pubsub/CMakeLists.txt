cmake_minimum_required(VERSION 3.8)
project(lab1_pubsub)

# Настройка компилятора
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # Добавьте эту строку
find_package(std_msgs REQUIRED)  # Добавьте эту строку

# Сборка узла listener_node
add_executable(listener_node src/listener.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)  # Укажите зависимости

# Сборка узла talker_node
add_executable(talker_node src/talker.cpp)
ament_target_dependencies(talker_node rclcpp std_msgs)  # Укажите зависимости

# Установка узлов в папку lib
install(TARGETS
  listener_node
  talker_node
  DESTINATION lib/${PROJECT_NAME})

# Включение тестирования (опционально)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Завершение настройки пакета
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
