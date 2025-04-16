## Инструкция по запуску

1. Находясь в папке line_control, открываем терминал:
   
   - Сорсим ros foxy:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   - Собираем проект через colcon:
   ```bash
    colcon build
   ```
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
   - Запускаем launch:
   ```bash
    ros2 launch line_control line.launch.xml
   ```
