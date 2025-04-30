## Инструкция по запуску

1. Находясь в папке SFCS, открываем первый терминал:
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
    ros2 launch lab4_control_selector cntrl_selector.launch.xml
   ```
2. Открываем второй терминал в папке SFCS:
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
   - Запускаем launch:
   ```bash
    ros2 run lab4_control_selector selector
   ```