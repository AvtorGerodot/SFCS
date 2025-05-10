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
    ros2 launch lab5_simple_map map.launch.xml
   ```
   В открывшейся программе rviz необходимо выставить `fixed_frame` в значение `odom`, добавить через меню окна `Displays` `Add`-> `By Topic` топики с данными дальномера `/base_scan` и карты '/simple_map'

2. Открываем второй терминал в папке SFCS:
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
   - Запускаем selector:
   ```bash
    ros2 run lab4_control_selector selector
   ```