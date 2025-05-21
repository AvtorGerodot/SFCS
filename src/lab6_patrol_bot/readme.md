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
    ros2 launch lab6_patrol_bot patrol.launch.xml
   ```
   

2. Открываем второй терминал в папке SFCS:
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
   - Запускаем slam:
   ```bash
    ros2 launch lab6_patrol_bot online_async_launch.py
   ```


3. Открываем третий терминал в папке SFCS:
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
   - Запускаем nav2:
   ```bash
    ros2 launch lab6_patrol_bot navigation_launch.py use_sim_time:=true
   ```


4. В открывшейся программе rviz необходимо добавить через меню окна `Displays` `Add`-> `By Topic` топики с карты `/map` и карты от nav2 `/global_costmap/costmap`, выставляем в `Color Scheme` 
   значение `costmap` для наглядности

   С помощью кнопки `2D Goal Pose` сверху экрану выставляем роботу позицию на карте 
