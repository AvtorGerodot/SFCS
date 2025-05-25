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

2. В открывшейся программе rviz необходимо добавить через меню окна `Displays` `Add`-> `By Topic` топик с карты `/map` 

   С помощью кнопки `2D Goal Pose` сверху экрану задаём маршрут робота, состоящий из нескольких точек.

3. Открываем второй терминал в папке SFCS, после того, как задали маршрут:
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
   - Запускаем "стартер":
   ```bash
    ros2 topic pub /navigation_status std_msgs/msg/String "{data: 'Reached goal'}" -1
   ```
