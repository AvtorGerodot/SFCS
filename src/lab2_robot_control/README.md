## Инструкция по запуску
1. Открываем первый терминал(там будет Noetic и roscore):
   - Сорсим ros noetic:
   ```bash 
   source /opt/ros/noetic/setup.bash
   ```
   ```bash 
   roscore
   ```
3. Открываем workpace с симуляцией во втором терминале:
   - Сорсим ros noetic:
   ```bash 
   source /opt/ros/noetic/setup.bash
   ```
   //если у вас нет симуляции, сначала качайте отсюда и собираете через catkin: https://github.com/AndreyMinin/ros_course
   - Находясь в workpace, где у вас лежит симуляция, сорсим:     
   ```bash 
   source devel/setup.bash           
   ```
   - Запускаем симуляцию:     
   ```bash 
   roslaunch stage_launch task1.launch          
   ```
4. Создаем мост между ros и ros2 в третьем терминале:
   - **ВАЖНО!!!** Сначала нужно засорсить первый ROS, потом второй в одном терминале:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   - Запускаем мост:
   ```bash
   ros2 run ros1_bridge dynamic_bridge
   ```
5. Запускаем управляющую программу:
   - Сорсим ros foxy:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   - Собираем проект через colcon, находясь в SFCS:
   ```bash
    coclcon build
   ```
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
   - Запускаем control_node_1 (робот вращается при виде стены)
   или control_node_1 (робот едет вдоль стены):
   ```bash
    ros2 run lab2_robot_control control_node_1
   ```
   ИЛИ
   ```bash
    ros2 run lab2_robot_control control_node_2
   ```
   
