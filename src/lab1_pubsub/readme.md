## Инструкция по запуску
1. Первый способ запуска (talker и listener в одном терминале):
   - Сорсим ros foxy:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   - Собираем проект через colcon, находясь в SFCS:
   ```bash
    colcon build
   ```
   - Сорсим install:
   ```bash
    source install/setup.bash
   ```
    - Запускаем launch:
   ```bash
    ros2 launch lab1_pubsub lab1_pubsub.launch.py
   ```

2. Второй способ запуска (talker и listener в разных терминалах):
    - Открываем первый терминал в папке SFCS и прописываем:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   ```bash
    colcon build
   ```
   ```bash
    source install/setup.bash
   ```
   ```bash
    ros2 run lab1_pubsub talker_node
   ```

   - Открываем второй терминал в папке SFCS и прописываем в первом терминале:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   ```bash
    source install/setup.bash
   ```
   ```bash
    ros2 run lab1_pubsub listener_node
   ```