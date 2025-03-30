# Software for control systems

Репозиторий команды №1 по предмету "Программное обеспечение управляющих комплексов"  
Состав Команды:
- Петров Михаил		СМ7-63Б  
- Примова Полина	СМ7-63Б  
- Гергет Данил		СМ7-64Б  
- Шишкин Максим		СМ7-64Б  
- Яковлев Егор		СМ7-64Б  

#Лабораторная работа №1:  
Пакет для лабораторной работы:  
**"lab1_pubsub"**

*описание*

## Инструкции
### Как настроить мост между ROS1 и ROS2?
1. Скачиваем пакет с мостом для ваших версий ROS-ов(для примера: Noetic-Foxy):
   
   ```bash
   sudo apt-get install ros-foxy-ros1-bridge
   ```
3. Как это запустить:
   - открываем первый терминал(там будет Noetic и roscore):   
   ```bash 
   source /opt/ros/noetic/setup.bash
   ```
   ```bash 
   roscore
   ```  
   - открываем второй терминал(там будет мост):
   - **ВАЖНО!!!** Сначала нужно засорсить первый ROS, потом второй в одном терминале:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   - запускаем мост:
   ```bash
   ros2 run ros1_bridge dynamic_bridge
   ```
   - УРА!
4. Тестируем:
   - открываем ещё один терминал и отпраляем сообщение из первого ROS-а:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```
   ```bash
   rostopic pub /test_topic std_msgs/String "data: 'Hello from ROS 1'"
   ```
   - открываем ещё один терминал и получаем сообщение на второй ROS:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   ```bash
   ros2 topic echo /test_topic std_msgs/String
   ```
   - чиназес)
5. (Не обязательно) Можно провести обратный тест - отправить сообщение с Foxy(ROS2) на Noetic(ROS1):
   - команды для терминала с Foxy:
   ```bash
   source /opt/ros/foxy/setup.bash
   ```
   ```bash
   ros2 topic pub /test_topic std_msgs/String "data: 'Hello from ROS 2'"
   ```
   - команды для терминала с Noetic:
   ```bash
   source /opt/ros/noetic/setup.bash
   ```
   ```bash
   rostopic echo /test_topic 
   ```
