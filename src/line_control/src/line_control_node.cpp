#include "rclcpp/rclcpp.hpp"
#include "line_control.h"

/*
* 1. Подобрать коэффициенты регулятора для устойчивого движения робота по траектории
* 2. Модифицировать функцию вычисления ошибки, чтобы робот двигался по овалу, задаваемому двумя окружностями одного радиуса и прямыми,
*   расстояние между центрами окружностей равно радиусу
*/

int main(int argc, char **argv)
{
  /**
   * Инициализация ROS2
   * Заменяет ros::init() из ROS1
   */
  rclcpp::init(argc, argv);

  /*
   * Создание объекта LineControl, который теперь является нодой ROS2
   * и выполняет всю работу
   */
  auto control = std::make_shared<LineControl>();
  
  RCLCPP_INFO(control->get_logger(), "go to spin");
  
  /**
   * Запуск обработки сообщений
   * Аналог ros::spin() в ROS1
   */
  rclcpp::spin(control);
  
  /**
   * Корректное завершение работы
   */
  rclcpp::shutdown();
  
  return 0;
}
