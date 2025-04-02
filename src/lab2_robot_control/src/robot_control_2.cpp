#include "rclcpp/rclcpp.hpp"        //Основной заголовочный файл ROS2 для C++, содержащий основные классы и функции для работы с нодами
  //Заголовочный файл для сообщения типа String из стандартного пакета std_msgs
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class ControlNode : public rclcpp::Node {  //Определение класса ListenerNode, который наследуется от rclcpp::Node
public:
    ControlNode() : Node("control_node"), obstacle(false) {  //Конструктор класса. Инициализирует ноду с именем "listener_node"
    //cоздаём подписку на топик "chatter" и функцией обработки callback
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "base_scan", 10, 
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->laserCallback(msg);
      });

    //создаём издателя-ответчика, публикующего сообщения типа String в топик "reply"
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);  

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::timerCallback, this));
  }

private:
    bool obstacle;
    double err = 0;
    double int_err = 0;
    double old_err = 0;
    double range = 1;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    const double kMinRange = 0.5;
    obstacle = false;
	double curr_range = msg->ranges[0];
    for (size_t i = 0; i<msg->ranges.size(); i++)  //проверим нет ли вблизи робота препятствия
    {
        if (msg->ranges[i] < curr_range)
		{
		     curr_range = msg->ranges[i];
		}
        if (msg->ranges[i] < kMinRange)
        {
            obstacle = true;
            RCLCPP_WARN(this->get_logger(),"OBSTACLE");
            break;
        }
    }
    err = range - curr_range;
  }

  void timerCallback(){
    geometry_msgs::msg::Twist cmd;
    if (!obstacle)
	{
		int_err += err; //интегральная ошибка
		double dif_err = err - old_err; //дифференциальная ошибка
		old_err = err;
		cmd.linear.x = 0.5;
		cmd.angular.z = 11 * err + 0.02 * int_err + 5.3 * dif_err; //ПИД-регулирование
	}
	else
	{
		cmd.linear.x = 0;
		cmd.angular.z = 0.5;
	}

    publisher_->publish(cmd);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
