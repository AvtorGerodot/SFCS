#include "rclcpp/rclcpp.hpp"        //Основной заголовочный файл ROS2 для C++, содержащий основные классы и функции для работы с нодами
  //Заголовочный файл для сообщения типа String из стандартного пакета std_msgs
#include <chrono>
#include "geometry_msgs/msg/Twist.msg"
#include "sensor_msgs/msg/LaserScan.msg"
#include "nav_msgs/msg/Odometry.msg"

class ControlNode : public rclcpp::Node {  //Определение класса ListenerNode, который наследуется от rclcpp::Node
public:
    ControlNode() : Node("control_node") {  //Конструктор класса. Инициализирует ноду с именем "listener_node"
    //cоздаём подписку на топик "chatter" и функцией обработки callback
    odomSubscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/base_scan", 10, std::bind(&ControlNode::laserCallback, this, std::placeholders::_1));

    //создаём издателя-ответчика, публикующего сообщения типа String в топик "reply"
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);  

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::timer_callback, this));
  }

private:
  bool obstacle;
//   void odomCallback(nav_msgs::msg::Odometry msg) {
//     // Вывод полученного сообщения
//     RCLCPP_INFO(this->get_logger(), "Pose msg: x = '%f'\n
//       y = %f'\n
//       theta =  %f'\n", msg.pose.pose.position.x, msg.pose.pose.position.y, 
//       2*atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w));
//   }

  void laserCallback(sensor_msgs::msg::LaserScan msg){
    const double kMinRange = 0.3;
    obstacle = false
    for (size_t i = 0; i<msg.ranges.size(); i++)  //проверим нет ли вблизи робота препятствия
    {
        if (msg.ranges[i] < kMinRange)
        {
            obstacle = true;
            RCLCPP_INFO(this->get_logger(),"OBSTACLE!!!");
            break;
        }
    }
  }

  void timerCallback(){
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0,5;
    publisher_->publish(cmd);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};