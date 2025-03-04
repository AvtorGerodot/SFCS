#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class TalkerNode : public rclcpp::Node {
public:
  TalkerNode() : Node("talker_node") {
    // Публикация в топик /chatter
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // Таймер для отправки сообщений
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&TalkerNode::callback, this));

    // Подписка на топик /reply
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "reply", 10, std::bind(&TalkerNode::reply_callback, this, std::placeholders::_1));
  }

private:
  void callback() {
    // Создание и отправка сообщения
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void reply_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Вывод ответного сообщения
    RCLCPP_INFO(this->get_logger(), "Received reply: '%s'", msg->data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerNode>());
  rclcpp::shutdown();
  return 0;
}
