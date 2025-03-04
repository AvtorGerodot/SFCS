#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node {
public:
  ListenerNode() : Node("listener_node") {
    // Подписка на топик /chatter
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, std::bind(&ListenerNode::callback, this, std::placeholders::_1));

    // Публикация в топик /reply
    publisher_ = this->create_publisher<std_msgs::msg::String>("reply", 10);
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    // Вывод полученного сообщения
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

    // Создание ответного сообщения
    auto reply_msg = std_msgs::msg::String();
    reply_msg.data = "Reply to: " + msg->data;

    // Отправка ответного сообщения
    publisher_->publish(reply_msg);
    RCLCPP_INFO(this->get_logger(), "Sent reply: '%s'", reply_msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ListenerNode>());
  rclcpp::shutdown();
  return 0;
}
