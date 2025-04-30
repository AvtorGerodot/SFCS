#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include <iostream>

void menu()
{
    std::cout << "\nChoose regime:\n";
    std::cout << "0) Dummy\n";
    std::cout << "1) Voyager\n";
    std::cout << "2) Wallfollower\n";
    std::cout << "Enter your choice (0-2): ";
}

class SelectorNode : public rclcpp::Node
{
public:
    SelectorNode() : Node("selector")
    {
        publisher_ = this->create_publisher<std_msgs::msg::UInt16>("/selector", 1000);
    }

    void publishSelection(int selection)
    {
        auto message = std_msgs::msg::UInt16();
        message.data = selection;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published control mode: %d", message.data);
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SelectorNode>();
    
    int selection;
    
    while (rclcpp::ok())
    {
        menu();
        std::cin >> selection;
        
        if (selection >= 0 && selection <= 2)
        {
            node->publishSelection(selection);
        }
        else
        {
            std::cout << "Invalid selection. Please choose 0-2.\n";
        }

        //rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    rclcpp::shutdown();
    return 0;
}