#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
//#include <move_base_msgs/action/move_base.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using MoveBaseAction = nav2_msgs::action::NavigateToPose;
using GoalHandleMoveBase = rclcpp_action::ClientGoalHandle<MoveBaseAction>;
using namespace std::placeholders;

class ControlNode : public rclcpp::Node
{
public:
    ControlNode() : Node("control_node"), trace_queue_(-1), count_(0), done_(true)
    {
        // Инициализация клиента действия
        move_base_client_ = rclcpp_action::create_client<MoveBaseAction>(this, "navigate_to_pose");

        // Подписка на тему clicked_point
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 1, std::bind(&ControlNode::clickPointCallback, this, _1));

        // Ожидание сервера действия
        this->wait_for_action_server();
    }

    void run()
    {
        rclcpp::Rate rate(10);
        while (rclcpp::ok()) {
            rclcpp::spin_some(this->get_node_base_interface());
            if (done_) {
                send_goal();
            }
            rate.sleep();
        }
    }

private:
    void wait_for_action_server()
    {
        while (!move_base_client_->wait_for_action_server(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
        RCLCPP_INFO(this->get_logger(), "Server move_base is connected");
    }

    void send_goal()
    {
        if (count_ == 0) return;

        trace_queue_++;
        if (trace_queue_ == 5 || trace_queue_ == count_) {
            trace_queue_ = 0;
        }

        auto goal_msg = goals_[trace_queue_];
        goal_msg.pose.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveBaseAction>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ControlNode::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&ControlNode::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&ControlNode::result_callback, this, _1);

        move_base_client_->async_send_goal(goal_msg, send_goal_options);
        done_ = false;
    }

    void goal_response_callback(std::shared_future<GoalHandleMoveBase::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleMoveBase::SharedPtr,
        const std::shared_ptr<const MoveBaseAction::Feedback> feedback)
    {
        /*RCLCPP_INFO(this->get_logger(), "Feedback: x=%f, y=%f",
            feedback->current_pose.pose.position.x,
            feedback->current_pose.pose.position.y);*/
    }

    void result_callback(const GoalHandleMoveBase::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Target is reached");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        done_ = true;
    }

    void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point)
    {
        double target_angle = M_PI/2;
        if (count_ < 5) count_++;
        RCLCPP_INFO(this->get_logger(), "Get point %f %f", point->point.x, point->point.y);

        // Формируем структуру цели для move_base Action
        goals_[count_ - 1].pose.pose.position = point->point;
        goals_[count_ - 1].pose.pose.orientation.z = sin(target_angle/2);
        goals_[count_ - 1].pose.pose.orientation.w = cos(target_angle/2);
        goals_[count_ - 1].pose.header.frame_id = "map";
        goals_[count_ - 1].pose.header.stamp = this->now();

        if (count_ == 1) done_ = true;
    }

    rclcpp_action::Client<MoveBaseAction>::SharedPtr move_base_client_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    MoveBaseAction::Goal goals_[5];
    int trace_queue_;
    int count_;
    bool done_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}