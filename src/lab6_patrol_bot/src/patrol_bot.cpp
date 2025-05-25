#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <queue>
#include <memory>

class GoalManager : public rclcpp::Node
{
public:
    // Флаг для немедленного старта
    //int flag = 0; 
    GoalManager() : Node("patrol_bot_node")
    {
        // Подписка на целевые позиции
        goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&GoalManager::goalPoseCallback, this, std::placeholders::_1));

        // Подписка на статус навигации
        nav_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/navigation_status", 10,
            std::bind(&GoalManager::navStatusCallback, this, std::placeholders::_1));

        // Публикатор для отправки целей навигатору
        single_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/single_goal_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Goal Manager node has been initialized");
    }

private:
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Добавляем новую цель в очередь
        
        goals_queue_.push(*msg);

        // Добавляем новую цель в вектор с сохранёнными точками
        saved_points_.push_back(*msg);
        RCLCPP_INFO(this->get_logger(), "New goal received. Queue size: %zu", goals_queue_.size());

        // Код для немедленного старта (без кнопки)

        // if(goals_queue_.size() == 1 && flag == 0) {
        //     geometry_msgs::msg::PoseStamped next_goal = goals_queue_.front();
        //     goals_queue_.pop();
        //     single_goal_pub_->publish(next_goal);
        //     flag = 1;
        // }
    }

    void navStatusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "Reached goal")
        {
            if (!goals_queue_.empty())
            {
                // Берем следующую цель из очереди
                geometry_msgs::msg::PoseStamped next_goal = goals_queue_.front();
                goals_queue_.pop();

                // Публикуем следующую цель
                single_goal_pub_->publish(next_goal);
                RCLCPP_INFO(this->get_logger(), "Sending next goal to navigator. Remaining goals: %zu", 
                           goals_queue_.size());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Goal reached but queue is empty, loop started");
                // Копируем из сохранённого вектора очередь из точек
                for (const auto& point : saved_points_) {
                    goals_queue_.push(point);
                }
                // Берем следующую цель из очереди
                geometry_msgs::msg::PoseStamped next_goal = goals_queue_.front();
                goals_queue_.pop();

                // Публикуем следующую цель
                single_goal_pub_->publish(next_goal);
                RCLCPP_INFO(this->get_logger(), "Sending next goal to navigator. Remaining goals: %zu", 
                           goals_queue_.size());
            }
        }
    }

    // Очередь целей
    std::queue<geometry_msgs::msg::PoseStamped> goals_queue_;

    // Сохранённые цели в векторе
    std::vector<geometry_msgs::msg::PoseStamped> saved_points_;

    // Подписки
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;

    // Публикатор
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr single_goal_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
