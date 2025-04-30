#include "dummy.h"
#include "voyager.h"
#include "wallFollower.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control.h"

using namespace std::chrono_literals;

class ControlSelector : public rclcpp::Node
{
public:
    ControlSelector() : Node("control_node")
    {
        // Инициализация алгоритмов управления
        initializeControls();

        // Инициализация издателя команд управления
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Инициализация подписчиков
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/base_scan", 10, std::bind(&ControlSelector::laserCallback, this, std::placeholders::_1));
            
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/base_pose_ground_truth", 10, std::bind(&ControlSelector::poseCallback, this, std::placeholders::_1));
            
        select_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
            "/selector", 100, std::bind(&ControlSelector::selectCallback, this, std::placeholders::_1));
            
        // Инициализация таймера (10 Гц)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ControlSelector::timerCallback, this));
    }

    ~ControlSelector()
    {
        // Освобождение памяти при завершении работы
        for (auto& control : controls_) {
            delete control;
        }
    }

private:
    // Перечисление для индексов алгоритмов управления
    enum ControlEnum {
        DUMMY,
        VOYAGER,
        WALLFOLLOWER,
        nControls
    };

    // Инициализация алгоритмов управления
    void initializeControls()
    {
        // Получение параметров из ROS2
        double min_range = this->declare_parameter("min_range", 1.0);
        double max_vel = this->declare_parameter("max_vel", 0.5);
        double max_omega = this->declare_parameter("max_omega", 0.5);
        
        // Создание экземпляров алгоритмов
        controls_[DUMMY] = new DummyControl();
        controls_[VOYAGER] = new VoyagerControl(min_range, max_vel, max_omega);
        controls_[WALLFOLLOWER] = new WallFollower();
        
        // Установка алгоритма по умолчанию
        control_ptr_ = nullptr;
    }

    // Обработчик выбора алгоритма управления
    void selectCallback(const std_msgs::msg::UInt16::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Получен запрос на выбор алгоритма: %d", msg->data);
        
        if (msg->data >= nControls) {
            RCLCPP_ERROR(this->get_logger(), "Неверный номер алгоритма: %d", msg->data);
            control_ptr_ = nullptr;
        } else {
            control_ptr_ = controls_[msg->data];
            RCLCPP_INFO(this->get_logger(), "Выбран алгоритм: %s", control_ptr_->getName().c_str());
        }
    }

    // Обработчик данных лазерного дальномера
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Получены данные от лидара");
        if (control_ptr_) {
            control_ptr_->setLaserData(msg->ranges);
        }
    }

    // Обработчик данных о положении робота
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double theta = 2 * atan2(msg->pose.pose.orientation.z,
                                msg->pose.pose.orientation.w);
                                
        RCLCPP_DEBUG(this->get_logger(), "Позиция робота: x=%.2f, y=%.2f, θ=%.2f", x, y, theta);
        
        if (control_ptr_) {
            control_ptr_->setRobotPose(x, y, theta);
        }
    }

    // Обработчик таймера (вызывается с частотой 10 Гц)
    void timerCallback()
    {
        auto cmd = geometry_msgs::msg::Twist();
        
        if (control_ptr_) {
            control_ptr_->getControl(cmd.linear.x, cmd.angular.z);
            RCLCPP_INFO_STREAM(this->get_logger(), "Алгоритм " << control_ptr_->getName());
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Алгоритм управления не выбран");
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Команда управления: v=%.2f, ω=%.2f", cmd.linear.x, cmd.angular.z);
        cmd_pub_->publish(cmd);
    }

    // Указатели на ROS2 интерфейсы
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr select_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Алгоритмы управления
    Control* controls_[nControls];
    Control* control_ptr_ = nullptr;
};

int main(int argc, char** argv)
{
    // Инициализация ROS2
    rclcpp::init(argc, argv);
    
    // Создание и запуск узла
    auto node = std::make_shared<ControlSelector>();
    rclcpp::spin(node);
    
    // Завершение работы
    rclcpp::shutdown();
    return 0;
}
