// #include "control.h"
#include "dummy.h"
#include "voyager.h"
#include "wallFollower.h"
// #include "std_msgs/msg/u_int16.hpp"


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control.h"
//#include "voyagercontrol.h"
//#include "dummy_control.h"
//#include "wallFollower.h"

using namespace std::chrono_literals;

/*
 * Класс для выбора алгоритмов управления роботом
 * Включает 3 режима:
 * - DUMMY: простейший алгоритм
 * - VOYAGER: алгоритм исследования пространства
 * - WALLFOLLOWER: алгоритм следования вдоль стены
 */
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
            "/scan", 10, std::bind(&ControlSelector::laserCallback, this, std::placeholders::_1));
            
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



/*
//  * Доработать проект control_selector, добавив класс движения вдоль стены
//  * (правой и левой) в виде наследника абстрактного класса Control
//  *  Добавить объекты нового класса в массив controls, обеспечив тем самым возможность
//  * их выбора спомощью selectCallback
 

// перечисление, задающее константы, соответствующее индексам алгоритмов в массиве
enum ControlEnum
{
    DUMMY,
    VOYAGER,
    WALLFOLLOWER, // раскомментировать после добавления нового алгоритма
    nControls
};

// указатель на текущий алгоритм управления
Control *controlPtr(nullptr);
// массив указателей на доступные алгоритмы из которых мы можем выбрать
Control *controls[nControls];

// объект издатель команд управления
class Node_cmd : public rclcpp::Node
{
public:
    Node_cmd() : Node("cmd_node")
    {
        //Подписываемся на данные с лазера
        subscription_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1000, std::bind(&Node_cmd::laserCallback, this, std::placeholders::_1));
        
        //Подписываемся на данные о положении робота
        subscription_pose = this->create_subscription<nav_msgs::msg::Odometry>(
            "/base_pose_ground_truth", 1000, std::bind(&Node_cmd::poseCallback, this, std::placeholders::_1));

        //Подписываемся на selector
        subscription_selector = this->create_subscription<std_msgs::msg::UInt16>(
            "/selector", 1000, std::bind(&Node_cmd::Selector_callback, this, std::placeholders::_1));


        //Публикуем данные о скорости робота в соответствии с выбранным управлением
        publisher_cmd = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 100);

        // Таймер для отправки сообщений
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&Node_cmd::Timer_callback, this));
    }

private:
    void Timer_callback()
    {
        RCLCPP_DEBUG(this->get_logger(), "on timer ");
        // Создание и отправка сообщения
        geometry_msgs::msg::Twist cmd;
        if (controlPtr)
        {
            controlPtr->getControl(cmd.linear.x, cmd.angular.z);
        }
        else
        {
            // это сообщение будет печататься не чаще 1 раза в секунду
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "no control");
        }
        RCLCPP_DEBUG_STREAM(this->get_logger(), "cmd v = " << cmd.linear.x << " cmd w = " << cmd.angular.z);
        publisher_cmd->publish(cmd);
    }

    // функция обработки сообщения по топику '/selector`, управляющего выбором алгоритма
    // можно отправить сообщение из консоли
    // rostopic pub /selector std_msgs/UInt16 <номер алгоритма начиная с 0>
    void Selector_callback(const std_msgs::msg::UInt16 msg)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "select callback " << msg.data);
        if (msg.data >= nControls)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Wrong algorithm number " << msg.data);
            controlPtr = nullptr;
        }
        else
        {
            controlPtr = controls[msg.data];
            RCLCPP_INFO_STREAM(this->get_logger(), "Select " << controlPtr->getName() << " control");
        }
    }

    //Функция, которая будет вызвана при получении данных от лазерного дальномера
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_msg){
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Laser msg: ");
        if (controlPtr) // если указатель не нулевой - вызываем функцию текущего алгоритма
            controlPtr->setLaserData(laser_msg->ranges);
    }

    //Функция, которая будет вызвана при получении сообщения с текущим положением робота
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        double x = odom_msg->pose.pose.position.x;
        double y = odom_msg->pose.pose.position.y;
        double theta = 2 * atan2(odom_msg->pose.pose.orientation.z,
                                 odom_msg->pose.pose.orientation.w);
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Pose msg: x = " << x << " y = " << y << " theta = " << theta);
        if (controlPtr) // если указатель не нулевой - вызываем функцию текущего алгоритма
            controlPtr->setRobotPose(x, y, theta);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr subscription_selector;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_laser;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_pose;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    controls[DUMMY] = new DummyControl();
    controls[VOYAGER] = new VoyagerControl(1.0, 0.5, 0.5);
    controls[WALLFOLLOWER] = new WallFollower();

    // устанавливаем указатель на действующий алгоритм управления на любой (например первый) элемент массива
    controlPtr = controls[VOYAGER];

    auto control = std::make_shared<Node_cmd>();
    rclcpp::spin(control);

    // Корректное завершение работы
    rclcpp::shutdown();
    for (std::size_t i = 0; i < nControls; ++i)
    {
        delete controls[i];
    }

    return 0;
}

*/