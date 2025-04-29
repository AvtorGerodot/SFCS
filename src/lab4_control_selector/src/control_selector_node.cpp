#include "control.h"
#include "dummy.h"
#include "voyager.h"
#include "wallFollower.h"
#include "std_msgs/UInt16.h"

/*
 * Доработать проект control_selector, добавив класс движения вдоль стены
 * (правой и левой) в виде наследника абстрактного класса Control
 *  Добавить объекты нового класса в массив controls, обеспечив тем самым возможность
 * их выбора спомощью selectCallback
 *
 * */

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
        subscription_selector = this->create_subscription<std_msgs::UInt16>(
            "/selector", 1000, std::bind(&Node_cmd::Selector_callback, this, std::placeholders::_1));

            
        //Подписываемся на данные с лазера
        subscription_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1000, std::bind(&Node_cmd::laserCallback, this, std::placeholders::_1));

        
        //Подписываемся на данные о положении робота
        subscription_pose = this->create_subscription<nav_msgs::msg::Odometry>(
            "/base_pose_ground_truth", 1000, std::bind(&Node_cmd::poseCallback, this, std::placeholders::_1));

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
    void Selector_callback(const std_msgs::UInt16 msg)
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
    rclcpp::Subscription<std_msgs::UInt16>::SharedPtr subscription_selector;
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
