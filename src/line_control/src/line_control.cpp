#include "line_control.h"
#include <cmath>
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

void LineControl::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Проверка наличия препятствий вблизи робота
    const double kMinObstacleDistance = 0.3;
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
        if (msg->ranges[i] < kMinObstacleDistance)
        {
            obstacle = true;
            RCLCPP_WARN(this->get_logger(), "OBSTACLE!!!");
            break;
        }
    }
}

void LineControl::poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Pose msg: x = %f y = %f theta = %f", 
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 2 * atan2(msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w));
    
    // Обновление переменных класса, отвечающих за положение робота
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    theta = 2 * atan2(msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
}

double LineControl::cross_track_err_line()
{
    return line_y - y;
}

double LineControl::cross_track_err_circle()
{
    double dx = cx - x;
    double dy = cy - y;
    double e = sqrt(dx * dx + dy * dy) - R;
    return e;
}

void LineControl::publish_error(double e)
{
    auto err = std_msgs::msg::Float64();
    err.data = e;
    err_pub->publish(err);
}

void LineControl::timerCallback()
{
    RCLCPP_DEBUG(this->get_logger(), "on timer");
    
    // Сообщение для управления угловой и линейной скоростью
    auto cmd = geometry_msgs::msg::Twist();
    
    // Если вблизи нет препятствия, задаем команды
    if (!obstacle)
    {
        // Вычисление текущей ошибки управления
        double err = cross_track_err_line();
        // Публикация текущей ошибки
        publish_error(err);
        // Интегрирование ошибки
        int_error += err;
        // Дифференцирование ошибки
        double diff_error = err - old_error;
        // Запоминание значения ошибки для следующего момента времени
        old_error = err;
        
        cmd.linear.x = task_vel;
        // ПИД регулятор угловой скорости w = k*err + k_и * инт_err + k_д * дифф_err
        cmd.angular.z = prop_factor * err + int_factor * int_error + diff_error * diff_factor;
        
        RCLCPP_DEBUG(this->get_logger(), "error = %f cmd v=%f w = %f", 
                     err, cmd.linear.x, cmd.angular.z);
    }
    
    // Отправка (публикация) команды
    cmd_pub->publish(cmd);
}

LineControl::LineControl() : Node("line_control"),
    int_error(0.0),
    old_error(0.0),
    obstacle(false)
{
    RCLCPP_INFO(this->get_logger(), "LineControl initialisation");
    
    // Чтение параметров с установкой значений по умолчанию
    this->declare_parameter("line_y", -10.0);
    this->declare_parameter("cx", -6.0);
    this->declare_parameter("cy", 0.0);
    this->declare_parameter("R", 6.0);
    this->declare_parameter("task_vel", 0.5);
    this->declare_parameter("prop_factor", 2.0);
    this->declare_parameter("int_factor", 0.1);
    this->declare_parameter("diff_factor", 0.01);
    this->declare_parameter("min_obstacle_range", 1.0);
    this->declare_parameter("dt", 0.1);
    
    line_y = this->get_parameter("line_y").as_double();
    cx = this->get_parameter("cx").as_double();
    cy = this->get_parameter("cy").as_double();
    R = this->get_parameter("R").as_double();
    task_vel = this->get_parameter("task_vel").as_double();
    prop_factor = this->get_parameter("prop_factor").as_double();
    int_factor = this->get_parameter("int_factor").as_double();
    diff_factor = this->get_parameter("diff_factor").as_double();
    min_obstacle_range = this->get_parameter("min_obstacle_range").as_double();
    double dt = this->get_parameter("dt").as_double();
    
    // Подписка на необходимые топики
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LineControl::laserCallback, this, _1));
        
    pose_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/base_pose_ground_truth", 10, std::bind(&LineControl::poseCallback, this, _1));
        
    // Создание таймера
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt), 
        std::bind(&LineControl::timerCallback, this));
        
    // Создание публикаторов
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    err_pub = this->create_publisher<std_msgs::msg::Float64>("/err", 10);
}
