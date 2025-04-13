#ifndef LINE_CONTROL_H
#define LINE_CONTROL_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class LineControl : public rclcpp::Node
{
public:
    LineControl();
    
private:
    // Функция вычисления ошибки управления для движения вдоль прямой
    double cross_track_err_line();
    // Функция вычисления ошибки управления для движения вдоль окружности
    double cross_track_err_circle();
    
    /**
     * Функция, которая будет вызвана
     * при получении данных от лазерного дальномера
     */
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    /**
     * Функция, которая будет вызвана при
     * получении сообщения с текущем положением робота
     */
    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * Функция обработчик таймера
     */
    void timerCallback();
    
    // Функция публикации ошибки
    void publish_error(double e);

private:
    // Параметры управления
    double line_y;          // заданная координата линии
    double cx, cy, R;       // параметры окружности
    double task_vel;        // заданная скорость движения
    double prop_factor;     // пропорциональный коэффициент ПИД
    double int_factor;      // интегральный коэффициент ПИД
    double diff_factor;     // дифференциальный коэффициент ПИД
    double int_error;       // интеграл ошибки
    double old_error;       // предыдущее значение ошибки
    double min_obstacle_range; // минимальное расстояние до препятствия
    bool obstacle;          // флаг наличия препятствия
    
    // Текущее положение робота
    double x, y, theta;

    // ROS2 интерфейсы
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // LINE_CONTROL_H
