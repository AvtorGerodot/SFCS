#pragma once
//#define CONTROL_H
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

//абстрактный класс - интерфейс системы управления
class Control
{
public:
    //абстрактные функции интерфейса

    //установка данных лазера
    virtual void setLaserData(const std::vector<float>& data) = 0;

    //установка текущей позиции робота
    virtual void setRobotPose(double x, double y, double theta) = 0;

    //получение управления
    virtual void getControl(double& v, double& w) = 0;

    virtual std::string getName() = 0;

    //виртуальный деструктор
    virtual ~Control() {}

};
