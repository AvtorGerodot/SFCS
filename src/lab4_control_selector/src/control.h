#pragma once
//#define CONTROL_H
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

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
