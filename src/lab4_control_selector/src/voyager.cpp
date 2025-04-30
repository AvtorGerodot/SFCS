#include "voyager.h"

void VoyagerControl::setLaserData(const std::vector<float> &ranges)
{
    obstacle = false;
    for (size_t i = 0; i < ranges.size(); i++) // проверим нет ли вблизи робота препятствия
    {
        if (ranges[i] < min_range)
        {
            obstacle = true;
            RCLCPP_WARN(logger_, "OBSTACLE");
            break;
        }
    }
}

void VoyagerControl::getControl(double &v, double &w) //в зависимости от наличия препятствия, отправляем сигналы управления
{
    if (!obstacle)
    {
        RCLCPP_INFO(logger_, "No obstacle - go forward");
        v = max_vel;
        w = 0;
    }
    else
    {
        RCLCPP_INFO(logger_, "Obstacle - spin");
        v = 0;
        w = max_omega;
    }
}
