#include "dummy.h"

void DummyControl::setLaserData(const std::vector<float> &data)
{
    obstacle = false;
    for (size_t i = 0; i < data.size(); i++)
    {
        if (data[i] < 0.3)
        {
            obstacle = true;
            break;
        }
    }
}

void DummyControl::getControl(double &v, double &w)
{
    if (obstacle)
    {
        RCLCPP_INFO(logger_, "Obstacle! Stop!");
        v = 0, w = 0;
    }
    else
    {
        RCLCPP_INFO(logger_, "No obstacle - go forward");
        v = 0.2;
        w = 0;
    }
}
