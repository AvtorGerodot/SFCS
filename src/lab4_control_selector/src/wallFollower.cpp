#include "wallFollower.h"

bool pastObstacle[5] = {0, 0, 0, 0, 0};
float obstacleRange = 0.0;

void obstaclePush()
{
    for (int i = 4; i > 0; i--)
    {
        pastObstacle[i] = pastObstacle[i - 1];
    }
    pastObstacle[0] = obstacleRange <= 0.2;
}

bool wasObstakleNearby()
{
    bool was = 0;
    for (int i = 0; i < 5; i++)
    {
        if (pastObstacle[i])
        {
            was = true;
            break;
        }
    }
    return was;
}

void WallFollower::setLaserData(const std::vector<float> &data)
{
    const float maxRange = 8.0;
    const double kMinRange = 0.3;
    const double nomRange = 0.6;
    float minRange = data[0];
    for (size_t i = 0; i < data.size(); i++)
    {
        if (minRange > data[i])
        {
            minRange = data[i];
        }
        if (minRange < nomRange)
        {
            obstacle = true;
            if (minRange < kMinRange)
            {
                RCLCPP_ERROR(logger_, "OBSTACLE!!!");
                break;
            }
        }
        else
        {
            obstacle = false;
        }
    }
    obstacleRange = minRange - nomRange;
    obstaclePush();
}

void WallFollower::getControl(double &v, double &w)
{
    if (obstacleRange < -0.2)
    {
        v = 0;
        w = 0.5;
        RCLCPP_WARN(logger_, "stop");
    }
    else
    {
        if (obstacleRange >= -0.2 && obstacleRange <= 0.2)
        {
            v = 0.5;
            w = -obstacleRange * fabs(obstacleRange) * 25;
            RCLCPP_INFO(logger_, "go near wall");
        }
        else
        {
            if (wasObstakleNearby())
            {
                v = 0;
                w = -1;
                RCLCPP_INFO(logger_, "Where is it?");
            }
            else
            {
                v = 0.5;
                w = -0.1;
                RCLCPP_INFO(logger_, "go");
            }
        }
    }
}