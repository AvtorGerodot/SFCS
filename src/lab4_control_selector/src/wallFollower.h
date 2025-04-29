#pragma once

#include "control.h"

class WallFollower : public Control
{
private:
    bool obstacle = false;
    rclcpp::Logger logger_;
public:
    WallFollower() : logger_(rclcpp::get_logger("VoyagerControl"))  {
        RCLCPP_DEBUG(logger_, "VoyagerControl constructor");
    }
    //установка данных лазера
    void setLaserData(const std::vector<float>& data) override;
    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    void setRobotPose(double x, double y, double theta) override {}
    //получение управления
    void getControl(double& v, double& w) override;
    std::string getName() override { return "Wall_Follower"; }
};
