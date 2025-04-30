#pragma once
#include "control.h"

class DummyControl : public Control
{
public:
    DummyControl() : logger_(rclcpp::get_logger("DummyControl")){
        RCLCPP_DEBUG(logger_, "DummyControl constructor");
    }
    //установка данных лазера
    void setLaserData(const std::vector<float>& data) override;

    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    void setRobotPose(double x, double y, double theta) override {}

    //получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override { return "Dummy"; }

private:
    bool obstacle = false;
    rclcpp::Logger logger_;
};

