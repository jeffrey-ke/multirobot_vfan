#include "Controller.h"

Controller::Controller(double k_p_angular, double k_p_distance)
{
    k_p_distance_ = k_p_distance;
    k_p_angular_ = k_p_angular;
    wp_.x = 5.544445;
    wp_.y = 5.544445;
}

Controller::~Controller()
{
}

void Controller::UpdatePose(double x, double y, double theta){
    pose_.x = x;
    pose_.y = y;
    pose_.theta = theta;
}

void Controller::UpdateWp(double x, double y) {
    wp_.x = x + pose_.x;
    wp_.y = y + pose_.y;
}

void Controller::UpdateCmdVel() {
    double angular_error = CalculateAngularError();
    double distance_error = CalculateDistanceError();

    cmdvel_.angular_speed = k_p_angular_ * angular_error;
    cmdvel_.speed = k_p_distance_ * distance_error;
}

double Controller::CalculateAngularError() {
    double heading_des = std::atan2(wp_.y - pose_.y, wp_.x - pose_.x);
    return heading_des - pose_.theta;
}

double Controller::CalculateDistanceError() {
    double deltaX = wp_.x - pose_.x;
    double deltaY = wp_.y - pose_.y;
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

Pose Controller::GetPose() {
    return pose_;
}

Pose Controller::GetWp() {
    return wp_;
}

CmdVel Controller::GetCmdVel() {
    return cmdvel_;
}


