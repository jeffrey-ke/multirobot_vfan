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
    auto heading_error = heading_des - pose_.theta;
    return (heading_error > MIN_HEAD_ERR)? heading_error : 0.0;
}

double Controller::CalculateDistanceError() {
    double deltaX = wp_.x - pose_.x;
    double deltaY = wp_.y - pose_.y;
    auto distance_error = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    return (distance_error > MIN_DIS_ERR)? distance_error : 0.0;
}

Quaternion Controller::Normalize(const Quaternion& q) {
    double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    
    // Avoid division by zero
    if (norm == 0) return {1, 0, 0, 0};

    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

Euler Controller::QuaternionToEuler(const Quaternion& q_in) {
    Quaternion q = Normalize(q_in);
    Euler angles;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
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


