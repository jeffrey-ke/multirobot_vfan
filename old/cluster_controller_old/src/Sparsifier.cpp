#include "Sparsifier.h"

custom_msgs::msg::Pose Sparsifier::CalculateWaypoint(double x_dot, double y_dot) {
    double cmd_speed = std::sqrt(x_dot * x_dot + y_dot * y_dot);
    double time_to_waypoint = step_size_ / cmd_speed;
    auto waypoint = custom_msgs::msg::Pose();
    waypoint.x = x_dot * time_to_waypoint;
    waypoint.y = y_dot * time_to_waypoint;
    waypoint.theta = -99.0;
    return waypoint;
}

Sparsifier::Sparsifier(double step_size)
{
    this->step_size_ = step_size;
}

Sparsifier::~Sparsifier()
{
}
