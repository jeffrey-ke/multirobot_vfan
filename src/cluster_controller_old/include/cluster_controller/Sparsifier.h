#pragma once

//standard imports
#include <cmath>
#include "custom_msgs/msg/pose.hpp"


class Sparsifier
{
private:
    double step_size_;
public:
    Sparsifier(double step_size = 2.0);
    ~Sparsifier();
    custom_msgs::msg::Pose CalculateWaypoint(double x_dot, double y_dot);
};
