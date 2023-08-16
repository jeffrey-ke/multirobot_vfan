#pragma once

// ros imports
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
// library imports
#include "Controller.h"

using namespace std::chrono_literals;

class RobotNode: public rclcpp::Node
{
private:
    rclcpp::Subscription<custom_msgs::msg::Pose>::SharedPtr wp_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_; 
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub_;
    Controller controller_;
    geometry_msgs::msg::Twist cmdvel_msg_;
public:
    RobotNode();
    ~RobotNode();
    void WaypointCb(const custom_msgs::msg::Pose& msg);
    void PoseCb(const nav_msgs::msg::Odometry& msg);
};

