#pragma once

// standard imports
#include <chrono>
#include <memory>

// ros imports
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/vel.hpp"
#include "custom_msgs/msg/pose.hpp"

// library imports
#include "Sparsifier.h"

using namespace std::chrono_literals;

class SparsifierNode: public rclcpp::Node
{
private:
    rclcpp::Subscription<custom_msgs::msg::Vel>::SharedPtr subscription_;
    rclcpp::Publisher<custom_msgs::msg::Pose>::SharedPtr publisher_;

    Sparsifier sparsifier_;

public:
    SparsifierNode();
    ~SparsifierNode();
    void CVelCb(const custom_msgs::msg::Vel& msg);
};

SparsifierNode::SparsifierNode(): Node("sparsifier")
{
    subscription_ = create_subscription<custom_msgs::msg::Vel>("/cluster/c_vel", 10, std::bind(&SparsifierNode::CVelCb, this, std::placeholders::_1));
    publisher_ = create_publisher<custom_msgs::msg::Pose>("/robot1/wp", 10);
}

SparsifierNode::~SparsifierNode()
{
}
