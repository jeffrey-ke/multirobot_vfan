#include "SparsifierNode.h"

void SparsifierNode::CVelCb(const custom_msgs::msg::Vel& msg){
    RCLCPP_INFO_STREAM(get_logger(), "c_vel: \n\t x_dot: " << msg.x_dot 
                                        << "\n\t y_dot: " << msg.y_dot);
    
    auto wp = sparsifier_.CalculateWaypoint(msg.x_dot, msg.y_dot);
    publisher_->publish(wp);

}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SparsifierNode>());
    rclcpp::shutdown();
    return 0;
}

SparsifierNode::SparsifierNode(): Node("sparsifier")
{
    subscription_ = create_subscription<custom_msgs::msg::Vel>("/cluster/c_vel", 10, std::bind(&SparsifierNode::CVelCb, this, std::placeholders::_1));
    publisher_ = create_publisher<custom_msgs::msg::Pose>("/robot1/wp", 10);
}

SparsifierNode::~SparsifierNode()
{
}
