#include "RobotNode.h" 


RobotNode::RobotNode(): Node("robot_node"), controller_(1.0, 0.2)
{
    wp_sub_ = create_subscription<custom_msgs::msg::Pose>(
                                "/robot1/wp", 10, 
                                std::bind(&RobotNode::WaypointCb, this, std::placeholders::_1));
    pose_sub_ = create_subscription<turtlesim::msg::Pose>(
                                "/turtle1/pose", 10, 
                                std::bind(&RobotNode::PoseCb, this, std::placeholders::_1));
    cmdvel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
                                "/turtle1/cmd_vel", 10);
    cmdvel_msg_.linear.x = 0.0;
    cmdvel_msg_.angular.z = 0.0;
    cmdvel_pub_->publish(cmdvel_msg_);
}

RobotNode::~RobotNode()
{
}

void RobotNode::WaypointCb(const custom_msgs::msg::Pose& msg) {
    controller_.UpdateWp(msg.x, msg.y);
    controller_.UpdateCmdVel();
    CmdVel cmdvel = controller_.GetCmdVel();
    cmdvel_msg_.angular.z = cmdvel.angular_speed;
    cmdvel_msg_.linear.x = cmdvel.speed;

    cmdvel_pub_->publish(cmdvel_msg_);
    RCLCPP_INFO_STREAM(get_logger(), "waypoint: \n\t x: " << controller_.GetWp().x 
                                            << "\n\t y: " << controller_.GetWp().y);
}

void RobotNode::PoseCb(const turtlesim::msg::Pose& msg) {
    controller_.UpdatePose(msg.x, msg.y, msg.theta);
    controller_.UpdateCmdVel();
    CmdVel cmdvel = controller_.GetCmdVel();
    cmdvel_msg_.angular.z = cmdvel.angular_speed;
    cmdvel_msg_.linear.x = cmdvel.speed;
    cmdvel_pub_->publish(cmdvel_msg_);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}