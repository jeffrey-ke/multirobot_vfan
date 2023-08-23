#include "RobotNode.h" 


RobotNode::RobotNode(): Node("robot_node"), controller_(1.0, 0.2)
{
    auto name = declare_parameter<std::string>("name", "robot");
    init_x_ = declare_parameter<double>("init_x", 0.0);
    init_y_ = declare_parameter<double>("init_y", 0.0);

    name = get_parameter("name").as_string();
    init_x_ = get_parameter("init_x").as_double();
    init_y_ = get_parameter("init_y").as_double();

    wp_sub_ = create_subscription<custom_msgs::msg::Pose>(
                                "/" + name + "/wp", 10, 
                                std::bind(&RobotNode::WaypointCb, this, std::placeholders::_1));
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                                "/" + name + "/odom", 10, 
                                std::bind(&RobotNode::PoseCb, this, std::placeholders::_1));
    cmdvel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
                                "/" + name + "/cmd_vel", 10);
                                
    cmdvel_msg_.linear.x = 0.0;
    cmdvel_msg_.angular.z = 0.0;
    cmdvel_pub_->publish(cmdvel_msg_);
}

RobotNode::~RobotNode()
{
}

void RobotNode::WaypointCb(const custom_msgs::msg::Pose& msg) {
    RCLCPP_INFO_STREAM(get_logger(), "Commanded to go: \n\t x: " << msg.x 
                                                    << "\n\t y: " << msg.y);
    controller_.UpdateWp(msg.x, msg.y);
    controller_.UpdateCmdVel();
    CmdVel cmdvel = controller_.GetCmdVel();
    cmdvel_msg_.angular.z = cmdvel.angular_speed;
    cmdvel_msg_.linear.x = cmdvel.speed;

    cmdvel_pub_->publish(cmdvel_msg_);
    RCLCPP_INFO_STREAM(get_logger(), "waypoint: \n\t x: " << controller_.GetWp().x 
                                            << "\n\t y: " << controller_.GetWp().y);
}

void RobotNode::PoseCb(const nav_msgs::msg::Odometry& msg) {
    auto pose = msg.pose.pose.position;
    auto quat = msg.pose.pose.orientation;
    auto yaw = controller_.QuaternionToEuler({quat.w, quat.x, quat.y, quat.z}).yaw;
    RCLCPP_INFO_STREAM(get_logger(), "Name: " << get_parameter("name").as_string() 
                                    << "Actual pose [x: " << init_x_ + pose.x 
                                                << " y:" << init_y_ + pose.y << "]");
    controller_.UpdatePose(init_x_ + pose.x, init_y_ + pose.y, yaw);
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