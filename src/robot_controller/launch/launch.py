from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    

    return LaunchDescription([
        Node(
            package="cluster_controller",
            namespace="cluster_controller",
            executable="clustercontroller",
            name="clustercontroller"
        ),
        Node(
            package="robot_controller",
            namespace="robot_controller",
            executable="RobotNode",
            name="robot1",
            parameters= [{
                "name" : "robot1"
            }]
        ),
        Node(
            package="robot_controller",
            namespace="robot_controller",
            executable="RobotNode",
            name="robot2",
            parameters= [{
                "name" : "robot2"
            }]
        ),
        Node(
            package="robot_controller",
            namespace="robot_controller",
            executable="RobotNode",
            name="robot3",
            parameters= [{
                "name" : "robot3"
            }]
        ),
        ExecuteProcess(cmd=[[
        "ign gazebo -v 4 -r /home/jeffrey/repo/ros_ws/src/sim_world/models/world.xml"
        ]]), ## odom msgs
        ExecuteProcess(cmd=[[
            "ros2 run ros_gz_bridge parameter_bridge /robot1/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
        ]]),
        ExecuteProcess(cmd=[[
            "ros2 run ros_gz_bridge parameter_bridge /robot2/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
        ]]),
        ExecuteProcess(cmd=[[
            "ros2 run ros_gz_bridge parameter_bridge /robot3/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry"
        ]]), ## cmdvel
        ExecuteProcess(cmd=[[
            "ros2 run ros_gz_bridge parameter_bridge /robot1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
        ]]),
        ExecuteProcess(cmd=[[
            "ros2 run ros_gz_bridge parameter_bridge /robot2/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
        ]]),
        ExecuteProcess(cmd=[[
            "ros2 run ros_gz_bridge parameter_bridge /robot3/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"
        ]])
        
    ])