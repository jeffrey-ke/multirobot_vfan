from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess
from ament_index_python import get_package_share_directory

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
                "name" : "robot1",
                "init_x": 0.0,
                "init_y": 0.0
            }]
        ),
        Node(
            package="robot_controller",
            namespace="robot_controller",
            executable="RobotNode",
            name="robot2",
            parameters= [{
                "name" : "robot2",
                "init_x": 5.0,
                "init_y": 2.5
            }]
        ),
        Node(
            package="robot_controller",
            namespace="robot_controller",
            executable="RobotNode",
            name="robot3",
            parameters= [{
                "name" : "robot3",
                "init_x": 0.0,
                "init_y": 5.0
            }]
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/robot1/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                       "/robot2/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                       "/robot3/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                       "/robot1/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                       "/robot2/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                       "/robot3/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist"]
        ),
        ExecuteProcess(
            cmd=["ign", "gazebo", 
                 PathJoinSubstitution([get_package_share_directory("sim_world"), "models", "world.xml"])]
        )
        
    ])