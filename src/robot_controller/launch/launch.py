from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package="robot_controller",
            namespace="robot_controller",
            executable="RobotNode",
            name="robotnode"
        ),
        Node(
            package="cluster_controller",
            namespace="cluster_controller",
            executable="SparsifierNode",
            name="sparsifiernode"
        )
    ])