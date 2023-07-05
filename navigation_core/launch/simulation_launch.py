#! /usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_core',
            namespace='robotspace',
            executable='Robot0',
            name='Robot0'
        ),
        Node(
            package='navigation_core',
            namespace='clusterspace',
            executable='Cluster',
            name='Cluster'
        )
    ])