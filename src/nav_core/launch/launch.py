from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_core',
            namespace='r0',
            executable='robot',
            name='r0',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'pose': [10.0, 10.0],
                 'id' : "0"}
            ]
        ),
        Node(
            package='nav_core',
            namespace='r1',
            executable='robot',
            name='r1',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'pose': [10.0, 10.0],
                 'id' : "1"}
            ]
        ),
        # Node(
        #     package='nav_core',
        #     namespace='r1',
        #     executable='robot',
        #     name='r1'
        # ),
        Node(
            package='nav_core',
            executable='Cluster',
            name='Cluster',
            parameters=[
                {"robot0_id": "0",
                 "robot1_id": "1",
                 "robot2_id": "2"}
            ]
        )
    ])