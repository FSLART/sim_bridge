from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_bridge',
            executable='sim_bridge',
            name='sim_bridge',
            output='screen'
        )
    ])
