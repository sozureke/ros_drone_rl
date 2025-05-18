from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_control',
            executable='command_gateway',
            name='command_gateway',
            output='screen',
        ),
        Node(
            package='drone_control',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
        ),
        Node(
            package='drone_control',
            executable='fsm_action_server',
            name='fsm_action_server',
            output='screen',
        ),
    ])
