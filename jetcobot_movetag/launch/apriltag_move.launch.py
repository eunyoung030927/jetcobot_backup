from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetcobot_movetag',
            executable='apriltag_move_node',
            name='apriltag_move_node',
            output='screen',
        ),
    ])
