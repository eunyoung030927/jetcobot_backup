from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jetcobot_movetag',
            executable='apriltag_flask_node',
            name='apriltag_flask_node',
            output='screen',
        ),
    ])