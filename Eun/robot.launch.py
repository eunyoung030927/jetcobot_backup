from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joint State Publisher (GUI 없이 코드로 직접 값 publish 가능)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'source_list': []}],  # 값 직접 publish
        ),
        # Robot State Publisher: URDF file 필요
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': "/home/jetcobot/colcon_ws/src/jetcobot/jetcobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"}],
        ),
    ])
