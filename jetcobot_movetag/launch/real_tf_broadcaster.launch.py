from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # URDF 경로 (절대 경로 또는 자동 경로 사용 가능)
    urdf_path = "/home/jetcobot/silver_ws/src/jetcobot_movetag/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([ # 실제 로봇을 위한 TF 브로드캐스터 (조인트 상태 퍼블리셔)
        Node(
            package='jetcobot_movetag',
            executable='real_joint_state_publisher',
            name='real_joint_state_publisher',
            output='screen'
        ),
        # real_joint_state를 구독하도록 remapping
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('/joint_states', '/real_joint_states')]
        ),

        # # apriltag_to_robot 노드 실행 (위 파이썬 코드)
        # Node(
        #     package='jetcobot_launch',  # ★ Python 코드 위치한 패키지명으로 변경하세요
        #     executable='apriltag_to_robot_node',
        #     name='apriltag_to_robot_node',
        #     output='screen',
        #     # prefix='python3',  # .py 파일 직접 실행 시
        # ),
    ])
