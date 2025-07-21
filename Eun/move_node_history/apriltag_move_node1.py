import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import numpy as np
import time
from pymycobot.mycobot import MyCobot
import math


URDF_PATH = "/home/jetcobot/silver_ws/src/jetcobot_movetag/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"
class AprilTagToRobot(Node):
    def __init__(self):
        super().__init__('apriltag_to_robot')
        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000) # MyCobot 초기화

        # 1. 실제 움직여야하는 조인트 (URDF 기준)
        self.joint_names = [
            "link2_to_link1", "link3_to_link2", "link4_to_link3",
            "link5_to_link4", "link6_to_link5", "link6output_to_link6"
        ]

        # 2. 역기구학용 Chain 로드
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["g_base"]) # last_link_name='tcp'
        self.joint_indices = [idx for idx, link in enumerate(self.chain.links) if link.name in self.joint_names] # IK 계산 결과 필터링할 인덱스

        # 3. 관절 상태 퍼블리셔
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 4. tf2 버퍼 + 리스너 (프레임 사이 변환 자동 관리)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. 태그 Pose 구독
        self.create_subscription(PoseStamped, '/apriltag_goal_pose', self.apriltag_callback, 10)

        self.current_tag_frame = None
        self.is_busy = False

    def apriltag_callback(self, msg: PoseStamped):
        if self.is_busy:
            return
        self.is_busy = True # 태그 처리 중 플래그 설정

        self.current_tag_frame = msg.header.frame_id  # ex: "tag36h11:4"
        self.get_logger().info(f"[TAG DETECTED] {self.current_tag_frame}")

        # 1) tf2에서 g_base → tagNN 변환 계산
        try:
            now = self.get_clock().now().to_msg()  # 현재 시간
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='g_base', # 이 좌표계로 바꿔줌 
                source_frame=self.current_tag_frame,
                time=now
                # time=rclpy.time.Time()
            )
            t = trans.transform.translation # 위치
            pos = np.array([t.x, t.y, t.z])
            # q = trans.transform.rotation # 회전(쿼터니안)
            # quat = np.array([q.x, q.y, q.z, q.w])

            self.get_logger().info(f"[TF2] g_base → {self.current_tag_frame} : pos={pos}")

            # 2) IK 계산: 목표 위치/회전을 전달
            self.move_robot(pos) # , quat

        except Exception as e:
            self.get_logger().error(f"[TF2 Lookup Error] {e}")
            self.is_busy = False

    def move_robot(self, pos, quat=None):
        # # 2-1) 쿼터니언을 회전행렬로 변환
        # rot_mat = R.from_quat(quat).as_matrix() 
        # sol1) 쿼터니안 직접 설정
        rot_mat = np.array([
            [-0.015,  0.375, -0.927],
            [ 0.998,  0.064,  0.010],
            [ 0.063, -0.925, -0.375]
        ])

        # tcp를 태그까지 보내기 위해 오프셋 설정
        tcp_offset = np.array([0.09, 0, 0])  # x축으로 9cm 앞으로 이동
        # tcp_offset = rot_mat @ tcp_offset
        pos -= tcp_offset  # 회전행렬을 이용해 오프셋 적용

        try:
            # 2-2) IK 계산 (pos, rot)
            self.get_logger().warn(f"[IK 계산용 체인]:{self.chain}")
            ik_result = self.chain.inverse_kinematics(pos, target_orientation=rot_mat)
            self.get_logger().warn(f"[IK 계산 결과]:{ik_result}") # IK 계산이 완료되면, 인덱스 1번부터 6번까지가 실제 관절값임(self.joint_names 길이만큼)

            radian_list = [ik_result[idx] for idx in self.joint_indices] # joint_indices로 필터링
            degree_list = [round(math.degrees(rad), 2) for rad in radian_list] # 라디안 → 각도 (float) 변환

            # 3) 각도 퍼블리시
            self.publish_joint_positions(radian_list)
            # 4) MyCobot에 각도 전송
            self.mc.send_angles(degree_list, 100, _async=True)

            self.get_logger().info(f"[IK] 이동 joint 각도: {degree_list}")
        except Exception as e:
            self.get_logger().error(f"[IK 계산 오류!] {e}")

        # 로봇이 움직일 시간 대기(실 하드웨어 sync)
        time.sleep(2.0)
        self.is_busy = False

    def publish_joint_positions(self, angles):
        # 3) JointState 메시지 생성 및 퍼블리시
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = angles[1:7]
        msg.velocity = [0.0]*len(self.joint_names)
        msg.effort = [0.0]*len(self.joint_names)
        self.joint_pub.publish(msg) # 퍼블리시

def main():
    rclpy.init()
    node = AprilTagToRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
