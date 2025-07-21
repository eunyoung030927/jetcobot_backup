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
# 굿 근데 각도가 이상함 (최신 ver) (0717 14:00)


URDF_PATH = "/home/jetcobot/colcon_ws/src/jetcobot/jetcobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"

class AprilTagToRobot(Node):
    def __init__(self):
        super().__init__('apriltag_to_robot')
        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000) # MyCobot 초기화

        # 1. 로봇 각 관절 이름(URDF 기준)
        self.joint_names = [
            "link2_to_link1", "link3_to_link2", "link4_to_link3",
            "link5_to_link4", "link6_to_link5", "link6output_to_link6"
        ]

        # 2. 역기구학용 Chain 로드
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["g_base"])

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
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='link1',
                source_frame=self.current_tag_frame,
                time=rclpy.time.Time()
            )
            t = trans.transform.translation # 위치
            pos = np.array([t.x, t.y, t.z])

            q = trans.transform.rotation # 회전(쿼터니안)
            quat = np.array([q.x, q.y, q.z, q.w])

            self.get_logger().info(f"[TF2] g_base → {self.current_tag_frame} : pos={pos}, quat={quat}")

            # 2) IK 계산: 목표 위치/회전을 전달
            self.move_robot(pos, quat)

        except Exception as e:
            self.get_logger().error(f"[TF2 Lookup Error] {e}")
            self.is_busy = False

    def move_robot(self, pos, quat):
        # 2-1) 쿼터니언을 회전행렬로 변환
        rot_mat = R.from_quat(quat).as_matrix() 
        try:
            # 2-2) IK 계산 (pos, rot)
            angles = self.chain.inverse_kinematics(pos, target_orientation=rot_mat)
            # IK 계산이 완료되면, 인덱스 1번부터 6번까지가 실제 관절값임(self.joint_names 길이만큼)
            
            # 3) 각도 퍼블리시
            self.publish_joint_positions(angles)
            # 4) MyCobot에 각도 전송 # MyCobot은 1번부터 6번까지의 관절값을 받음 (0은 그리퍼 관련)
            angles_subset = list(angles[1:7])  # 6개 값 추출 (라디안 단위)
            degree_list = [round(math.degrees(rad), 2) for rad in angles_subset] # 라디안 → 각도 (float) 변환
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
