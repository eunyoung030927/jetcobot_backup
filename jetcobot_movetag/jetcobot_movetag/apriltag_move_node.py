import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header, Bool
import tf2_ros
import numpy as np
import time
from pymycobot.mycobot import MyCobot
import math


URDF_PATH = "/home/jetcobot/silver_ws/src/jetcobot_movetag/urdf/jetcobot.urdf"
class AprilTagToRobot(Node):
    def __init__(self):
        super().__init__('apriltag_to_robot')
        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000) # MyCobot 초기화

        # 1. 실제 움직여야하는 조인트 (URDF 기준)
        self.joint_names = [
            "1_Joint", "2_Joint", "3_Joint",
            "4_Joint", "5_Joint", "6_Joint"
        ]

        # 2. 역기구학용 Chain 로드
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["base_link"]) # urdf 바꿨더니 마지막 링크가 tcp네? 
        # self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["base_link"], last_link_vector=np.array([0.1139, -0.0000566, 0.0123])) # last_link_name='tcp' # tcp가 y축으로 3.5cm 아래, 에 위치
        self.joint_indices = [idx for idx, link in enumerate(self.chain.links) if link.name in self.joint_names] # IK 계산 결과 필터링할 인덱스

        # 3. 관절 상태 퍼블리셔
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 4. tf2 버퍼 + 리스너 (프레임 사이 변환 자동 관리)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. 태그 Pose 구독
        self.create_subscription(PoseStamped, '/apriltag_goal_pose', self.apriltag_callback, 10)

        # 새로운 태그가 감지됐는지
        self.is_detected = False  # 태그 감지 상태 플래그 초기화
        self.create_subscription(Bool, 'apriltag_detected', self.detected_callback, 10)

        self.current_tag_frame = None
        self.is_busy = False

    def detected_callback(self, msg: Bool):
        self.is_detected = msg.data  # 태그 감지 상태 업데이트
        # self.get_logger().info(f"[NEW TAG DETECTED] {self.is_tag_detected}")

    def wait_lookup_transform(self, buffer, target_frame, source_frame, timeout):
        t_start = time.time() 
        while (time.time() - t_start) < timeout: # timeout 시간 동안 반복
            try:
                trans = buffer.lookup_transform(
                    target_frame=target_frame, # 이 좌표계로 바꿔줌 
                    source_frame=source_frame, 
                    time=rclpy.time.Time())
                return trans
            except tf2_ros.TransformException: # LookupException + ExtrapolationException 
                time.sleep(0.05)  # 잠시 대기 후 재시도 
        raise RuntimeError(f"Transform from {source_frame} to {target_frame}) not found within {timeout} seconds.")


    def apriltag_callback(self, msg: PoseStamped):
        if self.is_busy: # 바쁘면 무시 
            return

        if not self.is_detected:  # 새로운 태그가 감지되지 않았으면 무시
            self.get_logger().info("[TAG NOT DETECTED] Waiting for new tag...")
            return

        self.is_busy = True # 태그 처리 중 플래그 설정

        self.current_tag_frame = msg.header.frame_id  # ex: "tag36h11:4"
        self.get_logger().info(f"[NEW TAG DETECTED] {self.current_tag_frame}: pos={msg.pose.position}")

        # 1) tf2에서 base_link → tagNN 변환 계산
        try: # tf 변환을 기다리는 함수 호출 
            trans = self.wait_lookup_transform(self.tf_buffer, 'base_link', self.current_tag_frame, 2.0)
            
            t = trans.transform.translation # 위치
            pos = np.array([t.x, t.y, t.z])
            # q = trans.transform.rotation # 회전(쿼터니안)
            # quat = np.array([q.x, q.y, q.z, q.w])

            self.get_logger().info(f"[TF2] base_link → {self.current_tag_frame} : pos={pos}")

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
            [-0.277, -0.947, -0.162],
            [-0.936,  0.304, -0.178],
            [ 0.218,  0.102, -0.971]
        ])
        # rot_mat = np.array([[0.0525225,  0.8790978, -0.4737388], # x:45, y:0, z:-90
        #                    [-0.8790978, -0.1843469, -0.4395489],
        #                    [-0.4737388,  0.4395489,  0.7631306]])  # 회전행렬을 단위행렬로 설정 (회전 없음)

        # tcp를 태그까지 보내기 위해 오프셋 설정
        # tcp_offset = np.array([0.0, 0.0, 0.123])  # x축으로 9cm 앞으로 이동
        # tcp_offset = rot_mat @ tcp_offset
        # pos += tcp_offset  # 회전행렬을 이용해 오프셋 적용
        # self.get_logger().info(f"[TF2 offset] pos={pos}, offset={tcp_offset}")

        try:                
            # 2-2) IK 계산 (pos, rot)
            self.get_logger().warn(f"[IK 계산용 체인]:{self.chain}")
            ik_result = self.chain.inverse_kinematics(pos, target_orientation=rot_mat)
            self.get_logger().info(f"[IK 계산 결과]:{ik_result}") # IK 계산이 완료되면, 인덱스 1번부터 6번까지가 실제 관절값임(self.joint_names 길이만큼)

            radian_list = [ik_result[idx] for idx in self.joint_indices] # joint_indices로 필터링
            degree_list = [round(math.degrees(rad), 2) for rad in radian_list] # 라디안 → 각도 (float) 변환



            # 3) 각도 퍼블리시
            self.publish_joint_positions(radian_list)
            # 4) MyCobot에 각도 전송
            self.mc.send_angles(degree_list, 50, _async=True)

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
