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

# 함수 정리 및 approach pose + rotation matrix 설정

URDF_PATH = "/home/jetcobot/silver_ws/src/jetcobot_movetag/urdf/jetcobot.urdf"
class AprilTagToRobot(Node):
    def __init__(self):
        super().__init__('apriltag_to_robot')
        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000) # MyCobot 초기화
        self.init_chain() # 실제 움직일 joint 설정 후, chain load 
        self.init_pubsub() # publisher, subscriber 부분 코드 정리 

        self.is_detected = False  # 태그 감지 상태 플래그 초기화
        self.is_busy = False
        self.current_tag_frame = None

        # self.approach_offset = np.array([0.0, 0.0, 0.09]) # z축으로 9cm 위로 이동
        self.cam_offset = np.array([0.0, 0.0, 0.07]) # z축으로 7cm 위로 이동

    def init_chain(self): # 실제 움직일 joint 설정 후, chain load 
        # 1. 실제 움직여야하는 조인트 (URDF 기준)
        self.joint_names = [
            "1_Joint", "2_Joint", "3_Joint",
            "4_Joint", "5_Joint", "6_Joint"
        ]

        # 2. 역기구학용 Chain 로드
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["base_link"])
        self.joint_indices = [idx for idx, link in enumerate(self.chain.links) if link.name in self.joint_names] # IK 계산 결과 필터링할 인덱스

    def init_pubsub(self): # publisher, subscriber 부분 코드 정리 
        # 3. 관절 상태 퍼블리셔
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 4. tf2 버퍼 + 리스너 (프레임 사이 변환 자동 관리)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 5. 태그 Pose 구독
        self.create_subscription(PoseStamped, '/apriltag_goal_pose', self.apriltag_callback, 10)
        # 새로운 태그가 감지됐는지 여부 구독 
        self.create_subscription(Bool, 'apriltag_detected', self.detected_callback, 10)


    def wait_lookup_transform(self, target_frame, source_frame, timeout): # lookup_transform 에러 처리
        t_start = time.time() 
        while (time.time() - t_start) < timeout: # timeout 시간 동안 반복
            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame=target_frame, # 이 좌표계로 바꿔줌 
                    source_frame=source_frame, 
                    time=rclpy.time.Time())
                return trans
            except tf2_ros.TransformException: # LookupException + ExtrapolationException 
                time.sleep(0.1)  # 잠시 대기 후 재시도 
        raise RuntimeError(f"Transform from {source_frame} to {target_frame}) not found within {timeout} seconds.")

    def apply_offset(self, position, offset): # offset 계산
        position = np.array(position) + offset
        self.get_logger().info(f"[OFFSET APPLIED] pos={position}, offset={offset}")
        return position

    def compute_IK(self, pos, rot_mat): # IK 계산 (degree, radian list)
        self.get_logger().warn(f"[IK 계산용 체인]:{self.chain}")
        ik_result = self.chain.inverse_kinematics(pos, target_orientation=rot_mat)
        self.get_logger().info(f"[IK 계산 결과]:{ik_result}") # IK 계산이 완료되면, 인덱스 1번부터 6번까지가 실제 관절값임(self.joint_names 길이만큼)

        radian_list = [ik_result[idx] for idx in self.joint_indices] # joint_indices로 필터링
        degree_list = [round(math.degrees(rad), 2) for rad in radian_list] # 라디안 → 각도 (float) 변환
        self.get_logger().info(f"[IK] 이동할 joint 각도: {degree_list}")
        return radian_list, degree_list

    def send_joint_angles(self, radian_list, degree_list): # 실제로 로봇을 움직이는 부분 
        self.publish_joint_positions(radian_list) # 각도 퍼블리시
        self.mc.send_angles(degree_list, 20, _async=True) # MyCobot에 각도 전송
        time.sleep(2) # 움직임 대기

    def compute_rot_mat(self, quat=None, pos=None): # # 나중엔 quat 주고 집기 좋은 쿼터니안으로 설정할 것 
        """
        태그의 쿼터니언(quat)과 위치(pos)를 받아서,
        로봇 기준(base_link)에서 태그까지의 방향을 보고
        집기 좋은 회전행렬을 계산하는 함수

        Args:
            quat (np.array or list): [x, y, z, w] 순서의 태그 회전 쿼터니언 (base_link→tag)
            pos (np.array or list): [x, y, z] 태그 위치 (base_link 기준)

        Returns:
            rot_mat (np.ndarray): 3x3 직교 회전행렬 (로봇 엔드이펙터 목적 자세)
        """

        # 1) 입력값 검사: 없으면 단위행렬(회전없음) 반환
        if quat is None or pos is None:
            self.get_logger().warn("compute_rot_mat : quat or pos is None, returning identity.")
            return np.eye(3)

        # 2) 태그 쿼터니언 → 회전행렬 변환
        # 태그가 현재 로봇 기준으로 어떤 방향 회전했는지 3x3 행렬로 변환
        rot_mat_tag = R.from_quat(quat).as_matrix()

        # 3) 태그 고유(local) 좌표계 4개 주요 축 정의
        # 태그 기준 +X(앞), -X(뒤), +Y(오른쪽), -Y(왼쪽) 방향 벡터
        tag_axes_local = [
            np.array([1, 0, 0]),   # +X 축
            np.array([-1, 0, 0]),  # -X 축
            np.array([0, 1, 0]),   # +Y 축
            np.array([0, -1, 0])   # -Y 축
        ]

        # 4) 위 축들을 태그의 현재 회전상태(rot_mat_tag)로 회전 -> base_link 기준 좌표계상의 방향으로 변환
        tag_axes_world = []
        for v in tag_axes_local:
            axis = rot_mat_tag.dot(v) # 행렬곱으로 회전 적용
            axis[2] = 0  # Z 성분은 무시하고 XY 평면 방향만 고려 (바닥 평면)
            norm = np.linalg.norm(axis)
            if norm > 1e-6:
                axis /= norm # 크기를 1로 맞춘 단위벡터
            else:
                axis = np.array([0, 0, 0]) # 벡터 크기가 너무 작으면 0벡터 처리해 안정성 확보
            tag_axes_world.append(axis)

        # 5) base_link(원점)에서 태그 위치까지의 방향 벡터 계산 (XY 평면)
        tag_x, tag_y = pos[0], pos[1] # 태그 위치 x, y
        dist = np.hypot(tag_x, tag_y) # XY 평면 거리 계산 # sqrt(tag_x**2 + tag_y**2)
        if dist < 1e-6:
            base_direction = np.array([1, 0, 0]) # 태그와 같은 위치면 임의 축 지정 # 에러 방지 
        else:
            # 태그 위치에서 base_link 원점 방향으로 향하는 단위 벡터 (즉, tag → base 방향)
            base_direction = np.array([-tag_x / dist, -tag_y / dist, 0])

        # 6) 4개 축 중 base_direction과 내적(dot product)이 가장 큰 축을 찾아 선택
        dot_products = [np.dot(base_direction, axis) for axis in tag_axes_world] # 두 벡터 간 방향 유사도(1=완벽 일치)
        best_idx, max_dot = max(enumerate(dot_products), key=lambda x: x[1]) # 내적값이 최대인(가장 유사한) 축의 index 추출 

        # 7) 그리퍼의 TCP 좌표계 축 설정
        tcp_z = np.array([0, 0, -1]) # Z축은 항상 아래 방향 고정(바닥을 향함)
        tcp_y = tag_axes_world[best_idx] # Y축은 베이스 방향과 가장 정렬된 태그 축 (잡기 좋은 방향)
        tcp_x = np.cross(tcp_y, tcp_z) # X축은 Y축과 Z축의 외적으로 직교 보장

        norm_x = np.linalg.norm(tcp_x)
        if norm_x < 1e-6: # 외적 결과가 0에 가까우면 fallback 처리
            tcp_x = np.array([1, 0, 0])
        else:
            tcp_x /= norm_x # 단위벡터화

        # Y축도 다시 계산해서 완전 직교 보장
        tcp_y = np.cross(tcp_z, tcp_x)
        tcp_y /= np.linalg.norm(tcp_y)

        # 8) X,Y,Z 축을 열로 쌓아 3x3 회전행렬 생성
        # 이 행렬은 엔드이펙터의 목표 자세를 나타냄
        rot_mat = np.column_stack((tcp_x, tcp_y, tcp_z))

        # 최종 완성된 회전행렬 반환
        return rot_mat


    def apriltag_callback(self, msg: PoseStamped):
        if self.is_busy: # 바쁘면 무시 
            return
        if not self.is_detected:  # 새로운 태그가 감지되지 않았으면 무시
            self.get_logger().info("[TAG NOT DETECTED] Waiting for new tag...")
            return

        self.is_busy = True # 태그 처리 중 플래그 설정
        self.current_tag_frame = msg.header.frame_id  # ex: "tag41h12:2"
        self.get_logger().info(f"[NEW TAG DETECTED] {self.current_tag_frame}: pos={msg.pose.position}")

        # 1-1) tf2에서 base_link → tagNN 변환 계산
        try: # tf 변환을 기다리는 함수 호출 
            trans = self.wait_lookup_transform('base_link', self.current_tag_frame, timeout=5.0)
            
            t = trans.transform.translation # 위치
            pos = np.array([t.x, t.y, t.z])
            q = trans.transform.rotation # 회전(쿼터니안)
            quat = np.array([q.x, q.y, q.z, q.w])
            self.get_logger().info(f"[TF2] base_link → {self.current_tag_frame} : pos={pos}")

            pos = self.apply_offset(pos, self.cam_offset) # offset 설정 

            # 2) IK 계산: 목표 위치/회전을 전달
            self.move_robot(pos, quat) # 나중이 지금이네.?

        except Exception as e:
            self.get_logger().error(f"[TF2 Lookup Error] {e}")
            self.is_busy = False # 콜백 실행끝나면 False로 

    def move_robot(self, pos, quat):
        rot_mat = self.compute_rot_mat(pos, quat) # quat # 나중엔 quat 주고 집기 좋은 쿼터니안으로 설정할 것 
        try: # IK 계산 (pos, rot)             
            radian_list, degree_list = self.compute_IK(pos, rot_mat)
            self.send_joint_angles(radian_list, degree_list) # 실제 로봇 움직이는 코드 
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

    def detected_callback(self, msg: Bool):
        self.is_detected = msg.data  # 태그 감지 상태 업데이트
        # self.get_logger().info(f"[NEW TAG DETECTED] {self.is_tag_detected}")

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
