import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import tf2_ros
import numpy as np
import time
from pymycobot.mycobot import MyCobot
import math

URDF_PATH = "/home/jetcobot/silver_ws/src/jetcobot_movetag/urdf/jetcobot.urdf"
STATE_IDLE = 'IDLE'
STATE_APPROACHING = 'APPROACHING'
STATE_FINE_ADJUST = 'FINE_ADJUST'

class AprilTagToRobot(Node):
    def __init__(self):
        super().__init__('apriltag_to_robot')
        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000)
        self.init_chain()
        self.init_pubsub()

        self.is_detected = False
        self.state = STATE_IDLE
        self.current_tag_frame = None
        self.latest_tag_pose = None           # 최근 태그 포즈 tf 정보 저장

        self.approach_offset = np.array([0.0, 0.0, 0.09])
        self.cam_offset = np.array([0.0, 0.0, 0.07])

    def init_chain(self):
        self.joint_names = [
            "1_Joint", "2_Joint", "3_Joint",
            "4_Joint", "5_Joint", "6_Joint"
        ]
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["base_link"])
        self.joint_indices = [idx for idx, link in enumerate(self.chain.links) if link.name in self.joint_names]

    def init_pubsub(self):
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseStamped, '/apriltag_goal_pose', self.apriltag_callback, 10)
        self.create_subscription(Bool, 'apriltag_detected', self.detected_callback, 10)

    def wait_lookup_transform(self, target_frame, source_frame, timeout):
        t_start = time.time()
        while (time.time() - t_start) < timeout:
            try:
                trans = self.tf_buffer.lookup_transform(
                    target_frame=target_frame,
                    source_frame=source_frame,
                    time=rclpy.time.Time())
                return trans
            except tf2_ros.TransformException:
                time.sleep(0.1)
        raise RuntimeError(f"Transform from {source_frame} to {target_frame}) not found within {timeout} seconds.")

    def apply_offset(self, position, offset):
        position = np.array(position) + offset
        self.get_logger().info(f"[OFFSET APPLIED] pos={position}, offset={offset}")
        return position

    def compute_IK(self, pos, rot_mat):
        self.get_logger().warn(f"[IK 계산용 체인]:{self.chain}")
        ik_result = self.chain.inverse_kinematics(pos, target_orientation=rot_mat)
        radian_list = [ik_result[idx] for idx in self.joint_indices]
        degree_list = [round(math.degrees(rad), 2) for rad in radian_list]
        self.get_logger().info(f"[IK] 이동할 joint 각도: {degree_list}")
        return radian_list, degree_list

    def send_joint_angles(self, radian_list, degree_list):
        self.publish_joint_positions(radian_list)
        self.mc.send_angles(degree_list, 20, _async=True)
        time.sleep(2)  # 움직임 대기

    def compute_rot_mat(self, quat=None):
        rot_mat = np.array([
            [-0.277, -0.947, -0.162],
            [-0.936,  0.304, -0.178],
            [ 0.218,  0.102, -0.971]
        ])
        return rot_mat

    def apriltag_callback(self, msg: PoseStamped):
        if not self.is_detected:
            self.get_logger().info("[TAG NOT DETECTED] Waiting for new tag...")
            return

        self.current_tag_frame = msg.header.frame_id

        # 태그의 현재 base_link->tag frame 변환 정보 저장 (계속 최신값)
        try:
            trans = self.wait_lookup_transform('base_link', self.current_tag_frame, timeout=2.0)
            t = trans.transform.translation
            pos_tag = np.array([t.x, t.y, t.z])
            self.latest_tag_pose = pos_tag  # 최신 태그 위치 항상 갱신
        except Exception as e:
            self.get_logger().error(f"[TF2 Lookup Error] {e}")
            return

        # 상태 기반 처리 (State Machine)
        if self.state == STATE_IDLE:
            # 1단계: 접근 포즈 이동
            approach_pos = self.apply_offset(pos_tag, self.cam_offset + self.approach_offset)
            self.get_logger().info(f"[STATE: IDLE→APPROACHING] 접근 포즈 이동: {approach_pos}")
            self.state = STATE_APPROACHING
            self.move_robot(approach_pos)
        
        elif self.state == STATE_APPROACHING:
            # 2단계: 최신 태그 위치로 정확 포즈 이동
            final_pos = self.apply_offset(pos_tag, self.cam_offset)
            self.get_logger().info(f"[STATE: APPROACHING→FINE_ADJUST] 최종 위치 이동: {final_pos}")
            self.state = STATE_FINE_ADJUST
            self.move_robot(final_pos)

        elif self.state == STATE_FINE_ADJUST:
            # 필요 시 DONE 등 추가 처리, 아니면 바로 IDLE로 복원(한 번만 동작)
            self.get_logger().info("[STATE: FINE_ADJUST] 정밀 정지. 다음 태그를 기다립니다.")
            self.state = STATE_IDLE

    def move_robot(self, pos, quat=None):
        rot_mat = self.compute_rot_mat()
        try:
            radian_list, degree_list = self.compute_IK(pos, rot_mat)
            self.send_joint_angles(radian_list, degree_list)
        except Exception as e:
            self.get_logger().error(f"[IK 계산 오류!] {e}")

    def publish_joint_positions(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = angles[1:7]
        msg.velocity = [0.0]*len(self.joint_names)
        msg.effort = [0.0]*len(self.joint_names)
        self.joint_pub.publish(msg)

    def detected_callback(self, msg: Bool):
        self.is_detected = msg.data

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
