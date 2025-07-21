import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import time

URDF_PATH = "/home/jetcobot/colcon_ws/src/jetcobot/jetcobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"

class AprilTagToRobot(Node):
    def __init__(self):
        super().__init__('apriltag_to_robot')

        # 관절 이름 정의 (URDF 기준)
        self.joint_names = [
            "link2_to_link1", "link3_to_link2", "link4_to_link3",
            "link5_to_link4", "link6_to_link5", "link6output_to_link6"
        ]

        # IK 계산용 체인 로드
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["g_base"])

        # Publish: 로봇의 위치를 /joint_states로 전송
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # tf2 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 현재 처리 중인 태그 프레임 이름
        self.current_tag_frame = None
        self.is_busy = False

        # 태그 pose 수신 토픽
        self.create_subscription(PoseStamped, '/apriltag_goal_pose', self.apriltag_callback, 10)

    def apriltag_callback(self, msg: PoseStamped):
        if self.is_busy:
            return

        self.current_tag_frame = msg.header.frame_id  # 예: "tag36h11:1"
        self.get_logger().info(f"[TAG DETECTED] {self.current_tag_frame}")

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp  # ROS 시간 동기
        tf_msg.header.frame_id = "jetcocam"     # 카메라 프레임
        tf_msg.child_frame_id = msg.header.frame_id  # 태그 이름 (ex: tag36h11:1)
        tf_msg.transform.translation = msg.pose.position
        tf_msg.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        self.is_busy = True
        self.move_to_tag()

    def move_to_tag(self):
        if self.current_tag_frame is None:
            self.is_busy = False
            return

        try:
            # tf2를 사용해 g_base → 태그 프레임 변환 조회
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='g_base',
                source_frame=self.current_tag_frame,
                time=rclpy.time.Time()
            )
            # 위치 추출
            t = trans.transform.translation
            pos = np.array([t.x, t.y, t.z])

            # 회전 추출 (쿼터니언)
            q = trans.transform.rotation
            quat = np.array([q.x, q.y, q.z, q.w])

            self.get_logger().info(f"[TF2] Target pos: {pos}, quat: {quat}")
            self.move_robot(pos, quat)

        except Exception as e:
            self.get_logger().error(f"[TF2 Lookup Error] {e}")
            self.is_busy = False

    def publish_joint_positions(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = angles[1:7]
        self.publisher_.publish(msg)
        self.get_logger().info(f"[IK] Published joint positions: {angles[1:7]}")

    def move_robot(self, pos, quat):
        rot = R.from_quat(quat).as_matrix() # ㅎ회전 행렬로 변환
        angles = self.chain.inverse_kinematics(pos, target_orientation=rot) # IK 계산
        self.publish_joint_positions(angles) # 관절 위치 전송
        self.is_busy = True 
        self.get_logger().info("[MOVE] Robot moving to target position...")
        time.sleep(1.5)  # 동작 시간 고려 (하드웨어 연동 시 조정)
        self.is_busy = False

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
