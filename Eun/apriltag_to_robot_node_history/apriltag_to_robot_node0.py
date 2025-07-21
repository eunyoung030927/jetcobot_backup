import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
import tf2_ros
from geometry_msgs.msg import TransformStamped

# 겹치는 코드 정리 
URDF_PATH = "/home/jetcobot/colcon_ws/src/jetcobot/jetcobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"

CHAIN = [
    ('g_base', 'link1'),
    ('link1', 'link2'),
    ('link2', 'link3'),
    ('link3', 'link4'),
    ('link4', 'link5'),
    ('link5', 'link6'),
    ('link6', 'link6_flange'),
    ('link6_flange', 'gripper_base'),
    ('gripper_base', 'jetcocam')
    # ,('jetcocam', 'tag36h11:1')
]

FIXED_TF_DICT = { # 고정된 TF 정보
    ('map', 'g_base'): (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    ('g_base', 'link1'): (np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0])),
    ('link6_flange', 'gripper_base'): (
        np.array([0.0, 0.0, 0.034]),
        np.array([-0.2717039124397632, 0.65595638159753, 0.6505970827175301, -0.269484035456352])
    ),
    ('gripper_base', 'jetcocam'): (
        np.array([0.0, 0.0, -0.048]),
        np.array([2.597353007557415e-06, 0.707105482506466, 0.7071080798547033, -2.5973434669646147e-06])
    ),
}

class AprilTagToRobot(Node):
    def __init__(self):
        super().__init__('apriltag_to_robot')
        self.joint_names = [
            "link2_to_link1", "link3_to_link2", "link4_to_link3",
            "link5_to_link4", "link6_to_link5", "link6output_to_link6"
        ]
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["g_base"]) # IK 계산용 체인 로드
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10) # 로봇의 위치를 /joint_states로 전송
        self.tf_dict = {}
        self.tag = None
        self.is_busy = False  # 동작 중 여부
        self.tf_buffer = tf2_ros.Buffer() # tf2_ros로 태그 계산을 위한 버퍼 초기화
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # tf2 listener 초기화

        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(PoseStamped, '/apriltag_goal_pose', self.apriltag_callback, 10)

    def tf_callback(self, msg): # TF 메시지를 처리하는 콜백 함수 
        # link1부터 link6_flange까지의 TF 정보를 처리 
        if self.is_busy:
            return  # 동작 중이면 무시
        updated = False
        for tf in msg.transforms:
            parent, child = tf.header.frame_id, tf.child_frame_id
            t = tf.transform.translation
            q = tf.transform.rotation
            self.tf_dict[(parent, child)] = (
                np.array([t.x, t.y, t.z]),
                np.array([q.x, q.y, q.z, q.w])
            )
            if parent == 'jetcocam' and child.startswith('tag36h11:'):
                self.get_logger().warn(f"Detected tag: {child} at {t.x}, {t.y}, {t.z}")
                self.tag = (parent, child)
                updated = True
        if updated:
            self.is_busy = True  # 동작 시작
            self.move_to_tag()

    def apriltag_callback(self, msg):
        if self.is_busy:
            return  # 동작 중이면 무시

        # 메시지에서 태그 ID와 위치, 회전 정보를 추출
        self.tag_id = msg.header.frame_id # ex: "tag36h11:1"
        t = msg.pose.position
        q = msg.pose.orientation
        self.tf_dict[("jetcocam", self.tag_id)] = (
            np.array([t.x, t.y, t.z]),
            np.array([q.x, q.y, q.z, q.w])
        )
        self.get_logger().info(f"Detected tag: {self.tag_id} at {t.x}, {t.y}, {t.z}")
        self.tag = ("jetcocam", self.tag_id)
        self.is_busy = True  # 동작 시작
        self.move_to_tag()

    def merge_fixed_tf(self):
        for k, v in FIXED_TF_DICT.items():
            self.tf_dict[k] = v

    def compute_transform(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='g_base',
                source_frame=self.tag_id,  # ex: 'tag36h11:1'
                time=rclpy.time.Time()
            )
            # 위치
            t = trans.transform.translation
            pos = np.array([t.x, t.y, t.z])

            # 회전 (쿼터니언)
            q = trans.transform.rotation
            quat = np.array([q.x, q.y, q.z, q.w])

            self.get_logger().info(f"[TF2] Target pos: {pos}, quat: {quat}")
            self.move_robot(pos, quat)

        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {str(e)}")
            self.is_busy = False


    def move_to_tag(self):
        if not self.tag: # 태그가 없고 
            self.is_busy = False # 실행중인 것도 없으면 
            return
        self.merge_fixed_tf() # 고정된 TF 정보 병합
        self.get_logger().info(f"Moving to tag: {self.tag}")
        T = np.eye(4)
        for parent, child in CHAIN:
            if (parent, child) not in self.tf_dict and (parent != 'jetcocam'):
                self.get_logger().warn(f"Missing TF: {parent}->{child}")
                self.is_busy = False
                return
            t, q = self.tf_dict[(parent, child)]
            T_i = np.eye(4)
            T_i[:3, :3] = R.from_quat(q).as_matrix()
            T_i[:3, 3] = t
            T = T @ T_i
        pos = T[:3, 3]
        quat = R.from_matrix(T[:3, :3]).as_quat()
        self.get_logger().info(f"Target pos: {pos}, quat: {quat}")
        self.move_robot(pos, quat)

    def publish_joint_positions(self, angles):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = angles[1:7]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joint positions: {angles[1:7]}")

    def move_robot(self, pos, quat):
        rot = R.from_quat(quat).as_matrix()
        angles = self.chain.inverse_kinematics(pos, target_orientation=rot)
        self.publish_joint_positions(angles[1:7])
        self.is_busy = True 
        self.get_logger().info("Moving robot to target position") # 동작 완료 후 잠시 대기(실제 하드웨어에서는 동작 완료 신호로 대체)
        time.sleep(2.0)  # 2초간 대기 (필요시 조정)
        self.is_busy = False  # 다시 태그 감지 허용

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
