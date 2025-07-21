import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from pupil_apriltags import Detector
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
import transforms3d
from tf2_msgs.msg import TFMessage

# URDF 파일 경로
URDF_PATH = "/home/jetcobot/colcon_ws/src/jetcobot/jetcobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"

# 카메라 파라미터
CAMERA_MATRIX = np.array([
    [978.548555, 0.0, 293.112691],
    [0.0, 981.781244, 163.100487],
    [0.0, 0.0, 1.0]
])
DIST_COEFFS = np.array([-0.481485, 0.586547, -0.000026, 0.005891, 0.0])
TAG_SIZE = 0.028  # meter

# TF 체인 (base부터 flange까지)
CHAIN = [
    # ('map', 'g_base'),
    ('g_base', 'link1'),
    ('link1', 'link2'),
    ('link2', 'link3'),
    ('link3', 'link4'),
    ('link4', 'link5'),
    ('link5', 'link6'),
    ('link6', 'link6_flange'),
    ('link6_flange', 'gripper_base'),
    ('gripper_base', 'jetcocam'),
    ('jetcocam', 'tag36h11:4')
]

# 고정 변환 정보
FIXED_TF_DICT = {
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
        self.get_logger().warn("테스트 로그")
        self.joint_names = [
            "link2_to_link1",
            "link3_to_link2",
            "link4_to_link3",
            "link5_to_link4",
            "link6_to_link5",
            "link6output_to_link6"
        ]
        self.chain = Chain.from_urdf_file(URDF_PATH, base_elements=["g_base"])
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_dict = {}
        self.latest_tag = None

        # 카메라 및 태그 디텍터
        self.get_logger().warn("캠 테스트 로그")
        self.cap = cv2.VideoCapture('/dev/jetcocam0')
        self.detector = Detector(families='tag36h11')

        # TF 구독
        self.create_subscription(
            Pose, 'apriltag_goal_pose', self.pose_callback, 10
        )
        self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10
        )

    def process_apriltag(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None
        frame = cv2.undistort(frame, CAMERA_MATRIX, DIST_COEFFS)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[
                CAMERA_MATRIX[0, 0], CAMERA_MATRIX[1, 1],
                CAMERA_MATRIX[0, 2], CAMERA_MATRIX[1, 2]
            ],
            tag_size=TAG_SIZE
        )
        if not results:
            return None, None
        r = results[0]
        t = r.pose_t.flatten()
        R_mat = r.pose_R
        quat_wxyz = transforms3d.quaternions.mat2quat(R_mat)
        quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
        return t, quat_xyzw

    def tf_callback(self, msg):
        self.get_logger().warn("콜백 테스트 로그")
        for tf in msg.transforms:
            parent = tf.header.frame_id
            child = tf.child_frame_id
            t = tf.transform.translation
            q = tf.transform.rotation
            self.tf_dict[(parent, child)] = (
                np.array([t.x, t.y, t.z]),
                np.array([q.x, q.y, q.z, q.w])
            )
            if parent == 'jetcocam' and child.startswith('tag36h11:'):
                self.latest_tag = (parent, child)
                self.get_logger().warn(f"Detected tag: {child} at {t}, {q}")
        self.try_move_robot()

    def merge_fixed_tf(self):
        for k, v in FIXED_TF_DICT.items():
            self.tf_dict[k] = v

    def try_move_robot(self):
        # 고정 변환 병합
        self.merge_fixed_tf()

        # 누적곱 계산 (g_base~flange)
        T = np.eye(4)
        for parent, child in CHAIN:
            if (parent, child) not in self.tf_dict:
                return
            t, q = self.tf_dict[(parent, child)]
            T_i = np.eye(4)
            T_i[:3, :3] = R.from_quat(q).as_matrix()
            T_i[:3, 3] = t
            T = T @ T_i

        # 태그 TF
        if not (self.latest_tag and self.latest_tag in self.tf_dict):
            return
        t_tag, q_tag = self.tf_dict[self.latest_tag]
        T_tag = np.eye(4)
        T_tag[:3, :3] = R.from_quat(q_tag).as_matrix()
        T_tag[:3, 3] = t_tag
        T_goal = T @ T_tag
        pos = T_goal[:3, 3]
        quat = R.from_matrix(T_goal[:3, :3]).as_quat()
        self.get_logger().warn(f"Moving robot to position: {pos}, orientation: {quat}")
        self.move_robot(pos, quat)

    def move_robot(self, pos, quat):
        # IKPy에 목표 Pose 입력
        rot = R.from_quat(quat).as_matrix()
        angles = self.chain.inverse_kinematics(
            pos,
            target_orientation=rot,
        )
        msg_js = JointState()
        msg_js.header.stamp = self.get_clock().now().to_msg()
        msg_js.name = self.joint_names
        msg_js.position = angles[1:7]
        self.publisher_.publish(msg_js)
        self.get_logger().warn(f"Published joint positions: {angles[1:7]}")

    def pose_callback(self, msg):
        # 필요시 외부 Pose 메시지로도 이동 가능
        position = [msg.position.x, msg.position.y, msg.position.z]
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rot = R.from_quat(quat).as_matrix()
        angles = self.chain.inverse_kinematics(
            position,
            target_orientation=rot,
        )
        msg_js = JointState()
        msg_js.header.stamp = self.get_clock().now().to_msg()
        msg_js.name = self.joint_names
        msg_js.position = angles[1:7]
        self.publisher_.publish(msg_js)

def main():
    rclpy.init()
    node = AprilTagToRobot()
    try:
        while rclpy.ok():
            t, quat = node.process_apriltag()
            if t is not None and quat is not None:
                pose_msg = Pose()
                pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = t
                pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = quat
                node.pose_callback(pose_msg)
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
