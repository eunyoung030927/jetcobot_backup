import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
import numpy as np
import time
import transforms3d

class IKJointPublisher(Node):
    def __init__(self, urdf_path):
        super().__init__('ik_joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_names = [
            "link2_to_link1",
            "link3_to_link2",
            "link4_to_link3",
            "link5_to_link4",
            "link6_to_link5",
            "link6output_to_link6"
        ]
        self.chain = Chain.from_urdf_file(urdf_path, base_elements=["g_base"])
        self.subscription = self.create_subscription(
            Pose, 'apriltag_goal_pose', self.pose_callback, 10
        )

    def pose_callback(self, msg):
        # Pose → 위치/회전행렬
        position = [msg.position.x, msg.position.y, msg.position.z]
        # 쿼터니언 → 회전행렬
        quat = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]  # transforms3d: w, x, y, z
        rot = transforms3d.quaternions.quat2mat(quat)
        # IKPy 역기구학
        angles = self.chain.inverse_kinematics(
            position,
            target_orientation=rot, 
        )
        # JointState 퍼블리시 (IKPy의 0번은 base, 1~6번이 실제 관절)
        msg_js = JointState()
        msg_js.header.stamp = self.get_clock().now().to_msg()
        msg_js.name = self.joint_names
        msg_js.position = angles[1:7]
        self.publisher_.publish(msg_js)
        self.get_logger().info(f"Published joint positions: {angles[1:7]}")

def main():
    urdf_path = "/home/jetcobot/colcon_ws/src/jetcobot/jetcobot_description/urdf/mycobot_280_m5/mycobot_280m5_with_gripper_parallel.urdf"
    rclpy.init()
    node = IKJointPublisher(urdf_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
