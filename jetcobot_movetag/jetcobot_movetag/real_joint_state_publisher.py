# scripts/real_joint_state_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import pymycobot

from pymycobot.mycobot import MyCobot

class RealJointStatePublisher(Node):
    def __init__(self):
        super().__init__('real_joint_state_publisher')

        # MyCobot 초기화 - 포트와 속도는 실제 사용 환경에 맞춰 수정!
        self.mc = MyCobot("/dev/ttyJETCOBOT", 1000000)
        self.joint_names = [
            "1_Joint",
            "2_Joint",
            "3_Joint",
            "4_Joint",
            "5_Joint",
            "6_Joint",
        ]

        # 퍼블리셔 생성
        self.pub = self.create_publisher(JointState, '/real_joint_states', 10)  # 이 토픽이 중요
        self.timer = self.create_timer(0.002, self.publish_joint_states)  # 5Hz

    def publish_joint_states(self):
        angles = self.mc.get_radians()
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        if angles is not None and len(angles) == len(self.joint_names):
            joint_state.position = angles
        else:
            self.get_logger().warn('Failed to get valid angles. Publishing zeros.')
            joint_state.position = [0.0] * len(self.joint_names)

        joint_state.velocity = [0.0] * len(self.joint_names)
        joint_state.effort = [0.0] * len(self.joint_names)

        self.pub.publish(joint_state)
        self.get_logger().info(f"Published joint_state: {joint_state.position}")

def main():
    rclpy.init()
    rclpy.spin(RealJointStatePublisher())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
