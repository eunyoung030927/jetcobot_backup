import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R
import numpy as np

# 누적 곱을 위한 체인 정의
CHAIN = [
    ('link1', 'link2'),
    ('link2', 'link3'),
    ('link3', 'link4'),
    ('link4', 'link5'),
    ('link5', 'link6'),
    ('link6', 'link6_flange'),
]

class Link1ToTagTF(Node):
    def __init__(self):
        super().__init__('link1_to_tag_tf')
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10
        )
        self.tf_dict = {}  # (parent, child): (translation, quaternion)
        self.latest_tag = None

    def tf_callback(self, msg):
        # TF 토픽에서 변환 정보 저장
        for tf in msg.transforms:
            parent = tf.header.frame_id # 부모 프레임
            child = tf.child_frame_id  # 자식 프레임
            t = tf.transform.translation # Translation Vector
            q = tf.transform.rotation # Quaternion
            # (부모, 자식) 튜플을 키로 사용하여 변환 Translation Vector와 Quaternion을 저장

            self.tf_dict[(parent, child)] = (
                np.array([t.x, t.y, t.z]),
                np.array([q.x, q.y, q.z, q.w])
            )
            # 태그가 인식된 경우
            if parent == 'jetcocam' and child.startswith('tag36h11:'):
                self.latest_tag = (parent, child)

        # 누적 곱 계산 (link1~link6_flange까지)
        T = np.eye(4)
        valid = True
        for parent, child in CHAIN:
            if (parent, child) not in self.tf_dict:
                valid = False
                break
            t, q = self.tf_dict[(parent, child)] # Translation Vector와 Quaternion
            T_i = np.eye(4)
            T_i[:3, :3] = R.from_quat(q).as_matrix() # 쿼터니언을 회전행렬로 변환해 4x4 행렬의 상위 3x3에 할당
            T_i[:3, 3] = t
            T = T @ T_i

        if not valid:
            print("아직 모든 링크의 TF가 들어오지 않았습니다.")
            return

        # 태그가 인식되면 link1~flange 누적행렬에 곱해서 link1→tag 출력
        if self.latest_tag and self.latest_tag in self.tf_dict: # 태그가 인식되었고 TF 정보가 있는 경우
            t_tag, q_tag = self.tf_dict[self.latest_tag]
            T_tag = np.eye(4)
            T_tag[:3, :3] = R.from_quat(q_tag).as_matrix()
            T_tag[:3, 3] = t_tag
            T_link1_to_tag = T @ T_tag # 누적 행렬에 태그의 변환 행렬을 곱함
            pos = T_link1_to_tag[:3, 3] # 최종 위치 추출 
            quat = R.from_matrix(T_link1_to_tag[:3, :3]).as_quat()
            print(f"\nlink1 → {self.latest_tag[1]} 위치: {pos}")
            print(f"link1 → {self.latest_tag[1]} 쿼터니언(x, y, z, w): {quat}\n")

def main():
    rclpy.init()
    node = Link1ToTagTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
