import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFSubscriber(Node):
    def __init__(self):
        super().__init__('tf_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',  # 구독할 토픽 이름
            self.tf_callback,
            10
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            t = transform.transform.translation
            q = transform.transform.rotation
            print(f"{parent} -> {child}")
            print(f"  Translation: x={t.x:.5f}, y={t.y:.5f}, z={t.z:.5f}")
            print(f"  Quaternion:  x={q.x:.5f}, y={q.y:.5f}, z={q.z:.5f}, w={q.w:.5f}")
            print("-" * 40)

def main():
    rclpy.init()
    node = TFSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
