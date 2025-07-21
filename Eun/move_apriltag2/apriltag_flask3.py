import cv2
import numpy as np
from flask import Flask, Response
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import transforms3d
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
import tf2_ros

# publish 타입을 PoseStamped로 변경
# ROS2 퍼블리셔 노드 정의
class PosePublisher(Node):
    def __init__(self):
        super().__init__('apriltag_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'apriltag_goal_pose', 10)
        # TF broadcaster 추가!!
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def publish_pose(self, tag_id, position, orientation):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"tag36h11:{tag_id}"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = orientation
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published tag36h11:{tag_id}: {position}, {orientation}")

        # TF 트리 브로드캐스트 (자동 연결)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp           # 시간 동기화
        tf_msg.header.frame_id = "jetcocam"              # 카메라 프레임 명 (혹은 실제 setup에 맞게)
        tf_msg.child_frame_id = f"tag36h11:{tag_id}"     # 태그 프레임 명
        tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = position
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = orientation
        self.tf_broadcaster.sendTransform(tf_msg)        # tf 트리에 태그 프레임 연동

# Flask 및 AprilTag 설정
app = Flask(__name__)
cap = cv2.VideoCapture('/dev/jetcocam0')
detector = Detector(families='tag36h11') # AprilTag 디텍터 설정

camera_matrix = np.array([ # 카메라 캘리브레이션 매트릭스
    [978.548555, 0.0, 293.112691],
    [0.0, 981.781244, 163.100487],
    [0.0, 0.0, 1.0]
])
dist_coeffs = np.array([-0.481485, 0.586547, -0.000026, 0.005891, 0.0]) # 렌즈 왜곡 계수
tag_size = 0.028  # meter

# ROS2 초기화
rclpy.init()
pose_publisher = PosePublisher()

def gen_frames():
    while True:
        ret, frame = cap.read() # 프레임 성공 여부(bool), 실제 프레임(이미지 데이터)(numpy array)
        if not ret:
            break

        frame = cv2.undistort(frame, camera_matrix, dist_coeffs) # 렌즈 왜곡 보정
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 색상 변환 (BGR → Grayscale)
        results = detector.detect(
            gray,
            estimate_tag_pose=True, # 태그 3D 위치 추정
            # 태그 검출 시 보정할 때 사용한 파라미터를 전달해야 제대로 추청 가능 
            camera_params=[ # 카메라 캘리브레이션 설정
                camera_matrix[0, 0],  # fx
                camera_matrix[1, 1],  # fy
                camera_matrix[0, 2],  # cx
                camera_matrix[1, 2]   # cy
            ],
            tag_size=tag_size
        )
        # print(results)
        for r in results:

            # 태그의 위치(t)와 회전(R)
            tag_id = r.tag_id # 태그 ID
            t = r.pose_t.flatten()
            R = r.pose_R
            # 회전행렬 → 쿼터니언 변환 (w, x, y, z 순서)
            quat_wxyz = transforms3d.quaternions.mat2quat(R)
            quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
            # ROS2로 퍼블리시
            pose_publisher.publish_pose(tag_id, t, quat_xyzw) # tag_id, 위치, 쿼터니언 순서로 퍼블리시

            # 시각화 (생략 가능)
            (ptA, ptB, ptC, ptD) = r.corners
            ptA, ptB, ptC, ptD = map(lambda x: tuple(map(int, x)), [ptA, ptB, ptC, ptD])
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
            cX, cY = int(r.center[0]), int(r.center[1])
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(frame, str(r.tag_id), (ptA[0], ptA[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '''
    <html>
    <head>
        <title>AprilTag Detection Stream</title>
    </head>
    <body>
        <h1>AprilTag Detection Stream</h1>
        <img src="/video_feed">
    </body>
    </html>
    '''

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
