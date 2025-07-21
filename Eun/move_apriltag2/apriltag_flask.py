import cv2
import numpy as np
from flask import Flask, Response
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import transforms3d

# ROS2 퍼블리셔 노드 정의
class PosePublisher(Node):
    def __init__(self):
        super().__init__('apriltag_pose_publisher')
        self.publisher_ = self.create_publisher(Pose, 'apriltag_goal_pose', 10)

    def publish_pose(self, position, orientation):
        msg = Pose()
        msg.position.x = position[0]
        msg.position.y = position[1]
        msg.position.z = position[2]
        msg.orientation.x = orientation[0]
        msg.orientation.y = orientation[1]
        msg.orientation.z = orientation[2]
        msg.orientation.w = orientation[3]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published pose: {position}, {orientation}")

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
            t = r.pose_t.flatten()
            R = r.pose_R
            # 회전행렬 → 쿼터니언 변환 (w, x, y, z 순서)
            quat_wxyz = transforms3d.quaternions.mat2quat(R)
            quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
            # ROS2로 퍼블리시
            pose_publisher.publish_pose(t, quat_xyzw)

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
