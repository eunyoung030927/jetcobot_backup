import cv2
import numpy as np
from flask import Flask, Response
from pupil_apriltags import Detector
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
import transforms3d
import tf2_ros
import time

# ROS2 퍼블리셔 노드 정의
class PosePublisher(Node):
    def __init__(self):
        super().__init__('apriltag_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'apriltag_goal_pose', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 수정됨: 태그 감지 시간 추적 변수와 타임아웃 seconds
        self.last_tag_time = None
        self.timeout_sec = 1.0
        # Flask와 병렬 동작 위해 타이머 콜백 대신, 직접 poll 방식 사용 예정


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
        tf_msg.child_frame_id = f"tag36h11:{tag_id}"
        tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z = position
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z, tf_msg.transform.rotation.w = orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        # 수정됨: 최근 태그 감지 시간 갱신
        self.last_tag_time = time.time()


    # 수정됨: 태그 미검출 시 "빈" 메시지 퍼블리시(또는 무시)
    def publish_no_tag(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # 태그 없음 표시
        self.publisher_.publish(msg)
        self.get_logger().info("Published: No tag detected (빈 메시지)")


# Flask 및 AprilTag 설정
app = Flask(__name__)
cap = cv2.VideoCapture('/dev/jetcocam0')
detector = Detector(families='tag36h11')

camera_matrix = np.array([
    [978.548555, 0.0, 293.112691],
    [0.0, 981.781244, 163.100487],
    [0.0, 0.0, 1.0]
])
dist_coeffs = np.array([-0.481485, 0.586547, -0.000026, 0.005891, 0.0])
tag_size = 0.028  # meter

# ROS2 초기화
rclpy.init()
pose_publisher = PosePublisher()
last_detect_time = None

def gen_frames():
    global last_detect_time
    prev_no_tag_sent = False  # 수정됨: 마지막에 빈 메시지(per frame) 중복 발송 방지

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[
                camera_matrix[0, 0], camera_matrix[1, 1],
                camera_matrix[0, 2], camera_matrix[1, 2]
            ],
            tag_size=tag_size
        )

        if len(results) > 0:
            for r in results:
                tag_id = r.tag_id
                t = r.pose_t.flatten()
                R = r.pose_R
                quat_wxyz = transforms3d.quaternions.mat2quat(R)
                quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]
                pose_publisher.publish_pose(tag_id, t, quat_xyzw)
                last_detect_time = time.time()
                prev_no_tag_sent = False

                # 시각화
                (ptA, ptB, ptC, ptD) = r.corners
                ptA, ptB, ptC, ptD = map(lambda x: tuple(map(int, x)), [ptA, ptB, ptC, ptD])
                cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
                cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
                cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
                cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
                cX, cY = int(r.center[0]), int(r.center[1])
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(frame, str(r.tag_id), (ptA[0], ptA[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            # 수정됨: 일정 시간 이상 태그가 감지 안 되면 빈 메시지 단 한 번 퍼블리시
            now = time.time()
            if last_detect_time is not None and (now - last_detect_time) > pose_publisher.timeout_sec and not prev_no_tag_sent:
                pose_publisher.publish_no_tag()
                prev_no_tag_sent = True # 빈메세지 한번 퍼블리시함 

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

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

def main():
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    main()
