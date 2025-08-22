import cv2
import numpy as np
from flask import Flask, Response
from pupil_apriltags import Detector

import tf.transformations as tf_trans
from geometry_msgs.msg import Pose

app = Flask(__name__)

cap = cv2.VideoCapture('/dev/jetcocam0')
detector = Detector(families='tag36h11')

camera_matrix = np.array([
    [978.548555, 0.0, 293.112691],
    [0.0, 981.781244, 163.100487],
    [0.0, 0.0, 1.0]
])
dist_coeffs = np.array([-0.481485, 0.586547, -0.000026, 0.005891, 0.0])
tag_size = 0.028

def gen_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 왜곡 보정
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[
                camera_matrix[0, 0],  # fx
                camera_matrix[1, 1],  # fy
                camera_matrix[0, 2],  # cx
                camera_matrix[1, 2]   # cy
            ],
            tag_size=tag_size
        )

        for r in results:
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

            # TF(Translation Vector) 표시
            if hasattr(r, 'pose_t') and r.pose_t is not None:
                t = r.pose_t.flatten()
                tf_text = f"TF: X={t[0]:.2f} Y={t[1]:.2f} Z={t[2]:.2f} m"
                cv2.putText(frame, tf_text, (ptA[0], ptA[1] - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

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
