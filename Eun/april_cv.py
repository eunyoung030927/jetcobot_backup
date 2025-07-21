import cv2
from flask import Flask, Response
from pupil_apriltags import Detector

app = Flask(__name__)

cap = cv2.VideoCapture('/dev/jetcocam0')
detector = Detector(families='tag36h11')

def gen_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

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

        # 프레임을 JPEG로 인코딩
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # 스트림 전송
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
