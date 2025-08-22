import cv2
import datetime
import os
import threading
from flask import Flask, Response, render_template_string

os.environ['PYDEVD_DISABLE_FILE_VALIDATION'] = '1'

# Flask 애플리케이션 초기화
app = Flask(__name__)

# 카메라 장치 열기
cap = cv2.VideoCapture('/dev/jetcocam0')

# 캡처 이미지를 저장할 디렉터리가 없으면 생성
if not os.path.exists('./checkerboards'):
    os.makedirs('./checkerboards')
    print("checkerboards 디렉터리를 생성했습니다.")

capture_flag = False     # 캡처할지 여부 (키 입력으로 제어)
exit_flag = False        # 프로그램 종료 여부 (키 입력으로 제어)

# 비디오 스트리밍을 위한 제너레이터 함수
def generate_frames():
    global capture_flag, exit_flag
    while not exit_flag:
        ret, frame = cap.read()
        if not ret:
            print("카메라에서 프레임을 가져올 수 없습니다.")
            break

        # 키 입력 체크 (a-캡처, q-종료)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('a'):
            capture_flag = True
        elif key == ord('q'):
            exit_flag = True
            break

        if capture_flag:
            filename = datetime.datetime.now().strftime("./checkerboards/capture_%Y%m%d_%H%M%S.png")
            cv2.imwrite(filename, frame)
            print(f"{filename} 이미지가 저장되었습니다.")
            capture_flag = False

        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    cap.release()
    cv2.destroyAllWindows()

# 웹 페이지의 루트 경로
@app.route('/')
def index():
    # 간단한 웹페이지(템플릿파일 없이)
    return render_template_string("""
    <html>
      <head>
        <title>카메라 캡처</title>
      </head>
      <body>
        <h1>웹 카메라 캡처</h1>
        <img src="{{ url_for('video_feed') }}" width="640" height="480">
        <br><p>로컬창에서 a=캡처, q=종료</p>
      </body>
    </html>
    """)

# 비디오 스트림을 제공하는 경로
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask 앱을 별도 쓰레드에서 실행
def flask_thread():
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)

if __name__ == '__main__':
    t = threading.Thread(target=flask_thread)
    t.daemon = True
    t.start()

    print("플라스크 서버가 실행되었습니다. 창에서 실시간 스트림 확인, 터미널 포커스에서 'a', 'q'키로 제어하세요.")
    while not exit_flag:
        # waitKey(1) 루프는 generate_frames에서 처리, 수동 처리 불필요.
        pass

    print("종료합니다.")
