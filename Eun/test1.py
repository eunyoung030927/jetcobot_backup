import cv2
import numpy as np
from pupil_apriltags import Detector

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

cap = cv2.VideoCapture('/dev/jetcocam0')

if not cap.isOpened():
    print("오류: 카메라를 열 수 없습니다.")
    exit()

KNOWN_TAG_SIZE = 0.16
FOCAL_LENGTH = 800

def estimate_distance(pixel_width, known_width, focal_length):
    if pixel_width == 0:
        return 0
    return (known_width * focal_length) / pixel_width

print("카메라 피드를 시작합니다. 'q'를 눌러 종료하세요.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("오류: 프레임을 읽을 수 없습니다.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray, estimate_tag_pose=False, camera_params=None, tag_size=None)

    for tag in tags:
        (ptA, ptB, ptC, ptD) = tag.corners
        ptA = (int(ptA[0]), int(ptA[1]))
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))

        cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
        cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
        cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
        cv2.line(frame, ptD, ptA, (0, 255, 0), 2)

        (cX, cY) = (int(tag.center[0]), int(tag.center[1]))

        pixel_width = np.linalg.norm(np.array(ptA) - np.array(ptB))
        distance = estimate_distance(pixel_width, KNOWN_TAG_SIZE, FOCAL_LENGTH)

        tag_id = tag.tag_id
        info_text_id = f"ID: {tag_id}"
        info_text_coords = f"XY: ({cX}, {cY})"
        info_text_dist = f"Z: {distance:.2f} m"

        info_y_offset = cY - 15
        cv2.putText(frame, info_text_id, (cX + 10, info_y_offset - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, info_text_coords, (cX + 10, info_y_offset - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, info_text_dist, (cX + 10, info_y_offset),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        axis_length = 50
        center_point = (cX, cY)

        vec_x = np.array(ptB) - np.array(ptA)
        norm_vec_x = vec_x / (np.linalg.norm(vec_x) + 1e-6)
        end_point_x = tuple((np.array(center_point) + norm_vec_x * axis_length).astype(int))
        cv2.arrowedLine(frame, center_point, end_point_x, (255, 0, 0), 3)
        cv2.putText(frame, 'X', (end_point_x[0] + 5, end_point_x[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        vec_y = np.array(ptD) - np.array(ptA)
        norm_vec_y = vec_y / (np.linalg.norm(vec_y) + 1e-6)
        end_point_y = tuple((np.array(center_point) + norm_vec_y * axis_length).astype(int))
        cv2.arrowedLine(frame, center_point, end_point_y, (0, 0, 255), 3)
        cv2.putText(frame, 'Y', (end_point_y[0], end_point_y[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("AprilTag Detector", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

print("스크립트를 종료합니다.")
