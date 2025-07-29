import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True
print("로봇이 연결되었습니다.")

# # 로봇을 초기 위치로 리셋
# initial_angles = [0, 0, 0, 0, 0, 0]
# speed = 50
# print("로봇을 초기 위치로 리셋합니다.")

# mc.send_angles(initial_angles, speed)
# mc.set_gripper_value(100, speed) # 그리퍼 열기
# time.sleep(3) # 움직임이 완료될 때까지 대기
# print("리셋 완료")

# 현재 각도 읽기
angles = mc.get_angles()
print("현재 각도:", angles) # [0.08, -12.91, 0.52, -41.92, 3.6, -42.18]
# 현재 좌표 읽기
coords = mc.get_coords()
print("현재 좌표:", coords)
# 인코더 값 읽기
encoders = mc.get_encoders()
print("인코더:", encoders)
# 라디안 값 읽기
radians = mc.get_radians()
print("라디안:", radians)

