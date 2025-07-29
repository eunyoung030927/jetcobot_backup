import os
import time
import threading
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle, Coord

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True
print("로봇이 연결되었습니다.")

# # 현재 각도 읽기
# angles = mc.get_angles()
# print("현재 각도:", angles)
# # 현재 좌표 읽기
# coords = mc.get_coords()
# print("현재 좌표:", coords)
# # 인코더 값 읽기
# encoders = mc.get_encoders()
# print("인코더:", encoders)
# # 라디안 값 읽기
# radians = mc.get_radians()
# print("라디안:", radians)


# 그리퍼 완전히 열기
print("그리퍼를 완전히 엽니다.")
mc.set_gripper_value(100, 50)
time.sleep(1)
# # 그리퍼 반쯤 닫기
# print("그리퍼를 반쯤 닫습니다.")
# mc.set_gripper_value(50, 50)
# time.sleep(1)
# # 그리퍼 더 닫기
# print("그리퍼를 더 닫습니다.")
# mc.set_gripper_value(30, 50)
# time.sleep(1)
# # 그리퍼 완전히 닫기
# print("그리퍼를 완전히 닫습니다.")
# mc.set_gripper_value(0, 50)
# time.sleep(1)
# # 그리퍼 다시 열기
# print("그리퍼를 다시 엽니다.")
# mc.set_gripper_value(100, 50)
# time.sleep(1)