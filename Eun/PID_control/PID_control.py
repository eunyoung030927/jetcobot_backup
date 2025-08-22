# class PIDController:
#     def __init__(self, kp, ki, kd, setpoint):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.setpoint = setpoint
#         self.previous_error = 0
#         self.integral = 0

#     def compute(self, measurement, dt):
#         error = self.setpoint - measurement
#         self.integral += error * dt
#         derivative = (error - self.previous_error) / dt

#         output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
#         self.previous_error = error
#         return output

# # PID 제어기 초기화
# pid = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=10.0)

# # 제어 값 계산
# measurement = 8.0  # 현재 측정 값
# dt = 0.1  # 샘플링 시간
# control_signal = pid.compute(measurement, dt)
# print(f"Control Signal: {control_signal}")

import time
import matplotlib.pyplot as plt
from pymycobot import MyCobot280  # 또는 사용중인 모델에 따라 수정

mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)

# 1번 서보를 90도로 보낸 상황에서 오차 기록 예제
setpoint = 90  # 목표 각도

errors = []
timestamps = []

mc.send_angles([90, 0, 0, 0, 0, 0], 10)  # 1번 서보만 90도, 나머지는 기본값
start_time = time.time()

for i in range(40):  # 약 4초 동안 기록 (0.1초 간격)
    angles = mc.get_angles()
    current = angles[0]  # 1번 서보
    error = setpoint - current

    errors.append(error)
    timestamps.append(time.time() - start_time)

    print(f"t={timestamps[-1]:.2f}s, error={error:.2f}")

    time.sleep(0.1)

# 그래프 그리기
plt.plot(timestamps, errors, label='오차(Setpoint - 현재)')
plt.axhline(0, color='r', linestyle='--', label='Zero Error')
plt.xlabel('Time (s)')
plt.ylabel('Error (degrees)')
plt.title('PID 제어 오차 변화 추이')
plt.legend()
plt.grid(True)
plt.show()
