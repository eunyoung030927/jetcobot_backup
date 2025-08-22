from pymycobot import MyCobot280
import time

mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)

# memory address
POS_PROPORTIONAL_GAIN    = 0x15; #21
POS_DERIVATIVE_GAIN      = 0x16; #22
POS_INTEGRAL_GAIN        = 0x17; #23
data_id = POS_PROPORTIONAL_GAIN
time_sleep = 0.1

mc.set_servo_data(1,data_id,25)
time.sleep(time_sleep)
mc.set_servo_data(2,data_id,42)
time.sleep(time_sleep)
mc.set_servo_data(3,data_id,40)
time.sleep(time_sleep)
mc.set_servo_data(4,data_id,34)
time.sleep(time_sleep)
mc.set_servo_data(5,data_id,32)
time.sleep(time_sleep)
mc.set_servo_data(6,data_id,25)
time.sleep(time_sleep)
mc.set_servo_data(1,data_id,35)

for i in range(1,7):
    time.sleep(time_sleep)
    print(mc.get_servo_data(i,data_id))
