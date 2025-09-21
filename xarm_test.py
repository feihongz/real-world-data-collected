import os
import sys
import time
import math

from xarm.wrapper import XArmAPI



arm = XArmAPI('192.168.1.225')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(0)
arm.set_servo_angle(angle=[90, -25.7, -76.9, 1.6, 102.8, 0.3], speed=32, wait=True)
arm.set_mode(1)
arm.set_state(state=0)


x, y, z, roll, pitch, yaw = arm.get_position()[1]
while True:
    print(arm.get_position()[1])
    for _ in range(30):
        arm.set_servo_cartesian([x, y, z, roll, pitch, yaw])
        time.sleep(1 / 30)
        z += 5
    for _ in range(10):
        arm.set_servo_cartesian([x, y, z, roll, pitch, yaw])
        time.sleep(1 / 30)
        arm.set_servo_cartesian([x, y, z + 5, roll, pitch, yaw])
        time.sleep(1 / 30)
        arm.set_servo_cartesian([x, y, z + 10, roll, pitch, yaw])
        time.sleep(1 / 30)
        z += 15
    break

arm.disconnect()