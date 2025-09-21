import sys
import time
import math
import numpy as np

from xarm_wrapper import XArmWrapper
from franka_wrapper import FrankaWrapper
from robotiq_wrapper import RobotiqWrapper


if __name__ == '__main__':
    franka = FrankaWrapper(joints_init=(0.0845, -0.5603, -0.1064, -2.0000, -0.0475, 1.4359, 0))
    xarm = XArmWrapper(joints_init=[3.3, -14.1, -99.8, 1.3, 113.5, 4.3])
    gripper_franka = RobotiqWrapper(robot='franka')
    gripper_xarm = RobotiqWrapper(robot='xarm')
    exit()
    
    while True:
        franka_pose = franka.get_tcp_pose()
        xarm_pose = xarm.get_position()
        for i in range(30):
            start_time = time.time()
            franka_pose[2] += 0.005
            xarm_pose[2] += 5

            franka.franka.schedule_waypoint(franka_pose, time.time() + 1 / 30)
            xarm.xarm.set_servo_cartesian(xarm_pose)
            if i >= 10:
                gripper_franka.close()
                gripper_xarm.close()
            else:
                gripper_franka.open()
                gripper_xarm.open()
            while time.time() - start_time < 1 / 30:
                time.sleep(1 / 1000)

        franka.reset()
        xarm.reset()
        time.sleep(2)
