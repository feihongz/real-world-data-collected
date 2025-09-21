import time
import zerorpc
import numpy as np
from franka_wrapper import FrankaWrapper


if __name__ == "__main__":
    franka = FrankaWrapper(joints_init=(0.0160, -0.8264, -0.1298, -2.6534, -0.0669, 1.8296, -0.0200))
    print("********** Franka is initialized **********")

    time.sleep(3)
    while True:
        pose = franka.franka.get_state()['ActualTCPPose']
        for _ in range(30):
            pose[2] += 0.005
            # pose[1] += 0.01
            # franka.franka.servoL(pose, 1 / 30)
            franka.franka.schedule_waypoint(pose, time.time() + 1 / 30)
            time.sleep(1 / 30)
        # for _ in range(15):
        #     pose[2] -= 0.01
        #     # franka.franka.servoL(pose, 1 / 30)
        #     franka.franka.schedule_waypoint(pose, time.time() + 1 / 30)
        #     time.sleep(1 / 30)
        # for _ in range(15):
        #     pose[2] += 0.01
        #     # franka.franka.servoL(pose, 1 / 30)
        #     franka.franka.schedule_waypoint(pose, time.time() + 1 / 30)
        #     time.sleep(1 / 30)
        # for i in range(5):
        #     for _ in range(30):
        #         pose[1] -= 0.01
        #         franka.franka.servoL(pose, 1 / 60)
        #         # franka.franka.schedule_waypoint(pose, time.time() + 1 / 60)
        #         time.sleep(1 / 60)
        #     print('------------------------------------------------------')
        #     for _ in range(30):
        #         pose[1] += 0.01
        #         franka.franka.servoL(pose, 1 / 60)
        #         # franka.franka.schedule_waypoint(pose, time.time() + 1 / 60)
        #         time.sleep(1 / 60)
        #     print('------------------------------------------------------')
        franka.franka.reset_home()
        time.sleep(3)
        print("********** Franka is reset to home position **********")
    exit()