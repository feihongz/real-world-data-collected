from pyrobotiqgripper import RobotiqGripper

gripper_1 = RobotiqGripper(portname='/dev/ttyUSB0')
gripper_1.reset()
gripper_1.activate()
gripper_1.close()

gripper_2 = RobotiqGripper(portname='/dev/ttyUSB1')
gripper_2.reset()
gripper_2.activate()
# gripper_2.open()
while True:
    import time
    time.sleep(1)