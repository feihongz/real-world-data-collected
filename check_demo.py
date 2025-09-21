import os
import h5py
import cv2
import numpy as np
import time
import viser
from xarm_wrapper import XArmWrapper
from robotiq_wrapper import RobotiqWrapper
from franka_wrapper import FrankaWrapper

xarm = XArmWrapper(joints_init=[3.3, -14.1, -99.8, 1.3, 113.5, 4.3])
xarm_gripper = RobotiqWrapper(robot='xarm')
franka = FrankaWrapper(joints_init=(0.0845, -0.5603, -0.1064, -2.0000, -0.0475, 1.4359, 0.0))
franka_gripper = RobotiqWrapper(robot='franka')

traj_path = './data_offline_test'  # 替换成你自己的路径
episode_id = 0
for filename in sorted(os.listdir(traj_path)):
    print("###################################################################")
    if not filename.endswith('.h5'):
        continue

    file_path = os.path.join(traj_path, filename)
    print(f"读取文件: {file_path}")
    user_input = input("Check this demo? (y/n): ")
    if user_input == 'n':
        continue
    
    with h5py.File(file_path, 'r') as f:
        data = {key: f[key][()] for key in f.keys()}
    print(data.keys())
    print('point_cloud shape: ', data['point_cloud'].shape)
    print('action shape: ', data['action'].shape)

    xarm_gripper.open()
    franka_gripper.open()
    xarm.reset()
    franka.reset()
    time.sleep(3)
    for frame_idx in range(data['point_cloud'].shape[0]):
        action = data['action'][frame_idx]
        xarm_target = action[:6]
        xarm_target[:3] *= 1000
        xarm_target[3:] = xarm_target[3:] / np.pi * 180
        xarm_gripper_target = action[6]
        franka_target = action[7:13]
        franke_gripper_target = action[13]

        xarm.set_servo_cartesian(xarm_target)
        if xarm_gripper_target > 0.5:
            xarm_gripper.close()
        else:
            xarm_gripper.open()
        franka.franka.servoL(franka_target, 1 / 20)
        if franke_gripper_target > 0.5:
            franka_gripper.close()
        else:
            franka_gripper.open()
        time.sleep(1 / 20)


    def on_update(frame_idx):
        point_cloud = data['point_cloud'][frame_idx]
        value = point_cloud[:, 1] - point_cloud[:, 0]
        depth_norm = cv2.normalize(value, None, 0, 255, cv2.NORM_MINMAX)
        depth_8bit = np.uint8(depth_norm)
        colormap = np.array(cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET))[:, 0]
        # print(colormap.shape)

        server.scene.add_point_cloud(
            'point_cloud',
            point_cloud,
            point_size=0.005,
            point_shape="circle",
            colors=colormap
        )
        # server.scene.add_point_cloud(
        #     'point_cloud_next',
        #     data['next_point_cloud'][frame_idx],
        #     point_size=0.0025,
        #     point_shape="circle",
        #     colors=(255, 0, 0)
        # )
    server = viser.ViserServer(host='127.0.0.1', port=8080)

    slider = server.gui.add_slider(
        label='frame',
        min=0,
        max=data['point_cloud'].shape[0] - 1,
        step=1,
        initial_value=0
    )
    slider.on_update(lambda _: on_update(slider.value))
    