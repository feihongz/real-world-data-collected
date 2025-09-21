from re import T
import time
import viser
import numpy as np
import os
import h5py
from scipy.spatial.transform import Rotation as R
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
import zarr
from realsense import depth2pc, X_root_camera, point_cloud_downsample

def quat_to_rpy(quat):
    """
    将四元数转换为RPY (Roll-Pitch-Yaw)
    这是rpy_to_quat的逆操作
    
    Args:
        quat: 四元数 [x, y, z, w]
    
    Returns:
        rpy: [roll, pitch, yaw] 弧度制
    """
    rot = R.from_quat(quat)
    rpy = rot.as_euler('xyz')
    return rpy
# import pytorch3d.ops as torch3d_ops
import fpsample
from termcolor import cprint
from pathlib import Path
import os
from tqdm import tqdm
from copy import deepcopy
SHOW_REWARD_CURVE = False
SHOW_POINT_CLOUD = False
USE_XY_AGENT_POS = False
from collections import deque

LOAD_FROM_DATA = True
LOAD_COLLECTED = True
ONLY_SUCCESS = True
MAX_EPISODE_LEN = 200
NUM_POINTS = 1024
lambda_penalty = 0.05
smooth_panelty = 0 #0.001
dir_name = 'data_folding_50demo_0807_{}'.format(smooth_panelty)

def compute_return(reward, not_done, gamma: float == 0.99
    ):
        size_ = len(reward)
        return_ = np.zeros((size_, 1))
        pre_return = 0
        for i in tqdm(reversed(range(size_)), desc='Computing the returns'):
            return_[i] = reward[i] + gamma * pre_return * not_done[i]
            pre_return = return_[i]
        return return_


if __name__ == '__main__':
    if not LOAD_FROM_DATA:
        data = {
            'action': list(),
            'reward': list(),
            'is_success': list(),
            'done': list(),
            'agent_pos': list(),
            'point_cloud': list(),
            'timeout': list(),
            'ee_pose': list()
        }

        folder_path = Path('/home/jkzhao/xarm_franka_teleop/data/')
        file_names = sorted([f.name for f in folder_path.iterdir() if f.is_file()])
        for file_name in file_names:
            if not file_name.startswith('demo_'):
                continue
            print('demo id:', file_name)
            demo_path = "/home/jkzhao/xarm_franka_teleop/data/{}".format(str(file_name))
            demo = np.load(demo_path, allow_pickle=True)
            # print(len(demo))
            # continue
            for i in range(len(demo)):
                if i % 100 == 0:
                    print("Frame:", i)
                frame = demo[i]
                t = time.time()
                point_cloud = depth2pc(
                    frame['depth'] * frame['depth_scale'],
                    (228.45703125, 228.734375, 151.828125, 125.9453125),
                    X_root_camera
                )
                # print(time.time() - t)
                reward = int(i == len(demo) - 1)
                if i > 0:
                    reward -= smooth_panelty * np.linalg.norm(frame['action'] - demo[i-1]['action'])
                # if reward == 1:
                #     reward -= lambda_penalty * len(demo) / MAX_EPISODE_LEN
                data['action'].append(frame['action'])
                data['reward'].append(reward)
                data['done'].append(True if i == len(demo) - 1 else False) # TODO: all True, check it next time
                data['timeout'].append(True if i == len(demo) - 1 else False)
                data['agent_pos'].append(frame['qpos'])
                data['ee_pose'].append(frame['eepose'])
                # t = time.time()
                data['point_cloud'].append(point_cloud_downsample(point_cloud, NUM_POINTS))
                # print(time.time() - t)
            
            if SHOW_POINT_CLOUD:
                def on_update(frame_idx):
                    frame = demo[frame_idx]
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
                server = viser.ViserServer(host='127.0.0.1', port=8080)

                slider = server.gui.add_slider(
                    label='frame',
                    min=0,
                    max=len(demo) - 1,
                    step=1,
                    initial_value=0
                )
                slider.on_update(lambda _: on_update(slider.value))
                # while True:
                #     time.sleep(1)
            if SHOW_REWARD_CURVE:
                rewards = data['reward']
                plt.figure(figsize=(10, 5))
                plt.plot(rewards, marker='o', linestyle='-', color='b')  # 使用圆圈标记数据点
                plt.title('Reward Curve')
                plt.xlabel('Episode or Step')
                plt.ylabel('Reward')
                plt.grid()
                plt.show()
        np.save('/home/jkzhao/xarm_franka_teleop/data/{}.npy'.format(dir_name), data)

    else:   
        demo_path = "/home/jkzhao/xarm_franka_teleop/data/{}.npy".format(dir_name)
        demo = np.load(demo_path, allow_pickle=True)
        data = demo.item()
        # import pdb; pdb.set_trace()
        dir_name1 = 'data_folding_300demo_0806_0'
        demo_path1 = "/home/jkzhao/xarm_franka_teleop/_0907_pre/{}.npy".format(dir_name1)
        demo1 = np.load(demo_path1, allow_pickle=True)
        data1 = demo1.item()
        # import pdb; pdb.set_trace()
        # 正确的方式：将两个字典中对应key的列表连接起来
        for k in data.keys():
            if isinstance(data[k], list) and isinstance(data1[k], list):
                data[k].extend(data1[k])
            else:
                # 如果不是列表，则使用原来的加法操作
                print(f"Warning: {k} is not a list, using original addition.")
                data[k] = data[k] + data1[k]
        # # import pdb; pdb.set_trace()

        traj_end_index = np.where(data['timeout'])[0] # get the index of the last step of each episode
        save_dir = os.path.join('/home/jkzhao/xarm_franka_teleop/data', '{}_{}_{}_0807_350.zarr'.format(dir_name, ONLY_SUCCESS, LOAD_COLLECTED))
        if os.path.exists(save_dir):
            cprint('Data already exists at {}'.format(save_dir), 'red')
            cprint("If you want to overwrite, delete the existing directory first.", "red")
            cprint("Do you want to overwrite? (y/n)", "red")
            # user_input = input()
            user_input = 'y'
            if user_input == 'y':
                cprint('Overwriting {}'.format(save_dir), 'red')
                os.system('rm -rf {}'.format(save_dir))
            else:
                cprint('Exiting', 'red')
                exit()
        os.makedirs(save_dir, exist_ok=True)
        # pre process None type reward
        data['reward'] = np.array(data['reward'])

        # 检查 reward 列中为 None 或 NaN 的位置
        # data['reward'] = interpolate_rewards(data['reward'])
        # data['reward'][data['reward'] == 3] = 20

        # data['reward'] = data['reward'] - 1.
        total_count = 0
        img_arrays = []
        point_cloud_arrays = []

        next_img_arrays = []
        next_point_cloud_arrays = []
        next_state_arrays = []
        next_ee_pose_arrays = []
        next_action_arrays = []

        state_arrays = []
        ee_pose_arrays = []
        action_arrays = []
        next_action_arrays = []
        reward_arrays = []
        done_arrays = []
        timeout_arrays = []
        episode_ends_arrays = []
        
        all_total_rewards = []
        # loop over episodes
        episode_id = 0
        img = np.random.rand(3, 84, 84) # DON NOT USE IMAGE
        total_reward = 0.
        total_count_sub = 0
        for total_id, timeout in enumerate(data['timeout']):
            total_reward += data['reward'][total_id]
            img_arrays.append(img)
            point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id]))
            state_arrays.append(deepcopy(data['agent_pos'][total_id]))
            action_arrays.append(deepcopy(data['action'][total_id]))
            reward_arrays.append(deepcopy(data['reward'][total_id]))
            done_arrays.append(deepcopy(data['timeout'][total_id]))
            timeout_arrays.append(deepcopy(data['timeout'][total_id]))
            ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id]))
            total_count_sub += 1
            next_img_arrays.append(img)
            if total_id == len(data['timeout']) - 1:
                next_point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id]))
                next_state_arrays.append(deepcopy(data['agent_pos'][total_id]))
                next_action_arrays.append(deepcopy(data['action'][total_id]))
                next_ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id]))
            else:
                next_point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id+1]))
                next_state_arrays.append(deepcopy(data['agent_pos'][total_id+1]))
                next_action_arrays.append(deepcopy(data['action'][total_id+1]))
                next_ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id+1]))
            if timeout:
                episode_id += 1
                total_count += total_count_sub
                print('Episode: {}, Episode length: {}, Return: {}'.format(episode_id, total_count_sub, total_reward))

                episode_ends_arrays.append(deepcopy(total_count))
                all_total_rewards.append(deepcopy(total_reward))
                # reset sub arrays
                total_reward = 0.
                total_count_sub = 0
                #-----------------------
        if False: #LOAD_COLLECTED:
            traj_path = '/home/jkzhao/xarm_franka_teleop/data_0805_policy'  # 替换成你自己的路径
            # total_count = 0
            episode_id = 0
            # demo = np.load('./offline_data1/demo.npy', allow_pickle=True)
            # 遍历所有 HDF5 文件
            for filename in sorted(os.listdir(traj_path)):
                if not filename.endswith('.h5'):
                    continue

                file_path = os.path.join(traj_path, filename)
                
                with h5py.File(file_path, 'r') as f:
                    data = {key: f[key][()] for key in f.keys()}
                
                # 检查当前 trajectory 的最后一步的 is_success
                is_success = data['is_success'].any() # 取最后一个时间步的 is_success
                # 如果 is_success 为 False，跳过该 trajectory
                if True and not is_success:
                    continue
                if is_success:
                    data['reward'][-1] = 1
                print(f"读取文件: {file_path}")
                img = np.random.rand(3, 84, 84)  # dummy image
                total_reward = 0.
                total_count_sub = 0

                for total_id, timeout in enumerate(data['done']):
                    # if total_id > 0:
                    #     data['reward'][total_id] -= smooth_panelty * np.linalg.norm(data['action'][total_id] - data['action'][total_id-1])
                        
                    total_reward += data['reward'][total_id]
                    img_arrays.append(img)
                    point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id]))
                    state_arrays.append(deepcopy(data['state'][total_id]))  # 或者 'agent_pos' 如果你原来存的是那个字段
                    # ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id]))
                    action_arrays.append(deepcopy(data['action'][total_id].squeeze()))
                    next_action_arrays.append(deepcopy(data['next_action'][total_id].squeeze()))
                    reward_arrays.append(deepcopy(data['reward'][total_id]))
                    done_arrays.append(deepcopy(data['done'][total_id]))
                    timeout_arrays.append(deepcopy(data['timeout'][total_id]))
                    # import pdb; pdb.set_trace()
                    total_count_sub += 1

                    next_img_arrays.append(img)
                    if total_id == len(data['timeout']) - 1:
                        next_point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id]))
                        next_state_arrays.append(deepcopy(data['state'][total_id]))
                        # next_ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id]))
                        # next_action_arrays.append(deepcopy(data['action'][total_id]))
                    else:
                        next_point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id + 1]))
                        next_state_arrays.append(deepcopy(data['state'][total_id + 1]))
                        # next_ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id + 1]))
                    if timeout:
                        episode_id += 1
                        total_count += total_count_sub
                        print('Episode: {}, Episode length: {}, Return: {:.2f}'.format(episode_id, total_count_sub, total_reward))

                        episode_ends_arrays.append(deepcopy(total_count))
                        all_total_rewards.append(deepcopy(total_reward))

                        total_reward = 0.
                        total_count_sub = 0
            
            print('\n所有文件处理完成 data,总 episode 数:', total_count)
        if LOAD_COLLECTED:
            traj_path = '/home/jkzhao/xarm_franka_teleop/data_0805_policy'  # 替换成你自己的路径
            # total_count = 0
            episode_id = 0
            # demo = np.load('./offline_data1/demo.npy', allow_pickle=True)
            # 遍历所有 HDF5 文件
            for filename in sorted(os.listdir(traj_path)):
                if not filename.endswith('.h5'):
                    continue

                file_path = os.path.join(traj_path, filename)
                
                with h5py.File(file_path, 'r') as f:
                    data = {key: f[key][()] for key in f.keys()}
                
                # 检查当前 trajectory 的最后一步的 is_success
                is_success = data['is_success'].any() # 取最后一个时间步的 is_success
                # 如果 is_success 为 False，跳过该 trajectory
                if ONLY_SUCCESS and not is_success:
                    continue
                if is_success:
                    data['reward'][-1] = 1
                print(f"读取文件: {file_path}")
                img = np.random.rand(3, 84, 84)  # dummy image
                total_reward = 0.
                total_count_sub = 0

                for total_id, timeout in enumerate(data['done']):
                    # if total_id > 0:
                    #     data['reward'][total_id] -= smooth_panelty * np.linalg.norm(data['action'][total_id] - data['action'][total_id-1])
                        
                    total_reward += data['reward'][total_id]

                    img_arrays.append(img)
                    point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id]))
                    state_arrays.append(deepcopy(data['state'][total_id]))  # 或者 'agent_pos' 如果你原来存的是那个字段
                    # ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id]))
                    # 确保action数据形状一致并转换四元数到RPY
                    action_data = data['action'][total_id]
                    if action_data.ndim > 1:
                        action_data = action_data.squeeze()
                    
                    # 转换四元数到RPY (当action是16维时，说明包含四元数)
                    if len(action_data) == 16:
                        # 假设action结构: [xarm_pos(3), xarm_quat(4), xarm_gripper(1), franka_pos(3), franka_quat(4), franka_gripper(1)]
                        xarm_quat = action_data[3:7]  # 四元数 [x,y,z,w]
                        franka_quat = action_data[11:15]  # 四元数 [x,y,z,w]
                        
                        # 转换为RPY
                        xarm_rpy = quat_to_rpy(xarm_quat)
                        franka_rpy = quat_to_rpy(franka_quat)
                        
                        # 重新构建action数据为14维: [xarm_pos(3), xarm_rpy(3), xarm_gripper(1), franka_pos(3), franka_rpy(3), franka_gripper(1)]
                        new_action_data = np.zeros(14)
                        new_action_data[0:3] = action_data[0:3]  # XArm位置
                        new_action_data[3:6] = xarm_rpy  # XArm旋转 (RPY)
                        new_action_data[6] = action_data[7]  # XArm夹爪
                        new_action_data[7:10] = action_data[8:11]  # Franka位置
                        new_action_data[10:13] = franka_rpy  # Franka旋转 (RPY)
                        new_action_data[13] = action_data[15]  # Franka夹爪
                        
                        action_data = new_action_data
                    
                    action_arrays.append(deepcopy(action_data))
                    
                    next_action_data = data['next_action'][total_id]
                    if next_action_data.ndim > 1:
                        next_action_data = next_action_data.squeeze()
                    
                    # 同样转换next_action
                    if len(next_action_data) == 16:
                        # 假设next_action结构: [xarm_pos(3), xarm_quat(4), xarm_gripper(1), franka_pos(3), franka_quat(4), franka_gripper(1)]
                        xarm_quat = next_action_data[3:7]  # 四元数 [x,y,z,w]
                        franka_quat = next_action_data[11:15]  # 四元数 [x,y,z,w]
                        
                        # 转换为RPY
                        xarm_rpy = quat_to_rpy(xarm_quat)
                        franka_rpy = quat_to_rpy(franka_quat)
                        
                        # 重新构建next_action数据为14维: [xarm_pos(3), xarm_rpy(3), xarm_gripper(1), franka_pos(3), franka_rpy(3), franka_gripper(1)]
                        new_next_action_data = np.zeros(14)
                        new_next_action_data[0:3] = next_action_data[0:3]  # XArm位置
                        new_next_action_data[3:6] = xarm_rpy  # XArm旋转 (RPY)
                        new_next_action_data[6] = next_action_data[7]  # XArm夹爪
                        new_next_action_data[7:10] = next_action_data[8:11]  # Franka位置
                        new_next_action_data[10:13] = franka_rpy  # Franka旋转 (RPY)
                        new_next_action_data[13] = next_action_data[15]  # Franka夹爪
                        
                        next_action_data = new_next_action_data
                    
                    next_action_arrays.append(deepcopy(next_action_data))
                    reward_arrays.append(deepcopy(data['reward'][total_id]))
                    done_arrays.append(deepcopy(data['done'][total_id]))
                    timeout_arrays.append(deepcopy(data['timeout'][total_id]))
                    # import pdb; pdb.set_trace()
                    total_count_sub += 1

                    next_img_arrays.append(img)
                    if total_id == len(data['timeout']) - 1:
                        next_point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id]))
                        next_state_arrays.append(deepcopy(data['state'][total_id]))
                        # next_ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id]))
                        # next_action_arrays.append(deepcopy(data['action'][total_id]))
                    else:
                        next_point_cloud_arrays.append(deepcopy(data['point_cloud'][total_id + 1]))
                        next_state_arrays.append(deepcopy(data['state'][total_id + 1]))
                        # next_ee_pose_arrays.append(deepcopy(data['ee_pose'][total_id + 1]))
                    if timeout:
                        episode_id += 1
                        total_count += total_count_sub
                        print('Episode: {}, Episode length: {}, Return: {:.2f}'.format(episode_id, total_count_sub, total_reward))

                        episode_ends_arrays.append(deepcopy(total_count))
                        all_total_rewards.append(deepcopy(total_reward))

                        total_reward = 0.
                        total_count_sub = 0
            
            print('\n所有文件处理完成 data,总 episode 数:', total_count)
        print('Mean reward: {}'.format(np.mean(all_total_rewards[:])))
        # import pdb; pdb.set_trace()
        # tracemalloc.stop()
        ###############################
        # save data
        ###############################
        # create zarr file
        zarr_root = zarr.group(save_dir)
        zarr_data = zarr_root.create_group('data')
        zarr_meta = zarr_root.create_group('meta')
        # save img, state, action arrays into data, and episode ends arrays into meta
        # img_arrays = np.stack(img_arrays, axis=0)
        # next_img_arrays = np.stack(next_img_arrays, axis=0)
        # if img_arrays.shape[1] == 3: # make channel last
        #     img_arrays = np.transpose(img_arrays, (0,2,3,1))
            # next_img_arrays = np.transpose(next_img_arrays, (0,2,3,1))
        # 调试：检查action_arrays的形状
        print(f"[DEBUG] action_arrays长度: {len(action_arrays)}")
        
        # 检查所有action的形状
        shapes = set()
        problem_indices = []
        for i, action in enumerate(action_arrays):
            shape = action.shape
            shapes.add(shape)
            if shape != (14,):
                problem_indices.append(i)
                if len(problem_indices) <= 5:  # 只记录前5个问题
                    print(f"[DEBUG] 问题action_arrays[{i}] shape: {shape}, dtype: {action.dtype}")
        
        print(f"[DEBUG] 所有形状: {shapes}")
        print(f"[DEBUG] 问题索引数量: {len(problem_indices)}")
        if problem_indices:
            print(f"[DEBUG] 前5个问题索引: {problem_indices[:5]}")
        
        # 打印前10个正常形状的action
        for i, action in enumerate(action_arrays[:10]):
            print(f"[DEBUG] action_arrays[{i}] shape: {action.shape}, dtype: {action.dtype}")
        if len(action_arrays) > 10:
            print(f"[DEBUG] ... 还有 {len(action_arrays) - 10} 个action")
        
        # 打印最后5个action的形状
        print(f"[DEBUG] 最后5个action的形状:")
        for i in range(max(0, len(action_arrays) - 5), len(action_arrays)):
            print(f"[DEBUG] action_arrays[{i}] shape: {action_arrays[i].shape}, dtype: {action_arrays[i].dtype}")
        
        state_arrays = np.stack(state_arrays, axis=0)
        ee_pose_arrays = np.stack(ee_pose_arrays, axis=0)
        point_cloud_arrays = np.stack(point_cloud_arrays, axis=0)
        action_arrays = np.stack(action_arrays, axis=0)

        next_state_arrays = np.stack(next_state_arrays, axis=0)
        next_ee_pose_arrays = np.stack(next_ee_pose_arrays, axis=0)
        next_point_cloud_arrays = np.stack(next_point_cloud_arrays, axis=0)
        next_action_arrays = np.stack(next_action_arrays, axis=0)
        
        reward_arrays = np.array(reward_arrays).reshape(action_arrays.shape[0], -1)
        done_arrays = np.array(done_arrays).reshape(action_arrays.shape[0], -1)
        timeout_arrays = np.array(timeout_arrays).reshape(action_arrays.shape[0], -1)
        # done_arrays = deepcopy(timeout_arrays) # TODO: issues in done arrays, replace it with timeout arrays temporarily
        episode_ends_arrays = np.array(episode_ends_arrays)
        # print(done_arrays)



        compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)
        # img_chunk_size = (100, img_arrays.shape[1], img_arrays.shape[2], img_arrays.shape[3])
        state_chunk_size = (100, state_arrays.shape[1])
        ee_pose_chunk_size = (100, ee_pose_arrays.shape[1])
        point_cloud_chunk_size = (100, point_cloud_arrays.shape[1], point_cloud_arrays.shape[2])
        action_chunk_size = (100, action_arrays.shape[1])
        reward_chunk_size = (100, reward_arrays.shape[1])

        done_chunk_size = (100, done_arrays.shape[1])
        timeout_chunk_size = (100, timeout_arrays.shape[1])
        # compute return for each episode
        not_done_arrays =  1. - (done_arrays | timeout_arrays)
        done_timeout_arrays = done_arrays | timeout_arrays  
        done_indices = np.where(done_timeout_arrays.flatten())[0]
        return_arrays = compute_return(reward_arrays, not_done_arrays, 0.99)
        return_chunk_size = (100, return_arrays.shape[1])
        # import pdb
        # pdb.set_trace()
        
        # import pdb
        # pdb.set_trace()
        # zarr_data.create_dataset('img', data=img_arrays, chunks=img_chunk_size, dtype='uint8', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('state', data=state_arrays, chunks=state_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('ee_pose', data=ee_pose_arrays, chunks=ee_pose_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('point_cloud', data=point_cloud_arrays, chunks=point_cloud_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('action', data=action_arrays, chunks=action_chunk_size, dtype='float32', overwrite=True, compressor=compressor)

        # zarr_data.create_dataset('next_img', data=next_img_arrays, chunks=img_chunk_size, dtype='uint8', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('next_state', data=next_state_arrays, chunks=state_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('next_ee_pose', data=next_ee_pose_arrays, chunks=ee_pose_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('next_point_cloud', data=next_point_cloud_arrays, chunks=point_cloud_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('next_action', data=next_action_arrays, chunks=action_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('return', data=return_arrays, chunks=timeout_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('reward', data=reward_arrays, chunks=reward_chunk_size, dtype='float32', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('done', data=done_arrays, chunks=done_chunk_size, dtype='bool', overwrite=True, compressor=compressor)
        zarr_data.create_dataset('timeout', data=timeout_arrays, chunks=timeout_chunk_size, dtype='bool', overwrite=True, compressor=compressor)

        zarr_meta.create_dataset('episode_ends', data=episode_ends_arrays, dtype='int64', overwrite=True, compressor=compressor)
        
        
        # print shape
        # cprint(f'img shape: {img_arrays.shape}, range: [{np.min(img_arrays)}, {np.max(img_arrays)}]', 'green')
        cprint(f'point_cloud shape: {point_cloud_arrays.shape}, range: [{np.min(point_cloud_arrays)}, {np.max(point_cloud_arrays)}]', 'green')
        cprint(f'state shape: {state_arrays.shape}, range: [{np.min(state_arrays)}, {np.max(state_arrays)}]', 'green')
        cprint(f'ee_pose shape: {ee_pose_arrays.shape}, range: [{np.min(ee_pose_arrays)}, {np.max(ee_pose_arrays)}]', 'green')
        cprint(f'action shape: {action_arrays.shape}, range: [{np.min(action_arrays)}, {np.max(action_arrays)}]', 'green')

        # cprint(f'next_img shape: {next_img_arrays.shape}, range: [{np.min(next_img_arrays)}, {np.max(next_img_arrays)}]', 'green')
        cprint(f'next_point_cloud shape: {next_point_cloud_arrays.shape}, range: [{np.min(next_point_cloud_arrays)}, {np.max(next_point_cloud_arrays)}]', 'green')
        cprint(f'next_state shape: {next_state_arrays.shape}, range: [{np.min(next_state_arrays)}, {np.max(next_state_arrays)}]', 'green')
        cprint(f'next_ee_pose shape: {next_ee_pose_arrays.shape}, range: [{np.min(next_ee_pose_arrays)}, {np.max(next_ee_pose_arrays)}]', 'green')
        cprint(f'next_action shape: {next_action_arrays.shape}, range: [{np.min(next_action_arrays)}, {np.max(next_action_arrays)}]', 'green')
        
        cprint(f'reward shape: {reward_arrays.shape}, range: [{np.min(reward_arrays)}, {np.max(reward_arrays)}]', 'green')
        cprint(f'done shape: {done_arrays.shape}, range: [{np.min(done_arrays)}, {np.max(done_arrays)}]', 'green')
        cprint(f'timeout shape: {timeout_arrays.shape}, range: [{np.min(timeout_arrays)}, {np.max(timeout_arrays)}]', 'green')
        cprint(f'return shape: {return_arrays.shape}, range: [{np.min(return_arrays)}, {np.max(return_arrays)}]', 'green')
        cprint(f'Saved zarr file to {save_dir}', 'green')
        
        cprint(f'Saved zarr file to {save_dir}', 'green')
        
        # clean up
        del img_arrays, state_arrays, point_cloud_arrays, action_arrays, reward_arrays, done_arrays, timeout_arrays, episode_ends_arrays
        del zarr_root, zarr_data, zarr_meta
        # del expert_agent