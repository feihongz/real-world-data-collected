#!/usr/bin/env python3
"""
XArm环境包装器，用于数据采集
适配自DemoGen的数据采集格式
"""

import numpy as np
import cv2
import time
import threading
from gym import spaces
from scipy.spatial.transform import Rotation as R

from xarm_wrapper import XArmWrapper
from xarm_gripper_wrapper import XArmGripperWrapper

# 只使用USB相机的RGB信号

class XArmEnv:
    """XArm环境，兼容数据采集格式"""
    
    def __init__(self, 
                 xarm_ip='10.3.32.200',
                 camera_index=0,
                 use_camera=True,
                 safe_mode=False):
        
        self.arm_action_dim = 6  # x, y, z, roll, pitch, yaw
        self.gripper_action_dim = 1  # gripper position
        self.use_camera = use_camera
        self.safe_mode = safe_mode
        
        # 动作空间定义
        self.action_space = spaces.Box(
            low=-1,
            high=1,
            shape=(self.arm_action_dim + self.gripper_action_dim,),
            dtype=np.float32
        )
        
        # 观测空间定义
        obs_spaces = {
            'agent_pos': spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(7,),  # x,y,z,roll,pitch,yaw,gripper
                dtype=np.float32
            )
        }
        
        if self.use_camera:
            obs_spaces.update({
                'image': spaces.Box(
                    low=0,
                    high=255,
                    shape=(480, 640, 3),  # RGB format
                    dtype=np.uint8
                )
            })
        
        self.observation_space = spaces.Dict(obs_spaces)
        
        # 初始化机械臂
        if not self.safe_mode:
            print("[XArmEnv] 初始化XArm...")
            self.xarm = XArmWrapper(
                joints_init=None, 
                ip=xarm_ip, 
                move_to_init=False
            )
            self.gripper = XArmGripperWrapper(self.xarm.xarm)
        else:
            print("[XArmEnv] 安全模式：不连接真实机械臂")
            self.xarm = None
            self.gripper = None
        
        # 初始化相机
        self.camera = None
        self.camera_index = camera_index
        
        if self.use_camera:
            self._init_camera()
        
        # 记录当前状态
        self._last_robot_state = None
        self._update_robot_state()
        
        print("[XArmEnv] XArm环境初始化完成")
    
    def _init_camera(self):
        """初始化相机"""
        self._init_usb_camera()  # 只使用USB相机的RGB信号
    

    
    def _init_usb_camera(self):
        """初始化USB相机"""
        try:
            print(f"[XArmEnv] 初始化USB相机 (索引: {self.camera_index})...")
            self.camera = cv2.VideoCapture(self.camera_index)
            
            if not self.camera.isOpened():
                raise Exception(f"无法打开相机索引 {self.camera_index}")
            
            # 设置分辨率
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # 测试获取一帧
            ret, frame = self.camera.read()
            if not ret:
                raise Exception("无法获取相机帧")
            
            print("[XArmEnv] USB相机初始化成功")
            
        except Exception as e:
            print(f"[XArmEnv] USB相机初始化失败: {e}")
            self.use_camera = False
            self.camera = None
    
    def _update_robot_state(self):
        """更新机械臂状态"""
        if self.safe_mode:
            # 安全模式：使用模拟状态
            if self._last_robot_state is None:
                self._last_robot_state = np.array([0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 0.5], dtype=np.float32)
            return
            
        try:
            # 获取当前位置 (mm -> m)
            current_pos = self.xarm.xarm.get_position()[1][:6]
            current_pos[:3] = [x / 1000.0 for x in current_pos[:3]]  # mm to m
            
            # 获取夹爪状态
            gripper_state = self.gripper.get_state()
            gripper_pos = gripper_state.get('position', 0) / 850.0  # 归一化到[0,1]
            
            # 组合状态 [x, y, z, roll, pitch, yaw, gripper]
            self._last_robot_state = np.array(current_pos.tolist() + [gripper_pos], dtype=np.float32)
            
        except Exception as e:
            print(f"[XArmEnv] 获取机械臂状态失败: {e}")
            if self._last_robot_state is None:
                self._last_robot_state = np.zeros(7, dtype=np.float32)
    
    def _get_camera_data(self):
        """获取相机数据"""
        if not self.use_camera:
            return None
        
        return self._get_usb_camera_data()
    

    
    def _get_usb_camera_data(self):
        """获取USB相机数据"""
        try:
            ret, frame = self.camera.read()
            if not ret:
                return None
            
            # 转换BGR到RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            return rgb_frame
            
        except Exception as e:
            print(f"[XArmEnv] USB相机数据获取失败: {e}")
            return None
    
    def get_obs(self):
        """获取当前观测"""
        # 更新机械臂状态
        self._update_robot_state()
        
        obs_dict = {
            'agent_pos': self._last_robot_state.copy()
        }
        
        # 获取相机数据
        if self.use_camera:
            rgb_image = self._get_camera_data()
            
            if rgb_image is not None:
                obs_dict['image'] = rgb_image
            else:
                obs_dict['image'] = np.zeros((480, 640, 3), dtype=np.uint8)
        
        return obs_dict
    
    def step(self, action):
        """执行动作并返回观测"""
        assert len(action) == self.arm_action_dim + self.gripper_action_dim
        
        arm_action = action[:self.arm_action_dim]
        gripper_action = action[self.arm_action_dim]
        
        if self.safe_mode:
            # 安全模式：只更新模拟状态，不控制真实机械臂
            print(f"[XArmEnv] 安全模式动作: {action}")
            # 简单更新状态（可以根据需要调整）
            if self._last_robot_state is not None:
                self._last_robot_state = action.copy()
        else:
            # 真实模式：执行机械臂动作
            try:
                # 获取当前位置
                current_pos = self.xarm.xarm.get_position()[1][:6]
                
                # 转换动作为绝对位置 (action是相对位置或绝对位置，根据需要调整)
                target_pos = arm_action.copy()
                target_pos[:3] *= 1000  # m to mm
                
                # 发送位置命令
                self.xarm.set_servo_cartesian(target_pos)
                
                # 控制夹爪
                gripper_pos = int(gripper_action * 850)  # 0-850范围
                self.gripper.set_position(gripper_pos)
                
            except Exception as e:
                print(f"[XArmEnv] 动作执行失败: {e}")
        
        # 获取新的观测
        obs_dict = self.get_obs()
        
        return obs_dict, 0, False, {}
    
    def reset(self):
        """重置环境"""
        print("[XArmEnv] 重置环境...")
        # 注意：我们不移动到初始位置，保持当前位置
        return self.get_obs()
    
    def close(self):
        """关闭环境"""
        print("[XArmEnv] 关闭环境...")
        
        if not self.safe_mode and self.xarm:
            self.xarm.stop()
        
        if self.camera:
            self.camera.release()
        
        cv2.destroyAllWindows()
    
    def render(self, mode='rgb_array'):
        """渲染环境"""
        if not self.use_camera:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        rgb_image = self._get_camera_data()
        if rgb_image is not None:
            return rgb_image
        else:
            return np.zeros((480, 640, 3), dtype=np.uint8)

if __name__ == '__main__':
    # 测试环境（安全模式）
    env = XArmEnv(use_camera=True, safe_mode=True)
    
    try:
        print("测试环境...")
        obs = env.reset()
        print(f"观测空间: {list(obs.keys())}")
        print(f"机械臂状态: {obs['agent_pos']}")
        
        if 'image' in obs:
            print(f"图像形状: {obs['image'].shape}")
        
        # 测试动作
        action = np.zeros(7)  # 6维位置 + 1维夹爪
        obs, reward, done, info = env.step(action)
        print("动作测试完成")
        
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        env.close()
