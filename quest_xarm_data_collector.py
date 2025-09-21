#!/usr/bin/env python3
"""
Quest VR控制XArm机械臂的数据采集系统
基于成功的quest_xarm_control.py，增加数据记录功能
"""

import sys
import os
import time
import numpy as np
import threading
import cv2
import pickle
import argparse
from datetime import datetime
from termcolor import cprint

# 添加oculus_reader到路径
sys.path.append('/home/user/Desktop/oculus_reader')
from oculus_reader.reader import OculusReader

# 导入我们的机械臂控制模块
from xarm_wrapper import XArmWrapper
from xarm_gripper_wrapper import XArmGripperWrapper

class QuestXArmDataCollector:
    def __init__(self, xarm_ip='10.3.32.200', camera_index=0):
        print("[DataCollector] 初始化Quest-XArm数据采集系统...")
        
        # 初始化XArm机械臂
        print("[DataCollector] 连接XArm机械臂...")
        self.xarm = XArmWrapper(joints_init=None, move_to_init=False, ip=xarm_ip)
        self.gripper = XArmGripperWrapper(self.xarm.xarm)
        
        # 初始化Quest阅读器
        print("[DataCollector] 初始化Quest阅读器...")
        self.quest_reader = OculusReader(run=True, print_FPS=False)
        
        # 初始化相机
        print(f"[DataCollector] 初始化相机 (索引: {camera_index})...")
        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            raise Exception(f"无法打开相机索引 {camera_index}")
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # 控制参数 (与quest_xarm_control.py保持一致)
        self.running = False
        self.control_scale = 0.8  # 位置控制缩放因子
        self.rotation_scale = 0.5   # 旋转控制缩放因子
        
        # 初始位置（机械臂工作空间中心）
        self.base_position = np.array([0.35, 0.0, 0.25])  # [x, y, z]
        self.last_quest_position = None
        self.last_quest_rotation = None
        
        # 按钮状态
        self.last_button_states = {}
        
        # 数据采集相关
        self.recording = False
        self.control_started = False  # 是否已经开始过控制
        self.data_lock = threading.Lock()
        self.image_list = []
        self.robot_state_list = []
        self.action_list = []
        self.quest_data_list = []
        self.timestamp_list = []
        
        # 线程锁
        self.lock = threading.Lock()
        
        print("[DataCollector] 初始化完成！")


    def start_control_and_collection(self):
        """开始控制和数据采集"""
        print("[DataCollector] 开始Quest控制和数据采集...")
        self.running = True
        
        
        # 启动控制循环
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.start()
        
        print("[DataCollector] Quest数据采集系统已启动！")
        print("[DataCollector] 等待用户操作...")
        print("[DataCollector] 按A键开始控制和录制")
        print("[DataCollector] 按B键结束录制并退出程序")
        print("[DataCollector] 注意：机械臂控制仅在按A键后激活")

    def stop_control(self):
        """停止控制和数据采集"""
        if not self.running:
            return
            
        print("[DataCollector] 开始停止控制和数据采集...")
        self.running = False
        
        # 如果正在录制，先停止录制
        if self.recording:
            self._stop_recording()
        
        # 停止Quest阅读器线程
        if hasattr(self, 'quest_reader'):
            self.quest_reader.stop()
        
        # 等待控制循环线程结束
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            print("[DataCollector] 等待控制线程结束...")
            self.control_thread.join(timeout=2.0)

        # 安全关闭机械臂
        if hasattr(self, 'xarm') and self.xarm.xarm.connected:
            print("[DataCollector] 正在将机械臂恢复到安全状态...")
            try:
                self.xarm.xarm.set_mode(0)
                time.sleep(0.5)
                self.xarm.xarm.set_state(4)
                print("[DataCollector] 机械臂已设置为位置模式并停止。")
                self.xarm.xarm.disconnect()
                print("[DataCollector] 与机械臂的连接已断开。")
            except Exception as e:
                print(f"[DataCollector] 在安全关闭机械臂时发生错误: {e}")
        
        # 关闭相机
        if hasattr(self, 'camera'):
            self.camera.release()
        
        print("[DataCollector] 控制和数据采集已完全停止。")

    def _control_loop(self):
        """主控制循环（集成数据采集）"""
        control_rate = 15 
        dt = 1.0 / control_rate
        
        print("[DataCollector] 控制循环启动，频率: {}Hz".format(control_rate))
        
        while self.running:
            try:
                # 获取Quest数据
                transforms, buttons = self.quest_reader.get_transformations_and_buttons()
                
                if transforms and 'r' in transforms:  # 使用右手控制器
                    right_transform = transforms['r']
                    
                    # 提取位置和旋转
                    quest_position = right_transform[:3, 3]  # 位置
                    quest_rotation = right_transform[:3, :3]  # 旋转矩阵
                    
                    # 处理位置和旋转控制
                    action = self._handle_position_control(quest_position, quest_rotation)
                    
                    # 处理按钮控制
                    if buttons:
                        self._handle_button_control(buttons)
                    
                    # 如果正在录制，采集数据
                    if self.recording:
                        self._collect_data_point(transforms, buttons, action)
                
                time.sleep(dt)
                
            except Exception as e:
                print(f"[DataCollector] 控制循环错误: {e}")
                time.sleep(dt)

    def _handle_position_control(self, quest_position, quest_rotation):
        """处理位置和旋转控制（返回动作用于数据记录）"""
        action = np.zeros(7)  # [x, y, z, roll, pitch, yaw, gripper]
        
        if self.last_quest_position is None:
            # 第一次接收数据，记录初始位置和旋转
            self.last_quest_position = quest_position.copy()
            self.last_quest_rotation = quest_rotation.copy()
            return action
        
        # 计算Quest控制器的相对移动
        position_delta = quest_position - self.last_quest_position
        
        # 坐标系转换（与quest_xarm_control.py保持一致）
        arm_delta = np.array([
            position_delta[2],   # 机械臂X(前后) ← VR的Z
            position_delta[0],   # 机械臂Y(左右) ← VR的X
            position_delta[1]    # 机械臂Z(上下) ← VR的Y
        ])
        
        # 应用缩放并限制移动范围
        scaled_delta = arm_delta * self.control_scale
        scaled_delta = np.clip(scaled_delta, -0.05, 0.05)
        
        # 计算目标位置
        target_position = self.base_position + scaled_delta
        
        # 平滑边界限制
        workspace_limits = {
            'x': (0.2, 0.5),
            'y': (-0.3, 0.3), 
            'z': (0.17, 0.4)
        }
        
        smooth_target = target_position.copy()
        for i, (axis_name, (min_val, max_val)) in enumerate(zip(['x', 'y', 'z'], workspace_limits.values())):
            if target_position[i] < min_val:
                smooth_target[i] = min_val + 0.001
            elif target_position[i] > max_val:
                smooth_target[i] = max_val - 0.001
        
        target_position = smooth_target
        
        # 处理旋转变化
        rotation_delta = quest_rotation @ self.last_quest_rotation.T
        
        # 只有在开始控制后才发送命令到机械臂
        if self.control_started:
            # 发送位置和旋转命令到机械臂
            with self.lock:
                try:
                    code, current_pose = self.xarm.xarm.get_position()
                    if code == 0 and current_pose:
                        new_pose = current_pose.copy()
                        new_pose[:3] = target_position * 1000  # 米转毫米
                        
                        # 添加旋转控制
                        from scipy.spatial.transform import Rotation as R
                        
                        current_euler = new_pose[3:6]
                        rot_obj = R.from_matrix(rotation_delta)
                        euler_delta = rot_obj.as_euler('xyz', degrees=True)
                        
                        # 旋转映射（与quest_xarm_control.py保持一致）
                        remapped_euler_delta = np.array([
                            euler_delta[2], # 机械臂Roll增量 ← VR的Yaw增量
                            euler_delta[0], # 机械臂Pitch增量 ← VR的Roll增量
                            euler_delta[1]  # 机械臂Yaw增量 ← VR的Pitch增量
                        ])

                        scaled_euler_delta = remapped_euler_delta * self.rotation_scale
                        scaled_euler_delta = np.clip(scaled_euler_delta, -5, 5)
                        
                        new_pose[3:6] = current_euler + scaled_euler_delta
                        
                        # 设置新位置和姿态
                        self.xarm.set_servo_cartesian(new_pose)
                        
                        # 更新基准位置
                        self.base_position = target_position.copy()
                        
                        # 准备动作数据用于记录
                        action[:3] = target_position
                        action[3:6] = new_pose[3:6]
                        
                except Exception as e:
                    print(f"[DataCollector] 位置控制错误: {e}")
        else:
            # 即使没有开始控制，也生成动作数据用于记录
            action[:3] = target_position
            action[3:6] = [0, 0, 0]  # 默认姿态
        
        # 更新上次位置和旋转
        self.last_quest_position = quest_position.copy()
        self.last_quest_rotation = quest_rotation.copy()
        
        return action

    def _handle_button_control(self, button_data):
        """处理按钮控制"""
        try:
            # 检查右手触发器控制夹爪（只有在开始控制后才响应）
            if self.control_started and 'RTr' in button_data:
                trigger_pressed = button_data['RTr']
                
                if 'RTr' not in self.last_button_states or \
                   self.last_button_states['RTr'] != trigger_pressed:
                    
                    if trigger_pressed:
                        print("[DataCollector] 触发器按下 - 关闭夹爪")
                        self.gripper.close()
                    else:
                        print("[DataCollector] 触发器释放 - 打开夹爪")
                        self.gripper.open()
                    
                    self.last_button_states['RTr'] = trigger_pressed
            
            # 检查A按钮开始控制和录制
            if 'A' in button_data:
                a_pressed = button_data['A']
                if 'A' not in self.last_button_states or \
                   self.last_button_states['A'] != a_pressed:
                    
                    if a_pressed and not self.control_started:
                        print("[DataCollector] A按钮按下 - 开始控制和录制")
                        self._start_control_and_recording()
                    
                    self.last_button_states['A'] = a_pressed
            
            # 检查B按钮结束录制并退出
            if 'B' in button_data:
                b_pressed = button_data['B']
                
                if 'B' not in self.last_button_states or \
                   self.last_button_states['B'] != b_pressed:
                    
                    if b_pressed and self.control_started:
                        print("[DataCollector] B按钮按下 - 结束录制并退出程序")
                        self._stop_recording_and_exit()
                    
                    self.last_button_states['B'] = b_pressed
            
        except Exception as e:
            print(f"[DataCollector] 按钮控制错误: {e}")

    def _start_control_and_recording(self):
        """开始控制和录制（A按钮）"""
        print("\n[DataCollector] 🚀 开始控制和录制...")
        
        # 标记控制已开始
        self.control_started = True
        
        # 开始录制
        self._start_recording()

    def _stop_recording_and_exit(self):
        """停止录制并退出（B按钮）"""
        # 停止录制
        self._stop_recording()
        
        # 停止控制
        self.control_started = False
        print("[DataCollector] 🛑 停止控制")
        
        # 退出程序
        self.stop_control()

    def _start_recording(self):
        """开始录制数据"""
        with self.data_lock:
            print("\n[DataCollector] 🔴 开始录制数据...")
            self.recording = True
            
            # 清空之前的数据
            self.image_list = []
            self.robot_state_list = []
            self.action_list = []
            self.quest_data_list = []
            self.timestamp_list = []

    def _stop_recording(self):
        """停止录制数据"""
        with self.data_lock:
            print(f"\n[DataCollector] ⏹️  停止录制，共采集 {len(self.action_list)} 个数据点")
            self.recording = False

    def _collect_data_point(self, transforms, buttons, action):
        """采集一个数据点"""
        try:
            with self.data_lock:
                # 获取当前时间戳
                timestamp = time.time()
                
                # 获取相机图像
                ret, frame = self.camera.read()
                if ret:
                    rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                else:
                    rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
                
                # 获取机械臂状态
                try:
                    code, current_pose = self.xarm.xarm.get_position()
                    if code == 0 and current_pose:
                        # 转换为标准格式 [x, y, z, roll, pitch, yaw] + gripper
                        robot_state = np.array(current_pose[:6], dtype=np.float32)
                        robot_state[:3] /= 1000.0  # mm to m
                        
                        # 获取夹爪状态 (0=打开, 1=关闭)
                        gripper_state = self.gripper.get_state()
                        robot_state = np.append(robot_state, float(gripper_state))
                    else:
                        robot_state = np.zeros(7, dtype=np.float32)
                except:
                    robot_state = np.zeros(7, dtype=np.float32)
                
                # 获取夹爪状态并更新动作 (0=打开, 1=关闭)
                action[6] = float(self.gripper.get_state())
                
                # 存储数据
                self.image_list.append(rgb_image.copy())
                self.robot_state_list.append(robot_state.copy())
                self.action_list.append(action.copy())
                self.quest_data_list.append({'transforms': transforms, 'buttons': buttons})
                self.timestamp_list.append(timestamp)
                
                # 显示进度
                if len(self.action_list) % 25 == 0:
                    print(f"[DataCollector] 已采集 {len(self.action_list)} 个数据点")
                    
        except Exception as e:
            print(f"[DataCollector] 数据采集错误: {e}")

    def save_data(self, save_path):
        """保存采集的数据"""
        if len(self.action_list) == 0:
            print("[DataCollector] 没有数据可保存")
            return False
        
        try:
            with self.data_lock:
                # 转换为numpy数组
                image_arrays = np.stack(self.image_list, axis=0)
                robot_state_arrays = np.stack(self.robot_state_list, axis=0)
                action_arrays = np.stack(self.action_list, axis=0)
                timestamp_arrays = np.array(self.timestamp_list)
                
                # 准备数据字典（兼容DemoGen格式）
                data = {
                    'image': image_arrays,
                    'agent_pos': robot_state_arrays,
                    'action': action_arrays,
                    'quest_data': self.quest_data_list,
                    'timestamps': timestamp_arrays,
                    'metadata': {
                        'collection_time': datetime.now().isoformat(),
                        'num_samples': len(self.action_list),
                        'control_frequency': 50,
                        'workspace_limits': {
                            'x': (0.2, 0.5),
                            'y': (-0.3, 0.3), 
                            'z': (0.17, 0.4)
                        }
                    }
                }
                
                # 保存到文件
                os.makedirs(os.path.dirname(save_path), exist_ok=True)
                with open(save_path, 'wb') as f:
                    pickle.dump(data, f)
                
                print(f"[DataCollector] 数据已保存到: {save_path}")
                print(f"[DataCollector] 样本数量: {len(self.action_list)}")
                print(f"[DataCollector] 图像形状: {image_arrays.shape}")
                print(f"[DataCollector] 机械臂状态形状: {robot_state_arrays.shape}")
                print(f"[DataCollector] 动作形状: {action_arrays.shape}")
                
                return True
                
        except Exception as e:
            print(f"[DataCollector] 数据保存失败: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='Quest XArm数据采集系统')
    parser.add_argument('exp_name', type=str, help='实验名称')
    parser.add_argument('traj_name', type=str, help='轨迹名称')
    parser.add_argument('--camera-index', type=int, default=0, help='相机索引')
    
    args = parser.parse_args()
    
    # 创建保存路径
    save_base = "data/quest_demos"
    save_dir = os.path.join(save_base, args.exp_name)
    save_path = os.path.join(save_dir, f"{args.traj_name}.pkl")
    
    try:
        # 创建数据采集器
        collector = QuestXArmDataCollector(camera_index=args.camera_index)
        
        # 显示系统信息
        cprint("=== Quest XArm数据采集系统 ===", "green")
        cprint(f"实验: {args.exp_name}", "cyan")
        cprint(f"轨迹: {args.traj_name}", "cyan")
        cprint(f"保存: {save_path}", "cyan")
        cprint("控制说明:", "yellow")
        cprint("  - A按钮：开始控制和录制", "white")
        cprint("  - B按钮：结束录制并退出程序", "white")
        cprint("  - 右手触发器：控制夹爪开合（仅在录制时有效）", "white")
        cprint("  - 右手控制器：控制机械臂位置和姿态（仅在录制时有效）", "white")
        
        # 开始控制和采集
        collector.start_control_and_collection()
        
        # 等待程序结束
        while collector.running:
            time.sleep(1)
        
        # 保存数据
        if len(collector.action_list) > 0:
            collector.save_data(save_path)
        
    except KeyboardInterrupt:
        print("\n收到退出信号...")
    except Exception as e:
        print(f"程序错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'collector' in locals():
            collector.stop_control()
        print("程序已退出。")


if __name__ == "__main__":
    main()
