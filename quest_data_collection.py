#!/usr/bin/env python3
"""
基于Quest遥操作的XArm数据采集系统
结合Quest控制和数据记录功能
"""

import time
import sys
import numpy as np
import pickle
import os
import argparse
import threading
from datetime import datetime
from termcolor import cprint

# 导入我们的模块
from xarm_env_wrapper import XArmEnv
import oculus_reader


class QuestDataCollector:
    """Quest数据采集器"""
    
    def __init__(self, env, save_path, collect_frequency=10):
        self.env = env
        self.save_path = save_path
        self.collect_frequency = collect_frequency  # Hz
        
        # 数据存储
        self.image_list = []
        self.robot_state_list = []
        self.action_list = []
        self.quest_data_list = []  # 存储原始Quest数据
        
        # 控制变量
        self.collecting = False
        self.recording = False
        self.last_action = None
        
        # Quest读取器
        self.quest_reader = None
        self.quest_thread = None
        self.quest_running = False
        
        # 最后的Quest数据
        self.last_quest_data = None
        self.data_lock = threading.Lock()
        
        print(f"[DataCollector] 初始化完成，保存路径: {save_path}")
    
    def start_quest_reader(self):
        """启动Quest数据读取"""
        try:
            print("[DataCollector] 启动Quest读取器...")
            self.quest_reader = oculus_reader.reader.OculusReader()
            self.quest_running = True
            self.quest_thread = threading.Thread(target=self._quest_loop)
            self.quest_thread.start()
            print("[DataCollector] Quest读取器启动成功")
            return True
        except Exception as e:
            print(f"[DataCollector] Quest读取器启动失败: {e}")
            return False
    
    def _quest_loop(self):
        """Quest数据读取循环"""
        while self.quest_running:
            try:
                quest_data = self.quest_reader.get_transformations_and_buttons()
                if quest_data:
                    with self.data_lock:
                        self.last_quest_data = quest_data
                time.sleep(1/60)  # 60Hz读取
            except Exception as e:
                print(f"[DataCollector] Quest数据读取错误: {e}")
                time.sleep(0.1)
    
    def stop_quest_reader(self):
        """停止Quest数据读取"""
        self.quest_running = False
        if self.quest_thread:
            self.quest_thread.join()
        print("[DataCollector] Quest读取器已停止")
    
    def get_current_quest_data(self):
        """获取当前Quest数据"""
        with self.data_lock:
            return self.last_quest_data.copy() if self.last_quest_data else None
    
    def start_collecting(self):
        """开始数据采集"""
        print("[DataCollector] 开始数据采集...")
        self.collecting = True
        
        # 启动Quest读取器
        if not self.start_quest_reader():
            print("[DataCollector] 无法启动Quest读取器，退出")
            return False
        
        # 等待Quest数据
        print("[DataCollector] 等待Quest连接...")
        start_time = time.time()
        while time.time() - start_time < 10:  # 等待10秒
            if self.get_current_quest_data():
                print("[DataCollector] Quest连接成功！")
                break
            time.sleep(0.1)
        else:
            print("[DataCollector] Quest连接超时")
            self.stop_quest_reader()
            return False
        
        print("[DataCollector] 数据采集准备就绪")
        print("[DataCollector] 按右手触发器开始/停止记录")
        print("[DataCollector] 按A键退出")
        
        try:
            self._collection_loop()
        except KeyboardInterrupt:
            print("\n[DataCollector] 用户中断")
        finally:
            self.stop_quest_reader()
            self.collecting = False
        
        return True
    
    def _collection_loop(self):
        """数据采集主循环"""
        last_trigger_state = False
        last_a_state = False
        
        while self.collecting:
            start_time = time.time()
            
            # 获取Quest数据
            quest_data = self.get_current_quest_data()
            if not quest_data:
                time.sleep(0.01)
                continue
            
            # 检查按钮
            try:
                buttons = quest_data.get('buttons', {})
                
                # 检查A键退出
                a_pressed = buttons.get('A', False)
                if a_pressed and not last_a_state:
                    print("[DataCollector] A键按下，退出采集")
                    break
                last_a_state = a_pressed
                
                # 检查触发器开始/停止记录
                trigger_pressed = buttons.get('RTr', False)
                if trigger_pressed and not last_trigger_state:
                    if self.recording:
                        self._stop_recording()
                    else:
                        self._start_recording()
                last_trigger_state = trigger_pressed
                
            except Exception as e:
                print(f"[DataCollector] 按钮检查错误: {e}")
            
            # 如果正在记录，采集数据
            if self.recording:
                self._collect_data_point(quest_data)
            
            # 控制采集频率
            elapsed = time.time() - start_time
            sleep_time = (1.0 / self.collect_frequency) - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def _start_recording(self):
        """开始记录"""
        print("\n[DataCollector] 🔴 开始记录数据...")
        self.recording = True
        
        # 清空之前的数据
        self.image_list = []
        self.robot_state_list = []
        self.action_list = []
        self.quest_data_list = []
    
    def _stop_recording(self):
        """停止记录"""
        print(f"\n[DataCollector] ⏹️  停止记录，共采集 {len(self.action_list)} 个数据点")
        self.recording = False
        
        if len(self.action_list) > 0:
            self._save_data()
        else:
            print("[DataCollector] 没有数据可保存")
    
    def _collect_data_point(self, quest_data):
        """采集一个数据点"""
        try:
            # 获取当前观测
            obs = self.env.get_obs()
            
            # 提取数据
            robot_state = obs['agent_pos']
            image = obs.get('image', np.zeros((480, 640, 3), dtype=np.uint8))
            
            # 计算动作（这里可以根据需要调整）
            action = self._compute_action_from_quest(quest_data, robot_state)
            
            # 存储数据
            self.robot_state_list.append(robot_state.copy())
            self.image_list.append(image.copy())
            self.action_list.append(action.copy())
            self.quest_data_list.append(quest_data.copy())
            
            # 执行动作（如果需要实时控制）
            # self.env.step(action)
            
            # 显示进度
            if len(self.action_list) % 50 == 0:
                print(f"[DataCollector] 已采集 {len(self.action_list)} 个数据点")
                
        except Exception as e:
            print(f"[DataCollector] 数据采集错误: {e}")
    
    def _compute_action_from_quest(self, quest_data, current_robot_state):
        """从Quest数据计算动作"""
        try:
            # 获取右手控制器数据
            transforms = quest_data.get('transforms', {})
            right_hand = transforms.get('r', {})
            
            if not right_hand:
                # 如果没有数据，返回当前状态作为动作
                return current_robot_state.copy()
            
            # 提取位置和旋转
            position = right_hand.get('position', [0, 0, 0])
            rotation = right_hand.get('rotation', [0, 0, 0, 1])
            
            # 转换为机械臂动作
            # 这里需要根据实际情况调整坐标变换和缩放
            action_pos = np.array([
                position[1] * 0.001,  # VR Y -> Robot X (mm to m)
                position[0] * 0.001,  # VR X -> Robot Y (mm to m)  
                position[2] * 0.001   # VR Z -> Robot Z (mm to m)
            ])
            
            # 转换旋转（四元数到欧拉角）
            from scipy.spatial.transform import Rotation as R
            r = R.from_quat([rotation[0], rotation[1], rotation[2], rotation[3]])
            euler = r.as_euler('xyz')
            
            # 获取夹爪动作
            buttons = quest_data.get('buttons', {})
            gripper_action = 1.0 if buttons.get('RTr', False) else 0.0
            
            # 组合动作 [x, y, z, roll, pitch, yaw, gripper]
            action = np.array([
                action_pos[0], action_pos[1], action_pos[2],
                euler[0], euler[1], euler[2],
                gripper_action
            ])
            
            return action
            
        except Exception as e:
            print(f"[DataCollector] 动作计算错误: {e}")
            return current_robot_state.copy()
    
    def _save_data(self):
        """保存数据"""
        try:
            # 转换为numpy数组
            image_arrays = np.stack(self.image_list, axis=0)
            robot_state_arrays = np.stack(self.robot_state_list, axis=0)
            action_arrays = np.stack(self.action_list, axis=0)
            
            # 准备数据字典
            data = {
                'image': image_arrays,
                'agent_pos': robot_state_arrays,
                'action': action_arrays,
                'quest_data': self.quest_data_list,  # 原始Quest数据
                'metadata': {
                    'collection_time': datetime.now().isoformat(),
                    'num_samples': len(self.action_list),
                    'frequency': self.collect_frequency
                }
            }
            
            # 保存到文件
            with open(self.save_path, 'wb') as f:
                pickle.dump(data, f)
            
            print(f"[DataCollector] 数据已保存到: {self.save_path}")
            print(f"[DataCollector] 样本数量: {len(self.action_list)}")
            print(f"[DataCollector] 图像形状: {image_arrays.shape}")
            print(f"[DataCollector] 机械臂状态形状: {robot_state_arrays.shape}")
            print(f"[DataCollector] 动作形状: {action_arrays.shape}")
            
        except Exception as e:
            print(f"[DataCollector] 数据保存失败: {e}")


def main():
    parser = argparse.ArgumentParser(description='Quest XArm数据采集')
    parser.add_argument('exp_name', type=str, help='实验名称')
    parser.add_argument('traj_name', type=str, help='轨迹名称')
    parser.add_argument('--freq', type=int, default=10, help='采集频率(Hz)')
    parser.add_argument('--no-camera', action='store_true', help='不使用相机')
    parser.add_argument('--safe-mode', action='store_true', help='安全模式：不连接真实机械臂')
    
    args = parser.parse_args()
    
    # 创建保存目录
    save_base = "data/quest_demos"
    save_dir = os.path.join(save_base, args.exp_name)
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, f"{args.traj_name}.pkl")
    
    # 初始化环境
    print("[Main] 初始化XArm环境...")
    env = XArmEnv(use_camera=not args.no_camera, safe_mode=args.safe_mode)
    
    try:
        # 初始化数据采集器
        collector = QuestDataCollector(env, save_path, args.freq)
        
        # 开始采集
        cprint("=== Quest XArm数据采集系统 ===", "green")
        cprint(f"实验: {args.exp_name}", "cyan")
        cprint(f"轨迹: {args.traj_name}", "cyan")
        cprint(f"频率: {args.freq}Hz", "cyan")
        cprint(f"安全模式: {'开启' if args.safe_mode else '关闭'}", "yellow")
        cprint(f"保存: {save_path}", "cyan")
        
        collector.start_collecting()
        
    except Exception as e:
        print(f"[Main] 采集过程出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        env.close()
        print("[Main] 程序结束")


if __name__ == "__main__":
    main()
