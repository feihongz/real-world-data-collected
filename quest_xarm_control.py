#!/usr/bin/env python3
"""
Quest VR控制XArm机械臂
使用oculus_reader库直接读取Quest控制器数据
"""

import sys
import os
import time
import numpy as np
import threading

# 添加oculus_reader到路径
sys.path.append('/home/user/Desktop/oculus_reader')
from oculus_reader.reader import OculusReader

# 导入我们的机械臂控制模块
from xarm_wrapper import XArmWrapper
from xarm_gripper_wrapper import XArmGripperWrapper

class QuestXArmController:
    def __init__(self, xarm_ip='10.3.32.200'):
        print("[QuestXArm] 初始化Quest-XArm控制器...")
        
        # 初始化XArm机械臂
        print("[QuestXArm] 连接XArm机械臂...")
        self.xarm = XArmWrapper(joints_init=None, move_to_init=False, ip=xarm_ip)
        self.gripper = XArmGripperWrapper(self.xarm.xarm)
        
        # 初始化Quest阅读器
        print("[QuestXArm] 初始化Quest阅读器...")
        self.quest_reader = OculusReader(run=True, print_FPS=False)
        
        # 控制参数
        self.running = False
        self.control_scale = 0.6  # 位置控制缩放因子 (降低灵敏度)
        self.rotation_scale = 2  # 旋转控制缩放因子
        
        # 初始位置（机械臂工作空间中心）
        self.base_position = np.array([0.35, 0.0, 0.25])  # [x, y, z]
        self.last_quest_position = None
        self.last_quest_rotation = None
        
        # 按钮状态
        self.last_button_states = {}
        
        # 线程锁
        self.lock = threading.Lock()
        
        print("[QuestXArm] 初始化完成！")
    def _go_to_start_pose(self):
        """
        【最终工作版本】基于成功的时序测试，解决模式切换问题。
        核心是为控制器状态切换提供充足的延时。
        """
        print("[QuestXArm] 开始执行基于正确时序的归位与伺服准备程序...")

        # 1. 使能，并明确设置到位置模式
        print("[QuestXArm] Step 1: 清理并进入位置模式...")
        self.xarm.xarm.motion_enable(enable=True)
        self.xarm.xarm.set_mode(0)
        self.xarm.xarm.set_state(0)
        
        # 【核心修复】给控制器充足的时间稳定在位置模式
        print("[QuestXArm] 等待控制器在位置模式下稳定...")
        time.sleep(2.0)

        # 2. 在稳定的位置模式下，移动到目标点
        print("[QuestXArm] Step 2: 移动到初始姿态...")
        start_position_mm = [350, 0, 250]
        start_euler_angles = [180, 0, 0]
        ret = self.xarm.xarm.set_position(
            *start_position_mm,
            roll=start_euler_angles[0],
            pitch=start_euler_angles[1],
            yaw=start_euler_angles[2],
            speed=50,
            wait=True
        )
        if ret != 0:
            print(f"[QuestXArm] 移动失败，错误码: {ret}，请检查机械臂状态。")
            return

        # 【核心修复】移动完成后，让机械臂和控制器有时间完全静止
        print("[QuestXArm] 等待机械臂在目标点完全稳定...")
        time.sleep(3.0)

        # 3. 切换到伺服模式
        print("[QuestXArm] Step 3: 切换到伺服模式...")
        self.xarm.xarm.set_mode(1)

        # 【核心修复】给控制器充足的时间完成到伺服模式的内部切换
        print("[QuestXArm] 等待控制器在伺服模式下稳定...")
        time.sleep(2.0)

        # 4. 使能伺服模式
        print("[QuestXArm] Step 4: 使能伺服运动...")
        self.xarm.xarm.set_state(0)
        print("[QuestXArm] 伺服模式已激活，无跳变。")

        # 5. 重置程序逻辑变量（现在可以安全地获取位置）
        code, real_pose = self.xarm.xarm.get_position()
        self.base_position = np.array(real_pose[:3]) / 1000.0
        self.last_quest_position = None
        self.last_quest_rotation = None
        print(f"[QuestXArm] 程序基准位置已更新为: {self.base_position} 米")
        
        print("="*20 + " 系统准备就绪 " + "="*20)
    def start_control(self):
        # self._go_to_start_pose()
        """开始Quest控制"""
        print("[QuestXArm] 开始Quest控制...")
        self.running = True
        
        # 启动控制循环
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.start()
        
        print("[QuestXArm] Quest控制已启动！")
    
    def stop_control(self):
        """
        停止Quest控制，并执行一个健壮、安全的关闭流程。
        """
        if not self.running:
            # 如果已经停止，就不用重复执行了
            return
            
        print("[QuestXArm] 开始停止Quest控制和安全关闭流程...")
        self.running = False
        
        # 停止Quest阅读器线程
        if hasattr(self, 'quest_reader'):
            self.quest_reader.stop()
        
        # 等待控制循环线程结束
        if hasattr(self, 'control_thread') and self.control_thread.is_alive():
            print("[QuestXArm] 等待控制线程结束...")
            self.control_thread.join(timeout=2.0) # 设置一个超时

        # 检查机械臂是否还连接着
        if hasattr(self, 'xarm') and self.xarm.xarm.connected:
            print("[QuestXArm] 正在将机械臂恢复到安全状态...")
            try:
                # 1. 切换回位置模式
                self.xarm.xarm.set_mode(0)
                time.sleep(0.5)
                
                # 2. 设置为停止状态
                # State 4 是一个安全的停止/错误状态
                self.xarm.xarm.set_state(4)
                print("[QuestXArm] 机械臂已设置为位置模式并停止。")
                
                # 3. 断开连接
                self.xarm.xarm.disconnect()
                print("[QuestXArm] 与机械臂的连接已断开。")
                
            except Exception as e:
                print(f"[QuestXArm] 在安全关闭机械臂时发生错误: {e}")
        
        print("[QuestXArm] Quest控制已完全停止。")
    
    def _control_loop(self):
        """主控制循环"""
        control_rate = 50  # 50Hz
        dt = 1.0 / control_rate
        
        print("[QuestXArm] 控制循环启动，频率: {}Hz".format(control_rate))
        
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
                    self._handle_position_control(quest_position, quest_rotation)
                    print (buttons)
                    # 处理按钮控制
                    if buttons:
                        self._handle_button_control(buttons)
                
                time.sleep(dt)
                
            except Exception as e:
                print(f"[QuestXArm] 控制循环错误: {e}")
                time.sleep(dt)
    
    def _handle_position_control(self, quest_position, quest_rotation):
        """处理位置和旋转控制"""
        if self.last_quest_position is None:
            # 第一次接收数据，记录初始位置和旋转
            self.last_quest_position = quest_position.copy()
            self.last_quest_rotation = quest_rotation.copy()
            return
        
        # 计算Quest控制器的相对移动
        position_delta = quest_position - self.last_quest_position
        
        # 坐标系转换: 机械臂X ← VR的Z, 机械臂Y ← VR的X, 机械臂Z ← VR的Y
        arm_delta = np.array([
            position_delta[2],   # 机械臂X(前后) ← VR的Z
            position_delta[0],   # 机械臂Y(左右) ← VR的X
            position_delta[1]    # 机械臂Z(上下) ← VR的Y
        ])
        
        # 应用缩放并限制移动范围
        scaled_delta = arm_delta * self.control_scale
        scaled_delta = np.clip(scaled_delta, -0.05, 0.05)  # 限制单次移动距离
        
        # 计算目标位置
        target_position = self.base_position + scaled_delta
        
        # 平滑边界限制 - 避免突然限制导致的抖动
        workspace_limits = {
            'x': (0.2, 0.5),
            'y': (-0.3, 0.3), 
            'z': (0.17, 0.4)
        }
        
        # 检查是否接近边界，如果接近则减少移动幅度
        smooth_target = target_position.copy()
        for i, (axis_name, (min_val, max_val)) in enumerate(zip(['x', 'y', 'z'], workspace_limits.values())):
            if target_position[i] < min_val:
                # 接近下边界，平滑限制
                smooth_target[i] = min_val + 0.001  # 留一点缓冲
                print(f"[QuestXArm] {axis_name}轴接近下边界")
            elif target_position[i] > max_val:
                # 接近上边界，平滑限制  
                smooth_target[i] = max_val - 0.001  # 留一点缓冲
                print(f"[QuestXArm] {axis_name}轴接近上边界")
        
        target_position = smooth_target

        
        # 处理旋转变化
        rotation_delta = quest_rotation @ self.last_quest_rotation.T  # 相对旋转变化
        
        # 发送位置和旋转命令到机械臂
        with self.lock:
            try:
                # 获取当前末端执行器姿态 (原始API返回毫米，需要转换)
                code, current_pose = self.xarm.xarm.get_position()
                if code == 0 and current_pose:
                    # 更新位置 (转换为毫米)
                    new_pose = current_pose.copy()
                    new_pose[:3] = target_position * 1000  # 米转毫米
                    
                    # 添加旋转控制 - 将旋转矩阵转换为欧拉角
                    from scipy.spatial.transform import Rotation as R
                    
                    # 获取当前欧拉角 (度)
                    current_euler = new_pose[3:6]  # roll, pitch, yaw
                    
                    # 将旋转变化转换为欧拉角增量
                    rot_obj = R.from_matrix(rotation_delta)
                    euler_delta = rot_obj.as_euler('xyz', degrees=True) # [delta_x, delta_y, delta_z]
                    
                    # 【核心修复】根据位置坐标的映射关系，对旋转增量进行同样的映射
                    # 机械臂Roll(绕X) ← VR的Yaw(绕Z)
                    # 机械臂Pitch(绕Y) ← VR的Roll(绕X)
                    # 机械臂Yaw(绕Z) ← VR的Pitch(绕Y)
                    remapped_euler_delta = np.array([
                        euler_delta[2], # 机械臂Roll增量 ← VR的Yaw增量 (euler_delta的第3个元素)
                        euler_delta[0], # 机械臂Pitch增量 ← VR的Roll增量 (euler_delta的第1个元素)
                        euler_delta[1]  # 机械臂Yaw增量 ← VR的Pitch增量 (euler_delta的第2个元素)
                    ])

                    # 应用旋转缩放和限制
                    scaled_euler_delta = remapped_euler_delta * self.rotation_scale
                    scaled_euler_delta = np.clip(scaled_euler_delta, -5, 5)  # 限制旋转幅度
                    
                    # 更新欧拉角
                    new_pose[3:6] = current_euler + scaled_euler_delta
                    
                    # 设置新位置和姿态
                    self.xarm.set_servo_cartesian(new_pose)
                    
                    # 更新基准位置
                    self.base_position = target_position.copy()
                    
            except Exception as e:
                print(f"[QuestXArm] 位置控制错误: {e}")
        
        # 更新上次位置和旋转
        self.last_quest_position = quest_position.copy()
        self.last_quest_rotation = quest_rotation.copy()
    
    def _handle_button_control(self, button_data):
        """处理按钮控制"""
        try:
            # 检查右手触发器 (RTr) 控制夹爪
            if 'RTr' in button_data:
                trigger_pressed = button_data['RTr']
                print ('trigger_pressed', trigger_pressed)
                # 如果触发器状态改变
                if 'RTr' not in self.last_button_states or \
                   self.last_button_states['RTr'] != trigger_pressed:
                    
                    if trigger_pressed:
                        print("[QuestXArm] 触发器按下 - 关闭夹爪")
                        self.gripper.close()
                    else:
                        print("[QuestXArm] 触发器释放 - 打开夹爪")
                        self.gripper.open()
                    
                    self.last_button_states['RTr'] = trigger_pressed
            
            # 检查触发器模拟值 (rightTrig) 进行精确控制
            # if 'rightTrig' in button_data:
            #     trigger_value = button_data['rightTrig']
            #     if isinstance(trigger_value, tuple) and len(trigger_value) > 0:
            #         trigger_strength = trigger_value[0]  # 0.0-1.0
                    
            #         # 根据触发器强度控制夹爪位置
            #         gripper_position = int(trigger_strength * 850)  # 0-850是XArm夹爪范围
            #         self.gripper.set_position(gripper_position)
            
            # 检查右手握持按钮 (RG) 重置位置
            # if 'RG' in button_data:
            #     grip_pressed = button_data['RG']
            #     if 'RG' not in self.last_button_states or \
            #        self.last_button_states['RG'] != grip_pressed:
                    
            #         if grip_pressed:
            #             print("[QuestXArm] 握持按钮按下 - 重置到中心位置")
            #             self._reset_to_center()
                    
            #         self.last_button_states['RG'] = grip_pressed
            
            # 检查A按钮用于紧急停止
            if 'A' in button_data:
                a_pressed = button_data['A']
                if 'A' not in self.last_button_states or \
                   self.last_button_states['A'] != a_pressed:
                    
                    if a_pressed:
                        print("[QuestXArm] A按钮按下 - 紧急停止")
                        self.stop_control()
                    
                    self.last_button_states['A'] = a_pressed
            
        except Exception as e:
            print(f"[QuestXArm] 按钮控制错误: {e}")
            import traceback
            traceback.print_exc()
    
    def _reset_to_center(self):
        """重置机械臂到中心位置"""
        try:
            # 【核心修复】直接调用_go_to_start_pose函数，以确保所有状态都被正确重置
            print("[QuestXArm] 正在重置到中心位置...")
            self._go_to_start_pose()
            print("[QuestXArm] 机械臂已重置到中心位置")

        except Exception as e:
            print(f"[QuestXArm] 重置位置错误: {e}")
    
    def print_status(self):
        """打印当前状态"""
        try:
            transforms, buttons = self.quest_reader.get_transformations_and_buttons()
            
            print("\n" + "="*50)
            print("Quest-XArm 控制状态")
            print("="*50)
            
            if transforms:
                print("Quest控制器状态:")
                for hand, transform in transforms.items():
                    pos = transform[:3, 3]
                    print(f"  {hand}手: 位置 [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
            else:
                print("Quest控制器: 未检测到")
            
            if buttons:
                print("按钮状态:")
                for hand, button_data in buttons.items():
                    print(f"  {hand}手: {button_data}")
            else:
                print("按钮: 无数据")
            
            # 打印机械臂状态
            try:
                code, current_pose = self.xarm.xarm.get_position()
                if code == 0 and current_pose:
                    # 转换毫米到米显示
                    pos_m = np.array(current_pose[:3]) / 1000
                    print(f"XArm位置: [{pos_m[0]:.3f}, {pos_m[1]:.3f}, {pos_m[2]:.3f}]m")
                print(f"夹爪状态: {self.gripper.get_state()}")
            except Exception as e:
                print(f"XArm状态: 获取失败 - {e}")
            
            print("="*50)
            
        except Exception as e:
            print(f"状态打印错误: {e}")

def main():
    """主函数"""
    print("Quest-XArm 实时控制系统")
    print("按 Ctrl+C 退出")
    
    try:
        # 创建控制器
        controller = QuestXArmController()
        
        # 开始控制
        controller.start_control()
        
        # 状态监控循环
        while True:
            time.sleep(2)
            controller.print_status()
            
    except KeyboardInterrupt:
        print("\n收到退出信号...")
    except Exception as e:
        print(f"主程序错误: {e}")
    finally:
        # 无论发生什么，都尝试安全地停止
        if controller:
            controller.stop_control()
        print("程序已退出。")


if __name__ == "__main__":
    main()
