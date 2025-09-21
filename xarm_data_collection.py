#!/usr/bin/env python3
"""
XArm数据采集系统 - 支持键盘和VR控制
用于机器人学习的数据采集
"""
import os
import time
import numpy as np
import threading
import sys
import termios
import tty
from scipy.spatial.transform import Rotation as R

from xarm_wrapper import XArmWrapper
from robotiq_wrapper import RobotiqWrapper
from realsense import RealSense
from quest_streamer import QuestStreamer

class KeyboardController:
    """键盘输入控制器"""
    
    def __init__(self):
        self.running = True
        self.key_pressed = None
        self.key_lock = threading.Lock()
        self.old_settings = termios.tcgetattr(sys.stdin)
        
    def start(self):
        self.thread = threading.Thread(target=self._keyboard_listener)
        self.thread.daemon = True
        self.thread.start()
        
    def _keyboard_listener(self):
        try:
            tty.setraw(sys.stdin.fileno())
            while self.running:
                key = sys.stdin.read(1)
                with self.key_lock:
                    self.key_pressed = key
                time.sleep(0.01)
        except:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_key(self):
        with self.key_lock:
            key = self.key_pressed
            self.key_pressed = None
            return key
    
    def stop(self):
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

class XArmDataCollector:
    """XArm数据采集系统"""
    
    def __init__(self, control_mode='keyboard'):
        """
        初始化数据采集系统
        
        Args:
            control_mode: 'keyboard' 或 'vr'
        """
        self.control_mode = control_mode
        self.running = False
        
        # 控制参数
        self.position_step = 0.005  # 5mm
        self.rotation_step = np.radians(3)  # 3度
        self.control_frequency = 20  # Hz
        self.dt = 1.0 / self.control_frequency
        
        # 数据存储
        self.demo_data = []
        self.frame_idx = 0
        
    def initialize_hardware(self):
        """初始化硬件设备"""
        print("=" * 60)
        print("初始化硬件设备...")
        print("=" * 60)
        
        # 初始化相机
        print("1. 初始化RealSense相机...")
        try:
            self.camera = RealSense()
            self.camera.start()
            print("✓ 相机初始化成功")
        except Exception as e:
            print(f"⚠ 相机初始化失败: {e}")
            self.camera = None
        
        # 初始化机械臂 (保持当前位置)
        print("2. 初始化XArm机械臂...")
        try:
            self.xarm = XArmWrapper(joints_init=None, move_to_init=False)
            print("✓ XArm初始化成功")
        except Exception as e:
            print(f"✗ XArm初始化失败: {e}")
            raise
        
        # 初始化夹爪
        print("3. 初始化Robotiq夹爪...")
        try:
            self.gripper = RobotiqWrapper(robot='xarm')
            print("✓ 夹爪初始化成功")
        except Exception as e:
            print(f"⚠ 夹爪初始化失败: {e}")
            self.gripper = None
        
        # 初始化控制接口
        if self.control_mode == 'keyboard':
            print("4. 初始化键盘控制...")
            self.controller = KeyboardController()
            self.controller.start()
            print("✓ 键盘控制初始化成功")
        elif self.control_mode == 'vr':
            print("4. 初始化VR控制...")
            self.controller = QuestStreamer(record=False)
            self.controller.connect()
            print("✓ VR控制初始化成功")
        
        # 获取初始位置
        print("5. 获取机械臂初始状态...")
        self.initial_pose = self.xarm.get_position()
        self.target_pose = self.initial_pose.copy()
        print(f"✓ 初始位置: {self.initial_pose[:3]}")
        print(f"✓ 初始姿态: {np.degrees(self.initial_pose[3:]):.1f}°")
        
    def print_controls(self):
        """打印控制说明"""
        if self.control_mode == 'keyboard':
            print("\n" + "="*60)
            print("键盘控制说明:")
            print("="*60)
            print("位置控制: W/S(X), A/D(Y), Q/E(Z)")
            print("姿态控制: I/K(RX), J/L(RY), U/O(RZ)")  
            print("夹爪控制: G(开), H(关)")
            print("数据采集: SPACE(开始/停止记录)")
            print("其他: R(重置), ESC(退出)")
            print("="*60)
        elif self.control_mode == 'vr':
            print("\n" + "="*60)
            print("VR控制说明:")
            print("="*60)
            print("- 使用右手控制机械臂位置和姿态")
            print("- 右手捏合控制夹爪开合")
            print("- 左手紧捏结束数据采集")
            print("="*60)
            
    def get_control_commands(self):
        """获取控制命令"""
        if self.control_mode == 'keyboard':
            return self._get_keyboard_commands()
        elif self.control_mode == 'vr':
            return self._get_vr_commands()
        
    def _get_keyboard_commands(self):
        """处理键盘输入"""
        key = self.controller.get_key()
        pose_changed = False
        gripper_cmd = None
        special_cmd = None
        
        if key:
            if key == '\x1b':  # ESC
                special_cmd = 'quit'
            elif key == ' ':  # SPACE
                special_cmd = 'toggle_recording'
            elif key.lower() == 'r':
                self.target_pose = self.initial_pose.copy()
                pose_changed = True
                print("重置到初始位置")
            elif key.lower() == 'w':
                self.target_pose[0] += self.position_step
                pose_changed = True
            elif key.lower() == 's':
                self.target_pose[0] -= self.position_step
                pose_changed = True
            elif key.lower() == 'd':
                self.target_pose[1] += self.position_step
                pose_changed = True
            elif key.lower() == 'a':
                self.target_pose[1] -= self.position_step
                pose_changed = True
            elif key.lower() == 'q':
                self.target_pose[2] += self.position_step
                pose_changed = True
            elif key.lower() == 'e':
                self.target_pose[2] -= self.position_step
                pose_changed = True
            elif key.lower() == 'i':
                self.target_pose[3] += self.rotation_step
                pose_changed = True
            elif key.lower() == 'k':
                self.target_pose[3] -= self.rotation_step
                pose_changed = True
            elif key.lower() == 'j':
                self.target_pose[4] += self.rotation_step
                pose_changed = True
            elif key.lower() == 'l':
                self.target_pose[4] -= self.rotation_step
                pose_changed = True
            elif key.lower() == 'u':
                self.target_pose[5] += self.rotation_step
                pose_changed = True
            elif key.lower() == 'o':
                self.target_pose[5] -= self.rotation_step
                pose_changed = True
            elif key.lower() == 'g':
                gripper_cmd = 'open'
            elif key.lower() == 'h':
                gripper_cmd = 'close'
        
        return {
            'pose': self.target_pose if pose_changed else None,
            'gripper': gripper_cmd,
            'special': special_cmd
        }
    
    def _get_vr_commands(self):
        """处理VR输入"""
        try:
            vr_data = self.controller.latest
            
            # 简化的VR控制逻辑 (使用模拟数据)
            right_hand = vr_data['right_wrist'][0]
            right_pinch = vr_data['right_pinch_distance']
            left_pinch = vr_data['left_pinch_distance']
            
            # 计算目标位置 (简单映射)
            target_pose = self.initial_pose.copy()
            target_pose[:3] = right_hand[:3, 3]
            
            # 夹爪控制
            gripper_cmd = 'close' if right_pinch < 0.03 else 'open'
            
            # 终止控制
            special_cmd = 'quit' if left_pinch < 0.02 else None
            
            return {
                'pose': target_pose,
                'gripper': gripper_cmd,
                'special': special_cmd
            }
        except Exception as e:
            print(f"VR控制错误: {e}")
            return {'pose': None, 'gripper': None, 'special': None}
    
    def execute_commands(self, commands):
        """执行控制命令"""
        try:
            # 执行位置控制
            if commands['pose'] is not None:
                target_mm = commands['pose'].copy()
                target_mm[:3] *= 1000  # m -> mm
                target_mm[3:] = np.degrees(target_mm[3:])  # rad -> deg
                self.xarm.set_servo_cartesian(target_mm)
            
            # 执行夹爪控制
            if commands['gripper'] and self.gripper:
                if commands['gripper'] == 'open':
                    self.gripper.open()
                elif commands['gripper'] == 'close':
                    self.gripper.close()
            
            return commands['special']
            
        except Exception as e:
            print(f"执行控制命令错误: {e}")
            return None
    
    def collect_data(self):
        """采集当前状态数据"""
        try:
            # 获取机械臂状态
            xarm_joint_angles = self.xarm.get_joint()
            xarm_pose = self.xarm.get_position()
            xarm_gripper_state = self.gripper.get_state() if self.gripper else 0
            
            # 获取相机数据
            camera_frame = self.camera.get_frame() if self.camera else {
                'depth': np.zeros((240, 320), dtype=np.uint16),
                'depth_scale': 0.001
            }
            
            # 构建数据帧
            data_frame = {
                'frame_idx': self.frame_idx,
                'timestamp': time.time(),
                'xarm_joint_angles': xarm_joint_angles,
                'xarm_pose': xarm_pose,
                'xarm_gripper_state': xarm_gripper_state,
                'target_pose': self.target_pose.copy(),
                'depth_image': camera_frame['depth'],
                'depth_scale': camera_frame['depth_scale'],
                'control_mode': self.control_mode
            }
            
            return data_frame
            
        except Exception as e:
            print(f"数据采集错误: {e}")
            return None
    
    def save_data(self, filename=None):
        """保存采集的数据"""
        if not self.demo_data:
            print("没有数据需要保存")
            return
        
        if filename is None:
            # 自动生成文件名
            demo_idx = 0
            data_dir = '/home/user/Desktop/xarm_franka_teleop/data'
            os.makedirs(data_dir, exist_ok=True)
            filename = f'{data_dir}/xarm_demo_{demo_idx:03d}.npy'
            while os.path.exists(filename):
                demo_idx += 1
                filename = f'{data_dir}/xarm_demo_{demo_idx:03d}.npy'
        
        try:
            np.save(filename, self.demo_data, allow_pickle=True)
            print(f"✓ 数据已保存到: {filename}")
            print(f"  共 {len(self.demo_data)} 帧数据")
            print(f"  采集时长: {len(self.demo_data) * self.dt:.1f}秒")
        except Exception as e:
            print(f"✗ 保存数据失败: {e}")
    
    def run(self):
        """运行数据采集循环"""
        self.print_controls()
        
        print(f"\n控制参数:")
        print(f"  位置步长: {self.position_step*1000:.0f}mm")
        print(f"  旋转步长: {np.degrees(self.rotation_step):.0f}°")
        print(f"  控制频率: {self.control_frequency}Hz")
        print(f"  控制模式: {self.control_mode}")
        
        if self.control_mode == 'keyboard':
            print("\n按SPACE键开始数据采集...")
        else:
            print("\n开始VR数据采集...")
        
        recording = False
        self.running = True
        
        try:
            while self.running:
                start_time = time.time()
                
                # 获取控制命令
                commands = self.get_control_commands()
                
                # 处理特殊命令
                if commands['special'] == 'quit':
                    break
                elif commands['special'] == 'toggle_recording':
                    recording = not recording
                    if recording:
                        print("🔴 开始数据采集...")
                        self.demo_data = []
                        self.frame_idx = 0
                    else:
                        print("⏹ 停止数据采集")
                        self.save_data()
                
                # 执行控制命令
                self.execute_commands(commands)
                
                # 采集数据 (如果正在录制)
                if recording:
                    data_frame = self.collect_data()
                    if data_frame:
                        self.demo_data.append(data_frame)
                        self.frame_idx += 1
                        
                        if self.frame_idx % 100 == 0:
                            print(f"已采集 {self.frame_idx} 帧数据")
                
                # 控制循环频率
                elapsed = time.time() - start_time
                if elapsed < self.dt:
                    time.sleep(self.dt - elapsed)
                    
        except KeyboardInterrupt:
            print("\n程序被中断")
        except Exception as e:
            print(f"\n程序错误: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理资源"""
        print("\n清理资源...")
        
        # 停止控制器
        if hasattr(self, 'controller'):
            if self.control_mode == 'keyboard':
                self.controller.stop()
            elif self.control_mode == 'vr':
                self.controller.disconnect()
        
        # 复位夹爪
        if self.gripper:
            try:
                self.gripper.open()
            except:
                pass
        
        # 停止相机
        if self.camera:
            try:
                self.camera.stop()
            except:
                pass
        
        # 关闭机械臂
        if hasattr(self, 'xarm'):
            try:
                self.xarm.close()
            except:
                pass
        
        print("清理完成")

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='XArm数据采集系统')
    parser.add_argument('--mode', choices=['keyboard', 'vr'], default='keyboard',
                      help='控制模式: keyboard 或 vr')
    args = parser.parse_args()
    
    print("XArm数据采集系统")
    print(f"控制模式: {args.mode}")
    
    try:
        collector = XArmDataCollector(control_mode=args.mode)
        collector.initialize_hardware()
        collector.run()
    except Exception as e:
        print(f"系统错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
