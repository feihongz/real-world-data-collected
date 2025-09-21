#!/usr/bin/env python3
"""
键盘控制XArm机械臂测试程序
用于验证机械臂控制逻辑和准备VR集成
"""
import time
import numpy as np
import threading
import sys
import termios
import tty
from scipy.spatial.transform import Rotation as R

from xarm_wrapper import XArmWrapper
from xarm_gripper_wrapper import XArmGripperWrapper

class KeyboardController:
    """键盘输入控制器"""
    
    def __init__(self):
        self.running = True
        self.key_pressed = None
        self.key_lock = threading.Lock()
        
        # 保存终端设置
        self.old_settings = termios.tcgetattr(sys.stdin)
        
    def start(self):
        """启动键盘监听线程"""
        self.thread = threading.Thread(target=self._keyboard_listener)
        self.thread.daemon = True
        self.thread.start()
        
    def _keyboard_listener(self):
        """键盘监听线程"""
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
        """获取按键"""
        with self.key_lock:
            key = self.key_pressed
            self.key_pressed = None
            return key
    
    def stop(self):
        """停止键盘监听"""
        self.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def print_controls():
    """打印控制说明"""
    print("\n" + "="*60)
    print("键盘控制说明:")
    print("="*60)
    print("位置控制:")
    print("  W/S - X轴前进/后退")
    print("  A/D - Y轴左移/右移") 
    print("  Q/E - Z轴上升/下降")
    print("")
    print("姿态控制:")
    print("  I/K - 绕X轴旋转 (俯仰)")
    print("  J/L - 绕Y轴旋转 (偏航)")
    print("  U/O - 绕Z轴旋转 (滚动)")
    print("")
    print("夹爪控制:")
    print("  G - 打开夹爪")
    print("  H - 关闭夹爪")
    print("")
    print("其他:")
    print("  R - 重置到初始位置")
    print("  ESC - 退出程序")
    print("="*60)

def main():
    """主控制函数"""
    print("XArm键盘控制测试程序")
    print("="*40)
    
    # 初始化机械臂 (不移动到预设位置，保持当前状态)
    print("1. 初始化XArm机械臂...")
    try:
        # 使用None作为joints_init，表示不移动到特定位置
        xarm = XArmWrapper(joints_init=None, move_to_init=False)
        print("✓ XArm初始化成功")
    except Exception as e:
        print(f"✗ XArm初始化失败: {e}")
        return
    
    # 初始化XArm内置夹爪
    print("2. 初始化XArm内置夹爪...")
    try:
        gripper = XArmGripperWrapper(xarm.xarm)  # 传递XArmAPI实例
        print("✓ XArm夹爪初始化成功")
    except Exception as e:
        print(f"⚠ 夹爪初始化失败: {e}")
        print("  程序将继续运行，但夹爪控制不可用")
        gripper = None
    
    # 获取初始位置
    print("3. 获取当前位置...")
    try:
        current_pose = xarm.get_position()
        print(f"✓ 当前位置: {current_pose[:3]}")
        print(f"✓ 当前姿态: {np.degrees(current_pose[3:])}")
    except Exception as e:
        print(f"✗ 获取位置失败: {e}")
        return
    
    # 设置控制参数
    position_step = 0.01  # 1cm per step
    rotation_step = np.radians(5)  # 5度 per step
    control_frequency = 20  # Hz
    dt = 1.0 / control_frequency
    
    # 初始化键盘控制
    keyboard = KeyboardController()
    keyboard.start()
    
    print_controls()
    print(f"\n当前控制参数:")
    print(f"  位置步长: {position_step*1000:.0f}mm")
    print(f"  旋转步长: {np.degrees(rotation_step):.0f}°")
    print(f"  控制频率: {control_frequency}Hz")
    print("\n开始键盘控制...")
    
    # 保存初始位置
    initial_pose = current_pose.copy()
    target_pose = current_pose.copy()
    
    try:
        while True:
            start_time = time.time()
            
            # 获取键盘输入
            key = keyboard.get_key()
            pose_changed = False
            
            if key:
                if key == '\x1b':  # ESC键
                    print("\n退出程序...")
                    break
                elif key.lower() == 'r':  # 重置
                    print("重置到初始位置...")
                    target_pose = initial_pose.copy()
                    pose_changed = True
                elif key.lower() == 'w':  # X+
                    target_pose[0] += position_step
                    pose_changed = True
                    print(f"X+ -> {target_pose[0]:.3f}")
                elif key.lower() == 's':  # X-
                    target_pose[0] -= position_step
                    pose_changed = True
                    print(f"X- -> {target_pose[0]:.3f}")
                elif key.lower() == 'd':  # Y+
                    target_pose[1] += position_step
                    pose_changed = True
                    print(f"Y+ -> {target_pose[1]:.3f}")
                elif key.lower() == 'a':  # Y-
                    target_pose[1] -= position_step
                    pose_changed = True
                    print(f"Y- -> {target_pose[1]:.3f}")
                elif key.lower() == 'q':  # Z+
                    target_pose[2] += position_step
                    pose_changed = True
                    print(f"Z+ -> {target_pose[2]:.3f}")
                elif key.lower() == 'e':  # Z-
                    target_pose[2] -= position_step
                    pose_changed = True
                    print(f"Z- -> {target_pose[2]:.3f}")
                elif key.lower() == 'i':  # RX+
                    target_pose[3] += rotation_step
                    pose_changed = True
                    print(f"RX+ -> {np.degrees(target_pose[3]):.1f}°")
                elif key.lower() == 'k':  # RX-
                    target_pose[3] -= rotation_step
                    pose_changed = True
                    print(f"RX- -> {np.degrees(target_pose[3]):.1f}°")
                elif key.lower() == 'j':  # RY+
                    target_pose[4] += rotation_step
                    pose_changed = True
                    print(f"RY+ -> {np.degrees(target_pose[4]):.1f}°")
                elif key.lower() == 'l':  # RY-
                    target_pose[4] -= rotation_step
                    pose_changed = True
                    print(f"RY- -> {np.degrees(target_pose[4]):.1f}°")
                elif key.lower() == 'u':  # RZ+
                    target_pose[5] += rotation_step
                    pose_changed = True
                    print(f"RZ+ -> {np.degrees(target_pose[5]):.1f}°")
                elif key.lower() == 'o':  # RZ-
                    target_pose[5] -= rotation_step
                    pose_changed = True
                    print(f"RZ- -> {np.degrees(target_pose[5]):.1f}°")
                elif key.lower() == 'g':  # 打开夹爪
                    if gripper:
                        gripper.open()
                        print("夹爪打开")
                    else:
                        print("夹爪不可用")
                elif key.lower() == 'h':  # 关闭夹爪
                    if gripper:
                        gripper.close()
                        print("夹爪关闭")
                    else:
                        print("夹爪不可用")
            
            # 如果位置改变了，发送控制命令
            if pose_changed:
                try:
                    # 转换为机械臂格式 (mm, degrees)
                    target_mm = target_pose.copy()
                    target_mm[:3] *= 1000  # m -> mm
                    target_mm[3:] = np.degrees(target_mm[3:])  # rad -> deg
                    
                    # 发送控制命令
                    xarm.set_servo_cartesian(target_mm)
                    
                except Exception as e:
                    print(f"控制错误: {e}")
            
            # 控制循环频率
            elapsed = time.time() - start_time
            if elapsed < dt:
                time.sleep(dt - elapsed)
                
    except KeyboardInterrupt:
        print("\n程序被中断")
    except Exception as e:
        print(f"\n程序错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        print("\n清理资源...")
        keyboard.stop()
        if gripper:
            try:
                gripper.open()
            except:
                pass
        try:
            xarm.close()
        except:
            pass
        print("程序结束")

if __name__ == "__main__":
    main()
