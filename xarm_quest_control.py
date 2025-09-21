#!/usr/bin/env python3
"""
XArm机械臂 + Quest手部追踪控制系统
完整的VR遥操作数据采集pipeline
"""
import time
import numpy as np
import threading
import argparse
from scipy.spatial.transform import Rotation as R

from xarm_wrapper import XArmWrapper
from xarm_gripper_wrapper import XArmGripperWrapper
from quest_adb_streamer import QuestADBStreamer
from realsense import RealSense

class XArmQuestControl:
    """XArm + Quest VR遥操作控制系统"""
    
    def __init__(self, use_camera=True, use_gripper=True, record_data=True):
        """
        初始化控制系统
        
        Args:
            use_camera: 是否使用RealSense相机
            use_gripper: 是否使用夹爪
            record_data: 是否录制数据
        """
        self.use_camera = use_camera
        self.use_gripper = use_gripper
        self.record_data = record_data
        
        # 控制参数
        self.control_active = False
        self.control_thread = None
        self.data_lock = threading.Lock()
        
        # 缩放因子 (将Quest坐标映射到机械臂工作空间)
        self.position_scale = 0.5  # Quest的1米对应机械臂0.5米
        self.workspace_center = np.array([0.3, 0.0, 0.2])  # 机械臂工作空间中心
        
        # 安全限制
        self.max_velocity = 0.1  # m/s
        self.workspace_limits = {
            'x': [0.1, 0.6],
            'y': [-0.3, 0.3], 
            'z': [0.05, 0.4]
        }
        
        # 数据录制
        self.recorded_data = []
        
        print("=== XArm Quest控制系统初始化 ===")
        
    def initialize_components(self):
        """初始化所有组件"""
        success = True
        
        # 1. 初始化XArm机械臂
        print("1. 初始化XArm机械臂...")
        try:
            self.xarm = XArmWrapper(joints_init=None, move_to_init=False)
            print("✅ XArm机械臂初始化成功")
        except Exception as e:
            print(f"❌ XArm初始化失败: {e}")
            success = False
            
        # 2. 初始化夹爪
        if self.use_gripper:
            print("2. 初始化XArm夹爪...")
            try:
                self.gripper = XArmGripperWrapper(self.xarm.xarm)
                print("✅ XArm夹爪初始化成功")
            except Exception as e:
                print(f"❌ 夹爪初始化失败: {e}")
                self.use_gripper = False
        
        # 3. 初始化Quest手部追踪
        print("3. 初始化Quest手部追踪...")
        try:
            self.quest = QuestADBStreamer()
            if self.quest.start_tracking():
                print("✅ Quest手部追踪初始化成功")
            else:
                print("❌ Quest手部追踪初始化失败")
                success = False
        except Exception as e:
            print(f"❌ Quest初始化失败: {e}")
            success = False
            
        # 4. 初始化相机 (可选)
        if self.use_camera:
            print("4. 初始化RealSense相机...")
            try:
                self.camera = RealSense()
                print("✅ RealSense相机初始化成功")
            except Exception as e:
                print(f"⚠️ 相机初始化失败: {e}")
                self.use_camera = False
                
        return success
    
    def quest_to_robot_position(self, quest_pos):
        """将Quest坐标转换为机械臂坐标"""
        # Quest坐标系: X(左右), Y(上下), Z(前后)
        # 机械臂坐标系: X(前后), Y(左右), Z(上下)
        
        # 坐标变换和缩放
        robot_pos = np.array([
            -quest_pos[2] * self.position_scale,  # Quest -Z -> Robot X (前后)
            -quest_pos[0] * self.position_scale,  # Quest -X -> Robot Y (左右)
            quest_pos[1] * self.position_scale    # Quest Y -> Robot Z (上下)
        ])
        
        # 相对于工作空间中心
        robot_pos += self.workspace_center
        
        # 应用安全限制
        robot_pos[0] = np.clip(robot_pos[0], *self.workspace_limits['x'])
        robot_pos[1] = np.clip(robot_pos[1], *self.workspace_limits['y'])
        robot_pos[2] = np.clip(robot_pos[2], *self.workspace_limits['z'])
        
        return robot_pos
    
    def control_loop(self):
        """主控制循环"""
        print("🎮 开始VR遥操作控制循环...")
        
        last_position = None
        last_time = time.time()
        
        while self.control_active:
            try:
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time
                
                # 获取Quest手部追踪数据
                quest_data = self.quest.get_latest_data()
                
                # 使用右手控制机械臂 (可以改为左手)
                if quest_data['right_hand']['is_tracked']:
                    # 获取右手位置
                    quest_pos = quest_data['right_hand']['position']
                    robot_pos = self.quest_to_robot_position(quest_pos)
                    
                    # 速度限制
                    if last_position is not None:
                        velocity = np.linalg.norm(robot_pos - last_position) / dt
                        if velocity > self.max_velocity:
                            # 限制移动距离
                            direction = (robot_pos - last_position) / np.linalg.norm(robot_pos - last_position)
                            robot_pos = last_position + direction * self.max_velocity * dt
                    
                    # 发送位置指令到机械臂
                    target_pose = np.concatenate([robot_pos, [0, 0, 0]])  # 位置 + 固定姿态
                    self.xarm.set_servo_cartesian(target_pose)
                    
                    last_position = robot_pos.copy()
                    
                    # 夹爪控制 (基于捏合手势)
                    if self.use_gripper:
                        pinch_strength = quest_data['right_hand']['pinch_strength']
                        if pinch_strength > 0.7:  # 强捏合 -> 关闭夹爪
                            self.gripper.close()
                        elif pinch_strength < 0.3:  # 松开 -> 打开夹爪
                            self.gripper.open()
                    
                    # 数据录制
                    if self.record_data:
                        data_point = {
                            'timestamp': current_time,
                            'quest_position': quest_pos.tolist(),
                            'robot_position': robot_pos.tolist(),
                            'pinch_strength': pinch_strength,
                            'gripper_state': 'closed' if pinch_strength > 0.7 else 'open'
                        }
                        
                        # 相机数据 (如果可用)
                        if self.use_camera:
                            try:
                                camera_data = self.camera.get_data()
                                data_point['camera'] = {
                                    'color_shape': camera_data['color'].shape,
                                    'depth_shape': camera_data['depth'].shape
                                }
                            except:
                                pass
                        
                        with self.data_lock:
                            self.recorded_data.append(data_point)
                    
                    # 显示状态
                    print(f"\r[{time.strftime('%H:%M:%S')}] "
                          f"Quest: ({quest_pos[0]:.2f},{quest_pos[1]:.2f},{quest_pos[2]:.2f}) "
                          f"-> Robot: ({robot_pos[0]:.2f},{robot_pos[1]:.2f},{robot_pos[2]:.2f}) "
                          f"捏合: {pinch_strength:.2f}", end="", flush=True)
                
                else:
                    print(f"\r[{time.strftime('%H:%M:%S')}] 等待右手追踪... ", end="", flush=True)
                
                time.sleep(0.02)  # 50Hz控制频率
                
            except Exception as e:
                print(f"\n控制循环错误: {e}")
                time.sleep(0.1)
    
    def start_control(self):
        """开始控制"""
        if not self.control_active:
            self.control_active = True
            self.control_thread = threading.Thread(target=self.control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            print("🚀 VR遥操作控制已启动")
            return True
        return False
    
    def stop_control(self):
        """停止控制"""
        if self.control_active:
            self.control_active = False
            if self.control_thread:
                self.control_thread.join(timeout=2)
            print("\n🛑 VR遥操作控制已停止")
    
    def save_recorded_data(self, filename=None):
        """保存录制的数据"""
        if not self.record_data or not self.recorded_data:
            print("没有数据可保存")
            return
            
        if filename is None:
            filename = f"xarm_quest_data_{time.strftime('%Y%m%d_%H%M%S')}.json"
        
        import json
        with self.data_lock:
            with open(filename, 'w') as f:
                json.dump(self.recorded_data, f, indent=2)
        
        print(f"📊 数据已保存到: {filename} ({len(self.recorded_data)} 个数据点)")
    
    def cleanup(self):
        """清理资源"""
        self.stop_control()
        
        if hasattr(self, 'quest'):
            self.quest.stop_tracking()
        
        if hasattr(self, 'camera') and self.use_camera:
            try:
                self.camera.stop()
            except:
                pass
        
        print("🧹 系统清理完成")

def main():
    parser = argparse.ArgumentParser(description='XArm Quest VR遥操作控制')
    parser.add_argument('--no-camera', action='store_true', help='不使用相机')
    parser.add_argument('--no-gripper', action='store_true', help='不使用夹爪')
    parser.add_argument('--no-record', action='store_true', help='不录制数据')
    
    args = parser.parse_args()
    
    # 创建控制系统
    control_system = XArmQuestControl(
        use_camera=not args.no_camera,
        use_gripper=not args.no_gripper,
        record_data=not args.no_record
    )
    
    try:
        # 初始化所有组件
        if control_system.initialize_components():
            print("\n🎯 所有组件初始化成功!")
            print("\n📋 控制说明:")
            print("- 将右手放在Quest摄像头视野内")
            print("- 移动右手控制机械臂末端")
            print("- 捏合手指控制夹爪开关")
            print("- 按 Ctrl+C 停止控制")
            
            # 启动控制
            control_system.start_control()
            
            # 等待用户中断
            while True:
                time.sleep(1)
                
        else:
            print("❌ 系统初始化失败")
            
    except KeyboardInterrupt:
        print("\n\n收到中断信号...")
    
    finally:
        # 清理
        control_system.cleanup()
        
        # 保存数据
        if control_system.record_data:
            control_system.save_recorded_data()
        
        print("✅ 程序正常退出")

if __name__ == "__main__":
    main()


