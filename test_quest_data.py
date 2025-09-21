#!/usr/bin/env python3
"""
测试Quest数据读取
不连接机械臂，只验证Quest控制器数据
"""

import sys
import os
import time
import numpy as np

# 添加oculus_reader到路径
sys.path.append('/home/user/Desktop/oculus_reader')
from oculus_reader.reader import OculusReader

class QuestDataTester:
    def __init__(self):
        print("[测试] 初始化Quest数据测试器...")
        
        # 初始化Quest阅读器
        print("[测试] 连接Quest设备...")
        try:
            self.quest_reader = OculusReader(run=True, print_FPS=False)
            print("[测试] Quest连接成功！")
        except Exception as e:
            print(f"[测试] Quest连接失败: {e}")
            raise
        
        # 数据记录
        self.last_position = None
        self.position_history = []
        self.button_history = []
        
    def test_data_reading(self, duration=30):
        """测试数据读取，持续指定时间（秒）"""
        print(f"[测试] 开始数据读取测试，持续{duration}秒...")
        print("[测试] 请移动Quest右手控制器并按下按钮进行测试")
        print("-" * 60)
        
        start_time = time.time()
        last_print_time = start_time
        data_count = 0
        
        while time.time() - start_time < duration:
            try:
                # 获取Quest数据
                transforms, buttons = self.quest_reader.get_transformations_and_buttons()
                
                current_time = time.time()
                
                # 每秒打印一次状态
                if current_time - last_print_time >= 1.0:
                    self._print_status(transforms, buttons, data_count)
                    last_print_time = current_time
                    data_count = 0
                
                # 记录数据
                if transforms and 'r' in transforms:
                    data_count += 1
                    self._record_data(transforms['r'], buttons.get('r', {}) if buttons else {})
                
                time.sleep(0.02)  # 50Hz
                
            except KeyboardInterrupt:
                print("\n[测试] 用户中断测试")
                break
            except Exception as e:
                print(f"[测试] 数据读取错误: {e}")
                time.sleep(0.1)
        
        self._print_summary()
    
    def _record_data(self, transform, button_data):
        """记录数据用于分析"""
        position = transform[:3, 3]
        
        # 记录位置变化
        if self.last_position is not None:
            movement = np.linalg.norm(position - self.last_position)
            self.position_history.append(movement)
        
        self.last_position = position.copy()
        
        # 记录按钮状态
        if button_data:
            self.button_history.append(button_data.copy())
    
    def _print_status(self, transforms, buttons, data_count):
        """打印当前状态"""
        timestamp = time.strftime("%H:%M:%S")
        
        print(f"[{timestamp}] 数据接收率: {data_count}/秒")
        
        if transforms and 'r' in transforms:
            # 右手控制器数据
            right_transform = transforms['r']
            position = right_transform[:3, 3]
            
            # 计算旋转角度（欧拉角）
            rotation_matrix = right_transform[:3, :3]
            # 简单的旋转角度估算
            roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            pitch = np.arctan2(-rotation_matrix[2, 0], 
                             np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
            
            print(f"  右手位置: X={position[0]:.3f}, Y={position[1]:.3f}, Z={position[2]:.3f}")
            print(f"  右手旋转: Roll={np.degrees(roll):.1f}°, Pitch={np.degrees(pitch):.1f}°, Yaw={np.degrees(yaw):.1f}°")
            
            # 计算相对于初始位置的移动
            if hasattr(self, 'initial_position'):
                relative_pos = position - self.initial_position
                print(f"  相对移动: ΔX={relative_pos[0]:.3f}, ΔY={relative_pos[1]:.3f}, ΔZ={relative_pos[2]:.3f}")
            else:
                self.initial_position = position.copy()
                print("  (设置为初始位置)")
        else:
            print("  右手控制器: 未检测到")
        
        if buttons and 'r' in buttons:
            button_data = buttons['r']
            print(f"  按钮状态: {button_data}")
            
            # 详细解析按钮
            if 'trigger' in button_data:
                trigger_state = "按下" if button_data['trigger'] else "释放"
                print(f"    触发器: {trigger_state}")
            
            if 'grip' in button_data:
                grip_state = "按下" if button_data['grip'] else "释放"
                print(f"    握持键: {grip_state}")
                
            if 'menu' in button_data:
                menu_state = "按下" if button_data['menu'] else "释放"
                print(f"    菜单键: {menu_state}")
        else:
            print("  按钮状态: 无数据")
        
        # 检查左手（如果有的话）
        if transforms and 'l' in transforms:
            left_position = transforms['l'][:3, 3]
            print(f"  左手位置: X={left_position[0]:.3f}, Y={left_position[1]:.3f}, Z={left_position[2]:.3f}")
        
        print("-" * 60)
    
    def _print_summary(self):
        """打印测试总结"""
        print("\n" + "="*60)
        print("测试总结")
        print("="*60)
        
        if self.position_history:
            avg_movement = np.mean(self.position_history)
            max_movement = np.max(self.position_history)
            print(f"位置变化统计:")
            print(f"  平均移动距离: {avg_movement:.4f}m")
            print(f"  最大移动距离: {max_movement:.4f}m")
            print(f"  总数据点数: {len(self.position_history)}")
        
        if self.button_history:
            print(f"按钮操作记录: {len(self.button_history)}次")
            
            # 统计按钮按下次数
            button_counts = {}
            for button_data in self.button_history:
                for key, pressed in button_data.items():
                    if pressed:
                        button_counts[key] = button_counts.get(key, 0) + 1
            
            if button_counts:
                print("  按钮按下统计:")
                for button, count in button_counts.items():
                    print(f"    {button}: {count}次")
        
        print("="*60)
    
    def stop(self):
        """停止测试"""
        print("[测试] 停止Quest阅读器...")
        self.quest_reader.stop()
        print("[测试] 测试完成")

def main():
    """主函数"""
    print("Quest数据读取测试")
    print("=" * 40)
    print("这个测试将验证Quest控制器数据读取是否正常")
    print("请确保:")
    print("1. Quest设备已通过USB连接到电脑")
    print("2. Quest设备已开启开发者模式")
    print("3. 已允许USB调试")
    print("=" * 40)
    
    try:
        # 创建测试器
        tester = QuestDataTester()
        
        print("\n请拿起Quest控制器，准备开始测试...")
        input("按回车开始测试...")
        
        # 运行测试
        tester.test_data_reading(duration=30)
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'tester' in locals():
            tester.stop()
        print("\n测试程序退出")

if __name__ == "__main__":
    main()

