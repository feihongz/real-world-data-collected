#!/usr/bin/env python3
"""
测试XArm机械臂连接
"""
import time
import numpy as np
from xarm_wrapper import XArmWrapper

def test_xarm_connection():
    """测试XArm连接和基本功能"""
    print("=" * 50)
    print("XArm机械臂连接测试")
    print("=" * 50)
    
    try:
        print("1. 创建XArm连接...")
        # 连接机械臂但不移动到预设位置（更安全）
        xarm = XArmWrapper(joints_init=None, ip='10.3.32.200', move_to_init=False)
        print("✓ XArm连接成功")
        
        print("\n2. 等待机械臂初始化...")
        time.sleep(3)
        
        print("\n3. 获取当前状态...")
        try:
            joint_angles = xarm.get_joint()
            print(f"✓ 关节角度: {np.degrees(joint_angles)}")
            
            position = xarm.get_position()
            print(f"✓ 末端位置: {position[:3]} (m)")
            print(f"✓ 末端姿态: {np.degrees(position[3:])} (deg)")
            
        except Exception as e:
            print(f"✗ 获取状态失败: {e}")
            return False
        
        print("\n4. 测试小幅度运动...")
        try:
            # 获取当前位置
            current_pos = xarm.get_position()
            print(f"当前位置: {current_pos}")
            
            # 创建一个小的位移 (5mm in X direction)
            target_pos = current_pos.copy()
            target_pos[0] += 5  # 5mm in X
            target_pos_mm = target_pos.copy()
            target_pos_mm[:3] *= 1000  # convert to mm
            target_pos_mm[3:] = np.degrees(target_pos_mm[3:])  # convert to degrees
            
            print(f"目标位置: {target_pos}")
            print("执行小幅度运动...")
            
            xarm.set_servo_cartesian(target_pos_mm)
            time.sleep(2)
            
            # 回到原位置
            original_pos_mm = current_pos.copy()
            original_pos_mm[:3] *= 1000  # convert to mm
            original_pos_mm[3:] = np.degrees(original_pos_mm[3:])  # convert to degrees
            
            print("返回原位置...")
            xarm.set_servo_cartesian(original_pos_mm)
            time.sleep(2)
            
            print("✓ 运动测试成功")
            
        except Exception as e:
            print(f"✗ 运动测试失败: {e}")
            return False
        
        print("\n5. 清理资源...")
        xarm.close()
        print("✓ 资源清理完成")
        
        print("\n" + "=" * 50)
        print("✓ XArm连接测试成功！")
        print("机械臂已准备好进行遥操作控制")
        print("=" * 50)
        return True
        
    except Exception as e:
        print(f"\n✗ XArm连接测试失败: {e}")
        print("\n故障排除建议:")
        print("1. 检查XArm机械臂是否开机")
        print("2. 检查网络连接，确保IP地址 10.3.32.200 可达")
        print("3. 检查XArm控制箱的网络设置")
        print("4. 确保xarm-python-sdk安装正确")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_xarm_connection()
    if not success:
        exit(1)
