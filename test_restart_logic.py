#!/usr/bin/env python3
"""
测试FrankaWrapper的重启功能
"""

import time
import numpy as np
from franka_wrapper import FrankaWrapper

def test_franka_restart():
    """测试Franka重启功能"""
    print("=== 测试Franka重启功能 ===")
    
    # 初始化Franka
    joints_init = (0.0845, -0.5603, -0.1064, -2.0000, -0.0475, 1.4359, 0.0)
    
    try:
        print("1. 创建FrankaWrapper...")
        franka = FrankaWrapper(joints_init)
        print("   ✓ FrankaWrapper创建成功")
        
        # 等待一段时间确保启动完成
        time.sleep(3.0)
        
        print("2. 检查Franka是否存活...")
        if franka.is_alive():
            print("   ✓ Franka进程存活")
        else:
            print("   ✗ Franka进程未存活")
            return False
        
        print("3. 获取TCP位姿...")
        try:
            tcp_pose = franka.get_tcp_pose()
            print(f"   ✓ TCP位姿: {tcp_pose}")
        except Exception as e:
            print(f"   ✗ 获取TCP位姿失败: {e}")
            return False
        
        print("4. 测试重启功能...")
        try:
            franka.restart()
            print("   ✓ Franka重启成功")
        except Exception as e:
            print(f"   ✗ Franka重启失败: {e}")
            return False
        
        # 等待重启完成
        time.sleep(3.0)
        
        print("5. 检查重启后是否存活...")
        if franka.is_alive():
            print("   ✓ 重启后Franka进程存活")
        else:
            print("   ✗ 重启后Franka进程未存活")
            return False
        
        print("6. 获取重启后的TCP位姿...")
        try:
            tcp_pose_after = franka.get_tcp_pose()
            print(f"   ✓ 重启后TCP位姿: {tcp_pose_after}")
        except Exception as e:
            print(f"   ✗ 获取重启后TCP位姿失败: {e}")
            return False
        
        print("7. 测试第二次重启...")
        try:
            franka.restart()
            print("   ✓ 第二次重启成功")
        except Exception as e:
            print(f"   ✗ 第二次重启失败: {e}")
            return False
        
        print("=== 所有测试通过 ===")
        return True
        
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        return False
    
    finally:
        # 清理资源
        try:
            if 'franka' in locals():
                del franka
        except:
            pass

if __name__ == "__main__":
    success = test_franka_restart()
    if success:
        print("🎉 重启功能测试成功！")
    else:
        print("❌ 重启功能测试失败！") 