#!/usr/bin/env python3
"""
简单的FrankaWrapper测试
"""

import time
import numpy as np

def test_basic_creation():
    """测试基本的FrankaWrapper创建"""
    print("=== 测试基本创建 ===")
    
    try:
        # 检查是否可以导入
        print("1. 导入FrankaWrapper...")
        from franka_wrapper import FrankaWrapper
        print("   ✓ 导入成功")
        
        # 检查是否可以创建实例
        print("2. 创建FrankaWrapper实例...")
        joints_init = (0.0845, -0.5603, -0.1064, -2.0000, -0.0475, 1.4359, 0.0)
        franka = FrankaWrapper(joints_init)
        print("   ✓ 创建成功")
        
        # 等待一段时间
        print("3. 等待启动完成...")
        time.sleep(5.0)
        
        # 检查是否存活
        print("4. 检查进程状态...")
        if franka.is_alive():
            print("   ✓ 进程存活")
        else:
            print("   ✗ 进程未存活")
        
        # 清理
        print("5. 清理资源...")
        del franka
        print("   ✓ 清理完成")
        
        return True
        
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_basic_creation()
    if success:
        print("🎉 基本测试成功！")
    else:
        print("❌ 基本测试失败！") 