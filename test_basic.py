#!/usr/bin/env python3
"""
基本功能测试
"""

import time
import numpy as np

def test_imports():
    """测试导入功能"""
    print("=== 测试导入功能 ===")
    
    try:
        print("1. 导入基本模块...")
        import numpy as np
        import time
        print("   ✓ 基本模块导入成功")
        
        print("2. 导入机器人模块...")
        from xarm_wrapper import XArmWrapper
        print("   ✓ XArm模块导入成功")
        
        print("3. 导入夹爪模块...")
        from robotiq_wrapper import RobotiqWrapper
        print("   ✓ Robotiq模块导入成功")
        
        print("4. 导入Franka模块...")
        from franka_wrapper import FrankaWrapper
        print("   ✓ Franka模块导入成功")
        
        return True
        
    except Exception as e:
        print(f"导入失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_robotiq_wrapper():
    """测试RobotiqWrapper"""
    print("\n=== 测试RobotiqWrapper ===")
    
    try:
        print("1. 创建XArm夹爪...")
        xarm_gripper = RobotiqWrapper(robot='xarm')
        print("   ✓ XArm夹爪创建成功")
        
        print("2. 创建Franka夹爪...")
        franka_gripper = RobotiqWrapper(robot='franka')
        print("   ✓ Franka夹爪创建成功")
        
        return True
        
    except Exception as e:
        print(f"夹爪测试失败: {e}")
        return False

if __name__ == "__main__":
    print("开始基本功能测试...")
    
    success1 = test_imports()
    success2 = test_robotiq_wrapper()
    
    if success1 and success2:
        print("\n🎉 所有基本测试通过！")
    else:
        print("\n❌ 部分测试失败！") 