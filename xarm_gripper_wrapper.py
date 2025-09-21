#!/usr/bin/env python3
"""
XArm内置夹爪控制封装
"""
import time

class XArmGripperWrapper:
    """XArm内置夹爪控制类"""
    
    def __init__(self, xarm_api):
        """
        初始化XArm夹爪控制
        
        Args:
            xarm_api: XArmAPI实例
        """
        self.xarm = xarm_api
        self.current_state = 'open'  # 'open' 或 'close'
        
        # 检查夹爪状态
        self._check_gripper_status()
        
    def _check_gripper_status(self):
        """检查夹爪状态"""
        try:
            # 直接尝试使能夹爪
            print("[XArmGripper] 正在使能夹爪...")
            code = self.xarm.set_gripper_enable(True)
            if code == 0:
                print("[XArmGripper] 夹爪使能成功")
            else:
                print(f"[XArmGripper] 夹爪使能失败，错误代码: {code}")
            time.sleep(1)
        except Exception as e:
            print(f"[XArmGripper] 夹爪初始化异常: {e}")
    
    def open(self):
        """打开夹爪"""
        try:
            # XArm夹爪打开 - 通常使用较大的位置值 (例如850)
            code = self.xarm.set_gripper_position(850, wait=False, speed=5000, auto_enable=True)
            if code == 0:
                self.current_state = 'open'
                print("[XArmGripper] 夹爪打开")
            else:
                print(f"[XArmGripper] 夹爪打开失败，错误代码: {code}")
        except Exception as e:
            print(f"[XArmGripper] 夹爪打开异常: {e}")
    
    def close(self):
        """关闭夹爪"""
        try:
            # XArm夹爪关闭 - 通常使用较小的位置值 (例如0)
            code = self.xarm.set_gripper_position(0, wait=False, speed=5000, auto_enable=True)
            if code == 0:
                self.current_state = 'close'
                print("[XArmGripper] 夹爪关闭")
            else:
                print(f"[XArmGripper] 夹爪关闭失败，错误代码: {code}")
        except Exception as e:
            print(f"[XArmGripper] 夹爪关闭异常: {e}")
    
    def set_position(self, position, speed=5000):
        """
        设置夹爪位置
        
        Args:
            position: 夹爪位置 (0-850, 0为完全关闭，850为完全打开)
            speed: 移动速度
        """
        try:
            code = self.xarm.set_gripper_position(position, wait=False, speed=speed, auto_enable=True)
            if code == 0:
                self.current_state = 'close' if position < 400 else 'open'
                print(f"[XArmGripper] 夹爪移动到位置: {position}")
            else:
                print(f"[XArmGripper] 夹爪位置设置失败，错误代码: {code}")
        except Exception as e:
            print(f"[XArmGripper] 设置夹爪位置异常: {e}")
    
    def get_state(self):
        """
        获取夹爪状态
        
        Returns:
            int: 0表示打开，1表示关闭
        """
        return 1 if self.current_state == 'close' else 0
    
    def get_position(self):
        """
        获取当前夹爪位置
        
        Returns:
            float: 当前夹爪位置
        """
        try:
            code, pos = self.xarm.get_gripper_position()
            if code == 0:
                return pos
            else:
                print(f"[XArmGripper] 获取夹爪位置失败，错误代码: {code}")
                return 0
        except Exception as e:
            print(f"[XArmGripper] 获取夹爪位置异常: {e}")
            return 0
    
    def stop(self):
        """停止夹爪运动"""
        try:
            self.xarm.set_gripper_stop()
            print("[XArmGripper] 夹爪停止")
        except Exception as e:
            print(f"[XArmGripper] 夹爪停止异常: {e}")

if __name__ == "__main__":
    # 测试夹爪控制
    from xarm.wrapper import XArmAPI
    
    print("XArm夹爪控制测试")
    print("=" * 30)
    
    try:
        # 连接机械臂
        xarm = XArmAPI('10.3.32.200')
        gripper = XArmGripperWrapper(xarm)
        
        print("测试夹爪控制...")
        
        # 测试打开
        print("1. 打开夹爪...")
        gripper.open()
        time.sleep(2)
        
        # 测试关闭  
        print("2. 关闭夹爪...")
        gripper.close()
        time.sleep(2)
        
        # 测试部分打开
        print("3. 设置夹爪到中间位置...")
        gripper.set_position(400)
        time.sleep(2)
        
        # 获取当前位置
        pos = gripper.get_position()
        print(f"4. 当前夹爪位置: {pos}")
        
        print("夹爪测试完成！")
        
    except Exception as e:
        print(f"测试失败: {e}")
        import traceback
        traceback.print_exc()
