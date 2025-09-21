# a_minimal_test.py
import time
from xarm.wrapper import XArmAPI

# ================= 配置 =================
ARM_IP = '10.3.32.200'
# ========================================

print("开始极简时序测试...")

try:
    # 1. 连接
    arm = XArmAPI(ARM_IP, is_radian=False)
    print("已连接。")

    # 2. 使能
    arm.motion_enable(enable=True)
    print("已使能。")

    # 3. 明确设置到位置模式
    # 这是最可靠的移动模式
    arm.set_mode(0)
    arm.set_state(0)
    print("已设置为位置模式(Mode 0)。")
    time.sleep(1) # 【超长延时】给足时间稳定

    # 4. 使用最简单的指令移动到目标点
    start_pose = [350, 0, 250, 180, 0, 0]
    print(f"正在移动到 {start_pose}...")
    ret = arm.set_position(*start_pose, speed=30, wait=True)
    if ret != 0:
        print(f"移动失败，错误码: {ret}")
        arm.disconnect()
        exit()
    print("已到达目标位置。")
    time.sleep(1) # 【超长延时】让机械臂在目标点完全静止和稳定

    # 5. 切换到伺服模式
    print("准备切换到伺服模式(Mode 1)...")
    arm.set_mode(1)
    print("模式已设置为 1。")
    time.sleep(0.5) # 【超长延时】给足时间进行模式切换

    # 6. 使能伺服模式
    arm.set_state(0)
    print("伺服模式已使能 (set_state(0))。")


except Exception as e:
    print(f"测试过程中发生异常: {e}")

finally:
    # 8. 安全退出流程
    if 'arm' in locals() and arm.connected:
        print("开始执行安全退出...")
        # 回到位置模式并停止
        arm.set_mode(0)
        arm.set_state(4) 
        print("已设置回位置模式并停止。")
        arm.disconnect()
        print("连接已断开。")
    print("测试结束。")