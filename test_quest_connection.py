#!/usr/bin/env python3
"""
测试MetaQuest VR设备连接和手部追踪
"""
import time
import numpy as np
from quest_streamer import QuestStreamer

def test_quest_basic_connection():
    """测试Quest基本连接"""
    print("=" * 50)
    print("MetaQuest VR连接测试")
    print("=" * 50)
    
    try:
        print("1. 初始化Quest数据流...")
        quest = QuestStreamer(record=False)
        quest.connect()
        print("✓ Quest数据流初始化成功")
        
        print("\n2. 测试数据获取...")
        for i in range(5):
            data = quest.latest
            print(f"测试 {i+1}/5:")
            print(f"  - 左手位置: {data['left_wrist'][0][:3, 3]}")
            print(f"  - 右手位置: {data['right_wrist'][0][:3, 3]}")
            print(f"  - 左手捏合: {data['left_pinch_distance']:.3f}")
            print(f"  - 右手捏合: {data['right_pinch_distance']:.3f}")
            print(f"  - 时间戳: {data['timestamp']:.3f}")
            time.sleep(0.5)
        
        print("\n3. 断开连接...")
        quest.disconnect()
        print("✓ 连接测试成功")
        
        return True
        
    except Exception as e:
        print(f"✗ Quest连接测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_quest_hand_tracking():
    """测试手部追踪功能"""
    print("\n" + "=" * 50)
    print("MetaQuest手部追踪测试")
    print("=" * 50)
    
    try:
        print("初始化Quest手部追踪...")
        quest = QuestStreamer(record=False)
        quest.connect()
        
        print("\n请移动你的手，观察追踪数据变化:")
        print("(将运行10秒测试，按Ctrl+C提前结束)")
        
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < 10.0:
            data = quest.latest
            frame_count += 1
            
            # 计算手部移动速度
            left_pos = data['left_wrist'][0][:3, 3]
            right_pos = data['right_wrist'][0][:3, 3]
            
            if frame_count % 10 == 0:  # 每0.5秒打印一次
                print(f"Frame {frame_count:3d}: "
                      f"Left({left_pos[0]:.3f}, {left_pos[1]:.3f}, {left_pos[2]:.3f}) "
                      f"Right({right_pos[0]:.3f}, {right_pos[1]:.3f}, {right_pos[2]:.3f}) "
                      f"Pinch(L:{data['left_pinch_distance']:.3f}, R:{data['right_pinch_distance']:.3f})")
            
            time.sleep(0.05)  # 20Hz
        
        quest.disconnect()
        print(f"\n✓ 手部追踪测试完成，共处理{frame_count}帧")
        return True
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
        try:
            quest.disconnect()
        except:
            pass
        return True
        
    except Exception as e:
        print(f"✗ 手部追踪测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_quest_gesture_detection():
    """测试手势检测"""
    print("\n" + "=" * 50)
    print("MetaQuest手势检测测试")
    print("=" * 50)
    
    try:
        print("初始化手势检测...")
        quest = QuestStreamer(record=False)
        quest.connect()
        
        print("\n手势检测说明:")
        print("- 捏合手指: 拇指和食指靠近")
        print("- 开放手掌: 手指张开")
        print("\n开始检测 (运行10秒)...")
        
        start_time = time.time()
        last_left_gesture = "unknown"
        last_right_gesture = "unknown"
        
        while time.time() - start_time < 10.0:
            data = quest.latest
            
            # 简单的手势检测
            left_pinch = data['left_pinch_distance']
            right_pinch = data['right_pinch_distance']
            
            left_gesture = "pinch" if left_pinch < 0.03 else "open"
            right_gesture = "pinch" if right_pinch < 0.03 else "open"
            
            # 只在手势改变时打印
            if left_gesture != last_left_gesture or right_gesture != last_right_gesture:
                print(f"手势变化: 左手={left_gesture}, 右手={right_gesture} "
                      f"(距离: L:{left_pinch:.3f}, R:{right_pinch:.3f})")
                last_left_gesture = left_gesture
                last_right_gesture = right_gesture
            
            time.sleep(0.1)  # 10Hz
        
        quest.disconnect()
        print("✓ 手势检测测试完成")
        return True
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
        try:
            quest.disconnect()
        except:
            pass
        return True
        
    except Exception as e:
        print(f"✗ 手势检测测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主测试函数"""
    print("MetaQuest 3S 连接和手部追踪测试套件")
    print("注意: 当前使用模拟数据进行测试")
    print("=" * 60)
    
    # 运行基本连接测试
    success1 = test_quest_basic_connection()
    
    if success1:
        # 运行手部追踪测试
        success2 = test_quest_hand_tracking()
        
        if success2:
            # 运行手势检测测试
            success3 = test_quest_gesture_detection()
            
            if success3:
                print("\n" + "=" * 60)
                print("✓ 所有Quest测试通过！")
                print("系统已准备好进行VR遥操作控制")
                print("\n下一步:")
                print("1. 连接真实的MetaQuest 3S设备")
                print("2. 替换模拟数据接口为真实的VR API")
                print("3. 运行完整的遥操作数据采集")
                print("=" * 60)
                return True
    
    print("\n" + "=" * 60)
    print("❌ Quest测试失败，请检查设备连接")
    print("=" * 60)
    return False

if __name__ == "__main__":
    success = main()
    if not success:
        exit(1)


