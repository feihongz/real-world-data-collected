#!/usr/bin/env python3
"""
安全测试：只测试相机功能，不连接机械臂
"""

import cv2
import numpy as np
import time

def test_camera_safe(camera_index=0):
    """安全的相机测试"""
    print(f"[SafeTest] 测试相机索引: {camera_index}")
    
    try:
        # 初始化相机
        camera = cv2.VideoCapture(camera_index)
        if not camera.isOpened():
            print("[SafeTest] 无法打开相机")
            return False
        
        # 设置分辨率
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        print("[SafeTest] 相机初始化成功")
        print("[SafeTest] 按 'q' 退出，按 's' 保存图像")
        
        frame_count = 0
        
        while True:
            ret, frame = camera.read()
            if not ret:
                print("[SafeTest] 无法获取帧")
                break
            
            frame_count += 1
            
            # 转换BGR到RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 添加信息
            display_frame = frame.copy()
            cv2.putText(display_frame, f"Frame: {frame_count}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_frame, f"RGB Shape: {rgb_frame.shape}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(display_frame, "Press 'q' to quit, 's' to save", (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示
            cv2.imshow('Camera Test (Safe)', display_frame)
            
            # 检查按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                filename = f"test_rgb_frame_{frame_count}.jpg"
                # 保存RGB版本
                rgb_bgr = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                cv2.imwrite(filename, rgb_bgr)
                print(f"[SafeTest] 保存图像: {filename}")
        
        camera.release()
        cv2.destroyAllWindows()
        
        print(f"[SafeTest] 相机测试完成，总帧数: {frame_count}")
        return True
        
    except Exception as e:
        print(f"[SafeTest] 相机测试失败: {e}")
        return False

def test_data_structure():
    """测试数据结构"""
    print("[SafeTest] 测试数据结构...")
    
    # 模拟机械臂状态
    fake_robot_state = np.array([0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 0.5], dtype=np.float32)
    print(f"机械臂状态形状: {fake_robot_state.shape}")
    print(f"机械臂状态: {fake_robot_state}")
    
    # 模拟图像
    fake_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    print(f"图像形状: {fake_image.shape}")
    
    # 模拟动作
    fake_action = np.array([0.31, 0.01, 0.41, 0.1, 0.0, 0.0, 0.6], dtype=np.float32)
    print(f"动作形状: {fake_action.shape}")
    print(f"动作: {fake_action}")
    
    # 模拟Quest数据
    fake_quest_data = {
        'transforms': {
            'r': {
                'position': [10.0, 20.0, 30.0],
                'rotation': [0.0, 0.0, 0.0, 1.0]
            }
        },
        'buttons': {
            'RTr': True,
            'A': False
        }
    }
    print(f"Quest数据: {fake_quest_data}")
    
    print("[SafeTest] 数据结构测试完成")

def main():
    print("=== 安全相机测试 ===")
    
    # 1. 测试数据结构
    test_data_structure()
    print()
    
    # 2. 测试相机
    camera_works = test_camera_safe()
    
    if camera_works:
        print("[SafeTest] ✅ 相机测试通过")
    else:
        print("[SafeTest] ❌ 相机测试失败")

if __name__ == "__main__":
    main()
