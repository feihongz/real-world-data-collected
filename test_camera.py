#!/usr/bin/env python3
"""
测试相机连接和功能
支持RealSense相机和通用USB相机
"""

import cv2
import numpy as np
import time
import sys

def test_realsense_camera():
    """测试RealSense相机"""
    try:
        import pyrealsense2 as rs
        print("[CameraTest] 尝试连接RealSense相机...")
        
        # 配置RealSense管道
        pipeline = rs.pipeline()
        config = rs.config()
        
        # 尝试启动流
        pipeline.start(config)
        print("[CameraTest] RealSense相机连接成功！")
        
        # 获取几帧数据进行测试
        for i in range(10):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if color_frame:
                print(f"[CameraTest] 成功获取第{i+1}帧彩色图像")
                break
        
        pipeline.stop()
        print("[CameraTest] RealSense相机测试完成")
        return True
        
    except Exception as e:
        print(f"[CameraTest] RealSense相机测试失败: {e}")
        return False

def test_usb_camera():
    """测试通用USB相机"""
    print("[CameraTest] 尝试连接USB相机...")
    
    # 尝试不同的相机索引
    for camera_index in range(4):
        try:
            cap = cv2.VideoCapture(camera_index)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    height, width = frame.shape[:2]
                    print(f"[CameraTest] USB相机 {camera_index} 连接成功！")
                    print(f"[CameraTest] 分辨率: {width}x{height}")
                    cap.release()
                    return camera_index
                cap.release()
        except Exception as e:
            print(f"[CameraTest] 相机索引 {camera_index} 测试失败: {e}")
    
    print("[CameraTest] 未找到可用的USB相机")
    return None

def load_reference_image(ref_path="camera_test_frame_14915.jpg"):
    """加载参考图像"""
    try:
        ref_image = cv2.imread(ref_path)
        if ref_image is not None:
            print(f"[CameraTest] 成功加载参考图像: {ref_path}")
            print(f"[CameraTest] 参考图像尺寸: {ref_image.shape[1]}x{ref_image.shape[0]}")
            return ref_image
        else:
            print(f"[CameraTest] 警告: 无法加载参考图像 {ref_path}")
            return None
    except Exception as e:
        print(f"[CameraTest] 加载参考图像失败: {e}")
        return None

def create_overlay_display(current_frame, reference_image, overlay_mode='split'):
    """创建重叠显示"""
    if reference_image is None:
        return current_frame
    
    # 确保两个图像尺寸一致
    h, w = current_frame.shape[:2]
    ref_resized = cv2.resize(reference_image, (w, h))
    
    if overlay_mode == 'split':
        # 左右分屏模式
        display = np.zeros((h, w*2, 3), dtype=np.uint8)
        display[:, :w] = current_frame  # 左侧：当前画面
        display[:, w:] = ref_resized    # 右侧：参考画面
        
        # 添加分割线
        cv2.line(display, (w, 0), (w, h), (255, 255, 255), 2)
        
        # 添加标签
        cv2.putText(display, "Current", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display, "Reference", (w+10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
    elif overlay_mode == 'blend':
        # 半透明叠加模式
        alpha = 0.6  # 当前画面权重
        beta = 0.4   # 参考画面权重
        display = cv2.addWeighted(current_frame, alpha, ref_resized, beta, 0)
        
        # 添加标签
        cv2.putText(display, f"Blend: Current({alpha:.1f}) + Ref({beta:.1f})", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
    elif overlay_mode == 'diff':
        # 差异模式
        gray_current = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        gray_ref = cv2.cvtColor(ref_resized, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(gray_current, gray_ref)
        
        # 转换为彩色显示差异
        display = cv2.applyColorMap(diff, cv2.COLORMAP_JET)
        
        # 添加标签
        cv2.putText(display, "Difference (Blue=Similar, Red=Different)", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    else:  # 'current' - 仅显示当前画面
        display = current_frame.copy()
    
    return display

def test_camera_with_preview(camera_index, reference_path="camera_test_frame_14915.jpg"):
    """测试相机并显示预览，支持参考图像对比"""
    print(f"[CameraTest] 开始相机预览测试 (索引: {camera_index})")
    
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("[CameraTest] 无法打开相机")
        return False
    
    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 加载参考图像
    reference_image = load_reference_image(reference_path)
    
    frame_count = 0
    start_time = time.time()
    overlay_mode = 'split'  # 默认模式
    
    print("[CameraTest] 操作说明:")
    print("  'q' - 退出预览")
    print("  's' - 保存当前帧")
    print("  '1' - 左右分屏模式")
    print("  '2' - 半透明叠加模式") 
    print("  '3' - 差异对比模式")
    print("  '4' - 仅显示当前画面")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[CameraTest] 无法获取帧")
            break
        
        frame_count += 1
        
        # 创建显示画面
        display_frame = create_overlay_display(frame, reference_image, overlay_mode)
        
        # 添加帧计数和模式信息
        cv2.putText(display_frame, f"Frame: {frame_count}", (10, display_frame.shape[0]-60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Mode: {overlay_mode} (1-4 to change)", (10, display_frame.shape[0]-30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 显示帧
        cv2.imshow('Camera Test with Reference Overlay', display_frame)
        
        # 检查按键
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"camera_test_frame_{frame_count}.jpg"
            cv2.imwrite(filename, frame)  # 保存原始画面，不包含叠加效果
            print(f"[CameraTest] 保存图像: {filename}")
        elif key == ord('1'):
            overlay_mode = 'split'
            print("[CameraTest] 切换到左右分屏模式")
        elif key == ord('2'):
            overlay_mode = 'blend'
            print("[CameraTest] 切换到半透明叠加模式")
        elif key == ord('3'):
            overlay_mode = 'diff'
            print("[CameraTest] 切换到差异对比模式")
        elif key == ord('4'):
            overlay_mode = 'current'
            print("[CameraTest] 切换到仅显示当前画面")
    
    # 计算FPS
    elapsed_time = time.time() - start_time
    fps = frame_count / elapsed_time if elapsed_time > 0 else 0
    print(f"[CameraTest] 平均FPS: {fps:.2f}")
    
    cap.release()
    cv2.destroyAllWindows()
    return True

def main():
    print("=== 相机测试程序 ===")
    
    # 1. 测试RealSense相机
    realsense_available = test_realsense_camera()
    
    # 2. 测试USB相机
    usb_camera_index = test_usb_camera()
    
    if not realsense_available and usb_camera_index is None:
        print("[CameraTest] 错误: 未检测到任何可用相机")
        sys.exit(1)
    
    # 3. 选择相机进行预览测试
    if realsense_available:
        print("[CameraTest] 检测到RealSense相机，但我们先测试USB相机模式")
    
    if usb_camera_index is not None:
        print(f"[CameraTest] 使用USB相机索引 {usb_camera_index} 进行预览测试")
        test_camera_with_preview(usb_camera_index)
    else:
        print("[CameraTest] 跳过预览测试")
    
    print("[CameraTest] 相机测试完成")

if __name__ == "__main__":
    main()
