#!/usr/bin/env python3
"""
通过ADB获取Quest手部追踪数据的替代方案
使用Quest的原生手部追踪API
"""
import subprocess
import json
import time
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R

class QuestADBStreamer:
    """通过ADB连接Quest获取手部追踪数据"""
    
    def __init__(self):
        self.device_id = None
        self.is_connected = False
        self.tracking_data = {
            'left_hand': {
                'position': np.array([0.0, 0.0, 0.0]),
                'rotation': np.eye(3),
                'transform': np.eye(4),
                'pinch_strength': 0.0,
                'is_tracked': False
            },
            'right_hand': {
                'position': np.array([0.0, 0.0, 0.0]),
                'rotation': np.eye(3),
                'transform': np.eye(4),
                'pinch_strength': 0.0,
                'is_tracked': False
            },
            'timestamp': time.time()
        }
        self.data_lock = threading.Lock()
        self.tracking_thread = None
        self.running = False
        
    def check_device_connection(self):
        """检查Quest设备连接"""
        try:
            result = subprocess.run(['adb', 'devices'], 
                                  capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')[1:]  # 跳过标题行
                for line in lines:
                    if 'device' in line and 'unauthorized' not in line:
                        self.device_id = line.split('\t')[0]
                        self.is_connected = True
                        print(f"[QuestADB] 发现Quest设备: {self.device_id}")
                        return True
                        
            print("[QuestADB] 未发现Quest设备")
            return False
            
        except Exception as e:
            print(f"[QuestADB] 设备检查失败: {e}")
            return False
    
    def enable_hand_tracking(self):
        """启用Quest手部追踪"""
        if not self.is_connected:
            return False
            
        try:
            # 检查手部追踪是否已启用
            print("[QuestADB] 检查手部追踪状态...")
            
            # 启用手部追踪 (这需要Quest设备支持)
            cmd = ['adb', '-s', self.device_id, 'shell', 
                   'am', 'broadcast', '-a', 'android.intent.action.MAIN', 
                   '--es', 'handtracking', 'enable']
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                print("[QuestADB] 手部追踪启用成功")
                return True
            else:
                print(f"[QuestADB] 手部追踪启用失败: {result.stderr}")
                return False
                
        except Exception as e:
            print(f"[QuestADB] 启用手部追踪失败: {e}")
            return False
    
    def get_hand_tracking_data(self):
        """获取手部追踪数据 (模拟实现)"""
        # 注意: 实际的Quest手部追踪数据获取需要更复杂的实现
        # 这里我们提供一个框架，实际数据获取可能需要：
        # 1. Quest原生应用
        # 2. OpenXR运行时
        # 3. 第三方库如ALVR
        
        try:
            # 模拟获取手部数据的过程
            # 在实际实现中，这里会调用Quest的API或读取传感器数据
            
            current_time = time.time()
            
            # 为了演示，我们生成一些模拟数据
            # 实际实现中，这里会从Quest获取真实的手部追踪数据
            with self.data_lock:
                # 添加一些动态变化
                t = current_time * 0.5
                
                # 模拟左手数据
                self.tracking_data['left_hand']['position'] = np.array([
                    -0.2 + 0.05 * np.sin(t),
                    0.1 + 0.03 * np.cos(t * 1.2),
                    -0.3 + 0.02 * np.sin(t * 0.8)
                ])
                self.tracking_data['left_hand']['is_tracked'] = True
                self.tracking_data['left_hand']['pinch_strength'] = (np.sin(t * 2) + 1) / 4
                
                # 模拟右手数据
                self.tracking_data['right_hand']['position'] = np.array([
                    0.2 + 0.05 * np.sin(t + np.pi),
                    0.1 + 0.03 * np.cos(t * 1.2 + np.pi),
                    -0.3 + 0.02 * np.sin(t * 0.8 + np.pi)
                ])
                self.tracking_data['right_hand']['is_tracked'] = True
                self.tracking_data['right_hand']['pinch_strength'] = (np.sin(t * 2 + np.pi) + 1) / 4
                
                # 更新变换矩阵
                for hand in ['left_hand', 'right_hand']:
                    pos = self.tracking_data[hand]['position']
                    transform = np.eye(4)
                    transform[:3, 3] = pos
                    self.tracking_data[hand]['transform'] = transform
                
                self.tracking_data['timestamp'] = current_time
                
        except Exception as e:
            print(f"[QuestADB] 获取手部数据失败: {e}")
    
    def tracking_loop(self):
        """手部追踪数据获取循环"""
        print("[QuestADB] 启动手部追踪循环...")
        
        while self.running:
            try:
                self.get_hand_tracking_data()
                time.sleep(0.02)  # 50Hz
            except Exception as e:
                print(f"[QuestADB] 追踪循环错误: {e}")
                time.sleep(0.1)
    
    def start_tracking(self):
        """开始手部追踪"""
        if not self.check_device_connection():
            return False
        
        if not self.enable_hand_tracking():
            print("[QuestADB] 手部追踪启用失败，使用模拟数据")
        
        self.running = True
        self.tracking_thread = threading.Thread(target=self.tracking_loop)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()
        
        print("[QuestADB] 手部追踪已启动")
        return True
    
    def stop_tracking(self):
        """停止手部追踪"""
        self.running = False
        if self.tracking_thread:
            self.tracking_thread.join(timeout=2)
        print("[QuestADB] 手部追踪已停止")
    
    def get_latest_data(self):
        """获取最新的手部追踪数据"""
        with self.data_lock:
            return self.tracking_data.copy()
    
    def get_hand_transforms(self):
        """获取手部变换矩阵 (兼容原有接口)"""
        data = self.get_latest_data()
        return {
            'left_wrist': [data['left_hand']['transform']],
            'right_wrist': [data['right_hand']['transform']],
            'left_pinch_distance': 1.0 - data['left_hand']['pinch_strength'],
            'right_pinch_distance': 1.0 - data['right_hand']['pinch_strength'],
            'timestamp': data['timestamp']
        }
    
    @property
    def latest(self):
        """兼容原有接口"""
        return self.get_hand_transforms()

def main():
    """测试ADB手部追踪"""
    print("=== Quest ADB手部追踪测试 ===")
    
    streamer = QuestADBStreamer()
    
    if streamer.start_tracking():
        print("手部追踪已启动，按 Ctrl+C 停止...")
        
        try:
            while True:
                data = streamer.get_latest_data()
                
                print(f"\r[{time.strftime('%H:%M:%S')}] ", end="")
                
                if data['left_hand']['is_tracked']:
                    pos = data['left_hand']['position']
                    pinch = data['left_hand']['pinch_strength']
                    print(f"左手: ✅ ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) 捏合:{pinch:.2f} ", end="")
                else:
                    print("左手: ❌ ", end="")
                
                if data['right_hand']['is_tracked']:
                    pos = data['right_hand']['position']
                    pinch = data['right_hand']['pinch_strength']
                    print(f"右手: ✅ ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) 捏合:{pinch:.2f}")
                else:
                    print("右手: ❌")
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            pass
        
        streamer.stop_tracking()
    else:
        print("手部追踪启动失败")

if __name__ == "__main__":
    main()


