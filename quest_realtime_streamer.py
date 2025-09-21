#!/usr/bin/env python3
"""
MetaQuest 3S 实时手部追踪数据流
使用WebSocket和WebXR API获取手部追踪数据
"""
import asyncio
import websockets
import json
import numpy as np
import time
import threading
from scipy.spatial.transform import Rotation as R

class QuestRealtimeStreamer:
    """MetaQuest实时手部追踪数据流"""
    
    def __init__(self, port=8765):
        """
        初始化Quest实时数据流
        
        Args:
            port: WebSocket服务器端口
        """
        self.port = port
        self.server = None
        self.connected_clients = set()
        self.latest_data = None
        self.running = False
        
        # 手部追踪数据存储
        self.hand_data = {
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
        
    async def handle_client(self, websocket):
        """处理Quest客户端连接"""
        print(f"[QuestStreamer] 新的Quest客户端连接: {websocket.remote_address}")
        self.connected_clients.add(websocket)
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self._process_hand_data(data)
                except json.JSONDecodeError as e:
                    print(f"[QuestStreamer] JSON解析错误: {e}")
                except Exception as e:
                    print(f"[QuestStreamer] 数据处理错误: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            print(f"[QuestStreamer] Quest客户端断开连接")
        except Exception as e:
            print(f"[QuestStreamer] 连接错误: {e}")
        finally:
            self.connected_clients.remove(websocket)
            
    def _process_hand_data(self, data):
        """处理接收到的手部追踪数据"""
        with self.data_lock:
            try:
                # 解析左手数据
                if 'leftHand' in data:
                    left_data = data['leftHand']
                    if left_data.get('isTracked', False):
                        self.hand_data['left_hand']['position'] = np.array([
                            left_data['position']['x'],
                            left_data['position']['y'],
                            left_data['position']['z']
                        ])
                        
                        # 将四元数转换为旋转矩阵
                        quat = left_data['rotation']
                        rotation = R.from_quat([quat['x'], quat['y'], quat['z'], quat['w']])
                        self.hand_data['left_hand']['rotation'] = rotation.as_matrix()
                        
                        # 构建4x4变换矩阵
                        transform = np.eye(4)
                        transform[:3, :3] = self.hand_data['left_hand']['rotation']
                        transform[:3, 3] = self.hand_data['left_hand']['position']
                        self.hand_data['left_hand']['transform'] = transform
                        
                        self.hand_data['left_hand']['pinch_strength'] = left_data.get('pinchStrength', 0.0)
                        self.hand_data['left_hand']['is_tracked'] = True
                
                # 解析右手数据
                if 'rightHand' in data:
                    right_data = data['rightHand']
                    if right_data.get('isTracked', False):
                        self.hand_data['right_hand']['position'] = np.array([
                            right_data['position']['x'],
                            right_data['position']['y'],
                            right_data['position']['z']
                        ])
                        
                        # 将四元数转换为旋转矩阵
                        quat = right_data['rotation']
                        rotation = R.from_quat([quat['x'], quat['y'], quat['z'], quat['w']])
                        self.hand_data['right_hand']['rotation'] = rotation.as_matrix()
                        
                        # 构建4x4变换矩阵
                        transform = np.eye(4)
                        transform[:3, :3] = self.hand_data['right_hand']['rotation']
                        transform[:3, 3] = self.hand_data['right_hand']['position']
                        self.hand_data['right_hand']['transform'] = transform
                        
                        self.hand_data['right_hand']['pinch_strength'] = right_data.get('pinchStrength', 0.0)
                        self.hand_data['right_hand']['is_tracked'] = True
                
                self.hand_data['timestamp'] = time.time()
                
            except Exception as e:
                print(f"[QuestStreamer] 手部数据解析错误: {e}")
    
    async def start_server(self):
        """启动WebSocket服务器"""
        print(f"[QuestStreamer] 启动WebSocket服务器，端口: {self.port}")
        self.server = await websockets.serve(
            self.handle_client, 
            "0.0.0.0", 
            self.port
        )
        self.running = True
        print(f"[QuestStreamer] WebSocket服务器运行中...")
        
    def start(self):
        """启动数据流服务"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            loop.run_until_complete(self.start_server())
            loop.run_forever()
        except KeyboardInterrupt:
            print("[QuestStreamer] 收到中断信号，正在关闭...")
        finally:
            loop.close()
            
    def get_latest_data(self):
        """获取最新的手部追踪数据"""
        with self.data_lock:
            return self.hand_data.copy()
    
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
    """测试函数"""
    print("=== Quest实时手部追踪测试 ===")
    
    streamer = QuestRealtimeStreamer(port=8765)
    
    print(f"WebSocket服务器启动在端口 8765")
    print("请在Quest设备上访问对应的WebXR应用")
    print("按 Ctrl+C 停止服务器")
    
    try:
        streamer.start()
    except KeyboardInterrupt:
        print("\n测试结束")

if __name__ == "__main__":
    main()
