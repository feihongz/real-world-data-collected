#!/usr/bin/env python3
"""
Quest数据客户端
连接到运行中的Quest WebSocket服务器获取手部追踪数据
"""
import asyncio
import websockets
import json
import numpy as np
import time
import threading
from scipy.spatial.transform import Rotation as R

class QuestDataClient:
    """Quest数据客户端"""
    
    def __init__(self, server_host="localhost", server_port=8765):
        """
        初始化Quest数据客户端
        
        Args:
            server_host: Quest服务器主机地址
            server_port: Quest服务器端口
        """
        self.server_host = server_host
        self.server_port = server_port
        self.websocket = None
        self.connected = False
        self.client_thread = None
        self.running = False
        
        # 手部追踪数据
        self.latest_data = {
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
        
    async def connect_and_listen(self):
        """连接到Quest服务器并监听数据"""
        uri = f"ws://{self.server_host}:{self.server_port}"
        
        try:
            print(f"[QuestClient] 尝试连接到Quest服务器: {uri}")
            
            # 尝试连接
            async with websockets.connect(uri) as websocket:
                self.websocket = websocket
                self.connected = True
                print(f"[QuestClient] ✅ 已连接到Quest服务器")
                
                # 监听数据
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        self._process_quest_data(data)
                    except json.JSONDecodeError as e:
                        print(f"[QuestClient] JSON解析错误: {e}")
                    except Exception as e:
                        print(f"[QuestClient] 数据处理错误: {e}")
                        
        except ConnectionRefusedError:
            print(f"[QuestClient] ❌ 无法连接到Quest服务器 {uri}")
            print(f"[QuestClient] 请确保Quest服务器正在运行")
        except Exception as e:
            print(f"[QuestClient] 连接错误: {e}")
        finally:
            self.connected = False
            self.websocket = None
            print(f"[QuestClient] 与Quest服务器断开连接")
    
    def _process_quest_data(self, data):
        """处理接收到的Quest数据"""
        with self.data_lock:
            try:
                self.latest_data['timestamp'] = time.time()
                
                # 处理左手数据
                if 'leftHand' in data and data['leftHand']:
                    left_data = data['leftHand']
                    if left_data.get('isTracked', False):
                        pos = left_data['position']
                        self.latest_data['left_hand']['position'] = np.array([
                            float(pos['x']), float(pos['y']), float(pos['z'])
                        ])
                        
                        if 'rotation' in left_data:
                            quat = left_data['rotation']
                            rotation = R.from_quat([
                                float(quat['x']), float(quat['y']), 
                                float(quat['z']), float(quat['w'])
                            ])
                            self.latest_data['left_hand']['rotation'] = rotation.as_matrix()
                        
                        # 变换矩阵
                        transform = np.eye(4)
                        transform[:3, :3] = self.latest_data['left_hand']['rotation']
                        transform[:3, 3] = self.latest_data['left_hand']['position']
                        self.latest_data['left_hand']['transform'] = transform
                        
                        self.latest_data['left_hand']['pinch_strength'] = float(
                            left_data.get('pinchStrength', 0.0)
                        )
                        self.latest_data['left_hand']['is_tracked'] = True
                    else:
                        self.latest_data['left_hand']['is_tracked'] = False
                
                # 处理右手数据
                if 'rightHand' in data and data['rightHand']:
                    right_data = data['rightHand']
                    if right_data.get('isTracked', False):
                        pos = right_data['position']
                        self.latest_data['right_hand']['position'] = np.array([
                            float(pos['x']), float(pos['y']), float(pos['z'])
                        ])
                        
                        if 'rotation' in right_data:
                            quat = right_data['rotation']
                            rotation = R.from_quat([
                                float(quat['x']), float(quat['y']), 
                                float(quat['z']), float(quat['w'])
                            ])
                            self.latest_data['right_hand']['rotation'] = rotation.as_matrix()
                        
                        # 变换矩阵
                        transform = np.eye(4)
                        transform[:3, :3] = self.latest_data['right_hand']['rotation']
                        transform[:3, 3] = self.latest_data['right_hand']['position']
                        self.latest_data['right_hand']['transform'] = transform
                        
                        self.latest_data['right_hand']['pinch_strength'] = float(
                            right_data.get('pinchStrength', 0.0)
                        )
                        self.latest_data['right_hand']['is_tracked'] = True
                    else:
                        self.latest_data['right_hand']['is_tracked'] = False
                        
            except Exception as e:
                print(f"[QuestClient] 数据处理错误: {e}")
    
    def start_client(self):
        """启动客户端"""
        if not self.running:
            self.running = True
            
            def run_client():
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                try:
                    loop.run_until_complete(self.connect_and_listen())
                except Exception as e:
                    print(f"[QuestClient] 客户端错误: {e}")
                finally:
                    loop.close()
            
            self.client_thread = threading.Thread(target=run_client)
            self.client_thread.daemon = True
            self.client_thread.start()
            
            # 等待连接建立
            time.sleep(1)
            return self.connected
    
    def stop_client(self):
        """停止客户端"""
        self.running = False
        if self.websocket:
            asyncio.create_task(self.websocket.close())
        if self.client_thread:
            self.client_thread.join(timeout=2)
        print("[QuestClient] 客户端已停止")
    
    def get_latest_data(self):
        """获取最新的手部追踪数据"""
        with self.data_lock:
            return self.latest_data.copy()
    
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
    
    def is_connected(self):
        """检查是否连接"""
        return self.connected

def main():
    """测试函数"""
    print("=== Quest数据客户端测试 ===")
    
    client = QuestDataClient()
    
    if client.start_client():
        print("✅ 客户端启动成功")
        print("📊 监控Quest手部追踪数据...")
        print("按 Ctrl+C 停止测试")
        
        try:
            last_timestamp = 0
            print("等待Quest手部追踪数据...")
            
            while True:
                data = client.get_latest_data()
                
                if data['timestamp'] > last_timestamp:
                    last_timestamp = data['timestamp']
                    
                    # 只有在真正有追踪数据时才显示
                    left_tracked = data['left_hand']['is_tracked']
                    right_tracked = data['right_hand']['is_tracked']
                    
                    if left_tracked or right_tracked:
                        print(f"\r[{time.strftime('%H:%M:%S')}] 🔗 连接正常 | ", end="")
                        
                        if left_tracked:
                            pos = data['left_hand']['position']
                            pinch = data['left_hand']['pinch_strength']
                            print(f"左手: ✅ ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) 捏合:{pinch:.2f} ", end="")
                        
                        if right_tracked:
                            pos = data['right_hand']['position']
                            pinch = data['right_hand']['pinch_strength']
                            print(f"右手: ✅ ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) 捏合:{pinch:.2f} ", end="")
                        
                        print("", flush=True)
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\n🛑 停止测试")
        
        client.stop_client()
    else:
        print("❌ 客户端启动失败")
        print("请确保Quest服务器正在运行")

if __name__ == "__main__":
    main()
