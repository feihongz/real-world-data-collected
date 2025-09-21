#!/usr/bin/env python3
"""
真正的Quest手部追踪数据流接口
通过WebSocket接收来自Quest设备的真实手部追踪数据
"""
import asyncio
import websockets
import json
import numpy as np
import time
import threading
from scipy.spatial.transform import Rotation as R

class QuestRealStreamer:
    """真正的Quest手部追踪数据流接口"""
    
    def __init__(self, port=8765):
        """
        初始化Quest数据流
        
        Args:
            port: WebSocket服务器端口
        """
        self.port = port
        self.server = None
        self.connected_clients = set()
        self.latest_hand_data = None
        self.data_lock = threading.Lock()
        self.server_thread = None
        self.running = False
        
        # 默认手部追踪数据
        self.default_data = {
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
        
        print(f"[QuestReal] Quest真实数据流初始化完成")
        
    async def handle_client(self, websocket):
        """处理Quest客户端连接"""
        client_addr = websocket.remote_address
        print(f"[QuestReal] Quest设备连接: {client_addr}")
        self.connected_clients.add(websocket)
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    self._process_quest_data(data)
                    print(f"[QuestReal] 收到Quest数据: {len(self.connected_clients)} 客户端连接")
                except json.JSONDecodeError as e:
                    print(f"[QuestReal] JSON解析错误: {e}")
                except Exception as e:
                    print(f"[QuestReal] 数据处理错误: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            print(f"[QuestReal] Quest设备断开连接: {client_addr}")
        except Exception as e:
            print(f"[QuestReal] 连接错误: {e}")
        finally:
            self.connected_clients.discard(websocket)
            
    def _process_quest_data(self, data):
        """处理接收到的Quest手部追踪数据"""
        with self.data_lock:
            try:
                processed_data = self.default_data.copy()
                processed_data['timestamp'] = time.time()
                
                # 处理左手数据
                if 'leftHand' in data and data['leftHand']:
                    left_data = data['leftHand']
                    if left_data.get('isTracked', False):
                        # 位置数据
                        pos = left_data['position']
                        processed_data['left_hand']['position'] = np.array([
                            float(pos['x']),
                            float(pos['y']),
                            float(pos['z'])
                        ])
                        
                        # 旋转数据 (四元数转旋转矩阵)
                        if 'rotation' in left_data:
                            quat = left_data['rotation']
                            rotation = R.from_quat([
                                float(quat['x']), 
                                float(quat['y']), 
                                float(quat['z']), 
                                float(quat['w'])
                            ])
                            processed_data['left_hand']['rotation'] = rotation.as_matrix()
                        
                        # 构建变换矩阵
                        transform = np.eye(4)
                        transform[:3, :3] = processed_data['left_hand']['rotation']
                        transform[:3, 3] = processed_data['left_hand']['position']
                        processed_data['left_hand']['transform'] = transform
                        
                        # 捏合强度
                        processed_data['left_hand']['pinch_strength'] = float(
                            left_data.get('pinchStrength', 0.0)
                        )
                        processed_data['left_hand']['is_tracked'] = True
                
                # 处理右手数据
                if 'rightHand' in data and data['rightHand']:
                    right_data = data['rightHand']
                    if right_data.get('isTracked', False):
                        # 位置数据
                        pos = right_data['position']
                        processed_data['right_hand']['position'] = np.array([
                            float(pos['x']),
                            float(pos['y']),
                            float(pos['z'])
                        ])
                        
                        # 旋转数据 (四元数转旋转矩阵)
                        if 'rotation' in right_data:
                            quat = right_data['rotation']
                            rotation = R.from_quat([
                                float(quat['x']), 
                                float(quat['y']), 
                                float(quat['z']), 
                                float(quat['w'])
                            ])
                            processed_data['right_hand']['rotation'] = rotation.as_matrix()
                        
                        # 构建变换矩阵
                        transform = np.eye(4)
                        transform[:3, :3] = processed_data['right_hand']['rotation']
                        transform[:3, 3] = processed_data['right_hand']['position']
                        processed_data['right_hand']['transform'] = transform
                        
                        # 捏合强度
                        processed_data['right_hand']['pinch_strength'] = float(
                            right_data.get('pinchStrength', 0.0)
                        )
                        processed_data['right_hand']['is_tracked'] = True
                
                # 更新最新数据
                self.latest_hand_data = processed_data
                
                # 只有在真正接收到追踪数据时才打印
                left_tracked = processed_data['left_hand']['is_tracked']
                right_tracked = processed_data['right_hand']['is_tracked']
                if left_tracked or right_tracked:
                    print(f"[QuestReal] 接收到真实数据 - 左手: {'✅' if left_tracked else '❌'} 右手: {'✅' if right_tracked else '❌'}")
                
            except Exception as e:
                print(f"[QuestReal] 手部数据处理错误: {e}")
    
    async def start_server(self):
        """启动WebSocket服务器"""
        print(f"[QuestReal] 启动WebSocket服务器，端口: {self.port}")
        self.server = await websockets.serve(
            self.handle_client, 
            "0.0.0.0", 
            self.port
        )
        self.running = True
        print(f"[QuestReal] WebSocket服务器运行中，等待Quest连接...")
        
    def start_server_thread(self):
        """在后台线程中启动服务器"""
        def run_server():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            try:
                loop.run_until_complete(self.start_server())
                loop.run_forever()
            except Exception as e:
                print(f"[QuestReal] 服务器错误: {e}")
            finally:
                loop.close()
        
        self.server_thread = threading.Thread(target=run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # 等待服务器启动
        time.sleep(1)
        return True
    
    def stop_server(self):
        """停止服务器"""
        self.running = False
        if self.server:
            self.server.close()
        print("[QuestReal] WebSocket服务器已停止")
    
    def get_latest_data(self):
        """获取最新的手部追踪数据"""
        with self.data_lock:
            if self.latest_hand_data is not None:
                return self.latest_hand_data.copy()
            else:
                return self.default_data.copy()
    
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
        """检查是否有Quest设备连接"""
        return len(self.connected_clients) > 0
    
    def get_connection_count(self):
        """获取连接的Quest设备数量"""
        return len(self.connected_clients)

def main():
    """测试函数"""
    print("=== Quest真实手部追踪数据流测试 ===")
    
    streamer = QuestRealStreamer(port=8765)
    
    print("启动WebSocket服务器...")
    if streamer.start_server_thread():
        print("✅ 服务器启动成功")
        print(f"📱 请在Quest浏览器中访问:")
        print(f"   http://10.3.32.31:8080/quest_webxr_simple.html")
        print("🎮 点击'开始简单追踪'或'进入VR模式'")
        print("📊 观察下方的数据更新...")
        print("\n按 Ctrl+C 停止测试")
        
        try:
            last_update = 0
            while True:
                data = streamer.get_latest_data()
                current_time = data['timestamp']
                
                if current_time > last_update:
                    last_update = current_time
                    
                    print(f"\r[{time.strftime('%H:%M:%S')}] ", end="")
                    
                    # 显示连接状态
                    if streamer.is_connected():
                        print(f"🔗 {streamer.get_connection_count()} Quest连接 | ", end="")
                    else:
                        print("⏳ 等待Quest连接 | ", end="")
                    
                    # 显示手部追踪状态
                    left_status = "✅" if data['left_hand']['is_tracked'] else "❌"
                    right_status = "✅" if data['right_hand']['is_tracked'] else "❌"
                    print(f"左手: {left_status} 右手: {right_status}", end="")
                    
                    # 显示位置信息
                    if data['right_hand']['is_tracked']:
                        pos = data['right_hand']['position']
                        pinch = data['right_hand']['pinch_strength']
                        print(f" | 右手位置: ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) 捏合: {pinch:.2f}", end="")
                    
                    print("", flush=True)
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n\n🛑 停止测试")
        
        streamer.stop_server()
    else:
        print("❌ 服务器启动失败")

if __name__ == "__main__":
    main()
