#!/usr/bin/env python3
"""
简单的Quest数据监控脚本
连接到运行中的WebSocket服务器来查看手部追踪数据
"""
import asyncio
import websockets
import json
import time

async def monitor_quest_data():
    """监控Quest手部追踪数据"""
    print("=== Quest手部追踪数据监控 ===")
    print("尝试连接到WebSocket服务器...")
    
    try:
        # 连接到运行中的WebSocket服务器
        uri = "ws://localhost:8765"
        async with websockets.connect(uri) as websocket:
            print("✅ 已连接到WebSocket服务器")
            print("等待Quest手部追踪数据...\n")
            
            last_message_time = 0
            no_data_count = 0
            
            while True:
                try:
                    # 等待消息，设置超时
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    data = json.loads(message)
                    
                    current_time = time.time()
                    last_message_time = current_time
                    no_data_count = 0
                    
                    # 解析并显示数据
                    print(f"\r[{time.strftime('%H:%M:%S')}] ", end="")
                    
                    # 检查左手
                    if data.get('leftHand') and data['leftHand'].get('isTracked'):
                        pos = data['leftHand']['position']
                        pinch = data['leftHand'].get('pinchStrength', 0)
                        print(f"左手: ✅ 位置({pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f}) 捏合:{pinch:.2f} ", end="")
                    else:
                        print("左手: ❌ ", end="")
                    
                    # 检查右手
                    if data.get('rightHand') and data['rightHand'].get('isTracked'):
                        pos = data['rightHand']['position']
                        pinch = data['rightHand'].get('pinchStrength', 0)
                        print(f"右手: ✅ 位置({pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f}) 捏合:{pinch:.2f}")
                    else:
                        print("右手: ❌")
                        
                except asyncio.TimeoutError:
                    # 超时，检查是否长时间无数据
                    no_data_count += 1
                    if no_data_count > 5:  # 5秒无数据
                        print(f"\r[{time.strftime('%H:%M:%S')}] 等待Quest手部追踪数据... ", end="", flush=True)
                        no_data_count = 0
                except json.JSONDecodeError as e:
                    print(f"\nJSON解析错误: {e}")
                except Exception as e:
                    print(f"\n数据处理错误: {e}")
                    
    except ConnectionRefusedError:
        print("❌ 无法连接到WebSocket服务器")
        print("请确保Quest服务器正在运行")
    except Exception as e:
        print(f"❌ 连接错误: {e}")

def main():
    try:
        asyncio.run(monitor_quest_data())
    except KeyboardInterrupt:
        print("\n\n📊 数据监控结束")

if __name__ == "__main__":
    main()


