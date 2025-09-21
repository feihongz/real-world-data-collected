#!/usr/bin/env python3
"""
测试Quest实时手部追踪连接
"""
import time
import asyncio
import json
from quest_realtime_streamer import QuestRealtimeStreamer

async def test_quest_connection():
    """测试Quest实时连接"""
    print("=== Quest实时手部追踪测试 ===")
    
    # 创建实时数据流
    streamer = QuestRealtimeStreamer(port=8765)
    
    print("WebSocket服务器启动在端口 8765")
    print("请在Quest设备上访问: http://10.3.32.31:8080/quest_hand_tracking.html")
    print("然后点击'开始手部追踪'")
    print("\n等待Quest连接...")
    
    # 启动服务器
    await streamer.start_server()
    
    # 数据监控循环
    last_timestamp = 0
    no_data_count = 0
    
    while True:
        await asyncio.sleep(0.1)  # 10Hz检查频率
        
        data = streamer.get_latest_data()
        current_timestamp = data['timestamp']
        
        # 检查是否有新数据
        if current_timestamp > last_timestamp:
            last_timestamp = current_timestamp
            no_data_count = 0
            
            # 显示手部追踪状态
            left_tracked = data['left_hand']['is_tracked']
            right_tracked = data['right_hand']['is_tracked']
            
            print(f"\r[{time.strftime('%H:%M:%S')}] ", end="")
            print(f"左手: {'✅' if left_tracked else '❌'} ", end="")
            print(f"右手: {'✅' if right_tracked else '❌'} ", end="")
            
            if left_tracked:
                pos = data['left_hand']['position']
                pinch = data['left_hand']['pinch_strength']
                print(f"| 左手位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) 捏合: {pinch:.2f} ", end="")
            
            if right_tracked:
                pos = data['right_hand']['position']
                pinch = data['right_hand']['pinch_strength']
                print(f"| 右手位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) 捏合: {pinch:.2f} ", end="")
                
            print("", flush=True)
            
        else:
            no_data_count += 1
            if no_data_count > 50:  # 5秒无数据
                print(f"\r[{time.strftime('%H:%M:%S')}] 等待Quest连接... ", end="", flush=True)
                no_data_count = 0

def main():
    try:
        asyncio.run(test_quest_connection())
    except KeyboardInterrupt:
        print("\n\n测试结束")

if __name__ == "__main__":
    main()


