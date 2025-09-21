#!/usr/bin/env python3
"""
启动真实Quest手部追踪服务器
包括WebSocket数据接收服务器和HTTP文件服务器
"""
import threading
import time
import http.server
import socketserver
import os
from quest_real_streamer import QuestRealStreamer

def start_websocket_server():
    """启动WebSocket服务器"""
    print("[WebSocket] 启动真实Quest数据WebSocket服务器...")
    streamer = QuestRealStreamer(port=8765)
    
    try:
        if streamer.start_server_thread():
            print("[WebSocket] ✅ WebSocket服务器启动成功")
            
            # 保持服务器运行
            while True:
                time.sleep(1)
        else:
            print("[WebSocket] ❌ WebSocket服务器启动失败")
    except Exception as e:
        print(f"[WebSocket] 错误: {e}")

def start_http_server(port=8080):
    """启动HTTP文件服务器"""
    print(f"[HTTP] 启动HTTP服务器在端口 {port}...")
    
    class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory="/home/user/Desktop/xarm_franka_teleop", **kwargs)
    
    with socketserver.TCPServer(("", port), CustomHTTPRequestHandler) as httpd:
        print(f"[HTTP] ✅ HTTP服务器运行在: http://localhost:{port}")
        print(f"[HTTP] 📱 Quest访问地址: http://10.3.32.31:{port}/quest_webxr_simple.html")
        httpd.serve_forever()

def get_local_ip():
    """获取本机IP地址"""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"

def main():
    print("=" * 60)
    print("🤖 XArm Quest 真实手部追踪服务器")
    print("=" * 60)
    
    # 获取本机IP
    local_ip = get_local_ip()
    print(f"💻 本机IP地址: {local_ip}")
    
    print("\n📋 使用步骤:")
    print("1. 确保Quest和电脑在同一WiFi网络")
    print("2. 在Quest浏览器中访问:")
    print(f"   http://{local_ip}:8080/quest_webxr_simple.html")
    print("3. 先点击'开始简单追踪'测试连接")
    print("4. 如果简单追踪正常，可尝试'进入VR模式'")
    print("5. 在另一个终端中运行机械臂控制程序")
    print("\n🛑 按 Ctrl+C 停止所有服务器")
    
    try:
        # 启动WebSocket服务器线程
        websocket_thread = threading.Thread(target=start_websocket_server)
        websocket_thread.daemon = True
        websocket_thread.start()
        
        # 等待WebSocket服务器启动
        time.sleep(2)
        
        # 启动HTTP服务器 (主线程)
        start_http_server(8080)
        
    except KeyboardInterrupt:
        print("\n🛑 收到中断信号，正在关闭所有服务器...")
        print("✅ 所有服务器已关闭")

if __name__ == "__main__":
    main()
