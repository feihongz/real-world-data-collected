#!/usr/bin/env python3
"""
检查Quest VR设备网络连接设置
"""
import socket
import subprocess
import re

def get_local_ip():
    """获取本机IP地址"""
    try:
        # 连接到外部地址获取本机IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return None

def check_network_interfaces():
    """检查网络接口"""
    try:
        result = subprocess.run(['ip', 'addr', 'show'], capture_output=True, text=True)
        return result.stdout
    except:
        return None

def main():
    print("=" * 60)
    print("🔧 Quest VR网络连接检查")
    print("=" * 60)
    
    # 获取本机IP
    local_ip = get_local_ip()
    if local_ip:
        print(f"✅ 本机IP地址: {local_ip}")
    else:
        print("❌ 无法获取本机IP地址")
        return
    
    # 检查网络接口
    print(f"\n📡 网络接口信息:")
    interfaces = check_network_interfaces()
    if interfaces:
        # 提取WiFi接口信息
        wifi_pattern = r'(\w+):\s.*?inet\s+([\d.]+)'
        matches = re.findall(wifi_pattern, interfaces)
        for interface, ip in matches:
            if not ip.startswith('127.'):  # 排除localhost
                print(f"  - {interface}: {ip}")
    
    print(f"\n📋 Quest设备连接步骤:")
    print(f"1. 确保Quest和电脑连接到同一WiFi网络")
    print(f"2. 在Quest浏览器中访问:")
    print(f"   http://{local_ip}:8080/quest_hand_tracking.html")
    print(f"3. 或者访问简化版本:")
    print(f"   http://{local_ip}:8080/quest_webxr_simple.html")
    
    print(f"\n🔧 如果连接失败，请检查:")
    print(f"  - 防火墙设置 (允许端口8080和8765)")
    print(f"  - WiFi网络是否为同一个")
    print(f"  - Quest开发者模式是否已启用")
    
    # 测试端口是否可用
    print(f"\n🔍 测试端口可用性:")
    for port in [8080, 8765]:
        try:
            test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_socket.settimeout(1)
            result = test_socket.connect_ex((local_ip, port))
            if result == 0:
                print(f"  ⚠️  端口 {port} 已被占用")
            else:
                print(f"  ✅ 端口 {port} 可用")
            test_socket.close()
        except:
            print(f"  ✅ 端口 {port} 可用")

if __name__ == "__main__":
    main()


