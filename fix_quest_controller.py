#!/usr/bin/env python3
"""
Quest控制器问题快速修复工具
"""
import os
import time
import subprocess

def print_header(text):
    print("=" * 60)
    print(f"🔧 {text}")
    print("=" * 60)

def print_step(step_num, text):
    print(f"\n{step_num}. {text}")

def check_quest_setup():
    """检查Quest设备基本设置"""
    print_header("Quest设备设置检查")
    
    print_step(1, "检查Quest设备状态")
    print("请在Quest设备上检查以下设置：")
    print("  📱 设置 → 设备 → 开发者")
    print("     ✓ 开发者模式: 已启用") 
    print("     ✓ USB调试: 已启用")
    print("     ✓ 手部追踪: 已禁用 (使用控制器时)")
    
    print_step(2, "检查控制器状态")
    print("检查Quest控制器：")
    print("  🎮 左控制器状态: 开机 + 电量充足")
    print("  🎮 右控制器状态: 开机 + 电量充足")
    print("  🔋 控制器电量: > 20%")
    print("  📡 配对状态: 已连接到Quest")
    
    input("\n按Enter继续到下一步...")

def test_controller_activation():
    """测试控制器激活"""
    print_header("控制器激活测试")
    
    print_step(1, "激活控制器")
    print("请执行以下操作：")
    print("  1. 拿起两个Quest控制器")
    print("  2. 按下右控制器的 🏠 Home键")
    print("  3. 按下右控制器的 ☰ Menu键") 
    print("  4. 按下右控制器的扳机键")
    print("  5. 观察控制器LED指示灯是否亮起")
    
    print_step(2, "检查追踪状态")
    print("在Quest头显中：")
    print("  1. 进入Quest主界面")
    print("  2. 观察虚拟控制器是否显示")
    print("  3. 移动右控制器测试追踪")
    
    response = input("\n右控制器是否正常显示和追踪？(y/n): ")
    return response.lower() == 'y'

def fix_controller_pairing():
    """修复控制器配对"""
    print_header("控制器配对修复")
    
    print_step(1, "重新配对控制器")
    print("在Quest设备上：")
    print("  1. 设置 → 设备 → 控制器和追踪")
    print("  2. 选择 '重新配对控制器'")
    print("  3. 按住右控制器的 ☰ Menu键 + B键 3秒")
    print("  4. 等待配对完成")
    
    print_step(2, "测试配对结果")
    print("配对完成后：")
    print("  1. 按下右控制器任意按钮")
    print("  2. 观察Quest界面中的控制器状态")
    print("  3. 测试控制器响应")
    
    input("\n配对完成后按Enter继续...")

def create_simple_test_page():
    """创建简化的测试页面"""
    print_header("创建测试页面")
    
    test_html = """<!DOCTYPE html>
<html>
<head>
    <title>简单控制器测试</title>
    <style>
        body { font-family: Arial; padding: 20px; background: #222; color: white; }
        .status { padding: 10px; margin: 10px 0; border-radius: 5px; }
        .good { background: green; }
        .bad { background: red; }
    </style>
</head>
<body>
    <h1>Quest控制器简单测试</h1>
    <button onclick="startTest()">开始测试</button>
    <div id="status">点击开始测试</div>
    <div id="controllers"></div>
    
    <script>
        function startTest() {
            setInterval(checkControllers, 500);
        }
        
        function checkControllers() {
            const gamepads = navigator.getGamepads();
            const status = document.getElementById('status');
            const controllers = document.getElementById('controllers');
            
            let found = 0;
            let html = '';
            
            for (let i = 0; i < gamepads.length; i++) {
                if (gamepads[i] && gamepads[i].connected) {
                    found++;
                    html += `<div class="status good">
                        控制器 ${i}: ${gamepads[i].id} 
                        (手部: ${gamepads[i].hand || '未知'})
                    </div>`;
                }
            }
            
            if (found === 0) {
                status.innerHTML = '<div class="status bad">未发现控制器</div>';
                controllers.innerHTML = '<p>请确保控制器已开机并按下任意按钮</p>';
            } else {
                status.innerHTML = `<div class="status good">发现 ${found} 个控制器</div>`;
                controllers.innerHTML = html;
            }
        }
    </script>
</body>
</html>"""
    
    with open('/home/user/Desktop/xarm_franka_teleop/simple_controller_test.html', 'w') as f:
        f.write(test_html)
    
    print("✅ 创建了简化测试页面: simple_controller_test.html")
    print("   在Quest浏览器中访问这个页面测试控制器检测")

def main():
    print_header("Quest右控制器检测问题修复工具")
    
    print("🎯 这个工具将帮助你诊断和修复Quest右控制器检测问题")
    print("\n常见原因：")
    print("  • 控制器未正确激活")
    print("  • 控制器电量不足")
    print("  • 配对连接问题")
    print("  • 手部追踪与控制器冲突")
    print("  • WebXR权限问题")
    
    # 步骤1：检查基本设置
    check_quest_setup()
    
    # 步骤2：测试控制器激活
    if not test_controller_activation():
        print("\n⚠️  控制器激活有问题，尝试修复配对...")
        fix_controller_pairing()
    else:
        print("\n✅ 控制器激活正常！")
    
    # 步骤3：创建测试页面
    create_simple_test_page()
    
    print("\n" + "=" * 60)
    print("🏁 修复完成！接下来的步骤：")
    print("=" * 60)
    print("1. 在Quest浏览器中测试 simple_controller_test.html")
    print("2. 如果仍有问题，使用 quest_controller_debug.html 详细调试")
    print("3. 确认两个控制器都能正常检测后，运行完整的控制程序")
    
    print("\n💡 额外提示：")
    print("  • 控制器需要在Quest的追踪范围内")
    print("  • 确保房间光线充足")
    print("  • 避免反光表面干扰")
    print("  • 控制器电量低时追踪会不稳定")

if __name__ == "__main__":
    main()


