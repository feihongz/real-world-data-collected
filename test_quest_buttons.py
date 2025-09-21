#!/usr/bin/env python3
"""
测试Quest按钮数据格式
专门查看按钮数据的具体内容
"""

import sys
import os
import time
import numpy as np

# 添加oculus_reader到路径
sys.path.append('/home/user/Desktop/oculus_reader')
from oculus_reader.reader import OculusReader

def test_buttons():
    """测试按钮数据"""
    print("Quest按钮数据测试")
    print("="*50)
    print("请按下Quest控制器的各种按钮进行测试:")
    print("- 右手触发器 (食指)")
    print("- 右手握持按钮 (中指)")
    print("- A按钮")
    print("- B按钮")
    print("- 摇杆按下")
    print("="*50)
    
    try:
        # 初始化Quest阅读器
        quest_reader = OculusReader(run=True, print_FPS=False)
        
        last_button_data = {}
        
        while True:
            try:
                # 获取Quest数据
                transforms, buttons = quest_reader.get_transformations_and_buttons()
                
                if buttons:
                    # 检查是否有新的按钮数据
                    current_buttons = str(buttons)
                    if current_buttons != str(last_button_data):
                        print(f"\n[{time.strftime('%H:%M:%S')}] 按钮数据变化:")
                        print(f"完整数据: {buttons}")
                        
                        # 详细解析右手按钮
                        if 'r' in buttons:
                            right_buttons = buttons['r']
                            print(f"右手按钮: {right_buttons}")
                            
                            # 逐个检查按钮
                            button_names = {
                                'RTr': '触发器(数字)',
                                'RG': '握持按钮', 
                                'A': 'A按钮',
                                'B': 'B按钮',
                                'RJ': '摇杆按下',
                                'RThU': '拇指抬起',
                                'rightTrig': '触发器(模拟值)',
                                'rightGrip': '握持(模拟值)',
                                'rightJS': '摇杆位置'
                            }
                            
                            for key, name in button_names.items():
                                if key in right_buttons:
                                    value = right_buttons[key]
                                    print(f"  {name} ({key}): {value}")
                        
                        # 检查左手按钮
                        if 'l' in buttons:
                            left_buttons = buttons['l']
                            print(f"左手按钮: {left_buttons}")
                        
                        last_button_data = buttons.copy()
                        print("-" * 50)
                
                time.sleep(0.1)  # 10Hz检查频率
                
            except KeyboardInterrupt:
                print("\n用户中断测试")
                break
            except Exception as e:
                print(f"数据读取错误: {e}")
                time.sleep(0.1)
    
    except Exception as e:
        print(f"初始化失败: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if 'quest_reader' in locals():
            quest_reader.stop()
        print("测试结束")

if __name__ == "__main__":
    test_buttons()
