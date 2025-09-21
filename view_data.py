#!/usr/bin/env python3
"""
查看采集的数据文件内容
"""

import pickle
import numpy as np
import argparse
import sys
import cv2

def view_pkl_data(pkl_path):
    """查看pkl文件内容"""
    try:
        # 加载数据
        print(f"正在加载文件: {pkl_path}")
        with open(pkl_path, 'rb') as f:
            data = pickle.load(f)
        
        print("="*60)
        print("数据文件概览")
        print("="*60)
        
        # 显示基本信息
        if isinstance(data, dict):
            print(f"数据类型: 字典")
            print(f"键值: {list(data.keys())}")
            print()
            
            # 显示每个键的详细信息
            for key, value in data.items():
                print(f"[{key}]:")
                if isinstance(value, np.ndarray):
                    print(f"  类型: numpy数组")
                    print(f"  形状: {value.shape}")
                    print(f"  数据类型: {value.dtype}")
                    if value.size > 0:
                        print(f"  数值范围: [{np.min(value):.3f}, {np.max(value):.3f}]")
                        if len(value.shape) <= 2 and value.shape[0] <= 5:
                            print(f"  前几个值:\n{value}")
                        else:
                            print(f"  前几个值: {value.flat[:10]}")
                elif isinstance(value, list):
                    print(f"  类型: 列表")
                    print(f"  长度: {len(value)}")
                    if len(value) > 0:
                        print(f"  第一个元素类型: {type(value[0])}")
                        if len(value) <= 3:
                            print(f"  内容: {value}")
                        else:
                            print(f"  前3个元素: {value[:3]}")
                elif isinstance(value, dict):
                    print(f"  类型: 字典")
                    print(f"  键值: {list(value.keys())}")
                else:
                    print(f"  类型: {type(value)}")
                    print(f"  值: {value}")
                print()
            
            # 如果有图像数据，显示图像信息
            if 'image' in data:
                images = data['image']
                from IPython import embed; embed()
                print("图像数据详情:")
                print(f"  总帧数: {images.shape[0]}")
                print(f"  图像尺寸: {images.shape[1]}x{images.shape[2]}")
                print(f"  通道数: {images.shape[3] if len(images.shape) > 3 else 1}")
                print()
            
            # 如果有机械臂状态数据，显示状态信息
            if 'agent_pos' in data:
                states = data['agent_pos']
                print("机械臂状态详情:")
                print(f"  数据点数: {states.shape[0]}")
                print(f"  状态维度: {states.shape[1]}")
                if states.shape[1] == 7:
                    print("  状态含义: [x, y, z, roll, pitch, yaw, gripper]")
                    print(f"  位置范围:")
                    print(f"    X: [{np.min(states[:, 0]):.3f}, {np.max(states[:, 0]):.3f}]")
                    print(f"    Y: [{np.min(states[:, 1]):.3f}, {np.max(states[:, 1]):.3f}]")
                    print(f"    Z: [{np.min(states[:, 2]):.3f}, {np.max(states[:, 2]):.3f}]")
                    print(f"  夹爪状态: {np.unique(states[:, 6])}")
                print()
            
            # 如果有动作数据，显示动作信息
            if 'action' in data:
                actions = data['action']
                print("动作数据详情:")
                print(f"  数据点数: {actions.shape[0]}")
                print(f"  动作维度: {actions.shape[1]}")
                if actions.shape[1] == 7:
                    print("  动作含义: [x, y, z, roll, pitch, yaw, gripper]")
                    print(f"  夹爪动作: {np.unique(actions[:, 6])}")
                print()
            
            # 如果有Quest数据，显示Quest信息
            if 'quest_data' in data:
                quest_data = data['quest_data']
                print("Quest数据详情:")
                print(f"  数据点数: {len(quest_data)}")
                if len(quest_data) > 0:
                    first_data = quest_data[0]
                    if isinstance(first_data, dict):
                        print(f"  包含键值: {list(first_data.keys())}")
                        if 'transforms' in first_data and first_data['transforms']:
                            print(f"  控制器: {list(first_data['transforms'].keys())}")
                        if 'buttons' in first_data and first_data['buttons']:
                            print(f"  按钮: {list(first_data['buttons'].keys())}")
                print()
            
            # 如果有元数据，显示元数据
            if 'metadata' in data:
                metadata = data['metadata']
                print("元数据:")
                for k, v in metadata.items():
                    print(f"  {k}: {v}")
                print()
                
        else:
            print(f"数据类型: {type(data)}")
            print(f"数据内容: {data}")
        
        print("="*60)
        print("查看完成")
        
    except Exception as e:
        print(f"错误: 无法读取文件 {pkl_path}")
        print(f"详细错误: {e}")
        import traceback
        traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description='查看pkl数据文件内容')
    parser.add_argument('pkl_file', type=str, help='pkl文件路径')
    
    args = parser.parse_args()
    
    view_pkl_data(args.pkl_file)

if __name__ == "__main__":
    main()
