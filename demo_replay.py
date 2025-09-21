#!/usr/bin/env python3
"""
Demo重放功能
重放采集的演示数据，让机械臂执行相同的动作序列
"""

import pickle
import numpy as np
import time
import cv2
import argparse
import threading
import signal
import sys
from termcolor import cprint

# 导入我们的机械臂控制模块
from xarm_wrapper import XArmWrapper
from xarm_gripper_wrapper import XArmGripperWrapper

class DemoReplayer:
    def __init__(self, xarm_ip='10.3.32.200', safe_mode=False):
        print("[Replayer] 初始化Demo重放系统...")
        
        self.safe_mode = safe_mode
        self.interrupted = False  # 中断标志
        
        if not safe_mode:
            # 初始化XArm机械臂
            print("[Replayer] 连接XArm机械臂...")
            self.xarm = XArmWrapper(joints_init=None, move_to_init=False, ip=xarm_ip)
            self.gripper = XArmGripperWrapper(self.xarm.xarm)
        else:
            print("[Replayer] 安全模式：不连接真实机械臂")
            self.xarm = None
            self.gripper = None
        
        # 重放控制
        self.playing = False
        self.paused = False
        self.speed_scale = 1.0  # 重放速度倍数
        self.current_step = 0
        
        # 数据
        self.demo_data = None
        self.total_steps = 0
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        print("[Replayer] 初始化完成！")

    def cleanup(self):
        """清理资源"""
        print("[Replayer] 清理资源...")
        self.playing = False
        self.interrupted = True
        
        # 等待键盘线程结束
        if hasattr(self, 'keyboard_thread') and self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)
        
        # 关闭OpenCV窗口
        cv2.destroyAllWindows()
        
        # 停止机械臂
        if not self.safe_mode and hasattr(self, 'xarm') and self.xarm:
            self._safe_stop()

    def _signal_handler(self, signum, frame):
        """信号处理器，响应Ctrl+C"""
        print(f"\n[Replayer] 🚨 接收到中断信号！")
        self.interrupted = True
        self.playing = False
        
        # 立即停止机械臂
        if not self.safe_mode and hasattr(self, 'xarm') and self.xarm:
            self._emergency_stop()
        
        # 关闭OpenCV窗口
        cv2.destroyAllWindows()
        
        # 强制退出程序
        print("[Replayer] 🛑 程序正在安全退出...")
        sys.exit(0)

    def load_demo(self, pkl_path):
        """加载演示数据"""
        try:
            print(f"[Replayer] 加载演示数据: {pkl_path}")
            with open(pkl_path, 'rb') as f:
                self.demo_data = pickle.load(f)
            
            # 验证数据格式
            required_keys = ['image', 'agent_pos', 'action']
            for key in required_keys:
                if key not in self.demo_data:
                    raise ValueError(f"缺少必要的数据字段: {key}")
            
            self.total_steps = len(self.demo_data['action'])
            
            print(f"[Replayer] 演示数据加载成功")
            print(f"[Replayer] 总步数: {self.total_steps}")
            print(f"[Replayer] 数据维度:")
            print(f"  - 图像: {self.demo_data['image'].shape}")
            print(f"  - 状态: {self.demo_data['agent_pos'].shape}")
            print(f"  - 动作: {self.demo_data['action'].shape}")
            
            return True
            
        except Exception as e:
            print(f"[Replayer] 加载演示数据失败: {e}")
            return False

    def play_demo(self, show_images=True, start_step=0, end_step=None):
        """重放演示"""
        if not self.demo_data:
            print("[Replayer] 错误: 未加载演示数据")
            return False
        
        if end_step is None:
            end_step = self.total_steps
        
        end_step = min(end_step, self.total_steps)
        
        print(f"[Replayer] 开始重放演示 (步数: {start_step} -> {end_step})")
        print("[Replayer] 控制说明:")
        print("  - 空格键: 暂停/继续")
        print("  - 'q'键: 退出重放")
        print("  - '+'键: 加速 (最大2.0x)")
        print("  - '-'键: 减速 (最小0.1x)")
        print("  - 'r'键: 重置到开始")
        
        # 移动到起始位置
        
        self.playing = True
        self.paused = False
        self.current_step = start_step
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        try:
            while self.playing and self.current_step < end_step and not self.interrupted:
                # 检查键盘中断
                try:
                    if self.paused:
                        time.sleep(0.1)
                        continue
                    
                    # 检查中断标志
                    if self.interrupted:
                        print(f"\n[Replayer] 检测到中断标志，停止重放")
                        break
                    
                    start_time = time.time()
                    
                    # 获取当前步的数据
                    action = self.demo_data['action'][self.current_step]
                    image = self.demo_data['image'][self.current_step]
                    
                    # 显示进度
                    progress = (self.current_step - start_step) / (end_step - start_step) * 100
                    print(f"\r[Replayer] 步数: {self.current_step}/{end_step} ({progress:.1f}%) "
                          f"速度: {self.speed_scale:.1f}x", end='', flush=True)
                    
                    # 执行动作
                    if not self.safe_mode:
                        self._execute_action(action)
                    else:
                        print(f"\n[Replayer] 安全模式动作: {action}")
                    
                    # 显示图像
                    if show_images:
                        self._show_image(image, self.current_step)
                    
                    self.current_step += 1
                    
                    # 控制重放频率
                    target_dt = (1.0 / 15) / self.speed_scale  # 原始50Hz，除以速度倍数
                    elapsed = time.time() - start_time
                    sleep_time = target_dt - elapsed
                    if sleep_time > 0:
                        # 分段睡眠，以便更快响应中断
                        sleep_step = 0.01  # 10ms 步进
                        while sleep_time > 0 and not self.interrupted:
                            step_time = min(sleep_step, sleep_time)
                            time.sleep(step_time)
                            sleep_time -= step_time
                    
                    # 检查键盘输入
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
                except KeyboardInterrupt:
                    print(f"\n[Replayer] ⚠️  检测到Ctrl+C，立即停止机械臂！")
                    # 立即停止机械臂
                    if not self.safe_mode and self.xarm:
                        self._emergency_stop()
                    raise  # 重新抛出异常以退出循环
            
            print(f"\n[Replayer] 重放完成")
            
        except KeyboardInterrupt:
            print(f"\n[Replayer] 🛑 用户紧急中断重放")
        except Exception as e:
            print(f"\n[Replayer] 重放过程中出错: {e}")
        finally:
            self.playing = False
            
            # 确保所有线程停止
            if hasattr(self, 'keyboard_thread') and self.keyboard_thread.is_alive():
                print("[Replayer] 等待键盘线程结束...")
                self.keyboard_thread.join(timeout=1.0)
            
            # 关闭OpenCV窗口
            cv2.destroyAllWindows()
            
            # 安全停止机械臂
            if not self.safe_mode and self.xarm:
                self._safe_stop()
        
        return True

    def _execute_action(self, action):
        """执行单个动作"""
        try:
            # 执行位置和姿态控制
            target_pos = action[:3] * 1000  # m to mm
            target_euler = action[3:6]  # 欧拉角
            target_pose = np.concatenate([target_pos, target_euler])
            
            self.xarm.set_servo_cartesian(target_pose)
            
            # 执行夹爪控制
            gripper_action = action[6]
            if gripper_action > 0.5:  # 关闭
                self.gripper.close()
            else:  # 打开
                self.gripper.open()
                
        except Exception as e:
            print(f"\n[Replayer] 执行动作失败: {e}")

    def _show_image(self, image, step):
        """显示图像"""
        try:
            # 转换RGB到BGR用于显示
            display_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # 添加信息文字
            cv2.putText(display_image, f"Step: {step}/{self.total_steps}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display_image, f"Speed: {self.speed_scale:.1f}x", 
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            status = "PAUSED" if self.paused else "PLAYING"
            cv2.putText(display_image, status, 
                       (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow('Demo Replay', display_image)
            
        except Exception as e:
            print(f"\n[Replayer] 显示图像失败: {e}")

    def _keyboard_listener(self):
        """键盘监听线程"""
        old_settings = None
        try:
            import termios, tty, sys, select
            
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            
            while self.playing and not self.interrupted:
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        ch = sys.stdin.read(1)
                        
                        if ch == ' ':  # 空格键暂停/继续
                            self.paused = not self.paused
                            status = "暂停" if self.paused else "继续"
                            print(f"\n[Replayer] {status}")
                        elif ch == '+' or ch == '=':  # 加速
                            self.speed_scale = min(2.0, self.speed_scale + 0.1)
                            print(f"\n[Replayer] 速度: {self.speed_scale:.1f}x")
                        elif ch == '-':  # 减速
                            self.speed_scale = max(0.1, self.speed_scale - 0.1)
                            print(f"\n[Replayer] 速度: {self.speed_scale:.1f}x")
                        elif ch == 'r':  # 重置
                            self.current_step = 0
                            print(f"\n[Replayer] 重置到开始")
                        elif ch == 'q':  # 退出
                            self.playing = False
                            break
                except select.error:
                    # select被中断，继续循环
                    pass
                except:
                    # 其他错误，退出循环
                    break
            
        except Exception as e:
            print(f"\n[Replayer] 键盘监听错误: {e}")
        finally:
            # 确保恢复终端设置
            if old_settings:
                try:
                    import termios, sys
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                except:
                    pass

    def _emergency_stop(self):
        """紧急停止机械臂"""
        try:
            print("\n[Replayer] 🚨 紧急停止机械臂！")
            # 立即停止所有运动
            self.xarm.xarm.emergency_stop()
            # 设置到停止状态
            self.xarm.xarm.set_state(4)
            print("[Replayer] ✅ 机械臂已紧急停止")
        except Exception as e:
            print(f"[Replayer] ❌ 紧急停止失败: {e}")
            # 如果紧急停止失败，尝试常规停止
            try:
                self.xarm.xarm.set_mode(0)
                self.xarm.xarm.set_state(4)
                print("[Replayer] ✅ 已执行备用停止方案")
            except Exception as e2:
                print(f"[Replayer] ❌ 备用停止也失败: {e2}")

    def _safe_stop(self):
        """安全停止机械臂"""
        try:
            print("\n[Replayer] 安全停止机械臂...")
            self.xarm.xarm.set_mode(0)
            time.sleep(0.5)
            self.xarm.xarm.set_state(4)
            print("[Replayer] 机械臂已安全停止")
        except Exception as e:
            print(f"[Replayer] 安全停止失败: {e}")

    def analyze_demo(self):
        """分析演示数据"""
        if not self.demo_data:
            print("[Replayer] 错误: 未加载演示数据")
            return
        
        print("\n" + "="*60)
        print("演示数据分析")
        print("="*60)
        
        actions = self.demo_data['action']
        states = self.demo_data['agent_pos']
        
        # 分析位置轨迹
        positions = actions[:, :3]
        print(f"位置轨迹分析:")
        print(f"  X轴范围: [{np.min(positions[:, 0]):.3f}, {np.max(positions[:, 0]):.3f}] m")
        print(f"  Y轴范围: [{np.min(positions[:, 1]):.3f}, {np.max(positions[:, 1]):.3f}] m")
        print(f"  Z轴范围: [{np.min(positions[:, 2]):.3f}, {np.max(positions[:, 2]):.3f}] m")
        
        # 分析移动距离
        distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)
        print(f"  平均移动距离: {np.mean(distances)*1000:.2f} mm/step")
        print(f"  最大移动距离: {np.max(distances)*1000:.2f} mm/step")
        
        # 分析夹爪使用
        gripper_actions = actions[:, 6]
        gripper_changes = np.sum(np.diff(gripper_actions) != 0)
        open_ratio = np.mean(gripper_actions == 0) * 100
        print(f"夹爪使用分析:")
        print(f"  夹爪状态变化次数: {gripper_changes}")
        print(f"  打开状态占比: {open_ratio:.1f}%")
        print(f"  关闭状态占比: {100-open_ratio:.1f}%")
        
        # 分析时间
        if 'timestamps' in self.demo_data:
            timestamps = self.demo_data['timestamps']
            duration = timestamps[-1] - timestamps[0]
            print(f"时间分析:")
            print(f"  总时长: {duration:.2f} 秒")
            print(f"  平均频率: {len(timestamps)/duration:.1f} Hz")
        
        print("="*60)


def main():
    parser = argparse.ArgumentParser(description='Demo重放系统')
    parser.add_argument('pkl_file', type=str, help='演示数据文件路径')
    parser.add_argument('--safe-mode', action='store_true', help='安全模式：不连接真实机械臂')
    parser.add_argument('--no-images', action='store_true', help='不显示图像')
    parser.add_argument('--analyze-only', action='store_true', help='只分析数据，不重放')
    parser.add_argument('--start-step', type=int, default=0, help='开始步数')
    parser.add_argument('--end-step', type=int, default=None, help='结束步数')
    parser.add_argument('--speed', type=float, default=1.0, help='重放速度倍数')
    
    args = parser.parse_args()
    
    try:
        # 创建重放器
        replayer = DemoReplayer(safe_mode=args.safe_mode)
        replayer.speed_scale = args.speed
        
        # 加载演示数据
        if not replayer.load_demo(args.pkl_file):
            return
        
        # 显示系统信息
        cprint("=== Demo重放系统 ===", "green")
        cprint(f"演示文件: {args.pkl_file}", "cyan")
        cprint(f"安全模式: {'开启' if args.safe_mode else '关闭'}", "yellow")
        
        # 分析数据
        replayer.analyze_demo()
        
        if args.analyze_only:
            print("[Replayer] 仅分析模式，不执行重放")
            return
        
        # 确认是否继续
        if not args.safe_mode:
            response = input("\n是否继续重放演示？机械臂将开始移动！(y/N): ")
            if response.lower() != 'y':
                print("[Replayer] 用户取消重放")
                return
        
        # 开始重放
        replayer.play_demo(
            show_images=not args.no_images,
            start_step=args.start_step,
            end_step=args.end_step
        )
        
    except KeyboardInterrupt:
        print("\n[Replayer] 🛑 用户中断")
    except Exception as e:
        print(f"[Replayer] 程序错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保清理所有资源
        if 'replayer' in locals():
            replayer.cleanup()
        print("[Replayer] 程序结束")


if __name__ == "__main__":
    main()
