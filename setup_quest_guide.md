# MetaQuest 3S设置指南

## 1. 硬件连接

### 方法1: USB连接 (推荐用于开发)
1. 使用USB-C数据线连接Quest 3S到电脑
2. 在Quest设备上启用开发者模式
3. 允许USB调试

### 方法2: WiFi连接
1. 确保Quest 3S和电脑在同一WiFi网络
2. 获取Quest设备的IP地址
3. 启用无线调试

## 2. 软件依赖安装

我们需要安装OpenXR或类似的VR开发工具：

```bash
# 安装OpenXR相关依赖
conda activate xarm_teleop
pip install pyopenxr

# 或者使用Oculus SDK (如果可用)
# pip install oculus-sdk
```

## 3. 启用开发者模式

1. 在Quest设备上打开Settings
2. 进入Device -> Developer
3. 启用Developer Mode
4. 启用Hand Tracking

## 4. 手部追踪设置

1. 在Quest设备上打开Hand Tracking
2. 进行手部追踪校准
3. 确保手部追踪精度

## 5. 网络配置

如果使用WiFi连接，需要配置网络：

```python
# 在quest_streamer.py中更新IP地址
QUEST_IP = "192.168.1.XXX"  # 替换为实际Quest IP
```

## 6. 测试连接

运行Quest连接测试：

```bash
python test_quest_connection.py
```

## 注意事项

1. 确保Quest设备电量充足
2. 保持良好的WiFi信号强度
3. 避免手部追踪遮挡
4. 确保充足的照明条件

## 故障排除

### 连接问题
- 检查USB线缆是否支持数据传输
- 确认开发者模式已启用
- 重启Quest设备和电脑

### 手部追踪问题
- 重新校准手部追踪
- 检查照明条件
- 清洁Quest摄像头

### 网络问题
- 确认设备在同一网络
- 检查防火墙设置
- 尝试重新连接WiFi


