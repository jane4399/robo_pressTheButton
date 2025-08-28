# 双重检测系统使用指南 (Vision + Pressure)

本指南介绍如何使用结合视觉检测和压力感应的双重检测系统，实现更可靠的按钮状态检测。

## 为什么使用双重检测？

### **单一检测的局限性**
- ❌ **仅视觉检测**: 受光照、LED颜色、相机角度影响
- ❌ **仅压力检测**: 无法区分按钮是否真的被按下（LED状态）
- ❌ **环境变化**: 单一检测方法容易受环境因素干扰

### **双重检测的优势**
- ✅ **冗余检测**: 两种方法互相验证，提高可靠性
- ✅ **环境适应**: 适应不同的光照和压力条件
- ✅ **故障容错**: 一种方法失效时，另一种仍可工作
- ✅ **精确判断**: 结合视觉和触觉信息，更准确的状态判断

## 系统架构

```
按钮状态检测
├── 视觉检测 (Vision Detection)
│   ├── ROI亮度分析
│   ├── 学习型阈值
│   └── LED状态识别
└── 压力检测 (Pressure Detection)
    ├── 力传感器读数
    ├── 接触检测
    └── 压力阈值判断
```

## 安装和配置

### **1. 硬件要求**
- **固定相机**: 安装在固定位置，始终能看到按钮面板
- **压力传感器**: Lebai机械臂内置的力传感器
- **网络连接**: 机械臂和计算机的网络连接

### **2. 软件配置**
```bash
# 安装依赖
python -m pip install opencv-python numpy lebai-sdk

# 检查配置
python test_pressure.py --duration 5
```

## 压力传感器校准

### **首次校准**
```bash
# 校准压力传感器
python test_pressure.py --calibrate
```

**校准步骤:**
1. 确保机械臂在**中性位置**（无外力作用）
2. 按Enter开始校准
3. 系统自动采集10次读数建立基线
4. 校准完成后显示基线压力和检测阈值

### **测试压力传感器**
```bash
# 基本测试（10秒）
python test_pressure.py

# 测试特定阈值
python test_pressure.py --threshold 0.3

# 连续监控
python test_pressure.py --continuous

# 长时间测试
python test_pressure.py --duration 30
```

## 双重检测工作流程

### **1. 按钮示教和ROI设置**
```bash
# 示教按钮位置
python teach.py savei F1 --axis z --approach 0.03 --press 0.005

# 设置ROI
python teach.py roi set F1 --x 100 --y 150 --w 50 --h 50 --threshold 0.6
```

### **2. 学习按钮检测**
```bash
# 学习所有按钮
python learn_buttons.py

# 或学习特定按钮
python learn_buttons.py --button F1
```

### **3. 运行双重检测服务器**
```bash
# 启动服务器（自动使用双重检测）
python press_server.py

# 启用调试模式
python press_server.py --debug

# 自定义压力阈值
python press_server.py --pressure-threshold 0.4
```

## 检测逻辑详解

### **双重检测决策**
```python
# 检测逻辑
vision_result = check_vision_detection(button)
pressure_result = check_pressure_detection(button)

# 按钮被按下 = 视觉检测成功 OR 压力检测成功
button_pressed = vision_result or pressure_result
```

### **检测优先级**
1. **双重检测模式** (推荐): 视觉 + 压力
2. **单一检测模式**: 视觉检测
3. **压力检测模式**: 仅压力传感器
4. **禁用压力检测**: `--disable-pressure`

### **置信度计算**
- **视觉置信度**: 基于学习型阈值和亮度变化
- **压力置信度**: 基于压力变化和稳定性
- **综合结果**: 任一方法确认即认为按钮被按下

## 使用场景

### **场景1: 正常操作**
```bash
# 启动服务器
python press_server.py

# 检查按钮状态
curl http://localhost:2000/status/F1

# 执行按钮按压
curl -X POST http://localhost:2000/press/F1
```

**检测流程:**
1. **预检查**: 视觉+压力检测按钮是否已被按下
2. **执行按压**: 机械臂移动到按钮位置并按压
3. **确认按压**: 视觉+压力检测确认按压成功

### **场景2: 调试模式**
```bash
# 启用调试输出
python press_server.py --debug

# 查看详细检测信息
curl http://localhost:2000/status/F1
```

**调试信息包括:**
- 视觉检测结果和置信度
- 压力检测结果和置信度
- ROI亮度变化
- 压力传感器读数

### **场景3: 压力传感器测试**
```bash
# 测试压力传感器功能
python test_pressure.py --duration 20

# 测试特定阈值
python test_pressure.py --threshold 0.5

# 连续监控
python test_pressure.py --continuous
```

## 故障排除

### **常见问题**

#### **压力传感器无法读取**
```bash
# 检查连接
python test_pressure.py --duration 5

# 可能的解决方案:
# 1. 检查网络连接
# 2. 确认机械臂IP地址
# 3. 检查SDK版本兼容性
```

#### **检测不准确**
```bash
# 重新校准压力传感器
python test_pressure.py --calibrate

# 重新学习视觉检测
python learn_buttons.py --button F1

# 调整压力阈值
python press_server.py --pressure-threshold 0.3
```

#### **相机问题**
```bash
# 检查相机连接
ls /dev/video*

# 测试相机
python -c "import cv2; cap = cv2.VideoCapture(0); print(cap.isOpened())"
```

### **性能优化**

#### **调整检测参数**
```python
# 在 press_server.py 中调整
PRESSURE_THRESHOLD = 0.5          # 压力阈值 (N)
PRESSURE_CHECK_INTERVAL = 0.1     # 检测间隔 (秒)
PRESSURE_STABLE_COUNT = 3         # 稳定读数数量
```

#### **ROI优化**
- **大小**: ROI应覆盖整个按钮LED区域
- **位置**: 避免阴影和反光
- **阈值**: 使用学习型阈值，无需手动调整

## 高级功能

### **自定义压力检测**
```python
# 在 press_server.py 中修改 read_pressure_sensor 方法
# 根据你的Lebai机器人型号调整接口调用
```

### **检测方法选择**
```bash
# 仅使用视觉检测
python press_server.py --disable-pressure

# 仅使用压力检测
python press_server.py --disable-pressure  # 然后修改代码逻辑
```

### **数据记录和分析**
```bash
# 启用调试模式记录检测数据
python press_server.py --debug > detection_log.txt

# 分析检测性能
grep "DEBUG" detection_log.txt | grep "F1"
```

## 安全注意事项

### **机械臂安全**
- 确保压力阈值设置合理，避免过度施力
- 定期检查压力传感器校准
- 在安全环境中测试

### **系统可靠性**
- 双重检测提供冗余保护
- 定期测试两种检测方法
- 监控检测失败率

## 总结

双重检测系统结合了视觉和压力检测的优势，提供了：

1. **更高的可靠性**: 两种方法互相验证
2. **更好的适应性**: 适应不同环境条件
3. **更强的容错性**: 单一方法失效时仍可工作
4. **更精确的判断**: 综合视觉和触觉信息

通过合理配置和定期维护，双重检测系统可以显著提高按钮按压的准确性和可靠性！ 🎯✨
