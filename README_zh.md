## 2025 暑期实习——机器人视觉标定与远程操控

本仓库包含以下功能脚本：
- 相机内参标定（基于棋盘格图像）
- 相机与 Lebai 机械臂的手眼标定（Hand–Eye）
- 轻量级 HTTP 服务器，用于采集 USB 相机图像并同步记录机器人 TCP 位姿
- 简单的视觉引导示例：检测圆形按钮并驱动机械臂移动到目标
- 通过 Dynamixel 主控设备对机械臂进行关节级远程操控

### 主要组件
- `calibration/take_photo.py`：启动 HTTP 服务，采集 USB 相机图像，并将 Lebai TCP 位姿追加到内存列表；图像保存到 `handeye_images/`，按 Ctrl+C 结束时写出 `arm_data.npy`。
- `calibration/post.py`：触发端脚本，对 `take_photo.py` 所在主机的服务发送请求以采集图像。
- `calibration/calibrate.py`：相机内参标定（默认 9×6 内角点棋盘格）。输出 `calibration_data.npz`（`camera_matrix`、`dist_coeffs`）。
- `calibration/calibrate_handeye.py`：使用 OpenCV `calibrateHandEye`（Tsai 方法）进行手眼标定。输入：棋盘格姿态 + `arm_data.npy`。输出 `handeye_result.npz`。
- `camera_test.py`：视觉引导示例服务。GET `/` 时：采集图像 -> HoughCircles 检测圆按钮 -> 由像素直径估算深度 -> 计算目标位姿 -> 控制机械臂移动；同时返回处理后的 JPEG 图像。
- `teleop.py`：从 Dynamixel 读取关节角并推送到 Lebai 机械臂，实现关节级远程操控。

### 环境要求
- Python 3.10+
- 系统需可访问 USB 摄像头（示例路径 `/dev/video0`、`/dev/video1`，按需修改）
- 可网络访问的 Lebai 机械臂
- 硬件（按脚本需要）：
  - USB 相机
  - 棋盘格标定板（默认 9×6 内角点）
  - Dynamixel 伺服与 USB 转接（用于 `teleop.py`）

### Python 依赖
建议在虚拟环境中安装：
```bash
pip install opencv-python numpy requests
# 以下 SDK 为厂商/内部提供，请按需安装
# pip install lebai-sdk
# pip install dynamixel-sdk
```

### 网络与设备配置
- 示例中的 Lebai 与主机 IP：
  - 机械臂：`192.168.10.200`
  - 运行 HTTP 服务的主机：通常为 `192.168.10.201`（请按实际环境修改）
- USB 摄像头设备：优先 `/dev/video0`，失败时尝试 `/dev/video1`
- Dynamixel 串口设备：在 `teleop.py` 的 `DEVICENAME` 中配置（macOS 示例：`/dev/cu.usbmodem1101`）

---

## 工作流程

### 1) 采集标定数据集（图像 + 机器人位姿）
1. 连接 USB 相机与 Lebai 机械臂，确保可互相访问（ping 通）。
2. 在连接相机的主机上启动采集服务：
   ```bash
   python calibration/take_photo.py
   ```
   - 对 `/` 发送 POST 将采集一张图像（保存到 `handeye_images/capture_<ts>.jpg`）并将当前 TCP 位姿追加到内存列表。
   - 完成后按 Ctrl+C，脚本会写出 `arm_data.npy`。
3. 在另一终端运行触发端，机械臂移动到不同姿态时触发采集：
   ```bash
   python calibration/post.py
   # 回车触发一次采集；输入 q 回车退出
   ```

提示：
- 覆盖多种位置与姿态，但需保证棋盘格清晰可见。
- 保持图像数量与位姿数量一致（脚本在手眼阶段会进行数量一致性校验）。

### 2) 相机内参标定
1. 确保 `handeye_images/` 中有棋盘格图像，`calibration/calibrate.py` 的 `chessboard_size` 与实际一致（默认 9×6 内角点）。
2. 运行：
   ```bash
   python calibration/calibrate.py
   ```
3. 输出：`calibration_data.npz`，包含 `camera_matrix` 与 `dist_coeffs`。

### 3) 手眼标定（Camera–Gripper）
1. 准备：
   - 第一步采集的 `handeye_images/`
   - 第一阶段保存的 `arm_data.npy`
   - 相机内参（`calibration_data.npz`），或在 `calibration/calibrate_handeye.py` 中手动填写
2. 如不从文件读取，请在 `calibration/calibrate_handeye.py` 中更新 `camera_matrix` 与 `distortion_coefficients`。
3. 运行：
   ```bash
   python calibration/calibrate_handeye.py
   ```
4. 输出：`handeye_result.npz`，包含旋转矩阵 `R` 与平移向量 `t`（相机坐标在抓取器坐标中的位姿）。

### 4) 视觉引导按钮按压示例
1. 在 `camera_test.py` 中更新：
   - 第二步得到的 `camera_matrix`、`distortion_coefficients`
   - 第三步得到的 `R_gripper2camera`、`T_gripper2camera`（注意单位；代码中 `T` 以米为单位）
   - `REAL_BUTTON_DIAMETER`（mm）与 HoughCircles 的参数
2. 启动服务：
   ```bash
   python camera_test.py
   ```
3. 发送 GET 触发一次“采集-检测-移动”流程，并返回处理后的 JPEG：
   ```bash
   curl -v http://<host_ip>:2000/ --output result.jpg
   ```
   机械臂会复位、采集图像、检测圆、用小孔模型由像素直径估算深度、将像素坐标回投到 3D、转换为机器人坐标并移动，随后复位。

说明：
- GET 处理器声明了 `multipart/x-mixed-replace`，当前实现每次请求返回单张 JPEG。
- 首次验证建议降低速度并清空工作空间，确保安全。

### 5) Dynamixel 主控远程操控
1. 连接 Dynamixel 总线，并在 `teleop.py` 中设置 `DEVICENAME`。
2. 如硬件配置不同，调整 `DXL_IDS` 与关节映射。
3. 运行：
   ```bash
   python teleop.py
   ```
4. 脚本持续读取伺服角度，通过 `towardj` 推动 Lebai 关节运动。

---

## HTTP 接口
- `calibration/take_photo.py`（默认端口 2001）
  - POST `/`：采集并保存图像；将当前 TCP 位姿追加到内存
  - GET `/`：返回当前帧的单张 JPEG
- `camera_test.py`（默认端口 2000）
  - GET `/`：执行一次检测-移动流程并返回处理后的单张 JPEG

## 数据产物
- `handeye_images/`：用于标定的原始图像
- `arm_data.npy`：与图像一一对应的 TCP 位姿
- `calibration_data.npz`：相机内参（`camera_matrix`、`dist_coeffs`）
- `handeye_result.npz`：手眼标定结果（`R`、`t`）

## 安全与排障
- 确认 IP 与设备路径正确。
- 以低速调试，保证环境安全与避障。
- HoughCircles 未检测到时，调节模糊核、`param1`、`param2`、`minRadius`/`maxRadius` 等参数。
- 在 macOS 上可能没有 `/dev/video*`，请使用平台可用的视频索引或流地址。
- 请正确安装并授权厂商 SDK（`lebai_sdk`、`dynamixel_sdk`）。

## 项目结构
```
2025-summer-intern/
  calibration/
    calibrate.py
    calibrate_handeye.py
    post.py
    take_photo.py
  camera_test.py
  teleop.py
```

## 许可
仅供内部使用，除非另行说明。
