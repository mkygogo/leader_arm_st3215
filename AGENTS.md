# AGENTS.md

## 项目概览

ST3215 舵机机械臂遥操控制系统。主臂（Leader）使用 7 个飞特 STS3215 总线舵机，读取关节角度后驱动从臂（Follower）跟随运动。

当前支持：
- 单臂遥操 (`teleop_main.py`)
- 双臂遥操 (`teleop_dual_main.py`)
- WebSocket bridge 远程遥操 (`local_bridge.py`)
- 远端从臂 WebSocket server (`robot_server.py`)
- 舵机 ID 设置、校准等工具

## 硬件

| 组件 | 型号 | 说明 |
|------|------|------|
| 主臂舵机 | 飞特 STS3215 | 7轴，总线协议，1Mbaud |
| 从臂电机 | 达妙 DM4340/DM4310 | CAN 总线，位置+速度模式 |
| 串口芯片 | CH340 / FTDI | USB-to-Serial，Windows/Linux 通用 |

## 串口协议 (STS3215)

- 波特率：`1,000,000`
- 包格式：`[0xFF, 0xFF, ID, Length, Instruction, Params..., Checksum]`
- 关键地址：
  - `42` — Goal Position (写，2字节 LE)
  - `56` — Present Position (读，2字节 LE，0-4095 = 360°)
  - `58` — Present Speed (读，2字节 LE，有符号)
  - `60` — Present Load (读，2字节 LE，有符号)
  - `40` — Torque Enable (写，1=锁定 0=卸力)
- 分辨率：4096 步 / 360°
- 7 个舵机逐个轮询一遍约 10-20ms

## 文件结构

| 文件 | 用途 |
|------|------|
| `sts3215_driver.py` | 底层串口驱动，收发 STS 协议包 |
| `leader_arm_st3215.py` | `LeaderArm` 类：读角度、校准、方向配置 |
| `mk_driver.py` | `MKRobotStandalone` 从臂驱动（DM CAN 电机） |
| `teleop_main.py` | 单臂遥操主循环 |
| `teleop_dual_main.py` | 双臂遥操主循环 |
| `read_7_axis.py` | 实时打印 7 轴原始位置 |
| `read_daul_leader.py` | 双主臂读取验证 |
| `tool_*.py` | 舵机 ID 设置、USB 端口扫描工具 |
| `leader_config.json` | 校准文件（零点偏移 + 方向） |
| `local_bridge.py` | WebSocket bridge（Windows/Linux 通用） |
| `bridge_config.json` | bridge 配置（序列号 + 校准） |
| `robot_server.py` | 远端从臂服务，连接 scene-relay 接收遥操命令并反馈真实关节状态 |
| `robot_server_config.json` | robot_server 配置（从臂端口、方向、夹爪映射、安全超时） |
| `start_stereo_teleop.sh` | 从臂端一键启动脚本（Janus + relay + FFS + robot_server） |

## 端口识别策略

不依赖 Linux udev 规则（可移植性差），改用 `serial.tools.list_ports` 按设备序列号匹配：

```python
for p in serial.tools.list_ports.comports():
    if p.serial_number == "5A68009611":
        port = p.device  # → "COM3" 或 "/dev/ttyACM0"
```

## local_bridge.py

### 目的

在操作员电脑（Windows/Linux）上运行，将主臂关节数据通过 WebSocket 推送给浏览器页面（StereoSpatial RobotControl），实现远程遥操。

### 架构

```
┌─ 操作员电脑 ──────────────────────────────────┐
│                                                │
│  左主臂(USB) ─┐                                │
│               ├→ local_bridge.py (:7000 WS)    │
│  右主臂(USB) ─┘        ↕                       │
│                    浏览器页面                    │
│                    (ws://localhost:7000)         │
└────────────────────────────────────────────────┘
         │ 浏览器通过 scene-relay 转发到远端
         ↓
┌─ 远端机器人 ──────────────────────────────────┐
│  robot_server.py → DM CAN 从臂               │
└───────────────────────────────────────────────┘
```

### 通信协议

**Bridge → 浏览器（50Hz 推送）**：
```json
{
  "type": "joint_state",
  "ts": 1716345678.123,
  "left": [0.0, 12.5, -30.2, 5.1, 0.0, -10.3, 45.0],
  "right": [0.0, -5.2, 20.1, -3.0, 0.0, 8.7, 30.0]
}
```
角度单位：度（已校准、已应用方向修正）。7 个值 = 6 关节 + 1 夹爪。
`left`/`right` 可为 `null`（该臂未连接）。

**浏览器 → Bridge（命令）**：
```json
{"cmd": "calibrate", "side": "left"}
{"cmd": "calibrate", "side": "right"}
{"cmd": "calibrate"}
{"cmd": "torque", "side": "left", "enable": false}
{"cmd": "torque", "side": "right", "enable": true}
```

### Windows 兼容性

1. 纯 Python，仅依赖 `pyserial` + `websockets`
2. 通过序列号自动发现 COM 端口（无 udev）
3. 无 numpy 依赖
4. 单文件可运行：`python local_bridge.py`
5. USB 拔插后自动重连

### 运行

```bash
pip install pyserial websockets
python local_bridge.py
python local_bridge.py --config bridge_config.json --port 7000
```

## 与 StereoSpatial 集成

浏览器端同时连接：
1. `ws://localhost:7000` — 本地 local_bridge（主臂关节数据）
2. `ws://远端:8190` — scene-relay（视频 + 点云 + 从臂指令）

## robot_server.py

### 目的

在远端机器人电脑上运行，将 StereoSpatial `#leader-arm` 页面经 `scene-relay` 转发来的主臂输入转换为真实从臂动作，并把从臂实际关节状态反馈给所有 viewer。

v1 不单独监听 `:8800`，而是作为 scene-relay client 连接 `:8190`：

```
操作员主臂 → local_bridge.py (:7000)
  → 浏览器 #leader-arm
  → scene-relay.mjs (:8190)
  → robot_server.py
  → MKRobotStandalone / DM CAN 从臂
```

### 安全策略

- 默认 `teleop_enabled=false`，仅收到 `teleop_enable` 后执行动作。
- 收到 `teleop_disable` 后停止继续下发命令。
- 收到 `emergency_stop` 后进入 `estop`，忽略普通命令，直到收到 `clear_estop`。
- 超过 `command_timeout_ms` 未收到新 `leader_joint_command` 时进入 `stale`，不继续执行旧命令。
- 左右从臂可独立启用；未配置或未连接的一侧反馈为 `null`。

### scene-relay 协议

**浏览器 → robot_server（经 relay 转发）**：
```json
{
  "type": "leader_joint_command",
  "ts": 1716345678.123,
  "seq": 1024,
  "left": null,
  "right": [0.0, -5.2, 20.1, -3.0, 0.0, 8.7, 30.0],
  "units": "deg",
  "enabled": true
}
```

主臂输入单位为度。`robot_server.py` 内部将 J1-J6 转换为弧度，并按 `robot_server_config.json` 映射夹爪到 `0..1`。

**浏览器 → 远端视频 publisher（经 relay 转发）**：
```json
{"type": "video_mode", "mode": "rgb"}
{"type": "video_mode", "mode": "depth"}
```

`#leader-arm` 页面提供 RGB/Depth 切换；relay 只负责透传，真实视频源需要在远端 publisher 侧按 `video_mode` 输出对应 `video_frame`。

**robot_server → 浏览器**：
```json
{
  "type": "remote_joint_state",
  "ts": 1716345678.123,
  "left": null,
  "right": [0.12, -0.4, 1.1, 0.0, 0.2, -0.1, 0.6],
  "units": "rad",
  "enabled": true
}
```

**robot_server 状态**：
```json
{
  "type": "robot_server_status",
  "ts": 1716345678.123,
  "connected": true,
  "enabled": false,
  "mode": "idle",
  "error": ""
}
```

### 运行

无硬件联调协议：
```bash
python3 robot_server.py --mock --relay-url ws://127.0.0.1:8190
```

真实从臂：
```bash
python3 robot_server.py --config robot_server_config.json --relay-url ws://127.0.0.1:8190
```

## 验证命令

```bash
python -m py_compile sts3215_driver.py leader_arm_st3215.py local_bridge.py robot_server.py
python local_bridge.py  # 无硬件时显示等待连接
python robot_server.py --mock  # 无从臂时验证 scene-relay 协议
./start_stereo_teleop.sh status  # 查看全部服务状态
./start_stereo_teleop.sh         # 一键启动从臂端全栈
./start_stereo_teleop.sh stop    # 停止所有服务
```

## start_stereo_teleop.sh

### 目的

在远端机器人电脑上一键启动整个遥操后端栈。

### 启动顺序

1. **Janus** — 检查 8088/8188 端口，未运行则通过 systemd 或前台启动
2. **Janus VideoRoom** — 确保 `robot-scene` 房间存在（通过 HTTP API 创建）
3. **scene-relay** — 启动 `scene-relay.mjs` on :8190
4. **Fast-FoundationStereo** — conda `ffs` 环境启动感知服务 :8765（Decxin 双目相机）
5. **robot_server** — 连接 relay，等待遥操命令驱动 DM CAN 从臂

### 用法

```bash
./start_stereo_teleop.sh          # 启动全栈
./start_stereo_teleop.sh status   # 查看服务状态
./start_stereo_teleop.sh stop     # 停止所有本脚本启动的服务
./start_stereo_teleop.sh restart  # 重启

# 环境变量覆盖
ROBOT_MOCK=1 ./start_stereo_teleop.sh   # 无从臂硬件时 mock 模式
FFS_CAMERA=mujoco ./start_stereo_teleop.sh  # 用 MuJoCo 仿真相机
FFS_CMD='...' ./start_stereo_teleop.sh      # 完全自定义 FFS 启动命令
```

### PID 和日志

- PID 文件：`.teleop/janus.pid`, `scene-relay.pid`, `ffs.pid`, `robot_server.pid`
- 日志目录：`.teleop/logs/`
