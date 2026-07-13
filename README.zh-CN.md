# Dobot IMU 姿态平衡补偿工程

> 文档版本：v1.0
>
> 日期：2026-5-28

------

本工程是一个面向 ROS 2 Python 项目，用于展示 Dobot 四足机器人如何基于底层 IMU 姿态数据完成 roll/pitch 平衡补偿。

系统通过 DDS 只读订阅机器人底层状态话题 `rt/lower/state`，提取 IMU 四元数、角速度、加速度和 RPY 姿态；随后在 ROS 2 内完成姿态发布、滤波、偏差计算、PID/P 控制、限幅和冷却控制；当姿态偏差超过阈值时，通过 Dobot 高层 gRPC SDK 调用 `balance_roll`、`balance_pitch` 或 `dynamic_pose` 执行姿态补偿动作。

本 Demo 的定位是“底层只读 + 高层动作补偿”。它不直接发布 `LowerCmd`，不做电机级闭环控制（直接控制电机操作不当易**造成严重安全事故**），也不调用 `kill_robot`。机器人站立、关节保护、状态切换和动作安全边界由机器人主控和高层 SDK 负责。

## 工程功能：

工程当前包含以下核心能力：

| 功能模块     | 实现文件                                   | 主要职责                                                     |
| ------------ | ------------------------------------------ | ------------------------------------------------------------ |
| IMU 数据读取 | `imu_reader_node.py`                       | 通过 `dds_middleware_python` 订阅 `rt/lower/state`，解析 IMU 数据并发布 ROS 2 姿态话题 |
| 姿态补偿控制 | `balance_controller_node.py`               | 对 roll/pitch 做滑动平均和低通滤波，计算姿态误差和 PID 输出，并触发高层 SDK 姿态动作 |
| 终端监控面板 | `demo_monitor_node.py`                     | 面向演示场景汇总显示 IMU、原始姿态、滤波姿态、误差、命令和触发事件 |
| 曲线记录导出 | `attitude_plot_recorder_node.py`           | 可选记录 `/balance_control/attitude`，退出时生成 CSV 和自包含 HTML 曲线 |
| 参数配置     | `config.py`、`balance_control_config.yaml` | 加载 YAML，解析 DDS 和 CycloneDDS 路径，集中管理控制参数     |
| 一键启动     | `balance_control.launch.py`                | 启动 IMU 读取、控制器、监控面板，并按需启动曲线记录节点      |

## 数据流

```text
Dobot 底层状态 DDS: rt/lower/state
        |
        v
imu_reader_node
        |-- /balance_control/imu_raw
        |-- /balance_control/imu_rpy
        |-- /balance_control/imu_status
        v
balance_controller_node
        |-- 滑动平均 + 低通滤波
        |-- 姿态误差计算
        |-- PID/P 输出
        |-- 死区、限幅、动作冷却
        |-- 高层 SDK: balance_roll / balance_pitch / dynamic_pose
        |
        |-- /balance_control/attitude
        |-- /balance_control/attitude_report
        |-- /balance_control/state
        |
        +------------------------+
        |                        |
        v                        v
demo_monitor_node        attitude_plot_recorder_node
终端演示面板               CSV / HTML 曲线留档
```

## 控制逻辑

1. `imu_reader_node` 从 DDS 收到 IMU RPY，按弧度发布到 `/balance_control/imu_rpy`。
2. `balance_controller_node` 将 roll/pitch 转为角度，先做滑动平均，再做一阶低通滤波。
3. 控制器计算滤波姿态与目标姿态的误差，默认目标为 `roll=0 deg`、`pitch=0 deg`。
4. PID 输出经过方向符号、最大补偿角限制、稳定死区和动作冷却处理。
5. 当 roll 或 pitch 误差超过 `trigger_threshold_deg` 时，后台线程调用高层 SDK 执行补偿动作。
6. 节点持续发布姿态数组、文本报告和状态事件，供终端面板、曲线记录或 `rqt_plot` 使用。

## 安全边界

本工程适合演示“IMU 姿态感知 + 高层动作补偿”的闭环思路，但不是电机级控制器。

| 项目                | 当前策略                                                     |
| ------------------- | ------------------------------------------------------------ |
| DDS 传感器数据      | 只读订阅，读取 IMU 状态                                      |
| 机器人动作          | 通过高层 gRPC SDK 触发姿态动作                               |
| 电机命令 `LowerCmd` | 不发布，不直接控制关节                                       |
| `kill_robot`        | 不调用                                                       |
| 实机调试            | 建议先使用 `enable_execute: false` 观察数据和控制输出，再开启动作执行 |

如果需要开发关节级闭环控制，应另建底层控制项目，并准备保护绳、急停、支撑架、离地调试流程和严格的安全评审。

## 目录结构

```text
dobot_balance_control_demo/
├── README.md
├── assets
├── balance_outputs
├── dobot_quad_sdk-main
└── src/
    └── dobot_balance_control_demo/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── config/
        │   └── balance_control_config.yaml
        ├── launch/
        │   └── balance_control.launch.py
        ├── resource/
        │   └── dobot_balance_control_demo
        └── dobot_balance_control_demo/
            ├── __init__.py
            ├── attitude_plot_recorder_node.py
            ├── balance_controller_node.py
            ├── config.py
            ├── demo_monitor_node.py
            └── imu_reader_node.py
```

## 环境要求

| 项目 | 要求 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| Python | 3.10 |
| 网络 | 开发机与机器狗通过有线网络连接，位于 `192.168.5.x` 网段 |
| SDK | `dobot_quad_sdk-main` 位于工程根目录 |

默认机器人地址：

```text
192.168.5.2:50051
```

DDS IMU 链路依赖有线网络连接。使用虚拟机时，连接机器狗的网卡建议设置为桥接模式。

## 安装依赖

安装系统依赖：

```bash
sudo apt update
sudo apt install -y python3-venv python3-pip python3-colcon-common-extensions
```

创建并激活虚拟环境：

```bash
cd ~/dobot_balance_control_demo
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
```

安装 ROS 2 Python 构建依赖：

```bash
python -m pip install empy==3.3.4 catkin_pkg lark pyyaml numpy
```

安装 Dobot 高层 gRPC SDK：

```bash
cd ~/dobot_balance_control_demo/dobot_quad_sdk-main/high_level/python
python -m pip install -e .
```

安装 DDS 中间件和 Python 依赖：

```bash
cd ~/dobot_balance_control_demo/dobot_quad_sdk-main/dist
sudo dpkg -i dds-middleware-with-thirdparty*.deb
export CYCLONEDDS_HOME="/usr/local/"
python -m pip install dds_middleware_python-*.whl
python -m pip install cyclonedds pyyaml numpy
```

验证依赖：

```bash
python -c "import dobot_quad; print('dobot_quad ok')"
python -c "import dds_middleware_python; print('dds ok')"
```

## 网络配置

确认连接机器狗的有线网卡名称：

```bash
ip a
```

如需手动设置开发机 IP：

```bash
sudo ip addr add 192.168.5.100/24 dev <网卡名>
sudo ip link set <网卡名> up
```

编辑 CycloneDDS 配置：

```bash
nano ~/dobot_balance_control_demo/dobot_quad_sdk-main/cyclonedds.xml
```

将文件中的网卡名称修改为连接机器狗的有线网卡名。启动前设置：

```bash
export CYCLONEDDS_URI=file:///home/dobot/dobot_balance_control_demo/dobot_quad_sdk-main/cyclonedds.xml
```

如果工程路径不是 `/home/dobot/dobot_balance_control_demo`，请替换为实际路径。

## 配置说明

配置文件：

```text
src/dobot_balance_control_demo/config/balance_control_config.yaml
```

关键参数：

```yaml
robot:
  grpc_addr: "192.168.5.2:50051"
  enter_balance_stand_on_start: true
  enable_execute: true

dds:
  config_file: "dobot_quad_sdk-main/low_level/python/config/dds_config.yaml"
  domain_id: 0
  lower_state_topic: "rt/lower/state"
  publish_period_seconds: 0.02

filter:
  low_pass_alpha: 0.25
  moving_average_window: 5

control:
  target_roll_deg: 0.0
  target_pitch_deg: 0.0
  trigger_threshold_deg: 3.0
  settle_threshold_deg: 1.5
  max_compensation_deg: 10.0
  action_duration_seconds: 0.8
  action_cooldown_seconds: 1.2
  roll_output_sign: -1.0
  pitch_output_sign: -1.0
  kp_roll: 0.8
  ki_roll: 0.0
  kd_roll: 0.08
  kp_pitch: 0.8
  ki_pitch: 0.0
  kd_pitch: 0.08
  integral_limit_deg_s: 8.0
  compensation_mode: "combined"

display:
  monitor_period_seconds: 0.5
  stale_timeout_seconds: 2.0

plot:
  output_dir: "balance_outputs"
  max_samples: 5000
```

常用调参说明：

| 参数                                                     | 说明                                                         |
| -------------------------------------------------------- | ------------------------------------------------------------ |
| `robot.enable_execute`                                   | 为 `false` 时只发布控制计算和日志，不调用机器人动作，适合无实机预演或安全调参 |
| `robot.enter_balance_stand_on_start`                     | 启动控制器后是否自动调用高层 SDK 进入平衡站立                |
| `filter.low_pass_alpha`                                  | 低通滤波系数，越小越平滑，越大响应越快                       |
| `filter.moving_average_window`                           | 滑动平均窗口长度                                             |
| `control.trigger_threshold_deg`                          | 姿态偏差超过该阈值才触发补偿                                 |
| `control.settle_threshold_deg`                           | 稳定死区，误差低于该值时对应轴输出置零                       |
| `control.max_compensation_deg`                           | 单次补偿角度限幅                                             |
| `control.action_cooldown_seconds`                        | 两次补偿动作的最小间隔，避免连续触发高层动作接口             |
| `control.roll_output_sign` / `control.pitch_output_sign` | 补偿方向符号；实机补偿方向相反时优先改这里                   |
| `control.compensation_mode`                              | `combined` 使用 `dynamic_pose` 同时补偿 roll/pitch；`axis` 分别调用 `balance_roll` 和 `balance_pitch` |
| `plot.output_dir`                                        | 开启曲线记录后 CSV/HTML 输出目录；相对路径基于执行 `ros2 launch` 的当前目录 |

## 编译

```bash
cd ~/dobot_balance_control_demo
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select dobot_balance_control_demo
source install/setup.bash
```

每次新开终端运行前，建议按顺序加载环境：

```bash
cd ~/dobot_balance_control_demo
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 启动

演示启动：

```bash
cd ~/dobot_balance_control_demo
source .venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export CYCLONEDDS_URI=file:///home/dobot/dobot_balance_control_demo/dobot_quad_sdk-main/cyclonedds.xml
ros2 launch dobot_balance_control_demo balance_control.launch.py
```

启动并记录曲线：

```bash
ros2 launch dobot_balance_control_demo balance_control.launch.py record_plot:=true
```

正常日志示例：

![截图 2026-05-15 17-19-38](./assets/01.png)

控制台字段含义：

| 字段 | 含义 | 课堂观察方式 |
| --- | --- | --- |
| `[IMU]` | DDS 读取到的 IMU 姿态，单位为度 | `OK` 表示 IMU 数据正在更新 |
| `[RAW]` | 控制节点收到的原始 roll/pitch | 摇晃或轻按机器人时应快速变化 |
| `[FILTER]` | 滤波后的 roll/pitch | 比 RAW 更平滑，允许有轻微延迟 |
| `[ERROR]` | 当前姿态相对目标姿态的偏差 | 默认目标为 0 度，因此通常接近 FILTER |
| `[CMD]` | 控制器计算出的补偿角度 | 超过稳定阈值时可能出现非零输出 |
| `[EVENT]` | 最近事件和补偿触发次数 | 出现 `[TRIGGER] compensation` 表示触发补偿 |

## ROS 2 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/balance_control/imu_raw` | `std_msgs/Float32MultiArray` | `[quat4, gyro3, accel3, rpy3]`，rpy 单位 rad |
| `/balance_control/imu_rpy` | `std_msgs/Float32MultiArray` | `[roll, pitch, yaw]`，单位 rad |
| `/balance_control/attitude` | `std_msgs/Float32MultiArray` | `[raw_roll, raw_pitch, filtered_roll, filtered_pitch, cmd_roll, cmd_pitch]`，单位 deg |
| `/balance_control/attitude_report` | `std_msgs/String` | 可选调试话题：单帧姿态计算文字报告 |
| `/balance_control/state` | `std_msgs/String` | 控制状态和动作事件 |
| `/balance_control/imu_status` | `std_msgs/String` | IMU 等待或异常状态 |

## 曲线记录输出

开启曲线记录：

```bash
ros2 launch dobot_balance_control_demo balance_control.launch.py record_plot:=true
```

演示结束后按 `Ctrl+C` 正常退出，程序会在 `balance_outputs/` 下生成：

```text
balance_attitude_YYYYMMDD_HHMMSS.csv
balance_attitude_YYYYMMDD_HHMMSS.html
```

CSV 字段：

```text
time_s, raw_roll_deg, raw_pitch_deg, filtered_roll_deg, filtered_pitch_deg, command_roll_deg, command_pitch_deg
```

HTML 是自包含 SVG 曲线文件，可直接用浏览器打开，包含：

- `raw_roll` / `raw_pitch`：原始姿态角
- `filtered_roll` / `filtered_pitch`：滤波后的姿态角
- `cmd_roll` / `cmd_pitch`：控制器输出的补偿角度

示例曲线：

![截图 2026-05-15 17-20-37](./assets/02.png)

## 常见问题

### 订阅不到 IMU 数据

检查：

- `CYCLONEDDS_URI` 是否指向有效的 `cyclonedds.xml`。
- CycloneDDS 配置中的网卡名是否正确。
- 开发机是否与机器狗处于 `192.168.5.x` 有线网段。
- 启动终端是否已设置 `CYCLONEDDS_URI`。

### 没有生成 HTML 曲线

检查：

- 启动命令是否带了 `record_plot:=true`。
- 是否已经按 `Ctrl+C` 正常退出 launch。曲线文件在退出时生成。

- 是否在工程根目录启动。默认输出目录为：

```text
~/dobot_balance_control_demo/balance_outputs
```

