# 地理背景中的吊舱视觉验证

使用静态环境 [geo_32_217857_119_105878](../../worlds/geo_32_217857_119_105878/README.md)。`prepare_world.py` 从原始 world 生成运行副本，解析地形和影像资源路径，并加入固定三轴吊舱、动态棋盘测试车、带碰撞的平台和实体遮挡板。原始静态 world 不包含这些验证装置。不启动 PX4，不加载载机。

## 启动和安装方式

先按[控制器说明](../../models/gimbals/three_axis_pod/controller/README.md)构建插件，然后在仓库根目录运行：

```bash
bash scenarios/geographic_camera/run.sh --gui --mount-mode skyward
bash models/gimbals/three_axis_pod/ground_station/run.sh --decoder avdec
```

`--gui` 打开连接同一仿真服务的 Gazebo 3D 窗口，省略时只运行服务。地面站依赖 Python OpenCV、NumPy、Pillow、PyGObject 和 GStreamer；调用方可通过 `PYTHONPATH` 提供现有依赖。

| 安装参数 | 实际安装 | 朝天按钮 H | 复位按钮 R |
|---|---|---|---|
| `skyward`（默认） | 底座朝下、机械零位朝天空 | 指向正天 | 公共三轴 0°，水平朝前、画面水平 |
| `downward` | 底座朝上悬挂、安装 roll=π | 可达最高天空仰角约 44° | 公共三轴 0°，水平朝前、画面水平 |

启动入口同时设置模型 pose 和插件 `mount_mode`。两种标准安装中，吊舱固定在 `(0,0,50)` 米，测试车初始在 `(30,0,30.6)` 米；车落在平台后车体中心约 Z=30.35 米。平台为 50×50 米，顶面 Z=30 米。初始化脚本通过 UDP 点动闭环把相机对准车顶，再停止点动；这和 R 的水平复位是不同操作。按 H 可以查看天空。仅静态安装外参参与控制与遥测，不提供载机动态姿态补偿。

协议版本 2 使用 `angle_convention=px4_frd`，初始化与手动输入按公共 Euler 角速度执行，`attitude_deg` 是公共姿态；`joints_deg/cmd_dps` 保留机械诊断。两种安装使用相同方向：yaw 右转为正、pitch 抬头为正、roll 沿镜头朝前看顺时针为正。场景同时设置真实 sensor rpy：立装 `(π,−π/2,0)`、悬挂 `(0,−π/2,0)`，悬挂实际画面朝向随光学安装校正。默认静态参考前向为世界 `+X`，初始化对准车顶时公共 pitch 约 −32.76°。

立装 H 到正天后处于公共 Euler 奇异区，yaw/roll 非零手动输入与这两轴独立回零会被拒绝；先用 pitch 离开正天或按 R 回公共零位。完整姿态可用 `attitude_quat_frd` 表示，奇异时不要以 yaw/roll 单独数值判断朝向。

`sideward` 是安装 Y 轴旋转 +90°、车位于 `(0,0,30.6)` 米的侧装测试配置，用于组件测试；集成程序通常选择上表的标准安装。

场景默认 `GZ_PARTITION=geographic_camera`、运行目录 `runs/geographic_camera/`。视频使用 NVIDIA `nvcudah264enc` 硬编码，客户端 `avdec` 为软件解码。默认 UDP/RTSP/内部 RTP 端口为 8765/8554/5600。并行实例必须使用不同 partition、运行目录及三个端口，例如追加 `--udp-port 8766 --rtsp-port 8555 --rtp-port 5602`。场景控制脚本应使用同一 partition、`--port` 和 `--mount-mode`。

只生成世界副本：

```bash
python3 scenarios/geographic_camera/prepare_world.py --mount-mode skyward --output /tmp/geographic_camera.sdf
```

停止仿真及网关服务：

```bash
GIMBAL_RUNDIR="$PWD/runs/geographic_camera" bash models/gimbals/three_axis_pod/scripts/stop_bridge.sh
GIMBAL_RUNDIR="$PWD/runs/geographic_camera" bash models/gimbals/three_axis_pod/scripts/stop_plugin_sim.sh
```

关闭地面站和 Gazebo GUI 使用各自窗口的退出按钮。GUI 与服务日志及 PID 保存在运行目录。

## 移动目标验收

小车使用仓库的 `test_car`，由 Gazebo `VelocityControl` 插件驱动动态刚体，车轮仅用于显示。它不是 Fuel 下载车型，也没有 DiffDrive 轮式动力学。场景为小车增加世界位姿里程计；位姿真值只用于验收，不反馈跟踪控制。

```bash
python3 scenarios/geographic_camera/check_moving_target.py --mount-mode skyward
```

脚本先停止小车并放回初始位置、初始化相机、选框，然后以 0.5 m/s 完成前进、倒车、转弯和反向转弯，每段 10 秒仿真时间。使用速度指令持续运动，不用位姿服务生成运动轨迹。JSONL 保存到 `runs/geographic_camera/moving_target.jsonl`；重复测试会覆盖该文件。

验收要求：TRACK 且目标有效的遥测比例至少 95%；中心角误差 P95 不超过 3°；跟踪框对应方向与独立车顶真值方向的角差 P95 不超过 3°；实际累计位移至少 10 米；车体高度变化小于 0.3 米。仿真速度降低时墙钟耗时会增长。默认指标针对该速度、目标纹理、视角和轨迹，不代表任意速度或长期跟踪能力。

## 2 秒实体遮挡

```bash
python3 scenarios/geographic_camera/check_occlusion.py --mount-mode skyward
```

脚本自动初始化视角并选框，执行两轮真实遮挡板遮挡；每轮在服务确认后保持 2 秒墙钟，要求出现 TEMP_LOST、未进入终止 LOST、自动恢复 TRACK 且稳定 1 秒。证据保存到 `runs/geographic_camera/occlusion.jsonl`。

人工框选车顶并进入 TRACK 后，也可单次触发：

```bash
python3 scenarios/geographic_camera/control_scene.py occlude --mount-mode skyward --duration 2
python3 scenarios/geographic_camera/control_scene.py clear --mount-mode skyward
```

遮挡板为 20×20 米，放在初始相机与目标中间。服务确认表示位置提交成功，画面生效还受仿真步与视频延迟影响。遮挡板适配初始视角附近；大幅移动车辆或相机后应恢复初始条件再测。初始化相机可单独执行 `check_occlusion.py --prepare-only --mount-mode skyward`。

人工观察实际小车运动（60 秒后自动停止；运行中可按 Ctrl+C 停车）：

```bash
python3 scenarios/geographic_camera/control_scene.py drive --duration 60 --speed 0.5 --turn 0.12
```

目标位置复位使用 `control_scene.py target --mount-mode skyward`；显式 `--x`/`--y` 可用于重新布置位置。这是场景布置操作，不计作移动目标验收。

环境提供以 WGS84 `(32.21785737811048,119.1058778440883)` 为中心、覆盖半径 5 km 的真实高程、地表影像、静态天空和太阳。精度和数据来源见 world 说明。完整 400 块地形同时加载，必须确认图像和遥测就绪，不能只以进程存活判断可用。
