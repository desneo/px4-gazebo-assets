# 火箭穿越机自验证

本目录包含 `rocket_racing_quad` 的自验证世界、启动脚本和飞行验收工具。

- [world.sdf](world.sdf)：独立自验证世界，内置模型与所需系统插件。
- [run.sh](run.sh)：资源路径、IMU 就绪等待及 PX4 SITL 启动入口。
- [run_check.sh](run_check.sh)：自动启动、使用实际运行配置验收、退出时停止仿真的单入口。
- [flight_check.py](flight_check.py)：通过 MAVLink 控制飞行，使用选定数据源验收悬停、巡航和冲刺，并正常降落。
- [gazebo_truth.py](gazebo_truth.py)：订阅Gazebo世界位姿，转换坐标并计算物理真值速度。
- [check_model.py](check_model.py)：检查生成一致性、几何净空、质量、坐标映射和参数输入。
- [test_flight_check.py](test_flight_check.py)：MAVLink筛选、通信、仿真时间和连续达标逻辑回归。

## 使用

以下命令均从仓库根目录执行。需要 Python 3、NumPy、Gazebo Sim；SITL 模式需要已构建 PX4，飞行验收还需要 `pymavlink`；单入口真值验收需要 Gazebo 的 Python `gz.transport13` / `gz.msgs10` 模块。

```sh
# 单入口：自动启动、验收、停止仿真；默认结果为运行目录/flight_result.json
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless

# 自定义配置和结果路径；带界面可将headless改为sitl
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless \
  /tmp/rocket-test.json /tmp/rocket-flight-review.json

# 查看外形：自动从配置生成独立资产，无需PX4
sh models/vehicles/rocket_racing_quad/tests/run.sh view

# 接入PX4，无图形界面；带界面使用sitl
sh models/vehicles/rocket_racing_quad/tests/run.sh headless

# 自定义配置（第二个参数）与PX4工作区
PX4_ROOT=/path/to/PX4-Autopilot \
  sh models/vehicles/rocket_racing_quad/tests/run.sh headless /tmp/rocket-test.json

# PX4启动后，在另一终端使用启动器实际输出的配置路径
python3 models/vehicles/rocket_racing_quad/tests/flight_check.py \
  --config /tmp/rocket-racing.XXXXXX/models/rocket_racing_quad/parameters.json \
  --log /tmp/rocket-flight-review.json

# 覆盖本次验收速度、持续时间和高度，仍需提供实际配置
python3 models/vehicles/rocket_racing_quad/tests/flight_check.py \
  --config /tmp/rocket-racing.XXXXXX/models/rocket_racing_quad/parameters.json \
  --cruise-speed 200 --cruise-seconds 30 \
  --sprint-speed 300 --sprint-seconds 10 --altitude 30 \
  --log /tmp/rocket-flight-review.json
```

`run_check.sh` 自动选择启动器生成的配置快照和通信分区，使用 Gazebo 世界位姿真值判定速度、漂移和高度，并在验收结束、失败或Ctrl+C时停止本次仿真，运行目录及日志保留；验收退出码仍反映是否完整通过。分步启动模式适合保留仿真进行诊断。

`XXXXXX` 为示例占位符，使用启动器输出中的真实目录。默认PX4工作区为 `/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot`，可用 `PX4_ROOT` 覆盖；脚本要求该工作区有 `build/px4_sitl_default/bin/px4` 和构建的 `etc`。

`run.sh` 从所选JSON生成 `/tmp/rocket-racing.XXXXXX/models/rocket_racing_quad`，设置 `GZ_SIM_RESOURCE_PATH` 和独立 `GZ_PARTITION`。SITL模式复制已构建PX4的 `etc` 并装入本次生成的机体文件，使用独立rootfs，不修改相邻PX4源码或其已有参数存储。等待IMU首帧后，以 `PX4_GZ_STANDALONE=1` 接入PX4，估计器仍需初始化。运行目录保留 `gazebo.log`、`px4.log`、`imu_sample.txt`、配置快照和资产；按Ctrl+C停止启动器及其仿真进程。MAVLink的UDP端口未因Gazebo分区自动隔离，同时运行其他SITL时应检查端口冲突。

世界名为 `rocket_racing_quad_test`。原点WGS84为 `(47.397971057728974, 8.546163739800146)`，海拔0 m。模型机头朝+z，生成器按机身半长与离地间隙计算初始高度。世界参数来自JSON，非参数化场景内容来自 `tools/world_template.sdf`。

飞行脚本默认读取模型根目录的 [parameters.json](../parameters.json)，但用启动器复测时应通过 `--config` 指定**本次运行副本中的配置**，使验收与实际模型、PX4设置一致。源配置改变不会改变已启动的模型；模型、动力、PX4或仿真配置变化后须停止并重新启动。常调参数的单位、影响和生成流程见[参数说明](../docs/parameters.md)。

常用覆盖项见 `--help`。`--target-speed` 保留兼容，单位为m/s；`--cruise-speed` 与 `--sprint-speed` 单位为km/h。命令行覆盖只影响本次验收，不会改写配置文件或飞控。脚本启动前检查目标速度加控制余量不超过配置中的 `MPC_XY_VEL_MAX`；提高目标速度时应先调整实际PX4参数并重启。`MPC_XY_CRUISE` 是PX4任务模式设置，本脚本的OFFBOARD巡航目标取自 `flight_test.cruise_speed_km_h`。

## 静态检查与预览

```sh
# 从仓库根目录执行；静态检查会生成临时资产用于比对
python3 models/vehicles/rocket_racing_quad/tests/check_model.py
python3 -m unittest discover -s models/vehicles/rocket_racing_quad/tests -p 'test_*.py' -v

# 根据配置和网格生成docs/model_views.png；需要matplotlib
python3 models/vehicles/rocket_racing_quad/tools/render_views.py
```

`check_model.py` 核对仓库资产与配置重生成结果、桨盘/机身净空、网格质量、关节及总质量、世界地面显示/碰撞大小、PX4电机位置和参数映射，并检查修改配置确实影响输出及无效输入被拒绝。飞行回归测试验证遥测过滤、心跳、命令确认、时钟和严格连续速度/高度门槛；这些离线检查不能代替SITL飞行。

[多视图预览](../docs/model_views.png) 由网格和配置绘制，包含多个观察方向；它不是Gazebo运行截图，仍需人工确认外形。默认地面边长50 km，显示与碰撞同步，以容纳高速航段。

## 飞行流程与判定

脚本要求飞控初始未解锁，拒绝接管已有飞行。通过 PX4 模式接口发送主模式和子模式，并用飞控心跳确认模式、解锁状态。所有等待循环持续发送 20 Hz OFFBOARD 设定值及 1 Hz 地面站心跳。

1. 预发送设定值 2 秒，进入 OFFBOARD 并解锁。
2. 按配置爬升到相对起点 30 m，等待高度和垂直速度稳定。
3. 悬停 30 秒：水平位置相对悬停起点的最大距离不超过 1 m，高度误差不超过 2 m。
4. 平滑加速后，水平速度连续至少 30 秒达到 200 km/h，且每个计时样本的高度误差不超过 2 m。
5. 再加速后，水平速度连续至少 10 秒达到 300 km/h，同样满足高度限制。
6. 平滑减速至低于 1 m/s，切换 LAND，等待飞控报告落地并自动解除解锁。

XY 方向使用速度设定值，Z 方向使用位置设定值，由 PX4 保持高度。速度设定值默认比验收门槛高 15 km/h，验收仍严格采用 200 / 300 km/h，没有百分比折扣。速度低于门槛或高度条件中断后，连续计时重新开始；不累计分散的达标时间。速度是本地 NED 水平地速，当前世界无风时等于水平空速。

航段和加速调度采用 `LOCAL_POSITION_NED.time_boot_ms` 仿真时钟；`run_check.sh` 的连续保持和漂移/高度判定采用 Gazebo 位姿时间戳与 ENU→本地NED变换，速度由相邻位置差分计算。仅运行 `flight_check.py` 且未加 `--ground-truth` 时，指标使用 PX4 本地位置估计；通信新鲜度与安全超时采用单调墙钟。重复遥测不增加时间，时钟倒退、位置遥测或飞控心跳超时、OFFBOARD 意外退出均终止验收。等待达速另受 `speed_acquire_timeout_s` 限制，每个航段也有 `phase_wall_timeout_s` 安全期限。低实时率仿真下若安全期限不足，应调整配置。

异常或 Ctrl+C 后，脚本尝试发送 LAND 并等待落地；不会在空中发送 disarm。若通信断开，降落无法确认会记入日志，实际后续动作由 PX4 失联策略处理。脚本不会修改失联策略。拒绝接管已解锁飞控时不会发送 LAND。

## 可调参数

所有数值在 `flight_test` 节点内。长度单位为 m，时间为 s，速度字段以名称中的单位为准。

| 参数 | 用途 |
| --- | --- |
| `endpoint` | MAVLink 连接地址，默认 `udpin:0.0.0.0:14540` |
| `setpoint_rate_hz` | OFFBOARD 发送频率，至少 10 Hz，默认 20 Hz |
| `telemetry_timeout_s` / `heartbeat_timeout_s` | 位置/落地状态与心跳新鲜度限制 |
| `command_timeout_s` | 模式、解锁确认期限 |
| `phase_wall_timeout_s` / `landing_timeout_s` | 航段与降落墙钟期限 |
| `takeoff_altitude_m` / `climb_speed_m_s` | 起飞高度及高度设定值爬升速率 |
| `altitude_tolerance_m` | 悬停、巡航、冲刺高度容差 |
| `hover_seconds` / `hover_drift_limit_m` | 悬停时长及相对起点最大水平漂移 |
| `cruise_speed_km_h` / `cruise_seconds` | 巡航速度门槛及连续时长 |
| `sprint_speed_km_h` / `sprint_seconds` | 冲刺速度门槛及连续时长 |
| `ramp_seconds` | 每次加速设定值的线性爬升时长 |
| `speed_setpoint_margin_km_h` | 控制设定速度相对验收门槛的余量 |
| `speed_acquire_timeout_s` | 允许到达并稳定在验收条件的附加仿真时间 |
| `deceleration_m_s2` | 减速设定值的变化率 |

## 日志与验证状态

每次运行写入 `/tmp/rocket-flight-<时间戳>.json`，可用 `--log` 指定路径。包含实际配置与SHA-256、判定数据来源、Gazebo真值（启用时）、PX4 NED 位置和速度、仿真时间、航段、控制设定值、飞控状态文本、各项结果、错误及落地确认。只有所有航段通过且确认落地时，顶层 `passed` 才为 `true`，程序退出码为 0；其余情况为 1。可依据原始样本重新计算漂移、连续时长和高度误差。

当前低位电机与下置桨布局已通过静态/SDF检查，脚本回归通过；80 mm 动力舱外形已通过人工验收，当前配置完整SITL飞行验收待复测。具体状态见[验证说明](../docs/validation.md)。

独立运行真值验收时，先将 `GZ_PARTITION` 设置为启动器打印的分区，并给 `flight_check.py` 加 `--ground-truth`。真值模式不会改写飞控传感器；遥测和真值任一失联都会终止验收。地面显示为平面，碰撞采用顶部位于 z=0 的实体箱体。
