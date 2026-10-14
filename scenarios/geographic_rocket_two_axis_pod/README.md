# 地理环境中的火箭机与两轴吊舱

场景使用 `worlds/geo_32_217857_119_105878/world.sdf` 的完整运行副本，保留 400 块地形、影像与天空。源 world 不改写。飞机由 `assembledmodels/rocket_two_axis_pod/` 的动态生成器组合，静态外形已通过人工确认；当前配置通过两次完整往返、降落及 70 仿真秒真实图像跟踪自动检查，结果与范围见[验收说明](docs/validation.md)。场景效果已通过人工确认。

## 飞行与目标

PX4 控制真实动力学飞行。默认相对起飞点高度 **80 m**、航段中部速度 **15 m/s**，在起点 A 和北侧 300 m 的 B 点之间持续往返。Gazebo ENU 中北为 +Y，MAVLink NED 中北为 +X。根据 PX4 实际位置反馈减速，到端点低速后反向；保持航向连续，通过四旋翼倾转改变运动方向。相对起飞点高度不等于沿线各处的离地高度。

空中视觉靶标 `AERIAL_TRACKING_TARGET` 是固定在 ENU `(200,150,150) m` 的 8 m 立方体夹具，每面都有非对称彩色棋盘。它没有碰撞体，也不是另一架飞机。目标相对航线位于东侧和上方，供人通过吊舱视频框选。默认位置和尺寸在 `target_fixture.py` 中配置；修改后重新生成和启动世界。

吊舱保持已确认安装：底面在机体系 `(0,0,0.134) m`，各同名轴与机体对齐。yaw 绕机体 +Z，pitch 绕随动 +Y；零位朝机头 +Z，pitch=+90° 转向机体 +X。机械范围为 yaw ±90°、pitch −45°～+90°，朝安装 +Z 时偏航视线控制退化。两轴不提供独立滚转稳定，不能保证任意姿态下目标都可达。

图像跟踪使用组件 CSRT 与载机转动补偿；目标世界坐标不参与持续控制。可达时持续跟踪，限位、速度饱和、视线退化与目标丢失应在地面站明确显示；不能仅以 TRACK 字样判断稳定锁定。

## 启动与人工验收

以下命令在仓库根目录执行。需要 Gazebo Harmonic、已构建的相邻 `../PX4-Autopilot`、两轴控制插件，以及 Python 的 pymavlink、NumPy、Gazebo Transport/Messages 绑定。网关和地面站另需 GStreamer、PyGObject、Pillow、OpenCV；已有独立环境可用 `GIMBAL_PYTHON` 指定。`PX4_ROOT` 可覆盖 PX4 路径，`FLIGHT_PYTHON` 可覆盖飞行脚本解释器。

```bash
# 持续往返并打开 Gazebo；Ctrl+C 或 stop.sh 请求降落后停止
GIMBAL_PYTHON=../two_axis_pod_venv/bin/python \
  bash scenarios/geographic_rocket_two_axis_pod/run.sh --gui

# 另一个终端打开吊舱地面站
GIMBAL_PYTHON=../two_axis_pod_venv/bin/python \
  bash models/gimbals/two_axis_pod/ground_station/run.sh \
  --udp-port 8770 --rtsp-port 8558 --decoder avdec

# 有限飞行检查：两次完整往返后自动降落并停止服务
bash scenarios/geographic_rocket_two_axis_pod/run.sh --cycles 2 \
  --run-dir "$PWD/runs/geographic_rocket_two_axis_pod_check"

# 请求正常停止，等待启动器完成有界降落和进程清理
bash scenarios/geographic_rocket_two_axis_pod/stop.sh \
  "$PWD/runs/geographic_rocket_two_axis_pod"
```

默认运行目录为 `runs/geographic_rocket_two_axis_pod/`，可用 `--run-dir` 设置。每次启动生成独立 Gazebo 分区、动态资产、PX4 `etc` 与新的 rootfs；不修改相邻 PX4 源码。实际分区保存在运行目录 `gz_partition.txt`。同一个运行目录有活跃进程时拒绝重复启动。PX4/MAVLink 端口不由 Gazebo 分区隔离，端口占用时拒绝启动。场景使用 UDP 8770、RTSP 8558、内部 RTP 5608；额外主动状态 UDP 在本场景配置中关闭。

飞机起飞并达到巡航高度后，在吊舱地面站用两轴摇杆寻找空中棋盘靶标，再在实际视频中拖框选择它。观察来回航段及减速、反向时目标是否保持居中，并检查限制提示。只有一个客户端应操作吊舱；检查脚本自动初始化时，不要同时人工控制。

## 定位与相机

飞机实体名为 `rocket_two_axis_pod`。起飞点默认 `(0,0)`，机体出生原点世界 Z 为 0.24 m；`prepare_world.py --altitude` 配置的是出生 Z，不是巡航高度。起点旁标志名为 `ROCKET_TAKEOFF_LOCATION_MARKER`，橙黑杆位于起点西南侧 `(-2,-2)` 偏移处，顶牌约 3.1 m 高，黄色箭头指向起点；无碰撞，不随飞机移动。

在 Gazebo 实体树搜索飞机并聚焦或跟随，起飞后仍可定位它。`run.sh --close` 提供起飞点吊舱近景入口；查看已确认静态外形也可使用装配目录的独立预览，见[装配说明](../../assembledmodels/rocket_two_axis_pod/README.md)。

浏览相机近裁剪距离默认 **0.005 m**、鼠标灵敏度默认 **0.1**。近景缩放时将鼠标放在吊舱实体表面，避免指向远处地形或天空；右键缓慢拖动可微调。`config.json` 的 `view_near_clip_m` 在重新生成世界后生效，`view_sensitivity` 由启动器在 GUI 服务就绪后设置；`GZ_VIEW_SENSITIVITY` 可覆盖灵敏度。当前 GUI 可在同一分区手动调整：

```bash
GZ_PARTITION="$(cat runs/geographic_rocket_two_axis_pod/gz_partition.txt)" \
  gz service -s /gui/camera/view_control/sensitivity \
  --reqtype gz.msgs.Double --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: 0.1'
```

以上是 Gazebo 浏览相机。吊舱视频相机仍沿用基础组件 1920×1080、30 Hz、水平视场 60°；其参数与浏览相机独立。基础组件参数的修改须先按仓库规则取得用户确认。

## 配置、记录与验收范围

`config.json` 配置飞行高度、速度、航段长度、加速度、端点容差、超时和浏览相机参数；启动器 `--config` 可指定配置文件，修改后重新启动。`--cycles 0` 持续巡航，正整数指定完整往返次数。

运行目录保留 `world.sdf`、动态模型及参数指纹、`gazebo.log`、`px4.log`、`bridge.log`、`flight.log`、`flight_result.json` 和流式 `flight_result.jsonl`。飞行控制使用 PX4 本机位置；Gazebo 世界位姿只用于独立检查端点、高度、横向偏差和航段速度，不参与航迹控制。

`check_tracking.py` 默认观察人工框选。`--initialize` 明确启用一次机械对准与视频框选，用于自动检查；初始化结束后不发布持续目标控制指令。图像、实际关节遥测和独立载机位姿用于核对 CSRT 框与真实靶标，识别跟错天空或其他物体的情况。完整参数可运行 `--help` 查看；原始记录、摘要和带独立目标投影的快照保存在指定输出目录。真实飞行与跟踪通过状态需以实际运行报告为准。

```bash
# 飞行场景启动后，另一个终端只观察人工框选；自动检查可显式加 --initialize
GZ_PARTITION="$(cat runs/geographic_rocket_two_axis_pod/gz_partition.txt)" \
  ../two_axis_pod_venv/bin/python scenarios/geographic_rocket_two_axis_pod/check_tracking.py \
  --manual-only --duration 70 --wait-timeout 600 \
  --output runs/geographic_rocket_two_axis_pod/tracking

# 仅对飞行、降落和独立跟踪均已通过的完整运行记录生成摘要、曲线和压缩证据
python3 scenarios/geographic_rocket_two_axis_pod/report_validation.py \
  --run-dir runs/geographic_rocket_two_axis_pod_validation2
```
