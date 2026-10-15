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
  --udp-port 8770 --rtsp-port 8555 --aircraft-id 1 --decoder avdec

# 有限飞行检查：两次完整往返后自动降落并停止服务
bash scenarios/geographic_rocket_two_axis_pod/run.sh --cycles 2 \
  --run-dir "$PWD/runs/geographic_rocket_two_axis_pod_check"

# 请求正常停止，等待启动器完成有界降落和进程清理
bash scenarios/geographic_rocket_two_axis_pod/stop.sh \
  "$PWD/runs/geographic_rocket_two_axis_pod"
```

默认运行目录为 `runs/geographic_rocket_two_axis_pod/`，可用 `--run-dir` 设置。每次启动生成独立 Gazebo 分区、动态资产、PX4 `etc` 与新的 rootfs；不修改相邻 PX4 源码。实际分区保存在运行目录 `gz_partition.txt`。同一个运行目录有活跃进程时拒绝重复启动。PX4/MAVLink 端口不由 Gazebo 分区隔离，端口占用时拒绝启动。场景使用 UDP 8770、共享 RTSP 8555、自动分配的内部 RTP 端口，默认视频路径 `/two_axis_pod_1`；可用 `run.sh --aircraft-id ID` 修改视频与上报 ID，地面站配置相同 ID。此参数不修改 PX4 SYSID 或 MAVLink 端口，本场景仍为单载机飞行入口；额外主动状态 UDP 在本场景配置中关闭。

飞机起飞并达到巡航高度后，在吊舱地面站用两轴摇杆寻找空中棋盘靶标，再在实际视频中拖框选择它。观察来回航段及减速、反向时目标是否保持居中，并检查限制提示。只有一个客户端应操作吊舱；检查脚本自动初始化时，不要同时人工控制。

## 多机集成的地址与端口对应

每架飞机绑定一个吊舱网关。集成时为每架飞机记录对应的飞机ID、吊舱UDP控制/遥测地址、完整RTSP URL及Gazebo话题。**同一主机上的吊舱网关分别使用不同UDP控制端口；RTSP可以共用8555端口，通过不同路径区分飞机。** 吊舱UDP端口用于云台指令、遥测和回执，PX4飞行控制使用独立的MAVLink连接。

下表是同一主机上的两机接口配置示例，端口由启动配置显式分配；不是按飞机ID自动计算，也不表示当前场景已提供两机同时飞行入口。

| 配置项 | 飞机23 | 飞机24 |
|---|---|---|
| 网关与地面站 `--aircraft-id` | `23` | `24` |
| 吊舱UDP控制/遥测目标 | `127.0.0.1:8770` | `127.0.0.1:8771` |
| 网关与地面站 `--udp-port` | `8770` | `8771` |
| 共享 `--rtsp-port` | `8555` | `8555` |
| 网关与地面站 `--mount` 前缀 | `/two_axis_pod` | `/two_axis_pod` |
| 完整RTSP URL | `rtsp://127.0.0.1:8555/two_axis_pod_23` | `rtsp://127.0.0.1:8555/two_axis_pod_24` |
| 网关 `--topic-prefix`，同一Gazebo分区内分别配置 | `/uav_23/pod` | `/uav_24/pod` |
| 网关 `--image-topic`，须对应实际相机发布话题 | `/uav_23/pod/image` | `/uav_24/pod/image` |

RTSP路径由 `--mount` 前缀追加 `_飞机ID` 生成，传入 `/two_axis_pod`，不要再手工追加ID。地面站的目标主机、UDP端口、飞机ID、RTSP端口和路径前缀应与同一架飞机的网关配置一致。跨主机连接时将 `127.0.0.1` 替换为实际网关地址；网关的 `--udp-host` 是监听地址，地面站的 `--udp-host` 是连接目标。

以下地面站命令对应上表；前提是多机仿真已发布各自话题，并已按表分别启动两个网关：

```bash
GIMBAL_PYTHON=../two_axis_pod_venv/bin/python \
  bash models/gimbals/two_axis_pod/ground_station/run.sh \
  --udp-host 127.0.0.1 --udp-port 8770 --aircraft-id 23 \
  --rtsp-port 8555 --mount /two_axis_pod --decoder avdec

GIMBAL_PYTHON=../two_axis_pod_venv/bin/python \
  bash models/gimbals/two_axis_pod/ground_station/run.sh \
  --udp-host 127.0.0.1 --udp-port 8771 --aircraft-id 24 \
  --rtsp-port 8555 --mount /two_axis_pod --decoder avdec
```

同一Gazebo分区内，每个吊舱的command、telemetry、command_result及图像话题分别对应自己的插件和相机；仅更改网关参数不会修改模型发布的话题。不同 `GZ_PARTITION` 可以隔离Gazebo话题，但不会隔离UDP、RTSP或MAVLink端口。内部RTP端口默认自动分配，显式配置 `--rtp-port` 时也需分别分配。共享RTSP中继的使用条件和网关双路启动示例见[基础组件网关说明](../../models/gimbals/two_axis_pod/controller/service/README.md#多机共享-rtsp)。

**当前组合场景的实际限制：** `run.sh` 仍固定使用一架 `rocket_two_axis_pod`、PX4 instance0、吊舱UDP 8770及默认MAVLink接收端口14540。`--aircraft-id` 只改变吊舱身份和视频路径；不能通过双开不同ID直接启动两架飞机。集成多架PX4飞机时，启动器还需分别配置实体名称、插件/相机话题、PX4实例和SYSID、MAVLink端口、运行目录及rootfs，并保存逐机接口对应表。当前验收覆盖单组合机实飞和双路视频服务隔离。

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

## 每机RTSP身份集成检查

`check_tracking.py` 检查的是控制器与Gazebo原始图像。网关身份和RTSP拉流由 `check_rtsp_binding.py` 独立检查，不能仅凭跟踪通过推断视频路径绑定正确。场景已验证非默认飞机ID23，对应 `rtsp://127.0.0.1:8555/two_axis_pod_23`。

```bash
# 先启动有限往返飞行；下面两项检查在飞机运行期间另开终端执行
GIMBAL_PYTHON=../two_axis_pod_venv/bin/python \
  bash scenarios/geographic_rocket_two_axis_pod/run.sh --cycles 2 --aircraft-id 23 \
  --run-dir "$PWD/runs/rocket_pod_rtsp_check"

# 飞行运行期间：一次初始化后只观察图像闭环跟踪和独立真值
GZ_PARTITION="$(cat runs/rocket_pod_rtsp_check/gz_partition.txt)" \
  ../two_axis_pod_venv/bin/python scenarios/geographic_rocket_two_axis_pod/check_tracking.py \
  --initialize --duration 70 --wait-timeout 600 \
  --output runs/rocket_pod_rtsp_check/tracking

# 只读：检查hello与遥测ID/路径、实际H.264帧号与frame_table对应
../two_axis_pod_venv/bin/python scenarios/geographic_rocket_two_axis_pod/check_rtsp_binding.py \
  --aircraft-id 23 --check-station-identity --duration 8 \
  --output runs/rocket_pod_rtsp_check/rtsp_binding

# 飞行完成且上述检查、图像跟踪均通过后，归档完整证据
python3 scenarios/geographic_rocket_two_axis_pod/report_validation.py \
  --run-dir runs/rocket_pod_rtsp_check --binding-dir runs/rocket_pod_rtsp_check/rtsp_binding
```

跟踪检查沿用上文 `--initialize --duration 70`，其 `GZ_PARTITION` 和 `--output` 必须对应本次运行目录及 `tracking/` 子目录。绑定检查只发送ping，不发布云台运动指令；要求身份全部匹配、至少20条遥测和20个视频帧、帧序号识别及帧表匹配比例均至少95%。可用 `--absent-id 24` 检查已明确未注册的路径返回404；同端口已有其他飞机24时不得使用这一负例。

基础双流测试使用独立临时端口：

```bash
../two_axis_pod_venv/bin/python models/gimbals/two_axis_pod/controller/service/tests/test_rtsp_multi.py -v
```

它验证两个独立编码进程同端口红蓝画面隔离、路径/RTP端口冲突拒绝、单路停止、租约清理和中继重启恢复。该测试与组合单机实飞分别记录，不代表已验证两架PX4飞机同时飞行。地面站会核对配置ID与当前网关hello的ID，不一致时禁止控制并显示原因。`--check-station-identity` 使用真实hello验证ID23就绪和误配ID24禁用；误配的回零、框选、手动和清故障调用使用本地记录socket检查拦截结果，不发往飞机。RTSP路径与视频帧表由本场景额外交叉核对。
