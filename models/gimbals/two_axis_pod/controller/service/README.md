# 两轴吊舱网络网关

`gimbal_bridge.py` 将 `/two_axis_pod/command`、`telemetry`、`command_result` 三个 Gazebo StringMsg 话题桥接为 UDP 控制与遥测，并提供 RTSP 视频。网关使用单类 YOLO11 ONNX 检测飞机、ByteTrack 风格多目标跟踪和服务端红框/编号叠加；选定目标交给 C++ 插件配置的单目标后端与机械控制执行。

默认世界为 `two_axis_pod_demo`，模型为 `two_axis_pod`，图像来自 `camera_link/camera`。默认 UDP 8766、共享 RTSP 8555 `/two_axis_pod_1`；内部 RTP 端口自动分配。插件和已登记地面站的遥测频率共用本目录 `telemetry.json` 的 `hz`，默认 50 Hz。使用入口见[启动说明](../../scripts/README.md)。

`telemetry.json` 内容为 `{"hz": 50}`，`hz` 必须为 (0, 1000] 的有限数值。插件按单调真实时间（`steady_clock`）调度新姿态，在新的物理更新步采样，网关逐份转发新样本，不按独立定时器重复发送旧姿态。暂停仿真或插件断流时，周期遥测停止，UDP ping 和命令回执仍正常处理；显式 `status` 请求仍可读取最新快照。只要物理更新的真实时间频率足够，仿真慢于或快于实时都以配置 `hz` 为目标。每个物理步最多发布一份；物理更新不足时实际频率下降并限频告警，不补发重复样本。网络、线程调度和 UDP 丢包仍可能影响接收端的实测速率。姿态 `ts_mono` 及图像相关时间戳继续使用原来的仿真/帧时间语义，不能用它们计算真实时间发送 Hz。

原生模型与装配继承的插件配置为 `<telemetry_config>model://two_axis_pod/controller/service/telemetry.json</telemetry_config>`，通过 `GZ_SIM_RESOURCE_PATH` 解析基础模型文件。指定其他配置时，插件 SDF 的 `telemetry_config` 与网关 `--telemetry-config /path/to/telemetry.json` 必须指向内容一致的文件。配置在启动时读取，修改后重启 Gazebo 插件和网关。网关校验插件上报的 `telemetry_hz` 与 `telemetry_clock:"steady"`，频率或时基不同、字段缺失时暂停周期转发并记录错误，避免配置不一致时静默产生不同频率。待发队列最多保存 1000 份样本，过载丢弃最旧样本并告警；UDP 不保证送达。

组合模型可用 `--topic-prefix /rocket_two_axis_pod` 对应插件的 command、telemetry、command_result 话题，用 `--image-topic /rocket_two_axis_pod/image` 指定完整图像话题；未指定时保持独立组件默认值。`--world`、`--gimbal-model`、UDP 端口分别配置，RTSP 端口同机共享，内部 RTP 默认自动分配；Gazebo 分区只隔离 Gazebo Transport，不隔离网络端口。

首次有效请求收到 `hello`；插件已确认协议时为 `version=7`、`angle_convention=motion_head_joint_right_positive`、`plugin_compatible=true`。未收到插件遥测或插件版本/角度标记不匹配时，前两字段为 `null`、`plugin_compatible=false`，禁止非零手动、回零、复位和跟踪等运动请求，仍允许停止、双轴零速、关闭视觉和仅检测。网关不广播旧插件姿态；兼容状态变化时向已登记客户端重发 `hello`，每次 `ping` 还会在 `pong` 前补发当前 `hello`，使丢失握手的客户端在后续保活中恢复；失配时清除姿态队列。插件的真实时间频率校验仍由既有频率检查独立执行。

`manual` 只接受 yaw、pitch 机械角速度，单位 °/s；任何 roll 字段均拒绝。yaw 为绕安装 +X 的外轴，范围 ±60°；以从吊舱后方沿镜头视线观察为准，镜头左转为负、右转为正，静止背景向相反方向移动。pitch 为绕随动 −Y 的内轴，范围 −80°～+30°。CAD 原始零位沿安装 +Z；yaw=0 时 pitch 负值向安装 +X 前方转动。运动链为 `Rx(yaw) Ry(-pitch)`；外轴穿过球心，球心不随外轴公转。模型、插件、网关及客户端须使用同一机械定义。非零人工速度在网关层统一退出 `auto_lock/manual_track`，因此随仓库地面站与第三方 station 行为一致。`reset_axis` 仅接受 yaw 或 pitch。当前机械限位内无视线奇异点，零位两轴均能改变视线；即使出现视线退化标志也不禁止机械控制。完整会话、重传、回执、命令参数、视频叠加和第三方 station 接入流程见[控制与遥测协议](../../docs/control-protocol.md)。

每个客户端地址具有独立会话。带 seq 请求按 1024 项窗口去重；同 seq 不同内容拒绝，乱序控制拒绝。可靠命令通过唯一 request_id 关联插件回执和客户端 seq。超时、发布失败与过期会话明确返回失败，不自动再次执行。客户端应持续发送 ping 续期，默认 5 秒过期。数据报上限 8 KiB。

视频只允许 NVIDIA NVENC，不提供软件编码回退；初始化或运行期失败时网关显示中文错误并退出，避免在没有视频的情况下继续控制。生产管线输出 8 位 YUV420 H.264。视频帧烧录序号，遥测增加 frame_table 和视频计数；同机 boot_id 相符时可测单调时钟端到端时延。编码兼容性测试使用独立临时 RTSP 端口和生产 NVENC 管线检查码流格式与连续帧。

## 飞机检测与自动锁定

视觉模式为 `off`、`detect`、`auto_lock`、`manual_track`，同一时刻只允许一个。`detect` 对最新相机帧做飞机检测和多目标跟踪，只有连续命中达到配置值的轨迹才输出，红框和 `#轨迹号` 直接烧录进 RTSP，不驱动云台。`auto_lock` 的搜索阶段使用相同红框，在已确认轨迹中首次选择框中心离画面中心最近的一项，以检测帧序号和归一化框启动插件单目标后端；选中后暂停检测叠加，地面站显示绿色框。默认 `sim_truth` 将该帧框内、离框中心最近的已登记飞机实体固定绑定，后续不切换到其他飞机；显式 `csrt` 使用图像纹理跟踪。后端进入 LOST 后清除旧轨迹并恢复检测搜索。`manual_track` 才接受地面站拖框。

配置文件为 `vision.json`，`confirmation_hits` 默认 5，另含置信度、NMS、丢失保持、关联阈值和 OpenCV 线程数。模型位于 `../../models/yolo11/aircraft_yolo11n/`，只有 `aircraft` 类；来源、固定上游版本、AGPL-3.0 许可和 SHA-256 随模型保存。网关使用单元素最新帧语义，不积压过期相机帧。当前 1920×1080@30 Hz 场景实测输入 30.35 fps、视觉输出 30.20 fps、推理约 14.9 ms。

## 多机共享 RTSP

飞机 ID 由网关参数 `--aircraft-id` 配置，为非负整数，默认 1；与实际载机 ID 保持一致，不从 Gazebo 模型名推断。默认 URL 为 `rtsp://<host>:8555/two_axis_pod_<aircraft_id>`，例如 `/two_axis_pod_1`、`/two_axis_pod_2`。`--mount` 配置路径前缀，末尾始终追加 `_飞机ID`；地面站使用相同的 `--aircraft-id` 与 `--mount`。

同一主机、同一系统用户和同一 RTSP 端口的网关自动共用 `rtsp_relay.py` 中继进程。每路独立编码、独立内部 RTP 端口；无需手工启动中继。内部端口默认自动分配，显式 `--rtp-port` 时必须唯一。重复路径或内部 RTP 端口注册被拒绝，不覆盖已有视频；视频初始化失败的网关退出，错误写入日志，修正配置后重启该网关。

中继使用用户私有的临时目录 `two-axis-rtsp-<UID>/` 保存每端口的 `.sock`、`.lock`、`.log`，本地注册不对网络开放。RTSP 对外监听所配置端口；该端口被其他服务占用时启动失败。正常停止网关立即撤销本路，其他路继续工作；异常退出后 10 秒租约过期清理。所有路退出后中继空闲约 30 秒自动退出，不属于单个网关的进程组。中继重启后存活网关自动尝试重新注册；已有播放器断线时需重新连接。

每路网关的 `hello` 和 station 遥测包含 `aircraft_id`、`video_rtsp_path`，表示当前网关 ID 与完整视频路径。控制协议版本为 7。地面站核对 `hello.aircraft_id` 与所选 ID，未确认、缺失/非法 ID 或不一致时禁止控制并清除待重传指令；断线后须重新通过 hello 核验。UDP 控制端口、插件话题与图像话题必须分别隔离；飞机 ID 只区分视频路径和上报身份，不自动配置 PX4 SYSID 或 Gazebo 话题。

以下示例假定仿真已分别提供 `/uav_1`、`/uav_2` 插件话题和对应图像话题：

```bash
GIMBAL_RUNDIR="$PWD/runs/pod_1" bash models/gimbals/two_axis_pod/scripts/run_bridge.sh \
  --aircraft-id 1 --udp-port 8766 --topic-prefix /uav_1 --image-topic /uav_1/image --encoder nvenc
GIMBAL_RUNDIR="$PWD/runs/pod_2" bash models/gimbals/two_axis_pod/scripts/run_bridge.sh \
  --aircraft-id 2 --udp-port 8767 --topic-prefix /uav_2 --image-topic /uav_2/image --encoder nvenc
bash models/gimbals/two_axis_pod/ground_station/run.sh --aircraft-id 2 --udp-port 8767 --decoder nvdec
```

两路视频共用 8555，分别为 `rtsp://127.0.0.1:8555/two_axis_pod_1` 与 `rtsp://127.0.0.1:8555/two_axis_pod_2`。先运行 `bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --prepare-only` 自动配置环境，再用 `.venv-linux/bin/python models/gimbals/two_axis_pod/controller/service/tests/test_rtsp_multi.py` 测试。该测试需要 GstRtspServer、NVENC 与 NVDEC，验证双进程同端口拉流、颜色隔离、重复路径拒绝、独立停止、租约过期和中继重启恢复。

运行依赖：Python 3、Gazebo Harmonic 的 gz-transport13 / gz-msgs10 Python 绑定、NumPy、带 DNN 的 OpenCV、PyGObject、GStreamer、GstRtspServer、NVIDIA 驱动与 `nvcudah264enc`。`run_bridge.sh` 默认使用仓库根目录 `.venv-linux`，检测到缺失时自动创建并安装依赖；启动前实际加载 ONNX 并确认 NVENC。也可用 `GIMBAL_PYTHON` 指定已配置的等价解释器。`gz_source.py` 只管理节点和仿真时钟映射。
