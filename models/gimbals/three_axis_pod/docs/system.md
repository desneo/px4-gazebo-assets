# 三轴吊舱与地面站功能说明

## 组成与边界

`three_axis_pod` 与其三轴 gimbal 一一对应，模型内的 `controller/` 为这套吊舱提供专用控制与网络接口。

- `models/gimbals/three_axis_pod/controller/`：Gazebo Harmonic C++17 系统插件，负责相机图像跟踪、状态机、视觉伺服、关节速度驱动和 JSON 遥测；`service/` 提供配套网络网关及其本地模块。
- `models/gimbals/three_axis_pod/`：三轴朝天吊舱及网格，相机 1920×1080、30 Hz、水平视场 60°；三轴零位时视轴朝天。yaw 可连续旋转，pitch 机械范围 −135°～45°，roll −45°～45°。
- `assemblies/three_axis_pod_stand/`：1.5 m 支架与吊舱装配，通过重力与摩擦落地。
- `models/gimbals/three_axis_pod/tests/world.sdf`：含棋盘地面、固定吊舱、测试小车的独立世界，不启动 PX4；地理背景环境仍独立保存在原目录。
- `models/gimbals/three_axis_pod/ground_station/`：通过 UDP 控制/遥测和 RTSP 视频连接网关的桌面客户端。
- `models/gimbals/three_axis_pod/scripts/`：环境配置与启停入口。

数据流：相机 → 插件跟踪/伺服 → 关节；相机 → 网关 H.264 编码 → RTSP → 地面站；地面站指令 → UDP 网关 → gz-transport 插件；插件遥测/回执反向返回。

## 当前功能

跟踪采用 OpenCV CSRT，初始化检查最小 8 像素选框与灰度纹理标准差；跟踪失败后以保存的灰度模板在局部区域匹配重捕获。没有使用目标仿真真值控制。像素误差转换为方位/俯仰角，使用 PI+D 和目标角速度前馈驱动 yaw/pitch；roll 在跟踪时输出零速度，没有实现载机姿态补偿。

控制默认 100 Hz，遥测默认 50 Hz；速度上限 100°/s，加速度上限 160°/s²。pitch/roll 设有 1° 限位余量，yaw 不设机械限位；限位、饱和和稳定性均有遥测。稳定性用约 1 秒误差窗口、1°/2° 双阈值与进入持续时间判定。限位或饱和时可判为不稳定。

运行模式为 `IDLE/TRACK/MANUAL/HOME`；跟踪状态为 `IDLE/INIT/TRACK/TEMP_LOST/LOST/FAULT`；稳定性为 `UNDETERMINED/STABLE/UNSTABLE`。

手动命令须持续续期：距最近命令达到 0.5 秒，在下一控制周期立即将速度指令归零并进入 `IDLE`。图像距最近新帧达到 0.5 秒时立即归零、目标失效并进入 `TEMP_LOST`；达到 2 秒进入 `LOST`，需重新选框。2 秒内新帧成功跟踪可恢复；连续无效帧和初始化失败也有丢失期限。插件用真实单调时钟判断超时，控制滤波仍使用仿真时间；仿真暂停时物理不更新，恢复后过期手动命令立即清除。过期图像不能用于新选框初始化。

正常速度指令受加速度限幅；超时停转与机械限位优先于速度平滑，立即裁剪指令，pitch/roll 到限位不输出向外残留速度，允许向内恢复。这里的“停止”指速度指令归零，不代表物理惯性瞬间消失。模式切换清理目标框、滤波和稳定性信息，重捕获成功时目标有效性与跟踪状态一致。

地面站支持鼠标拖框、实时跟踪框与状态叠加、关节角/角速度、限位/饱和、帧率、丢帧率和时延显示。方向键与方括号键以至少 10 Hz 发送约 0.4 秒的点动指令，再发零速度；视频无帧会重建管线。解码元件缺失时可回退另一个解码器，运行中的硬解故障不保证自动软解回退。

## 指令与网络接口

默认 UDP `8765`，RTSP `rtsp://<host>:8554/gimbal`。单个 UDP 数据报为 UTF-8 JSON，上限 8 KiB。客户端以递增 `seq` 标识请求，约每秒 `ping` 登记续期；网关默认 5 秒无请求即注销客户端，支持多客户端遥测广播。`hello` 返回 `proto=gimbal-udp`、`version=1`、遥测频率、视频路径和 `clock_domain`（Linux boot_id，无法读取时为 null）；`pong` 回传 `t/seq`。

| cmd | 参数 | 当前作用 |
|---|---|---|
| `track` | `box=[x,y,w,h]`，相对完整图像归一化 | 初始化选框跟踪，插件产生回执 |
| `stop` | 无 | 停止跟踪，速度渐降为零；无位置保持闭环，插件产生回执 |
| `home` | 无 | 三轴返回零位朝天，yaw按360°周期取最近零位方向；插件产生接收回执 |
| `reset` | 无 | 停止、清故障并转水平前向，插件产生接收回执 |
| `manual` | `yaw/pitch/roll`，°/s | 持续角速度，正常无回执；需0.5秒内续期，零速度命令用于停止 |
| `reset_fault` | 无 | 清跟踪故障，无回执 |
| `ping` | `t` 可选 | 网关登记续期、回复 pong |
| `status` | 无 | 网关立即返回最近遥测 |
| `bye` | 无 | 注销网关客户端 |

地面站对 `track/stop/home/reset` 默认每 0.2 秒重传同一 seq，最多发送 5 次。网关在每个客户端最近 1024 个序号窗口中记录请求内容与结果：同一 seq、同一内容不会重复执行，已有结果会重发；同一 seq 改用不同内容、超出去重窗口或落后于已接收控制请求的旧控制命令，明确返回 `ok=false`。因此迟到的非零 manual 不能覆盖较新的零速度命令。status/ping 不抢占控制顺序；未携带 seq 的请求不能去重或判断控制新旧。

网关为 `track/stop/home/reset` 生成唯一 `request_id` 传给插件，插件在成功或失败回执中原样返回该字段；网关仅将结果送给绑定的原客户端并附回原 seq。manual/reset_fault 正常无回执，不占等待队列；参数校验、发布失败和过时请求返回带 seq 的失败结果。待回执队列最多 4096 条；30 秒无插件回执返回“执行结果未知”，不重新执行。客户端会话过期或 bye 后，去重记录短期保留 60 秒，防止重登记后重执行；新客户端应使用新的本地端口，复用原地址须延续序号。网关重启或保留窗口结束后不保证先前请求的去重。

地面站完成回执后删除待处理项，保留最近有界结果供查询；接收时验证来源 IP/端口、数据报大小及结构后才刷新存活状态。`home/reset` 的成功回执表示请求接受，不能代替实际关节到位判断。

插件使用 `gz.msgs.StringMsg` 的 JSON 消息：订阅 `/gimbal/command`，发布 `/gimbal/telemetry` 和 `/gimbal/command_result`。图像为 `/world/three_axis_pod_test/model/three_axis_pod/link/camera_link/sensor/camera/image`。世界、实例和传感器名决定图像话题；插件指令/遥测话题可在 SDF 配置。网关的三个控制话题在源码中固定。

遥测包含：`mode`、`tracking`（状态、故障、提示、有效/无效帧数）、`stability`、`target`（有效性、像素/归一化选框）、`joints_deg`（位置、速度、加速度）、`camera`（姿态与帧号）、`limits`、`cmd_dps`、`frame`、`sim_uptime`。yaw 的 `pos` 为 [−180°,180°] 航向偏差，`pos_raw` 为连续关节角。插件 `ts_mono`、帧时间实际使用仿真时钟；相机 world 姿态由关节运动学推算，未融合载机实际世界姿态。

网关补充 `ts_wall/server_uptime/frame_table/video`。视频左下角烧录 8 bit 魔数 0xA5 和 24 bit 帧号，`frame_table` 提供最近 60 帧的网关单调时间。地面站用帧号关联时间，仅 hello.clock_domain 与本地 boot_id 相同才计算时延，并过滤负值和不少于3秒的样本；跨机器或时钟域未知时显示“时延不可用”。这不提供跨机器时钟同步。

## 环境与使用

依赖 Ubuntu 24.04、Gazebo Harmonic（gz-sim8、transport13、msgs10、plugin2 开发包）、CMake ≥3.22.1、C++17、OpenCV4（含 tracking）。Python 侧需要 gz Python bindings、NumPy、OpenCV、Pillow、PyGObject、GStreamer 与 GstRtspServer；视频需 H.264 编解码插件。地面站的 UDP 自检也会顶层导入 cv2/NumPy/Pillow。依赖可安装到独立环境或由调用方通过 PYTHONPATH 提供，仓库不包含 vendor 目录。

在仓库根目录运行：

```bash
cmake -S models/gimbals/three_axis_pod/controller -B models/gimbals/three_axis_pod/controller/build
cmake --build models/gimbals/three_axis_pod/controller/build -j4
export GZ_PARTITION=three_axis_pod_test
bash models/gimbals/three_axis_pod/scripts/run_plugin_sim.sh
bash models/gimbals/three_axis_pod/scripts/run_bridge.sh --encoder x264
bash models/gimbals/three_axis_pod/ground_station/run.sh --decoder avdec
```

仿真和网关必须使用同一 `GZ_PARTITION`；地面站是网络客户端，不需要该变量。默认编码为 NVIDIA `nvcudah264enc`，可显式选 `x264`；解码默认 `nvh264dec`，可选 `avdec`。无显示渲染需可用 EGL/GPU；如设备要求，可由调用方设置 `__EGL_VENDOR_LIBRARY_FILENAMES`。

日志/PID 默认保存 `runs/three_axis_pod/`，可用 `GIMBAL_RUNDIR` 改写；启停命令须使用相同运行目录。停止：

```bash
bash models/gimbals/three_axis_pod/scripts/stop_bridge.sh
bash models/gimbals/three_axis_pod/scripts/stop_plugin_sim.sh
```

生成器仅用于从包含 `sim/models/gimbal_up` 与 `sim/models/gimbal_stand` 的参考工程重新生成 SDF：设置 `GIMBAL_REFERENCE_ROOT` 后运行 `python3 models/gimbals/three_axis_pod/controller/tools/make_model.py`。日常运行使用已保存模型，无需该参考工程；网格随模型维护。

## 当前限制与验证边界

- 跟踪闭环驱动 yaw/pitch，roll 不做视觉闭环；没有载机姿态补偿、PX4联动或真实飞行验证。
- 视频采用显式选择的编解码器；运行中的硬解故障不保证自动软解回退。
- 跨机器时延显示不可用；没有实现跨机器时钟同步。
- 多客户端允许发指令，请求回执彼此隔离，但没有独占控制权仲裁；多个操作员的有效请求会按实际接收顺序控制同一吊舱。
- 没有真实飞行、载机姿态扰动、长时间跟踪、跨机器弱网或 GUI 鼠标键盘完整验收结果。

验证命令、观测数据及环境限制见 [自验证说明](validation.md)。
