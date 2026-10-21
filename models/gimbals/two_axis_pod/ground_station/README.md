# 两轴吊舱地面站

地面站显示 RTSP 视频、机械 yaw/pitch 角度和角速度、视觉模式、检测/锁定状态及限位状态。Linux 默认配置保存在同目录 `config.ini`，按 `[network]`、`[bridge]`、`[video]`、`[forward]` 和 `[session]` 分组，每个配置项上方都有用途注释。Station 只接受 INI 配置，不解析旧 JSON 文件。直接执行启动命令；入口自动检查环境，缺少项目虚拟环境或 Python 包时调用 `setup_linux.sh` 安装，然后使用仓库根目录 `.venv-linux` 并读取 `config.ini`：

```bash
bash models/gimbals/two_axis_pod/ground_station/run.sh
```

仓库默认 Linux 配置连接 1 号机：UDP 控制/遥测为 `192.168.1.56:8770`，RTSP 为 `rtsp://192.168.1.56:8555/two_axis_pod_1`。修改 `config.ini` 可切换到其他飞机或网关地址。

Linux `config.ini` 的 `[bridge] manage=true` 时，`run.sh` 会检查并管理本机同仓库、同 UDP 端口和飞机 ID 的网关。已有网关的 Python 缺少 DNN 等依赖时，先自动配置 `.venv-linux` 并实际加载 ONNX、校验 NVENC，再用原参数、工作目录和 Gazebo 分区恢复网关；存在其他启动器监管时会要求先通过该启动器停止。没有网关时，使用 `[bridge]` 的 world、model、topic_prefix、image_topic 和 gz_partition 启动。默认这些参数对应 1 号机场景。停止操作如下：

```bash
bash models/gimbals/two_axis_pod/ground_station/stop.sh
```

关闭 GUI 或按 Ctrl+C 也会停止本次管理的 bridge。使用自定义配置或临时端口时，`stop.sh` 传入相同的 `--config`、`--udp-port`、`--aircraft-id`。进程记录在 `runs/station_<ID>_<UDP端口>/session.json`，停止时核对 PID、启动时间与命令，避免误停复用 PID 的进程。直接运行 `ground_station.py`、`--selftest` 或 `--decode-test` 不管理网关。连接其他主机或希望由场景单独管理网关时设置 `[bridge] manage=false` 或传入 `--no-manage-bridge`；此时关闭 Station 不停止网关，远端依赖须在网关主机初始化。Windows 入口只连接已启动的网关。

界面提供四个互斥视觉模式按钮，按钮活动态以网关 `vision.mode` 遥测为准，而不是以最近一次点击为准：

- **关闭视觉**：不执行检测或跟踪。
- **目标检测**：显示服务端已烧录进 RTSP 画面的红色检测框和轨迹编号，不驱动云台。
- **自动锁定**：搜索时显示检测红框和轨迹号，选中中心最近目标后转入 CSRT 单目标跟踪；此时停止多目标红框，界面显示 `LOCK #轨迹号` 和一个绿色跟踪框。CSRT 最终丢失后重新搜索。
- **手动框选**：只有该模式允许在视频区拖拽画框，CSRT 跟踪框以绿色显示。其他模式下拖框会被拒绝并显示中文提示。

视觉状态行显示 `vision.state`、目标数、锁定轨迹号、推理耗时、输入/输出帧率和错误。点击模式按钮会可靠发送 `set_vision_mode`，并等待网关遥测确认实际模式。

右上角控制按钮下方提供 IP、端口输入框和“发送艾晟报文/停止发送”。单击输入框后可直接输入，点击时选中全部内容；支持 Ctrl+A、退格、Tab 切换、Enter 完成编辑，Esc 只结束本次编辑。编辑期间不触发回零、停止或方向键等云台快捷键。发送中同样可以单击输入框修改；进入编辑会自动暂停发送并清空待发队列。修改后点击“发送艾晟报文”才启用新地址并保存配置，Enter/Esc 只结束编辑，不自动恢复发送。地址接受 IPv4/IPv6 字面量，端口为 1～65535；无效地址在界面显示原因。

点击“发送艾晟报文”后，地面站将新收到、已确认飞机身份且符合协议 v5 的遥测转换为 UTF-8 JSON，发送到指定接收端。每个数据报仅包含以下 9 个字段，不附带 `type`、`frame_table` 或其他原始状态字段。`track_status` 为 JSON 整数，其余为 JSON 数值；JSON 本身不承载二进制 `uint8/float32` 类型。

| 字段 | 来源及发送值 |
| --- | --- |
| `track_status` | 处于手动框选或已确认锁定且有轨迹 ID 的自动锁定模式，无视觉错误，且 `mode=TRACK`、`tracking.state=TRACK`、`target.valid=true`、`target.miss.valid=true` 同时满足时为 `1`，否则为 `0` |
| `pitch_angle` | `attitude_deg.pitch.pos × 100`，直接使用负值朝前的机械角，不再次反号 |
| `yaw_angle` | `attitude_deg.yaw.pos × 100` |
| `pitch_misalignment` | 有效跟踪时 `target.miss.offset_deg[1] × 100`，画面向下正；否则 `0.0` |
| `yaw_misalignment` | 有效跟踪时 `target.miss.offset_deg[0] × 100`，画面向右正；否则 `0.0` |
| `tracking_confidence` | 有效跟踪时 `255.0`，否则 `0.0`；接收端乘 `0.1` 后为 `25.5` 或 `0.0`，只表示有效性占位，不是真实跟踪置信度 |
| `target_confidence` | 固定发送 `80.0`，不缩放，不随跟踪状态变化，也不代表实时检测评分 |
| `pitch_los_rate` | `attitude_deg.pitch.vel × 100`，单位换算前为 °/s |
| `yaw_los_rate` | `attitude_deg.yaw.vel × 100`，单位换算前为 °/s |

角度、脱靶量和角速度字段均由接收端乘 `0.01` 还原；目标丢失时姿态角和速度仍发送当前测量值。`pitch_los_rate/yaw_los_rate` 按接收端字段名承载机械轴角速度，不是独立估计的真实目标视线角速度。关闭视觉、仅检测、自动搜索/交接、未选框、初始化、丢失或视觉错误时，`track_status/tracking_confidence` 为 `0/0.0`，不能沿用插件残留的跟踪值。当前 CSRT 不输出真实跟踪置信度，不能把上述占位值或 YOLO 检测置信度解释为单目标跟踪可信程度。

发送频率跟随新收到的遥测，网关默认约 50 Hz。独立发送线程不阻塞视频或摇杆，停止/关闭窗口后不再发送，无新遥测时不重发旧状态。发送计数是本机已交给 UDP socket 的数量，不代表对端已收到。最后一次有效 IP/端口保存在 INI `[forward]`，默认 `127.0.0.1:9000`；不保存发送开关，每次启动默认不发送。此设置与网关额外主动上报 `status_udp.json` 分别生效，后者仍输出完整原始状态。

单目标跟踪时显示水平/垂直的像素、角度脱靶量：原图左上角为坐标原点，X 右正、Y 下正，偏差为目标中心减画面中心。自动锁定与手动框选共用吊舱 `target.miss` 遥测；初始化、丢失、停止或遥测过期时显示无效，旧插件没有该字段时不猜测数值。字段、测量帧号和无效值定义见[脱靶量协议](../docs/control-protocol.md#跟踪脱靶量)。

底部只有一个 yaw/pitch 虚拟摇杆：左负偏航、右正偏航，上正俯仰、下负俯仰。方向键同号，方向表示机械角增减。松开鼠标、失去焦点或停止输入后速度归零。控制区不参与视频选框坐标换算。两轴零位朝天，俯仰 −90° 朝前、+45° 朝后上方，范围 −90°～+45°；偏航绕竖直 `+Z` 轴旋转，范围 ±90°，俯仰绕随动水平 `−Y`。零位朝天时偏航不改变视线方向，机械关节仍可操作。

`H` 回零朝天，`R` 停止跟踪并回零朝天，`S` 停止跟踪，`M` 清除故障，`Esc` 退出。底部提供 yaw 和 pitch 单轴复位按钮。可靠命令保留 seq 重传和回执；手动流持续发送。收到协议版本 5、`upright_joint` 标记及新鲜有效机械角度遥测后才启用手动和单轴复位。没有滚转控制或滚转遥测；不同机械定义的插件、网关和地面站不得混用。

自动锁定或手动框选期间使用虚拟摇杆、方向键、复位、回零、停止或单轴复位时，客户端先可靠发送 `set_vision_mode(mode=off)`，再发送人工控制命令，避免跟踪控制和人工控制竞争。目标检测模式允许继续人工操纵，且不会关闭检测。

机载装配遥测 `mount.static=false` 时，安装说明显示“机载立装（偏航绕机体 +Z）”，“朝天”按钮改为“回零”；此时回零朝安装 +Z，火箭机上对应机头方向，未必是世界天空。跟踪提示显示机械限位、速度饱和及视线退化，失去目标时显示 TEMP_LOST/LOST；CSRT TRACK 状态需结合误差和稳定性判断。

依赖 Python 3、NumPy、OpenCV、Pillow、PyGObject、GStreamer、可用的 NVIDIA 驱动及 GStreamer `nvh264dec` 插件。地面站强制使用 NVDEC，不提供 `avdec_h264` 软件回退；缺少插件、NVDEC 管线创建/启动失败或运行时报错时会显示中文错误并以非零状态退出。`decoder` 默认值为 `nvdec`；旧配置名 `nvenc` 仅作为同一 `nvh264dec` 元件的兼容别名，不会启用软件解码。中文字体可通过配置指定。查看全部参数：

```bash
bash models/gimbals/two_axis_pod/ground_station/run.sh --help
python3 -m unittest discover -s models/gimbals/two_axis_pod/ground_station/tests
```

离线测试覆盖 UDP 会话/去重/回执、视觉模式协议和遥测校验、模式互斥、两轴机械协议、拒绝滚转、拖动续发、释放与焦点保护。在线自检使用 `--selftest 5 --send-home --send-manual`，硬解视频检查使用 `--decode-test 5 --snapshot /tmp/two_axis_pod.png`。自检会发出实际运动命令。

`.venv-linux` 是 Linux 网关与地面站共用的可重建本机环境，已被 Git 忽略，不随仓库归档；仓库归档的是环境检查、依赖列表和安装脚本。安装脚本优先使用系统 `python3-venv`，系统缺少 `ensurepip` 时自动使用已有 `uv`；两者都不可用或缺少 GStreamer/PyGObject 时，在 Debian/Ubuntu 上调用 `sudo apt-get` 安装基础系统包。NVIDIA 驱动或 `nvh264dec` 最终仍不可用时明确报错，不回退软件解码。`GIMBAL_PYTHON` 仅保留为开发或诊断时的高级覆盖入口，正常启动不需要设置。

多机视频路径为 `/two_axis_pod_<aircraft_id>`。在 `config.ini` 设置 `aircraft_id`（默认 1）；`rtsp_mount` 是路径前缀，末尾自动追加 ID。各机共用 RTSP 8555，UDP 控制端口仍需配置为对应网关端口；配置示例见[网关说明](../controller/service/README.md#多机共享-rtsp)。命令行的 `--udp-port`、`--rtsp-port`、`--aircraft-id`、`--mount` 和 `--decoder` 仅用于临时覆盖配置，不是正常启动必需项。

地面站按配置的 `aircraft_id` 核对当前网关 `hello.aircraft_id`，必须是相同的非负整数。尚未确认、缺失/非法 ID、ID 不符或连接超时时禁止所有控制（含停止、零速度、回零、跟踪和清故障），仅允许 ping、status、bye。身份不符时清除待重传控制，界面和日志显示期望 ID 与网关 ID；连接超时后仅收到 pong 或遥测不会恢复控制，须重新收到匹配 hello。GUI 与 `--selftest` 均使用相同校验，自检身份未确认时返回非零退出码。旧网关缺少 ID 时须升级网关，不能跳过校验。

## Windows 专用版本

Windows 入口使用 OpenCV wheel 自带的 FFmpeg 后端接收 RTSP，复用相同的四种视觉模式、UDP 协议、身份校验、界面和控制逻辑。入口固定请求 Windows D3D11 硬件解码，并在 RTSP 打开后核对实际加速类型；OpenCV 缺少接口、驱动不支持、打开失败或后端回退到 CPU 时均显示中文错误并退出，不提供软件解码回退。它不安装或使用 PyGObject、GStreamer、系统级 FFmpeg。Python 包只安装在仓库根目录的 `.venv-windows` 虚拟环境中。

默认 Windows 配置位于带注释的 `config.windows.ini`，连接飞机 1：

- 指令/遥测：`udp://192.168.1.56:8770`
- 视频：`rtsp://192.168.1.56:8555/two_axis_pod_1`
- RTSP 默认使用 TCP；网关只允许 UDP 时传入 `--rtsp-transport udp`

首次运行可双击 `setup_windows.cmd`，或在仓库根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File models/gimbals/two_axis_pod/ground_station/setup_windows.ps1 -InstallPython
```

安装脚本只在缺少 Python 时按当前用户安装官方 Python 3.12，然后创建项目专用虚拟环境。完成后双击 `run_windows.cmd`，只显示一个地面站 GUI 窗口；程序使用 Windows 命名互斥量阻止重复启动。需要查看运行日志时执行：

```powershell
models/gimbals/two_axis_pod/ground_station/run_windows.ps1
```

无窗口检查命令：

```powershell
# UDP 身份、协议与遥测，不发送运动指令
models/gimbals/two_axis_pod/ground_station/run_windows.ps1 --selftest 5

# D3D11 RTSP 硬件解码并保存截图；失败时返回非零状态
models/gimbals/two_axis_pod/ground_station/run_windows.ps1 --decode-test 5 --snapshot runs/two_axis_pod_windows.png
```

地址可使用原有 `--udp-host`、`--udp-port`、`--rtsp-port`、`--aircraft-id` 和 `--mount` 参数覆盖。Windows 版本不会改写 Linux 的 `config.ini` 和 `run.sh`。由于当前环境不能执行 Windows D3D11 解码，发布前应在目标 Windows 机器上先运行上述 `--decode-test`，再人工检查四个视觉模式按钮、绿色锁定框及人工摇杆退出锁定行为。
