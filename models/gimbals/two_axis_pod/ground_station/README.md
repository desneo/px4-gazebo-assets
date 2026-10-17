# 两轴吊舱地面站

地面站显示 RTSP 视频、机械 yaw/pitch 角度和角速度、视觉模式、检测/锁定状态及限位状态。默认连接本机 UDP 8766 和 `rtsp://127.0.0.1:8555/two_axis_pod_1`；可修改 `config.json` 或命令行参数。

```bash
bash models/gimbals/two_axis_pod/ground_station/run.sh
```

界面提供四个互斥视觉模式按钮，按钮活动态以网关 `vision.mode` 遥测为准，而不是以最近一次点击为准：

- **关闭视觉**：不执行检测或跟踪。
- **目标检测**：显示服务端已烧录进 RTSP 画面的红色检测框和轨迹编号，不驱动云台。
- **自动锁定**：搜索时显示检测红框和轨迹号，选中中心最近目标后转入 CSRT 单目标跟踪；此时停止多目标红框，界面显示 `LOCK #轨迹号` 和一个绿色跟踪框。CSRT 最终丢失后重新搜索。
- **手动框选**：只有该模式允许在视频区拖拽画框，CSRT 跟踪框以绿色显示。其他模式下拖框会被拒绝并显示中文提示。

视觉状态行显示 `vision.state`、目标数、锁定轨迹号、推理耗时、输入/输出帧率和错误。点击模式按钮会可靠发送 `set_vision_mode`，并等待网关遥测确认实际模式。

底部只有一个 yaw/pitch 虚拟摇杆：左负偏航、右正偏航，上正俯仰、下负俯仰。方向键同号。松开鼠标、失去焦点或停止输入后速度归零。控制区不参与视频选框坐标换算。两轴零位朝天，俯仰正值朝前，范围 −45°～+90°；偏航绕竖直 `Z` 轴旋转，范围 ±90°，俯仰轴保持水平。零位朝天时偏航不改变视线方向，机械关节仍可操作。

`H` 回零朝天，`R` 停止跟踪并回零朝天，`S` 停止跟踪，`M` 清除故障，`Esc` 退出。底部提供 yaw 和 pitch 单轴复位按钮。可靠命令保留 seq 重传和回执；手动流持续发送。收到协议版本 4、`upright_joint` 标记及新鲜有效机械角度遥测后才启用手动和单轴复位。没有滚转控制或滚转遥测。

自动锁定或手动框选期间使用虚拟摇杆、方向键、复位、回零、停止或单轴复位时，客户端先可靠发送 `set_vision_mode(mode=off)`，再发送人工控制命令，避免跟踪控制和人工控制竞争。目标检测模式允许继续人工操纵，且不会关闭检测。

机载装配遥测 `mount.static=false` 时，安装说明显示“机载立装（偏航绕机体 +Z）”，“朝天”按钮改为“回零”；此时回零朝安装 +Z，火箭机上对应机头方向，未必是世界天空。跟踪提示显示机械限位、速度饱和及视线退化，失去目标时显示 TEMP_LOST/LOST；CSRT TRACK 状态需结合误差和稳定性判断。

依赖 Python 3、NumPy、OpenCV、Pillow、PyGObject、GStreamer、可用的 NVIDIA 驱动及 GStreamer `nvh264dec` 插件。地面站强制使用 NVDEC，不提供 `avdec_h264` 软件回退；缺少插件、NVDEC 管线创建/启动失败或运行时报错时会显示中文错误并以非零状态退出。`decoder` 默认值为 `nvdec`；旧配置名 `nvenc` 仅作为同一 `nvh264dec` 元件的兼容别名，不会启用软件解码。中文字体可通过配置指定。查看全部参数：

```bash
bash models/gimbals/two_axis_pod/ground_station/run.sh --help
python3 -m unittest discover -s models/gimbals/two_axis_pod/ground_station/tests
```

离线测试覆盖 UDP 会话/去重/回执、视觉模式协议和遥测校验、模式互斥、两轴机械协议、拒绝滚转、拖动续发、释放与焦点保护。在线自检使用 `--selftest 5 --send-home --send-manual`，硬解视频检查使用 `--decode-test 5 --snapshot /tmp/two_axis_pod.png`。自检会发出实际运动命令。

可用 `GIMBAL_PYTHON` 指定含依赖的 Python 解释器。依赖安装在仓库上一级的独立环境时，例如：

```bash
GIMBAL_PYTHON=../two_axis_pod_venv/bin/python bash models/gimbals/two_axis_pod/ground_station/run.sh
```

多机视频路径为 `/two_axis_pod_<aircraft_id>`。在 `config.json` 设置 `aircraft_id`（默认 1），或用 `--aircraft-id 2` 选择飞机 2；`rtsp_mount` / `--mount` 是路径前缀，末尾自动追加 ID。各机共用 RTSP 8555，UDP 控制端口仍需配置为对应网关端口；配置示例见[网关说明](../controller/service/README.md#多机共享-rtsp)。

地面站按配置的 `aircraft_id` 核对当前网关 `hello.aircraft_id`，必须是相同的非负整数。尚未确认、缺失/非法 ID、ID 不符或连接超时时禁止所有控制（含停止、零速度、回零、跟踪和清故障），仅允许 ping、status、bye。身份不符时清除待重传控制，界面和日志显示期望 ID 与网关 ID；连接超时后仅收到 pong 或遥测不会恢复控制，须重新收到匹配 hello。GUI 与 `--selftest` 均使用相同校验，自检身份未确认时返回非零退出码。旧网关缺少 ID 时须升级网关，不能跳过校验。

## Windows 专用版本

Windows 入口使用 OpenCV wheel 自带的 FFmpeg 后端接收 RTSP，复用相同的四种视觉模式、UDP 协议、身份校验、界面和控制逻辑。入口固定请求 Windows D3D11 硬件解码，并在 RTSP 打开后核对实际加速类型；OpenCV 缺少接口、驱动不支持、打开失败或后端回退到 CPU 时均显示中文错误并退出，不提供软件解码回退。它不安装或使用 PyGObject、GStreamer、系统级 FFmpeg。Python 包只安装在仓库根目录的 `.venv-windows` 虚拟环境中。

默认 Windows 配置位于 `config.windows.json`，连接飞机 1：

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

地址可使用原有 `--udp-host`、`--udp-port`、`--rtsp-port`、`--aircraft-id` 和 `--mount` 参数覆盖。Windows 版本不会改写 Linux 的 `config.json` 和 `run.sh`。由于当前环境不能执行 Windows D3D11 解码，发布前应在目标 Windows 机器上先运行上述 `--decode-test`，再人工检查四个视觉模式按钮、绿色锁定框及人工摇杆退出锁定行为。
