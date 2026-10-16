# 两轴吊舱地面站

地面站显示 RTSP 视频、机械 yaw/pitch 角度和角速度、跟踪状态及限位状态，并支持鼠标画框跟踪。默认连接本机 UDP 8766 和 `rtsp://127.0.0.1:8555/two_axis_pod_1`；可修改 `config.json` 或命令行参数。

```bash
bash models/gimbals/two_axis_pod/ground_station/run.sh --decoder avdec
```

底部只有一个 yaw/pitch 虚拟摇杆：左负偏航、右正偏航，上正俯仰、下负俯仰。方向键同号。松开鼠标、失去焦点或停止输入后速度归零。控制区不参与视频选框坐标换算。两轴零位朝天，俯仰正值朝前，范围 −45°～+90°；偏航绕竖直 `Z` 轴旋转，范围 ±90°，俯仰轴保持水平。零位朝天时偏航不改变视线方向，机械关节仍可操作。

`H` 回零朝天，`R` 停止跟踪并回零朝天，`S` 停止跟踪，`M` 清除故障，`Esc` 退出。底部提供 yaw 和 pitch 单轴复位按钮。可靠命令保留 seq 重传和回执；手动流持续发送。收到协议版本 4、`upright_joint` 标记及新鲜有效机械角度遥测后才启用手动和单轴复位。没有滚转控制或滚转遥测。

机载装配遥测 `mount.static=false` 时，安装说明显示“机载立装（偏航绕机体 +Z）”，“朝天”按钮改为“回零”；此时回零朝安装 +Z，火箭机上对应机头方向，未必是世界天空。跟踪提示显示机械限位、速度饱和及视线退化，失去目标时显示 TEMP_LOST/LOST；CSRT TRACK 状态需结合误差和稳定性判断。

依赖 Python 3、NumPy、OpenCV、Pillow、PyGObject、GStreamer；硬解需要对应 NVIDIA 插件，`--decoder avdec` 选择软件解码。中文字体可通过配置指定。查看全部参数：

```bash
bash models/gimbals/two_axis_pod/ground_station/run.sh --help
python3 -m unittest discover -s models/gimbals/two_axis_pod/ground_station/tests
```

离线测试只依赖 Python 标准库，覆盖 UDP 会话/去重/回执、两轴机械协议、拒绝滚转、拖动续发、释放与焦点保护。在线自检使用 `--selftest 5 --send-home --send-manual`，视频检查使用 `--decode-test 5 --decoder avdec --snapshot /tmp/two_axis_pod.png`。自检会发出实际运动命令。

可用 `GIMBAL_PYTHON` 指定含依赖的 Python 解释器。依赖安装在仓库上一级的独立环境时，例如：

```bash
GIMBAL_PYTHON=../two_axis_pod_venv/bin/python bash models/gimbals/two_axis_pod/ground_station/run.sh --decoder avdec
```

多机视频路径为 `/two_axis_pod_<aircraft_id>`。在 `config.json` 设置 `aircraft_id`（默认 1），或用 `--aircraft-id 2` 选择飞机 2；`rtsp_mount` / `--mount` 是路径前缀，末尾自动追加 ID。各机共用 RTSP 8555，UDP 控制端口仍需配置为对应网关端口；配置示例见[网关说明](../controller/service/README.md#多机共享-rtsp)。

地面站按配置的 `aircraft_id` 核对当前网关 `hello.aircraft_id`，必须是相同的非负整数。尚未确认、缺失/非法 ID、ID 不符或连接超时时禁止所有控制（含停止、零速度、回零、跟踪和清故障），仅允许 ping、status、bye。身份不符时清除待重传控制，界面和日志显示期望 ID 与网关 ID；连接超时后仅收到 pong 或遥测不会恢复控制，须重新收到匹配 hello。GUI 与 `--selftest` 均使用相同校验，自检身份未确认时返回非零退出码。旧网关缺少 ID 时须升级网关，不能跳过校验。

## Windows 专用版本

Windows 入口使用 OpenCV wheel 自带的 FFmpeg 后端接收 RTSP，复用相同的 UDP 协议、身份校验、界面和控制逻辑。它不安装或使用 PyGObject、GStreamer、系统级 FFmpeg/NVIDIA 插件。Python 包只安装在仓库根目录的 `.venv-windows` 虚拟环境中。

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

# RTSP 解码并保存截图
models/gimbals/two_axis_pod/ground_station/run_windows.ps1 --decode-test 5 --snapshot runs/two_axis_pod_windows.png
```

地址可使用原有 `--udp-host`、`--udp-port`、`--rtsp-port`、`--aircraft-id` 和 `--mount` 参数覆盖。Windows 版本不会改写 Linux 的 `config.json` 和 `run.sh`。
