# 三轴吊舱地面站

网络桌面客户端：RTSP 显示、鼠标框选跟踪、UDP 遥测、按钮控制、虚拟摇杆和手动点动。依赖 Python OpenCV、NumPy、Pillow、PyGObject/GStreamer；无需 Gazebo Python bindings。

仓库根目录执行 `bash models/gimbals/three_axis_pod/ground_station/run.sh --decoder avdec`。默认读取本目录 `config.json`，命令行覆盖配置；远程使用 `--udp-host <IP> --udp-port 8765 --rtsp-port 8554`。全部参数见 `bash models/gimbals/three_axis_pod/ground_station/run.sh --help`。

| 操作 | 作用 |
|---|---|
| 鼠标左键拖框 | 提交整帧归一化跟踪框 |
| 底部 YAW / PITCH 摇杆 | 按住拖动控制偏航/俯仰角速度，松手归零 |
| 底部 ROLL 摇杆 | 按住左右拖动控制滚转角速度，松手归零 |
| Yaw复位 / Pitch复位 / Roll复位 | 指定轴回到当前遥测显示零，另外两轴保持当前角度 |
| R / S / H / M | 复位 / 停止 / 朝天 / 清故障 |
| 左右 / 上下 / [ ] | yaw / pitch / roll 点动 |
| ESC | 退出 |

摇杆区位于视频下方，不参与跟踪选框或视频归一化。二维摇杆有 8% 中心死区，行程对应最大 yaw 30°/s、pitch 20°/s；滚转摇杆最大 20°/s。三轴均通过 `manual` JSON 指令以 10 Hz 连续发送 **PX4 FRD 公共 Euler 角速度**。右移为 yaw 正向（向右转）、左移为负向；上移为 pitch 正向（抬头）、下移为负向；roll 右移为正向（沿镜头朝向看顺时针转）、左移为负向。方向键与 yaw/pitch 摇杆一致，`[` 为 roll 负向、`]` 为正向。立装和悬挂使用相同的公共角度含义。

三个单轴复位按钮通过可靠指令 `reset_axis` 下发，例如 `{"cmd":"reset_axis","seq":10,"axis":"yaw"}`，`axis` 也可以是 `pitch` 或 `roll`。回执按 `seq` 匹配，未收到回执会重发同一报文。Yaw复位到公共偏航 0（参考前方）；Pitch复位到公共俯仰 0（水平）；Roll复位到公共滚转 0（画面水平）。单轴复位会停止跟踪并保持其余两个公共角度，不等同于整体复位。界面“安装”一行读取遥测 `mount.mode`，区分立装（底座朝下）与悬挂（底座朝上），缺失或无法识别时显示“未知”。

界面的 yaw/pitch/roll 数值来自 `attitude_deg`，其 `pos/vel/acc` 分别为公共角度、角速度和角加速度；`joints_deg` 保留为机械关节诊断，不作为公共姿态显示。状态行中的“机械输出”来自 `cmd_dps`，用于诊断底层电机速度。程序必须同时收到 `hello.version=2`、hello 与遥测上的 `angle_convention="px4_frd"`，以及新鲜、完整的 `attitude_deg`，才启用摇杆、按键手动控制和单轴复位。旧服务或协议未确认时显示“公共姿态待确认”，禁止非零 `manual` 和 `reset_axis`；零角速度停止报文始终允许。协议失效或遥测超时会停止当前摇杆输入。

当 `attitude_deg.singular=true` 时，俯仰接近 ±90°，yaw/roll 无法独立解释，`vel/acc` 可为 `null`，界面显示角速度不可用。此时只启用俯仰手动控制和 Pitch复位；偏航/滚转控制与单轴复位会提示先调整俯仰。二维摇杆仍可向上/下拖动离开奇异点，横向输入归零。`attitude_quat_frd` 可用于完整姿态计算。无界面 `--send-manual` 自检使用 pitch −20°/s，支持从朝天姿态离开奇异点。

松开左键会立即发送三轴零角速度；点击复位、停止、朝天、清故障，以及退出/关闭窗口也会先取消手动输入。Linux X11 下检查窗口焦点与物理左键状态，切换窗口或拖出窗口松手都会归零，按住静止时仍保持连续控制。无法读取 X11 状态的环境采用 300 ms 鼠标事件租约：停止移动鼠标超过此时长即归零，需要继续拖动才能保持控制。发送线程另外使用 300 ms 手动输入租约，GUI 停顿时不会无限续发旧速度；插件端还有手动指令超时保护。

无界面自检：

```bash
bash models/gimbals/three_axis_pod/ground_station/run.sh --selftest 4 --send-home --send-manual --test-timeout 2.5
bash models/gimbals/three_axis_pod/ground_station/run.sh --decode-test 6 --decoder avdec --snapshot /tmp/gimbal.png
python3 -m unittest discover -s models/gimbals/three_axis_pod/ground_station/tests -v
```

配置中的 `reconnect` 当前为预留字段，程序默认执行重登记与视频重建。超时保护、可靠回执和时延显示条件见 [系统说明](../docs/system.md)。
