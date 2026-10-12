# 三轴吊舱地面站

网络桌面客户端：RTSP 显示、鼠标框选跟踪、UDP 遥测、按钮控制和手动点动。依赖 Python OpenCV、NumPy、Pillow、PyGObject/GStreamer；无需 Gazebo Python bindings。

仓库根目录执行 `bash models/gimbals/three_axis_pod/ground_station/run.sh --decoder avdec`。默认读取本目录 `config.json`，命令行覆盖配置；远程使用 `--udp-host <IP> --udp-port 8765 --rtsp-port 8554`。全部参数见 `bash models/gimbals/three_axis_pod/ground_station/run.sh --help`。

| 操作 | 作用 |
|---|---|
| 鼠标左键拖框 | 提交整帧归一化跟踪框 |
| R / S / H / M | 复位 / 停止 / 朝天 / 清故障 |
| 左右 / 上下 / [ ] | yaw / pitch / roll 点动 |
| ESC | 退出 |

无界面自检：

```bash
bash models/gimbals/three_axis_pod/ground_station/run.sh --selftest 4 --send-home --send-manual --test-timeout 2.5
bash models/gimbals/three_axis_pod/ground_station/run.sh --decode-test 6 --decoder avdec --snapshot /tmp/gimbal.png
python3 -m unittest discover -s models/gimbals/three_axis_pod/ground_station/tests -v
```

配置中的 `reconnect` 当前为预留字段，程序默认执行重登记与视频重建。超时保护、可靠回执和时延显示条件见 [系统说明](../docs/system.md)。
