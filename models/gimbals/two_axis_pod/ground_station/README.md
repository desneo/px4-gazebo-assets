# 两轴吊舱地面站

地面站显示 RTSP 视频、机械 yaw/pitch 角度和角速度、跟踪状态及限位状态，并支持鼠标画框跟踪。默认连接本机 UDP 8766 和 `rtsp://127.0.0.1:8555/two_axis_pod`；可修改 `config.json` 或命令行参数。

```bash
bash models/gimbals/two_axis_pod/ground_station/run.sh --decoder avdec
```

底部只有一个 yaw/pitch 虚拟摇杆：左负偏航、右正偏航，上正俯仰、下负俯仰。方向键同号。松开鼠标、失去焦点或停止输入后速度归零。控制区不参与视频选框坐标换算。两轴零位朝天，俯仰正值朝前，范围 −45°～+90°；偏航绕侧面安装板法线旋转，范围 ±90°。俯仰 +90° 时偏航转动画面，仍可操作机械关节。

`H` 回零朝天，`R` 停止跟踪并回零朝天，`S` 停止跟踪，`M` 清除故障，`Esc` 退出。底部提供 yaw 和 pitch 单轴复位按钮。可靠命令保留 seq 重传和回执；手动流持续发送。收到协议版本 3、`side_mount_joint` 标记及新鲜有效机械角度遥测后才启用手动和单轴复位。没有滚转控制或滚转遥测。

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
