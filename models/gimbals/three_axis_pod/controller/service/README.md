# 吊舱网络网关

`gimbal_bridge.py` 订阅插件 JSON 遥测、回执与相机图像，转发 UDP 指令并提供 H.264 RTSP。依赖模块保存在本目录：`gz_source.py/video.py/stamp.py/attitude.py/kinematics.py`。

在仓库根目录运行 `bash models/gimbals/three_axis_pod/scripts/run_bridge.sh --encoder nvenc`，先启动同一 GZ_PARTITION 下的仿真。默认 UDP 8765、RTSP 8554、挂载点 `/gimbal`；用 `python3 models/gimbals/three_axis_pod/controller/service/gimbal_bridge.py --help` 查看参数。

默认硬编码元件是 GStreamer `nvcudah264enc`，需要 NVIDIA GPU、驱动及对应插件。编码管线失败时不会自动改用软件编码；`--encoder x264` 是显式软件编码选项。地面站的 `--decoder avdec` 仅选择客户端软件解码，不改变网关的硬编码。

使用自定义 world 时，通过 `--world`、`--gimbal-model`、`--camera-link`、`--camera-sensor` 对应真实图像话题；默认依次为 `three_axis_pod_test`、`three_axis_pod`、`camera_link`、`camera`。三个控制话题 `/gimbal/command`、`/gimbal/telemetry`、`/gimbal/command_result` 在网关源码中固定；修改插件话题时需同步网关。

接口、去重窗口、回执关联与会话限制见 [系统说明](../../docs/system.md)。

网关 `hello` 声明 `version=2`、`angle_convention=px4_frd`。公共 `manual` 角速度、`reset/reset_axis` 姿态目标和 `attitude_deg` 由插件执行/提供；`joints_deg/cmd_dps` 仅作机械诊断。客户端须检查版本和约定，不可用旧机械角度接口解释公共字段。每条命令的 JSON 字符串示例见[控制协议](../../docs/control-protocol.md)。

并行运行网关实例时，UDP、RTSP 和本机 RTP 中继端口均须不同；通过 `--udp-port`、`--rtsp-port`、`--rtp-port` 配置，默认分别为 8765、8554、5600。
