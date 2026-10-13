# 两轴吊舱网络网关

`gimbal_bridge.py` 将 `/two_axis_pod/command`、`telemetry`、`command_result` 三个 Gazebo StringMsg 话题桥接为 UDP 控制与遥测，并提供 RTSP 视频。跟踪和机械控制在 C++ 插件内执行，网关不重复计算姿态。

默认世界为 `two_axis_pod_demo`，模型为 `two_axis_pod`，图像来自 `camera_link/camera`。默认 UDP 8766、RTSP 8555 `/two_axis_pod`、内部 RTP 5602，与三轴组件使用独立端口。使用入口见[启动说明](../../scripts/README.md)。

首次有效请求收到 `hello`，`version=4`，`angle_convention=upright_joint`。`manual` 只接受 yaw、pitch 机械角速度，单位 °/s；任何 roll 字段均拒绝。`reset_axis` 仅接受 yaw 或 pitch。视线奇异点不禁止机械控制；俯仰 0°、镜头朝天时，偏航转动不改变视线。

每个客户端地址具有独立会话。带 seq 请求按 1024 项窗口去重；同 seq 不同内容拒绝，乱序控制拒绝。可靠命令通过唯一 request_id 关联插件回执和客户端 seq。超时、发布失败与过期会话明确返回失败，不自动再次执行。客户端应持续发送 ping 续期，默认 5 秒过期。数据报上限 8 KiB。

视频默认 NVENC，也可显式指定 `--encoder x264`。初始化失败仍保留控制与遥测服务；视频帧烧录序号，遥测增加 frame_table 和视频计数。同机 boot_id 相符时可测单调时钟端到端时延。

运行依赖：Python 3、Gazebo Harmonic 的 gz-transport13 / gz-msgs10 Python 绑定、NumPy、PyGObject、GStreamer 及 GstRtspServer；视频还需要对应 H.264 编码器。`gz_source.py` 只管理节点和仿真时钟映射。
