# 两轴吊舱网络网关

`gimbal_bridge.py` 将 `/two_axis_pod/command`、`telemetry`、`command_result` 三个 Gazebo StringMsg 话题桥接为 UDP 控制与遥测，并提供 RTSP 视频。跟踪和机械控制在 C++ 插件内执行，网关不重复计算姿态。

默认世界为 `two_axis_pod_demo`，模型为 `two_axis_pod`，图像来自 `camera_link/camera`。默认 UDP 8766、RTSP 8555 `/two_axis_pod`、内部 RTP 5602，与三轴组件使用独立端口。使用入口见[启动说明](../../scripts/README.md)。

组合模型可用 `--topic-prefix /rocket_two_axis_pod` 对应插件的 command、telemetry、command_result 话题，用 `--image-topic /rocket_two_axis_pod/image` 指定完整图像话题；未指定时保持独立组件默认值。`--world`、`--gimbal-model`、UDP/RTSP/RTP 端口分别配置，Gazebo 分区只隔离 Gazebo Transport，不隔离网络端口。

首次有效请求收到 `hello`，`version=4`，`angle_convention=upright_joint`。`manual` 只接受 yaw、pitch 机械角速度，单位 °/s；任何 roll 字段均拒绝。`reset_axis` 仅接受 yaw 或 pitch。视线奇异点不禁止机械控制；俯仰 0°、镜头朝天时，偏航转动不改变视线。

每个客户端地址具有独立会话。带 seq 请求按 1024 项窗口去重；同 seq 不同内容拒绝，乱序控制拒绝。可靠命令通过唯一 request_id 关联插件回执和客户端 seq。超时、发布失败与过期会话明确返回失败，不自动再次执行。客户端应持续发送 ping 续期，默认 5 秒过期。数据报上限 8 KiB。

视频默认 NVENC，也可显式指定 `--encoder x264`。初始化失败仍保留控制与遥测服务；视频帧烧录序号，遥测增加 frame_table 和视频计数。同机 boot_id 相符时可测单调时钟端到端时延。

## 额外 UDP JSON 状态上报

`status_udp.json` 配置向指定 IP/端口主动发送一份状态副本，默认关闭、频率 20 Hz。使用独立 UDP socket 和独立发送线程，不登记为 station 客户端，不改 station 的默认 50 Hz 遥测、会话或控制回执。接收端无需发送 ping、注册或确认，只需监听配置的端口。

修改本目录的 `status_udp.json` 后重新启动网关，例如：

```json
{
  "enabled": true,
  "ip": "192.168.1.100",
  "port": 9000,
  "hz": 20,
  "stale_timeout_s": 1.0
}
```

`ip` 接受 IPv4/IPv6 字面地址；`port` 为 1～65535 的整数；`hz` 为 (0, 100] 的有限数值。`stale_timeout_s` 为插件遥测接收时效，须为正的有限秒数。未收到首条插件遥测时不发送；超过时效时暂停额外上报，收到新遥测后自动恢复，避免持续发送旧状态。配置错误会在网关初始化前明确报错。

可指定其他配置文件，或用命令行覆盖目标与频率：

```bash
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --encoder x264 \
  --status-config /path/to/status_udp.json
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --encoder x264 \
  --status-ip 192.168.1.100 --status-port 9000 --status-hz 20
```

`--status-ip` 与 `--status-port` 必须同时指定，指定后启用额外上报；`--status-hz` 可单独覆盖配置频率。配置在启动时读取，修改后需重启网关。地理场景启动器会透传这些参数。

每个 UDP 数据报是一条完整的 UTF-8 JSON，沿用原 station 遥测的所有字段：`type:"telemetry"`、`protocol_version:4`、`angle_convention:"upright_joint"`，以及 `attitude_deg`、`joints_deg`、`tracking`、`limits`、`camera`、`mount`、`frame`、`frame_table`、`video` 等；不增加外层封装，不附带 hello 或指令回执。字段定义见[控制与遥测协议](../../docs/control-protocol.md)。额外 socket 的源端口由系统分配，目标端口是此处配置的接收端口，不能据此将接收端当作 station 控制客户端。

独立发送使用非阻塞 UDP，单包上限 8 KiB。发送错误、非法 JSON 数值或超长报文会丢弃当前包并限频记录日志；后续按配置周期继续尝试，不影响 station，不重传积压报文。UDP 不保证到达或顺序。

运行依赖：Python 3、Gazebo Harmonic 的 gz-transport13 / gz-msgs10 Python 绑定、NumPy、PyGObject、GStreamer 及 GstRtspServer；视频还需要对应 H.264 编码器。`gz_source.py` 只管理节点和仿真时钟映射。
