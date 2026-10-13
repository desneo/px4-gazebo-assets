# 两轴控制与遥测协议

使用 UTF-8 JSON，UDP 默认 `8766`；视频为 `rtsp://<host>:8555/two_axis_pod`。每条 UDP 指令包含单调递增整数 `seq`，单条数据报不超过 8 KiB。网关保留会话去重、旧控制指令拒绝、接收回执和插件执行回执，行为见[网关说明](../controller/service/README.md)。

## 角度

遥测标识 `protocol_version=3`、`angle_convention=side_mount_joint`。yaw/pitch 为机械角，单位度；速度为度/秒，加速度为度/秒²。`attitude_deg` 与 `joints_deg` 均为两轴机械诊断对象，不是 PX4 FRD Euler 姿态。零位朝天、pitch 正值朝前，yaw 轴绕侧板法线。限位为 yaw ±90°、pitch −45°～+90°。

不支持 roll，`manual` 中出现 roll 字段（即使为 0）和 `reset_axis` 指定 roll 均拒绝。客户端必须验证协议与角度约定，不能把本组件当作三轴组件发送角速度。

## 指令

| cmd | 参数 | 行为 |
|---|---|---|
| `status` | `seq` | 请求当前遥测；首次有效请求自动登记并收到 hello |
| `ping` | `seq` | 保活与时延测量 |
| `bye` | `seq` | 注销客户端 |
| `manual` | `yaw`, `pitch` | 两轴机械角速度，缺省轴为 0；需持续发送，超过 0.5 s 停止 |
| `stop` | 无 | 立即停止控制并清除跟踪目标 |
| `reset` | 无 | 两轴回机械 0°、镜头朝天 |
| `home` | 无 | 与 reset 相同，两轴回机械零位 |
| `reset_axis` | `axis: yaw/pitch` | 指定轴回 0°，另一轴保持收到指令时的机械位置 |
| `track` | `box: [x,y,w,h]` | 在最新视频中选择归一化矩形，运行 CSRT 与视觉伺服 |
| `reset_fault` | 无 | 清除故障并停止 |

```json
{"cmd":"manual","seq":1,"yaw":15,"pitch":20}
{"cmd":"reset_axis","seq":2,"axis":"pitch"}
{"cmd":"track","seq":3,"box":[0.4,0.35,0.2,0.3]}
```

`manual` 输入为角速度而非目标角。选框必须完整位于图像内且具有足够纹理；无视频或帧超时拒绝跟踪。网关命令接收和插件执行为不同阶段，可靠命令应以 `command_result.ok` 判断执行结果；连续 `manual` 流不返回成功回执，应依据新鲜遥测确认生效，参数错误仍返回失败。

## 遥测

- `attitude_deg.yaw/pitch`、`joints_deg.yaw/pitch`：各含 `pos`、`vel`、`acc`。
- `cmd_dps.yaw/pitch`：关节速度指令；`limits.yaw/pitch/saturated`：限位与饱和。
- `los_mount`：安装系镜头单位方向；`singular`：俯仰接近 +90° 时视线控制退化提示，不禁止机械手动或回零。
- `camera.quat_body/quat_world`：实际光学系的四元数，顺序 wxyz；`rpy_body_deg/rpy_world_deg` 是光学姿态的诊断表示，与机械角不同。
- `mount.mode=side`、`mount.static=true`：侧装和固定安装参考，插件不执行动态载机补偿。
- `mode`：IDLE/MANUAL/HOME/TRACK；`tracking.state`、`target`、`stability`：跟踪状态、选框和中心误差。
- `frame`、网关时间戳、视频信息：供视频与时延显示。网关补充帧尺寸和RTSP地址。

俯仰 +90° 时偏航轴与视线平行，偏航仍可控制画面旋转，但无法提供横向视线调整。视觉伺服以阻尼最小二乘处理该处退化；机械可达范围与两轴结构约束仍然存在。视频断流时停止追踪速度，持续丢失后需重新选框。

Gazebo 专用话题为 `/two_axis_pod/command`、`/two_axis_pod/telemetry` 和 `/two_axis_pod/command_result`，消息均为 `gz.msgs.StringMsg` 中的 JSON。插件执行结果携带 `cmd`、`ok`、`message` 与网关生成的 `request_id`。
