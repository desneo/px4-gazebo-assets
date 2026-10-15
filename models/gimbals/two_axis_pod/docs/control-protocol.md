# 两轴控制与遥测协议

使用 UTF-8 JSON，UDP 默认 `8766`；视频为 `rtsp://<host>:8555/two_axis_pod_<aircraft_id>`。每条 UDP 指令包含单调递增整数 `seq`，单条数据报不超过 8 KiB。网关保留会话去重、旧控制指令拒绝和插件执行回执，行为见[网关说明](../controller/service/README.md)。

## 角度

遥测标识 `protocol_version=4`、`angle_convention=upright_joint`。yaw/pitch 为机械角，单位度；速度为度/秒，加速度为度/秒²。`attitude_deg` 与 `joints_deg` 均为两轴机械诊断对象，不是 PX4 FRD Euler 姿态。立装零位朝天，yaw 绕竖直 +Z，pitch 绕随动水平 +Y 且正值朝前。限位为 yaw ±90°、pitch −45°～+90°。

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

`manual` 输入为角速度而非目标角。选框必须完整位于图像内且具有足够纹理；无视频或帧超时拒绝跟踪。网关将 `track/stop/home/reset/reset_axis` 的插件执行结果关联为 UDP `command_result`，客户端以 `ok` 判断执行结果。`manual/reset_fault` 不转发插件成功回执，应依据新鲜遥测确认生效；网关参数校验或发布失败仍返回失败。

## 遥测

- `attitude_deg.yaw/pitch`、`joints_deg.yaw/pitch`：各含 `pos`、`vel`、`acc`。
- `cmd_dps.yaw/pitch`：关节速度指令；`limits.yaw/pitch/saturated`：限位与饱和。
- `los_mount`：安装系镜头单位方向；`singular`：俯仰接近 0°、视线朝天时偏航视线控制退化，不禁止机械手动或回零。
- `camera.quat_body/quat_world`：实际光学系的四元数，顺序 wxyz；`rpy_body_deg/rpy_world_deg` 是光学姿态的诊断表示，与机械角不同。
- `mount.mode=upright`：立装机械结构。默认 `mount.static=true`；装配插件设置 `mount_dynamic=true` 时为 `false`，每步更新安装系实际世界姿态，TRACK 中以载机角速度前馈抵消载机转动。
- `mount.quat_world`：安装参考系相对世界的四元数，顺序 wxyz；动态模式随载机变化。
- `tracking.constraint`：`NONE`、`SINGULAR`（偏航视线退化）、`LIMITED`（机械限位）、`RATE_LIMITED`（速度输出饱和）。`tracking.state=TRACK` 表示图像跟踪器仍识别选框，不等于所有轴均可稳定指向；应同时检查中心误差、稳定性和约束。
- `mode`：IDLE/MANUAL/HOME/TRACK；`tracking.state`、`target`、`stability`：跟踪状态、选框和中心误差。
- `frame`、`frame_table`、`ts_wall`、`server_uptime`、`video`：供视频、时序和网关状态显示。`hello.video_rtsp_path` 上报完整 RTSP 路径；地面站按配置的前缀和飞机 ID 生成 URL，两端配置必须一致。

俯仰 0° 时偏航轴与视线方向平行，偏航仍可机械转动，球心和画面方向随支架变化，但无法改变镜头朝天方向。视觉伺服以阻尼最小二乘处理该处退化；机械可达范围与两轴结构约束仍然存在。视频断流时停止追踪速度，持续丢失后需重新选框。

Gazebo 专用话题为 `/two_axis_pod/command`、`/two_axis_pod/telemetry` 和 `/two_axis_pod/command_result`，消息均为 `gz.msgs.StringMsg` 中的 JSON。插件执行结果携带 `cmd`、`ok`、`message` 与网关生成的 `request_id`。

组合模型可以在插件 SDF 中配置上述话题和 `image_topic`。网关的 `--topic-prefix` 与 `--image-topic` 必须对应装配配置；未指定时沿用独立组件的话题。动态补偿不使用目标世界坐标，也不提供第三轴画面水平稳定；平移、目标运动与模型误差由图像反馈修正。

## 额外主动状态副本

网关可通过独立 UDP socket 将上述遥测 JSON 额外发送到配置的 IP/端口。默认频率 20 Hz，可配置；station 原有默认 50 Hz 广播与控制会话保持独立。报文仍为 `type:"telemetry"`、协议 v4，沿用原字段，不添加外层封装。接收端无需 ping 或登记，不会收到 hello 或控制回执。

默认不开启，启用、目标地址和发送频率见[网关配置](../controller/service/README.md#额外-udp-json-状态上报)。尚无插件遥测或超过配置时效时暂停额外发送；接收端应按配置频率自行判断连续报文缺失。目标接收端口与 station 控制端口分别配置。

网关在 `hello` 和遥测中附加 `aircraft_id`（非负整数，默认 1）与 `video_rtsp_path`（例如 `/two_axis_pod_1`）。同机多路共用 RTSP 8555，按路径末尾的飞机 ID 区分；ID 不改变机械角度语义，UDP 控制仍使用各机独立端口。详见[多机视频配置](../controller/service/README.md#多机共享-rtsp)。

地面站按配置的 `aircraft_id` 核对当前网关 `hello.aircraft_id`，必须是相同的非负整数。尚未确认、缺失/非法 ID、ID 不符或连接超时时禁止所有控制（含停止、零速度、回零、跟踪和清故障），仅允许 ping、status、bye。身份不符时清除待重传控制，界面和日志显示期望 ID 与网关 ID；连接超时后仅收到 pong 或遥测不会恢复控制，须重新收到匹配 hello。GUI 与 `--selftest` 均使用相同校验，自检身份未确认时返回非零退出码。旧网关缺少 ID 时须升级网关，不能跳过校验。
