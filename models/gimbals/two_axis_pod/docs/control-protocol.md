# 两轴控制与遥测协议

使用 UTF-8 JSON，UDP 默认 `8766`；视频为 `rtsp://<host>:8555/two_axis_pod_<aircraft_id>`。每条 UDP 指令包含单调递增的非负整数 `seq`，单条数据报不超过 8 KiB。网关保留会话去重、旧控制指令拒绝和插件执行回执，行为见[网关说明](../controller/service/README.md)。本协议不依赖随仓库提供的图形地面站；保持同一个 UDP socket 的第三方 station、机载计算机或调试 CLI 均可接入。

## 传输、会话与兼容性

- 一个客户端会话由源 IP 和源 UDP 端口共同标识。客户端必须使用同一个已绑定 socket 发送指令并接收 `hello`、遥测和回执；不能每条命令换一个临时源端口。
- 第一个语法与命令名均有效的数据报登记会话。网关先发送 `hello`，随后处理该数据报；UDP 不保证接收顺序，客户端必须按 `type` 和 `seq` 分流，不能假设 `hello` 一定先于本次命令的响应到达。
- `hello.version` 必须为 `8`，`hello.angle_convention` 必须为 `motion_head_joint_right_positive`，`hello.aircraft_id` 必须与预期载机一致。未确认飞机身份时只允许 `ping/status/bye`。身份确认后，停止、双轴零速、关闭视觉和只检测可用；非零手动速度、回零、单轴复位、选框跟踪和自动锁定还须由 hello 与新鲜有效机械角度遥测共同确认版本及角度约定。
- 未收到兼容插件姿态或收到旧协议姿态时，hello 的 `plugin_compatible=false`，`version` 与 `angle_convention` 为 `null`，不转发姿态并拒绝运动请求；兼容状态改变后重新发送 hello；后续 ping 保活也补发当前 hello，使丢失的就绪通知可以恢复。插件、网关、地面站均需配套更新。
- 已登记客户端逐份接收插件新姿态。频率统一读取 `controller/service/telemetry.json` 中的 `hz`，默认 50 Hz（单调真实时间）；暂停或断流时不重发旧姿态，物理更新不足配置频率时实际速率下降并告警。任意有效命令均刷新会话；空闲客户端应以不超过 2 秒的周期发送 `ping`。默认连续 5 秒没有有效命令即注销，恢复时必须重新收到并核验 `hello`。
- UDP 不保证到达、顺序或唯一性。可靠命令在超时后重发时必须逐字义保持同一 JSON 内容和同一 `seq`；收到对应结果后才分配下一项操作。连续手动速度流则为每个采样分配新的递增 `seq`，不重发旧速度。
- JSON 数值必须有限，布尔值不视为数值。客户端接收缓冲区建议至少 64 KiB，并应忽略未知字段，以便协议向后兼容扩展。
- 网关允许多个地址同时登记以观察遥测，但当前不提供操作员所有权或控制租约；所有控制命令作用于同一个吊舱，后到的有效命令可能覆盖前一控制。部署时同一吊舱只允许一个 station 获得控制权，其他界面应只读。

最小接入顺序：

1. 创建并绑定 UDP socket，保持其源端口不变。
2. 发送 `status` 或 `ping`，接收并核验 `hello`。
3. 等待一条新鲜 `telemetry`，再次核验 `protocol_version=8`、`angle_convention=motion_head_joint_right_positive` 和 `aircraft_id`。
4. 使用单调递增 `seq` 下发控制，并按下文的响应类型确认结果。
5. 空闲时持续 `ping`；退出前发送 `manual(0,0)` 停车，再发送 `bye`。

## 角度

遥测标识 `protocol_version=8`、`angle_convention=motion_head_joint_right_positive`。yaw/pitch 为机械角，单位度；速度为度/秒，加速度为度/秒²。`attitude_deg` 与 `joints_deg` 均为两轴机械诊断对象，不是 PX4 FRD Euler 姿态，也不是视线方位角/仰角。底座圆环水平、底座向下时，安装系为 +X 前、+Y 左、+Z 上。yaw 是绕安装 +X 的水平外轴，范围 ±60°；pitch 是绕随动 −Y 的内轴，范围 −80°～+30°。两轴均为 CAD 原始 0° 时光轴沿安装 +Z 朝天；yaw=0 时，pitch 负值向 +X 前方转动，−80° 时光轴高于前方水平面 10°，+30° 时朝后上方。

运动链为 `Rx(yaw) Ry(-pitch)`，镜头单位方向为 `[-sin(pitch), -sin(yaw)cos(pitch), cos(yaw)cos(pitch)]`。外轴穿过球心，球心不随外轴公转。零位图像上方沿安装 −X、图像右方沿安装 −Y；yaw 以镜头视线向右转动为正、向左转动为负；静止背景移动方向与镜头视线相反。pitch 仍沿原机械定义。位置、速度、加速度和指令使用同一机械方向，不能只对显示值或艾晟输出反号。模型插件、网关和 Linux/Windows 控制端必须配套使用协议 8 与 `motion_head_joint_right_positive`；旧机械约定不兼容。

不支持 roll，`manual` 中出现 roll 字段（即使为 0）和 `reset_axis` 指定 roll 均拒绝。客户端必须验证协议与角度约定，不能把本组件当作三轴组件发送角速度。

## 指令

| cmd | 参数 | 行为 |
|---|---|---|
| `status` | `seq` | 请求当前遥测；首次有效请求自动登记并收到 hello |
| `ping` | `seq` | 保活与时延测量 |
| `bye` | `seq` | 注销客户端 |
| `manual` | `yaw`, `pitch` | 两轴机械角速度，缺省轴为 0；非零输入需持续发送，超过 0.5 s 停止；双轴为 0 时立即正常停车 |
| `stop` | 无 | 立即停止控制并清除跟踪目标 |
| `reset` | 无 | 两轴回 CAD 原始机械 0°、镜头朝安装 +Z；底座水平静置时朝天 |
| `home` | 无 | 与 reset 相同，两轴回机械零位 |
| `reset_axis` | `axis: yaw/pitch` | 指定轴回 0°，另一轴保持收到指令时的机械位置 |
| `set_vision_mode` | `mode` | 切换 `off/detect/auto_lock/manual_track` 互斥视觉模式 |
| `track` | `box: [x,y,w,h]`，可选 `frame_seq` | 仅在 `manual_track` 接受外部选框；网关自动锁定按检测帧启动插件单目标后端 |
| `reset_fault` | 无 | 清除故障并停止 |

```json
{"cmd":"status","seq":1}
{"cmd":"manual","seq":2,"yaw":15,"pitch":20}
{"cmd":"reset_axis","seq":3,"axis":"pitch"}
{"cmd":"set_vision_mode","seq":4,"mode":"manual_track"}
{"cmd":"track","seq":5,"box":[0.4,0.35,0.2,0.3]}
```

`manual` 输入为机械角速度而非目标角。两轴最终控制速度均限制为 ±20°/s，适用于人工、回零、单轴复位和跟踪补偿后的输出；超过范围的有效输入限幅执行。任一轴非零时，网关会自动将 `auto_lock` 或 `manual_track` 切换为 `off`，停止当前跟踪后再执行人工速度；因此第三方 station 无需复现随仓库地面站的内部抢占逻辑。为让界面立即显示确定状态，station 也可先可靠发送 `set_vision_mode(mode=off)`，收到成功结果后再发送速度流。非零速度至少以 10 Hz、每包新 `seq` 连续发送；双轴零速是正常松手停车。只有非零流超过 0.5 秒未续发时才产生“控制输入超时，已停止”安全告警。

选框坐标相对完整视频帧归一化：`x/y` 为左上角，`w/h` 为宽高，均不是像素；必须满足 `0≤x,y<1`、`w,h>0`、`x+w≤1`、`y+h≤1`，并具有足够纹理。station 显示视频时应先去除黑边并把显示坐标反算到完整原始帧，再生成归一化选框。无视频、帧超时或未处于 `manual_track` 时拒绝外部跟踪。`frame_seq` 是非负整数；插件只在最近 4 个已处理图像中存在该序号时接受。普通 station 手动画框应省略 `frame_seq` 以使用最新帧；只有能够从 RTSP 帧戳与 `frame_table` 建立精确对应的客户端才应填写。

### 响应类型与确认规则

| 请求 | 正常响应 | 控制端处理要求 |
|---|---|---|
| 首个有效请求 | `hello`，并继续返回该请求自身响应 | 按 `type` 分流并完成身份核验 |
| `ping` | `pong`，回显 `seq` 与可选 `t` | 可计算往返时延，不代表身份重新核验完成 |
| `status` | 一条 `telemetry`，带本次 `seq` | 用于主动读取；注册后仍会收到无请求 `seq` 的周期遥测 |
| `set_vision_mode` | `command_result` | 网关本地完成切换后返回 |
| `track/stop/home/reset/reset_axis` | `command_result` | 等待插件实际执行结果；只以 `ok=true` 判定成功 |
| `manual/reset_fault` | 无成功回执 | 依据后续新鲜遥测确认；参数或发布失败仍可能返回失败结果 |
| `bye` | 无响应 | 本地关闭 socket |

典型响应如下；`message` 为空表示没有补充信息，失败时包含可直接显示的中文原因：

```json
{"type":"hello","proto":"gimbal-udp","version":8,"angle_convention":"motion_head_joint_right_positive","plugin_compatible":true,"aircraft_id":1,"video_rtsp_path":"/two_axis_pod_1","capabilities":["vision_mode","server_video_overlay","auto_lock","frame_exact_track"]}
{"type":"pong","seq":10,"t":123.456}
{"type":"command_result","cmd":"set_vision_mode","seq":11,"ok":true,"message":""}
{"type":"command_result","cmd":"track","seq":12,"ok":false,"message":"无效选框或视频"}
{"type":"error","seq":13,"message":"暂无遥测"}
```

`command_result` 的 `seq` 对应控制端请求，而网关与 Gazebo 插件间使用的 `request_id` 不对外暴露。可靠命令等待回执期间，可按同一 `seq` 和相同内容重发；网关不会重复执行，已有结果会被重放。同一会话中将同一 `seq` 用于不同内容会失败；比最近控制序号更旧的控制不会执行。建议 station 对单条可靠命令设置约 0.2 秒重试间隔和有限重试次数，同时继续接收遥测；插件回执最终超过 30 秒时网关返回“执行结果未知”。

`detect` 只检测并在 RTSP 画面烧录红框和轨迹号，不控制云台。`auto_lock` 搜索已确认轨迹时显示相同红框，首次选择离画面中心最近的飞机后把选框交给插件，并暂停检测叠加；随后通过 `target.box_px` 上报单目标框，客户端可将其绘制为绿色。默认 `sim_truth` 在对应 `frame_seq` 中从框内已登记飞机选取离框中心最近者并固定绑定，其距离缩放框上下左右各扩展 5 px 并裁剪到画面边界后写入 `target.box_px`；显式 `csrt` 则初始化图像跟踪且不增加该真值框边距。后端进入 LOST 后重新搜索。`manual_track` 才允许客户端提交 `track`，其真值绑定规则同样为框内飞机。切换任一模式会停止前一模式留下的跟踪并清空多目标轨迹；人工停止、回零、复位，以及自动锁定或手动框选期间的非零人工转动，都会先切换到 `off` 并退出目标跟踪。

## 第三方 station 接入

第三方 station 应将 UDP 控制/遥测、RTSP 解码和界面渲染做成三个互不阻塞的执行单元。UDP 接收循环持续处理 `hello/telemetry/pong/command_result/error`；视频循环只保留最新解码帧；界面线程读取两者最新快照，不应让窗口刷新或模型推理阻塞手动指令续发。

| station 操作 | 下发 | 以遥测同步界面 |
|---|---|---|
| 关闭视觉 | `set_vision_mode(off)` | `vision.mode=off` |
| 目标检测 | `set_vision_mode(detect)` | 红框/编号已烧录在 RTSP；读取 `vision.target_count` 和速率 |
| 自动锁定 | `set_vision_mode(auto_lock)` | 搜索阶段使用 RTSP 红框；锁定后读取 `target.box_px` 自行叠加单个绿色框 |
| 手动框选 | `set_vision_mode(manual_track)`，再发 `track` | 使用 `target.box_px` 叠加绿色框，跟踪状态取 `tracking.state` |
| 人工转动 | 连续发送 `manual`，松手发双轴零速 | 非零输入会使 `vision.mode` 变为 `off`；界面应以遥测为准取消锁定/框选按钮高亮 |
| 停止、回零或复位 | 对应可靠命令 | 等待 `command_result`，再以遥测显示最终机械状态 |

检测和自动搜索阶段的红框属于服务端 RTSP 画面。自动选中或手动框选后的绿色框属于 station 叠加层，不会烧录进 RTSP；其坐标来自 `target.box_px`，颜色和样式可由 station 决定。按钮状态必须以最新 `vision.mode` 遥测为准，不能只依据本地点击结果，因为人工输入、其他客户端或故障处理可能改变模式。

### 控制流程示例

下面是与具体语言无关的推荐顺序：

```text
bind one UDP socket
send status(seq=1)
receive hello -> verify version, angle_convention, aircraft_id
receive telemetry -> verify the same identity and fresh two-axis attitude

# 只检测
send set_vision_mode(seq=2, mode="detect") -> wait command_result(ok=true)

# 自动搜索并锁定
send set_vision_mode(seq=3, mode="auto_lock") -> wait command_result(ok=true)
observe telemetry until vision.state="AUTO_LOCKED" and tracking.state="TRACK"

# 手动画框
send set_vision_mode(seq=4, mode="manual_track") -> wait command_result(ok=true)
send track(seq=5, box=[x,y,w,h]) -> wait command_result(ok=true)

# 人工速度抢占；每 0.1 s 使用新 seq 发送一次
send manual(seq=6, yaw=15, pitch=0)
send manual(seq=7, yaw=15, pitch=0)
...
send manual(seq=N, yaw=0, pitch=0)  # 正常松手停车

send bye(seq=N+1)
close socket
```

一个进程只监听遥测时发送 `status` 完成登记并周期 `ping` 即可。调试 CLI 可以复用同一协议，但不要使用每次启动一个新进程、发送一个 UDP 包就退出的方式做连续控制：它会改变源端口、丢失回执和身份状态，也无法满足手动速度看门狗。

## 遥测

- `attitude_deg.yaw/pitch`、`joints_deg.yaw/pitch`：各含 `pos`、`vel`、`acc`。
- `cmd_dps.yaw/pitch`：关节速度指令；`limits.yaw/pitch/saturated`：限位与饱和。
- `los_mount`：安装系镜头单位方向；`singular`：光轴接近外轴方向时视线控制退化，不禁止机械手动或回零；当前机械限位内不出现该退化。
- `camera.quat_body/quat_world`：实际光学系的四元数，顺序 wxyz；`rpy_body_deg/rpy_world_deg` 是光学姿态的诊断表示，与机械角不同。
- `mount.mode=upright`：底座向下的立装机械结构，外轴沿安装 +X；此字段不表示竖直偏航轴。默认 `mount.static=true`；装配插件设置 `mount_dynamic=true` 时为 `false`，每步更新安装系实际世界姿态。动态安装优先把Gazebo `mount_link` 世界角速度转换到当前安装局部系；仅在速度数据不可用时使用带35 Hz一阶低通和单步跳变拒绝的姿态差分。TRACK 中以前馈抵消载机转动，IDLE不主动稳像。
- `mount.quat_world`：安装参考系相对世界的四元数，顺序 wxyz；动态模式随载机变化。
- `tracking.backend`：`sim_truth` 或 `csrt`；`tracking.constraint`：`NONE`、`SINGULAR`（外轴视线退化）、`LIMITED`（机械限位）、`RATE_LIMITED`（速度输出饱和）。`tracking.state=TRACK` 表示当前后端持续提供有效目标，不等于所有轴均可稳定指向；应同时检查中心误差、稳定性和约束。
- `mode`：IDLE/MANUAL/HOME/TRACK；`tracking.state`、`target`、`stability`：跟踪状态、选框和中心误差。真值绑定期间 `target.entity_id/entity_name` 为固定实体，CSRT 或未绑定时为 `null`。
- `target.miss`：被跟踪目标的二维脱靶量，自动锁定和手动框选及两种单目标后端使用相同字段，定义如下。
- `frame`、`frame_table`、`ts_wall`、`server_uptime`、`video`：供视频、时序和网关状态显示。`hello.video_rtsp_path` 上报完整 RTSP 路径；地面站按配置的前缀和飞机 ID 生成 URL，两端配置必须一致。
- `vision.mode/state/target_count/locked_track_id/error`：当前互斥模式、检测/搜索/锁定状态、已确认目标数、锁定轨迹号和视觉错误。`inference_ms/input_fps/output_fps` 是模型推理和最新帧处理速率诊断。

CAD 零位两轴均能改变视线方向。数学上 pitch 接近 ±90° 时光轴与外轴平行，外轴转动只改变画面方向，视觉伺服以阻尼最小二乘处理退化；当前 pitch −80°～+30° 限位不包含该位置。底座水平时，当前可达光轴最低约高于水平面 5°，这与机械 yaw/pitch 的数值范围不同。视频断流时停止追踪速度，持续丢失后需重新选框。

### 目标惯性视线角速度

`target_los` 使用目标模型原点（当前真值跟踪使用的目标参考点）相对无人机**零关节固定名义重心**的视线，不使用镜头光心、关节转速、控制指令或机体 Euler 角速度替代。`frame=NED`，N 北、E 东、D 地；Gazebo 世界位置结合世界球坐标 heading 旋转到 ENU 后转换为 `(N,E,D)=(Y,X,-Z)`。未配置球坐标时，世界轴按标准 ENU 处理。

令 `r=(N,E,D)=p_target-p_cg`，`v=dr/dt`，`h=hypot(N,E)`，方位角 `psi=atan2(E,N)`，仰角 `theta=atan2(-D,h)`：

```text
yaw_rate   = (N*vE - E*vN) / h²
pitch_rate = (D*(N*vN + E*vE)/h - h*vD) / (h² + D²)
```

上式为 rad/s，遥测转换为 °/s。方位角向东/绕下轴向右增加，仰角向上增加；这是角度导数，不是三维视线角速度向量的两个分量。计算使用同一 PostUpdate 物理状态的双方位置和仿真时间差分，不差分包裹到 ±180° 的角度。50 Hz 配置是单调真实时间的发送目标，不是物理角速度的时间分母。

SDF `los_observer_cg_model` 为三项有限米制模型坐标，装配生成器从零关节质量/惯性数据派生，与 PX4 名义重心参考一致。世界观察点为 `p_model + R_world_model * cg_model`，关节转动不重新定义该固定偏移。缺失或非法配置只让 LOS 无效，不禁用既有控制；独立吊舱不是无人机，不自动把吊舱重心或光心当作无人机名义重心。

`sim_truth` 使用当前绑定的目标实体。`csrt` 需配置 `los_target_name`，并在选框初始化时用同帧真值投影验证该模型原点的投影位于选框内；未匹配时 CSRT 仍可跟踪，但 LOS 无效，不猜测实体，也不因同名实体重建而自动换绑。采样独立于图像更新频率；非 TRACK 时 LOS 无效。

对象包含 `omega_valid`、`omega_reason`、`omega_ned_dps`（按N/E/D排列的三维向量），以及 `valid`、`frame`、`observer_reference=aircraft_nominal_cg`、`target_name`、`reason`、`sim_stamp`、`yaw_rate_dps`、`pitch_rate_dps`、`relative_position_ned_m`、`relative_velocity_ned_mps`。`sim_stamp` 为 LOS 采样仿真时刻；现有 PreUpdate 遥测携带最近 PostUpdate 结果，与关节采样可相差一个物理步，LOS 双方位置始终来自同一状态。

内部真实 LOS 不受机械 ±20°/s 限制；限幅只在艾晟输出边界执行。未绑定、首样本、目标删除、时间回退/暂停/缺采样、非有限数据、位置跳变、零距离或近竖直视线等情况使 `valid=false`，两个内部速率为 `null`，不沿用旧有效值。竖直附近的方位角数学上不可定义；界面跟踪状态不改变此有效性。

三维向量直接由原始相对位置和相对速度计算：

```text
omega_N = (E*vD - D*vE) / |r|²
omega_E = (D*vN - N*vD) / |r|²
omega_D = (N*vE - E*vN) / |r|²
```

转换为°/s后写入`omega_ned_dps=[N,E,D]`，不使用限幅后的方位/仰角导数重建。该向量垂直于视线，是不含绕视线自转的角速度；N/E/D分量不是机械roll/pitch/yaw。向量不受±20°/s限制。

`omega_valid/omega_reason`独立判定三维向量，向量无效时`omega_ned_dps=null`；首样本、未绑定、时间异常等保护同时影响两类结果，但竖直视线可以保持向量有效，而`valid=false`、方位/仰角导数为null。非有限向量仍拒绝。v8将此作为新增字段扩展，升级接收端须同步识别这些字段；机械控制与原有字段定义保持一致。

### 跟踪脱靶量

像素坐标采用原始相机图像：左上角为原点，X 向右为正，Y 向下为正。`center_px=[cx,cy]` 为后端目标中心的绝对像素坐标：`sim_truth` 使用绑定实体的真实投影中心，`csrt` 使用跟踪框中心；`sim_truth` 的 5 px 扩框和画面边界裁剪不改变该中心。`offset_px=[cx-W/2,cy-H/2]` 为相对画面中心的有符号脱靶量。界面缩放不改变这些数值。

`offset_deg=[horizontal,vertical]` 是相对相机光轴的两分量角偏差，单位度，同样右、下为正；它不是机械 yaw/pitch 角度，也不是地理坐标偏差。当前相机为 60° 水平视场、方形像素，焦距 `f=W/(2*tan(60°/2))`；两分量为 `atan2(offset_px[i],f)*180/pi`，与控制器使用同一相机射线定义。`stability.center_err_deg` 保持原有无符号合成角误差定义。

有效样例（1920×1080 图像）：

```json
{
  "valid": true,
  "center_px": [980, 550],
  "offset_px": [20, 10],
  "offset_deg": [0.6891, 0.3446],
  "frame_seq": 1234
}
```

上述对象位于 `target.miss`。`frame_seq` 标识产生该测量的图像帧；没有新图像且未到断帧超时时保留原测量和帧号。跟踪初始化、失败、TEMP_LOST、LOST、停止或断帧超时后，`valid=false`，`center_px`、`offset_px`、`offset_deg`、`frame_seq` 均为 `null`，不把最后一次误差标记为当前测量。只有目标检测而未交接单目标跟踪时没有有效脱靶量。插件 Gazebo 遥测与网关 station 遥测均包含该字段。

Gazebo 专用话题为 `/two_axis_pod/command`、`/two_axis_pod/telemetry` 和 `/two_axis_pod/command_result`，消息均为 `gz.msgs.StringMsg` 中的 JSON。插件遥测与网关 hello 顶层 `telemetry_hz` 表示配置目标，`telemetry_clock:"steady"` 表示以单调真实时间计时；网关须确认插件频率及时基一致才进行周期转发。`ts_mono` 仍是插件采样的仿真时间，实际 Hz 须用接收或发送端单调时钟计算。插件执行结果携带 `cmd`、`ok`、`message` 与网关生成的 `request_id`。

组合模型可以在插件 SDF 中配置上述话题、`image_topic`、`tracking_backend` 及重复的 `sim_truth_target_name`，并按载机与目标配置 `los_observer_cg_model`、`los_target_name`。网关的 `--topic-prefix` 与 `--image-topic` 必须对应装配配置；未指定时沿用独立组件的话题。`sim_truth` 使用与图像帧同步的目标和相机世界坐标；`csrt` 的平移、目标运动与模型误差由图像反馈修正。动态补偿不提供第三轴画面水平稳定。

## 地面站艾晟报文

Linux/Windows 地面站右上角提供独立的 IP、端口输入框和“发送艾晟报文/停止发送”按钮。此功能在地面站本机执行。每份来自已配置网关、已确认飞机身份的协议 8 `telemetry` 转换为一个 UTF-8 JSON 数据报；仅包含下表十九个字段，不包含飞机 ID、帧表、type 或外层封装。`hello`、`pong`、回执和指令不发送。

| 字段 | 类型 | 发送数值与来源 |
|---|---|---|
| `timestamp` | JSON整数 | 地面站发送线程在发送前生成的Unix毫秒时间戳，`time.time_ns() // 1_000_000`，起点1970-01-01 00:00:00 UTC；不使用仿真时间或源遥测时间 |
| `track_status` | uint8 数值 | 视觉模式为 `manual_track`，或已确认 `AUTO_LOCKED` 且有锁定 ID 的 `auto_lock`；同时 `mode=TRACK`、`tracking.state=TRACK`、`target.valid=true`、`target.miss.valid=true` 且无视觉错误时为 1，其他为 0 |
| `pitch_angle` | float32 数值 | `attitude_deg.pitch.pos`，朝前为负，不额外反号 |
| `yaw_angle` | float32 数值 | `attitude_deg.yaw.pos` |
| `pitch_misalignment` | float32 数值 | 有效时 `target.miss.offset_deg[1]`，向下为正；无效为 0 |
| `yaw_misalignment` | float32 数值 | 有效时 `target.miss.offset_deg[0]`，向右为正；无效为 0 |
| `tracking_confidence` | float32 数值 | 有效时 25.5，无效时 0.0，仅表示有效性，不是真实概率或连续质量分 |
| `target_confidence` | float32 数值 | 固定 0.8，不缩放，不随跟踪有效/丢失状态变化，不取自模型评分 |
| `gyro_pitch_rate` | float32 数值 | `clamp(attitude_deg.pitch.vel, −20, +20)`，机械内轴相对转速，单位 °/s |
| `gyro_yaw_rate` | float32 数值 | `clamp(attitude_deg.yaw.vel, −20, +20)`，机械外轴相对转速，左负右正，单位 °/s |
| `pitch_los_rate` | float32 数值 | 有效时 `clamp(target_los.pitch_rate_dps, −20, +20)`，目标惯性视线仰角变化率，向上正；无效时 0.0 |
| `yaw_los_rate` | float32 数值 | 有效时 `clamp(target_los.yaw_rate_dps, −20, +20)`，目标惯性视线方位角变化率，向右正；无效时 0.0 |
| `los_omega_n` | float32 数值 | `omega_valid=true`时取`omega_ned_dps[0]`，固定NED的北轴角速度，°/s、不限幅；无效时0.0 |
| `los_omega_e` | float32 数值 | 东轴角速度，取`omega_ned_dps[1]`，°/s、不限幅；无效时0.0 |
| `los_omega_d` | float32 数值 | 地轴角速度，取`omega_ned_dps[2]`，°/s、不限幅；无效时0.0 |
| `gate_x` | int16 数值（JSON整数） | 有效跟踪时取`floor(target.box_px[0]+target.box_px[2]/2+0.5)`，原图左上原点、向右正，单位px；无效时整数0 |
| `gate_y` | int16 数值（JSON整数） | 有效跟踪时取`floor(target.box_px[1]+target.box_px[3]/2+0.5)`，向下正，单位px；无效时整数0 |
| `tracking_gate_w` | float32 数值 | 有效跟踪时为最终 `target.box_px[2]`，单位 px；无效时为 0.0 |
| `tracking_gate_h` | float32 数值 | 有效跟踪时为最终 `target.box_px[3]`，单位 px；无效时为 0.0 |

`timestamp`在测量字段转换后、序列化和`sendto`之前生成，表示地面站尝试发送该包时的系统时间，单位ms；与地面站入队时刻、Gazebo物理采样时间不同。每包独立生成，毫秒内多包可同值。PX4的uORB `timestamp`仍为接收端本地`hrt_absolute_time()`微秒，不被JSON时间戳覆盖。

JSON 中发送数值而非二进制结构体；框中心按上述规则取整，其余角度等浮点字段不进行 int16 量化；角度、脱靶角直接使用度，角速度直接使用度/秒，不进行倍率换算。丢失或未跟踪时仍发送当前机械角及机械角速度，脱靶角、`tracking_confidence`、`tracking_gate_w` 和 `tracking_gate_h` 置零，`target_confidence` 始终为 0.8。有效跟踪时门宽高取最终显式框经画面边界裁剪后的实际像素尺寸。`gyro_*_rate` 承载原机械关节相对转速，按接收端字段命名映射，不是真实陀螺仪的绝对角速度测量。`*_los_rate` 承载目标相对无人机固定名义重心的 NED 方位角/仰角变化率，不是镜头光轴角速度。原有四个速率字段均在有限值和float32范围校验后对称限幅到−20～+20°/s；三维`los_omega_n/e/d`仅校验有限值及float32范围，不裁剪，向量无效时发送0/0/0；内部原始测量不裁剪。明确 `target_los.valid=false` 且两个内部速率为 `null` 时，外部两个 LOS 字段发送 0.0，机械 gyro 及其它字段继续发送；0/0 也可能是真实零变化，外部报文不另加有效标记，内部遥测保留独立有效标记。`track_status` 仍表示原跟踪有效性，不代替 LOS 数学有效性。缺少整个 `target_los`、坐标参考不符或对象畸形属于转换错误，不默认伪造为无效零值。非有限值、缺少有效机械角、有效跟踪却缺少有效像素框或角脱靶量、数值超出 float32 范围或协议不符时丢弃该包并显示转换错误，不发送 NaN/Infinity 或旧测量替代值。

例如最终框为`[935,520,50,40] px`、中心`(960,540) px`，pitch −56.37°、yaw 12.34°、垂直/水平脱靶 +0.5°/−0.25°、机械俯仰/偏航角速度 −1.23°/s、+2.34°/s，目标仰角/方位角变化率+0.6°/s、−0.4°/s，目标位于正北水平面时三维向量为(0,+0.6,−0.4)°/s：

```json
{"timestamp":1792999999123,"track_status":1,"pitch_angle":-56.37,"yaw_angle":12.34,"pitch_misalignment":0.5,"yaw_misalignment":-0.25,"tracking_confidence":25.5,"target_confidence":0.8,"gyro_pitch_rate":-1.23,"gyro_yaw_rate":2.34,"pitch_los_rate":0.6,"yaw_los_rate":-0.4,"los_omega_n":0.0,"los_omega_e":0.6,"los_omega_d":-0.4,"tracking_gate_w":50.0,"tracking_gate_h":40.0,"gate_x":960,"gate_y":540}
```

关闭视觉、仅检测、自动搜索/交接、未选框、初始化、临时/最终丢失和视觉错误均发送 `track_status=0`、`tracking_confidence=0.0`。即使插件暂时残留 TRACK 标志，也不能绕过视觉模式判定。网关在 off/detect 下发现插件仍处于 TRACK 时，每 0.25 秒最多发布一次停止，直到插件遥测实际退出 TRACK；不伪造插件停车状态，不干扰 MANUAL/HOME。显式再次关闭视觉仍会发布停止指令。

发送频率跟随收到的 station 遥测包，默认约 50 Hz。发送线程使用有界队列和非阻塞 socket，过载或发送失败时丢弃对应包，不阻塞图像/控制，不重发积压旧状态；没有新遥测包就没有新副本。停止或退出地面站后停止发送。接收端无需登记或 ping；UDP 不保证到达。接收 IP/端口可直接单击编辑；发送中进入编辑会先暂停并清空待发队列，避免输入未完成的地址生效。修改完成后须点击“发送艾晟报文”才使用新地址并保存到 INI `[forward]`；Enter/Esc 不自动恢复发送，每次启动默认关闭发送。

网关在 `hello` 和遥测中附加 `aircraft_id`（非负整数，默认 1）与 `video_rtsp_path`（例如 `/two_axis_pod_1`）。`hello.capabilities` 包含 `vision_mode`、`server_video_overlay`、`auto_lock` 和 `frame_exact_track`。同机多路共用 RTSP 8555，按路径末尾的飞机 ID 区分；ID 不改变机械角度语义，UDP 控制仍使用各机独立端口。详见[多机视频配置](../controller/service/README.md#多机共享-rtsp)。

地面站按配置的 `aircraft_id` 核对当前网关 `hello.aircraft_id`，必须是相同的非负整数。尚未确认、缺失/非法 ID、ID 不符或连接超时时禁止所有控制（含停止、零速度、回零、跟踪和清故障），仅允许 ping、status、bye。身份不符时清除待重传控制，界面和日志显示期望 ID 与网关 ID；连接超时后仅收到 pong 或遥测不会恢复控制，须重新收到匹配 hello。GUI 与 `--selftest` 均使用相同校验，自检身份未确认时返回非零退出码。旧网关缺少 ID 时须升级网关，不能跳过校验。

`gate_x/gate_y`使用最终显式框中心，按原始未做六位小数舍入的框坐标和尺寸计算后四舍五入，不能使用Python银行家舍入规则。坐标相对于原始视频帧（当前1920×1080），不随地面站窗口缩放；也不是以画面中心为原点的脱靶量或未裁剪实体投影中心。跟踪有效条件与`track_status`一致，无效时0/0；零值本身不单独表示无效。PX4 JSON接收映射到已有int16 `gate_x/gate_y`，仅接受0..32767的整数字面量，旧报文缺整对时按0/0兼容，残缺或非法字段拒绝。
