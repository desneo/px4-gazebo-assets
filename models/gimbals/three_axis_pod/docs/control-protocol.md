# 三轴吊舱控制指令协议

本文对应当前 `controller/service/gimbal_bridge.py`、`controller/src/gimbal_controller.cc` 和 `controller/src/control.cc`。接口使用 UDP JSON 字符串，协议版本 2、`angle_convention=px4_frd`；采用 PX4 对外角度语义，但不是 MAVLink 接口。

## 公共角度与机械诊断

公共姿态使用前/右/下（FRD）坐标约定，参考前向由 SDF 插件参数 `forward_az_deg` 指定的 Gazebo 世界水平前向定义：yaw 0° 朝前、向右为正；pitch 0° 水平、抬头为正；roll 0° 画面水平、沿镜头朝前看顺时针为正。roll 表示相机横轴转动，实际图像地平线的转动与其相反。`forward_az_deg=0` 对应世界 `+X`，向世界 `+Y` 增加；不是北为 0° 的地理航向。该静态参考不随载机运动更新，不提供载机动态姿态补偿。

`manual.yaw/pitch/roll` 输入上述公共 Euler 角速度；`reset/reset_axis` 设置公共姿态目标。遥测 `attitude_deg` 提供公共姿态位置、速度和加速度。`joints_deg`、`cmd_dps` 保留机械关节诊断定义，不是公共姿态或公共控制速度；不能混用。

接收端应检查 `hello.version=2` 和 `angle_convention=px4_frd`，以及遥测的对应标记。机械关节全零不等于公共三轴 0°；公共水平朝前/画面水平需通过安装外参、关节运动学与真实 sensor pose 换算。

公共遥测字段如下（示例仅摘录相关字段，不是完整遥测）：

```json
{"angle_convention":"px4_frd","attitude_deg":{"singular":false,"yaw":{"pos":0,"vel":0,"acc":0},"pitch":{"pos":0,"vel":0,"acc":0},"roll":{"pos":0,"vel":0,"acc":0}},"attitude_quat_frd":[1,0,0,0]}
```

`pos/vel/acc` 单位依次为 `°`、`°/s`、`°/s²`；yaw/roll 范围为 [−180°,180°]，pitch 为 [−90°,90°]。非奇异时角速度由实际机械速度通过完整运动学换算，加速度由连续遥测的公共角速度按仿真时间差分；刚离开奇异区的首个有效样本加速度为 0。`attitude_quat_frd` 顺序为 `[w,x,y,z]`，表示静态参考 FRD 系下的相机 FRD 姿态，不受 Euler 分解奇异影响。

`camera.quat_body/quat_world` 与 `camera.rpy_body_deg/rpy_world_deg` 表示 Gazebo 光学传感器相对安装系/世界系的姿态，不是公共 FRD Euler 字段。集成控制和界面显示应使用 `attitude_deg`；完整公共姿态使用 `attitude_quat_frd`。机械 yaw 的 `joints_deg.yaw.pos=WrapPi(pos_raw−180°)` 仅是诊断显示偏移，pitch/roll 的诊断位置仍为机械角度。

当 `|cos(pitch)|<sin(1°)`，即离正天/正地不足约 1°时，`attitude_deg.singular=true`。此时 Euler `pos` 采用确定的分解分支，yaw/roll 不能作为独立方向解释，三轴 `vel/acc` 均为 `null`，应使用 `attitude_quat_frd` 表示完整姿态。非零 yaw/roll 手动请求被拒绝并停止，pitch 可离开奇异区；`reset_axis` 的 yaw/roll 请求拒绝，pitch 可用于回到水平。

## 传输与会话

- 默认服务端地址为 `<吊舱主机 IP>:8765`，可用网关参数 `--udp-host/--udp-port` 调整。
- 一个 UDP 数据报只传一个 UTF-8 编码的 JSON 对象字符串，最大 8192 字节。直接发送 `{...}` 的字符内容，不要再次序列化成带转义引号的 JSON 字符串，例如不要发送 `"{\"cmd\":\"stop\"}"`。
- `cmd` 区分大小写。`seq` 建议始终提供非负整数，按同一个客户端的本地 IP/端口持续递增；不要用布尔值或数字字符串。有限的整数浮点值也会被网关接受，但集成程序应发送整数。
- 首条已识别命令会登记客户端，并先发 `hello`；随后从相同服务端地址接收回执、心跳和约 50 Hz 遥测，接收端必须按 `type` 分流，不能假定收到的第一条就是本次命令结果。
- 默认 5 秒未收到该客户端请求即注销。建议每秒发送一次新 `seq` 的 `ping`。发送端须持续使用同一个 UDP socket，回复会返回该 socket 的本地端口。
- 客户端地址是会话身份，没有用户认证或独占操作权；多个客户端都能控制同一吊舱。
- RTSP 视频独立传输，默认 `rtsp://<吊舱主机 IP>:8554/gimbal`。UDP 控制报文不包含视频。

首次登记时的响应示例（时钟域为运行主机的实际 Linux boot_id，无法读取时为 `null`）：

```json
{"type":"hello","proto":"gimbal-udp","version":2,"angle_convention":"px4_frd","clock_domain":"11111111-2222-3333-4444-555555555555","telemetry_hz":50,"video_rtsp_path":"/gimbal"}
```

以下每个 `json` 代码块均为可直接发送的数据报字符内容。示例 `seq` 仅用于展示；实际发送须使用会话内的新序号。

## 全部 10 条指令

### 1. `ping`：登记与心跳

```json
{"cmd":"ping","seq":1,"t":1720000000.125}
```

`t` 可省略，也可为客户端自定义 JSON 值；网关原样回显，不负责解析时间、校时或换算单位。它不会续期手动转动，仅续期网络会话。响应：

```json
{"type":"pong","t":1720000000.125,"seq":1}
```

### 2. `status`：读取最近遥测

```json
{"cmd":"status","seq":2}
```

立即返回最近的 `type=telemetry` 完整遥测，并增加本次 `seq`。它不触发新的物理测量、不改变控制模式，也不续期手动转动。尚无插件遥测时返回：

```json
{"type":"error","message":"暂无遥测","seq":2}
```

普通定时广播遥测没有请求 `seq`。遥测字段和时间域见[系统说明](system.md#指令与网络接口)。

### 3. `track`：选框跟踪

```json
{"cmd":"track","seq":3,"box":[0.4,0.35,0.2,0.2]}
```

`box=[x,y,w,h]` 分别为目标框左上角横坐标、纵坐标、宽、高，均相对完整视频图像归一化，原点在图像左上角，横向向右、纵向向下。要求四项为有限数值、`x>=0`、`y>=0`、`w>0`、`h>0`、`x+w<=1`、`y+h<=1`。选框不受地面站窗口缩放、黑边影响；发送前须转换回完整图像坐标。

示例在 1920×1080 图像中对应左上角约 `(768,378)`、大小约 `384×216` 像素。初始化还要求裁剪后宽高至少 8 像素，灰度标准差至少 6，并存在未超时的视频帧（新帧到达间隔小于 0.5 秒）。

成功回执表示初始化调用成功，模式进入 `TRACK`、跟踪状态先为 `INIT`；应继续观察 `tracking.state=TRACK` 和 `target.valid=true` 判断已建立有效跟踪。目标会驱动 yaw/pitch，roll 不执行视觉补偿。新选框会替换原目标。成功回执：

```json
{"type":"command_result","cmd":"track","ok":true,"message":null,"seq":3}
```

典型失败回执（纹理不足）：

```json
{"type":"command_result","cmd":"track","ok":false,"message":"选框内纹理不足（可能对准天空），请重新框选","seq":3}
```

格式/范围/新帧检查失败会拒绝执行；进入跟踪初始化后失败则使跟踪状态进入 `FAULT`、模式进入 `IDLE`。恢复可重新选框，或使用 `reset_fault` 清理故障。

### 4. `manual`：三轴公共 Euler 角速度

```json
{"cmd":"manual","seq":4,"yaw":20,"pitch":-10,"roll":5}
```

`yaw/pitch/roll` 为公共姿态的 Euler 目标角速度，单位 `°/s`；不是目标位置角，也不是机械关节速度。各字段可省略，省略项为 0。仅接受有限数值，不接受字符串、布尔值、`NaN` 或 `Infinity`。命令切换到 `MANUAL` 并取消原跟踪/归位任务；插件根据当前姿态进行三轴耦合换算，再施加机械关节速度、加速度和限位保护。因此实际公共角速度可能因机械约束而低于请求值。

例如 `yaw=20` 表示向参考右方改变公共 yaw，`pitch=-10` 表示低头，`roll=5` 表示沿镜头朝前看顺时针滚转。两种正式安装使用相同公共符号约定，客户端不应按安装方式再次取反。

**必须用新 `seq` 持续发送，建议 10～20 Hz。** 距最近一次被插件执行的 `manual` 达到 0.5 秒后，在下一个控制周期将速度指令归零并转为 `IDLE`。只发 `ping`、重传同一个已经执行的 `manual` 序号均不能续期手动转动。零速度停止示例：

```json
{"cmd":"manual","seq":5,"yaw":0,"pitch":0,"roll":0}
```

零速度命令仍处于 `MANUAL`，正常按加速度限制减速；若希望退出手动模式，发送 `stop`。正常执行无成功回执；网关参数校验、过时序号、发布失败及基于最近遥测的奇异预检查失败会返回带 `seq` 的失败。`manual` 不登记插件回执等待关联，因此插件端稍后产生的拒绝结果不保证通过 UDP 返回；不能把“未收到失败”当作命令已执行。以 `mode`、`attitude_deg.*.vel` 观察公共执行情况，运行中转换停转原因可查看 `tracking.message`，机械保护诊断查看 `cmd_dps/joints_deg/limits`。

天顶/天底附近公共 Euler 角奇异，非零 yaw 或 roll 请求会被拒绝并停止当前手动运动，pitch 请求可用于离开该区域；不能把天顶遥测的某个 yaw/roll 数值当作独立可控方向。网关与插件分别检查奇异，网络延迟期间姿态可能变化，集成端仍需持续观察实际遥测。全零停止请求仍可发送，详见上文奇异区说明。

### 5. `stop`：退出控制任务并减速

```json
{"cmd":"stop","seq":6}
```

进入 `IDLE`，取消跟踪框、归位及手动目标，清理跟踪故障和滤波信息；速度指令按正常加速度限制降到 0。没有位置保持闭环，也不是物理瞬停。成功回执：

```json
{"type":"command_result","cmd":"stop","ok":true,"message":null,"seq":6}
```

### 6. `home`：朝世界天空方向

```json
{"cmd":"home","seq":7}
```

取消原任务，按加载时静态安装外参选择朝世界 `+Z` 的可达关节目标，模式进入 `HOME`；到位后进入 `IDLE`。它不是“所有机械关节回零”。`skyward` 立装可到正天，`downward` 悬挂受当前机械限位约束，最高仰角约 44°。成功回执仅表示接受请求：

```json
{"type":"command_result","cmd":"home","ok":true,"message":null,"seq":7}
```

实际到位需观察相机姿态、关节角速度和模式，不能用回执代替到位确认。

### 7. `reset`：世界水平前向复位

```json
{"cmd":"reset","seq":8}
```

停止原任务、清理跟踪故障，然后回公共 `yaw=0,pitch=0,roll=0`，即朝静态参考前向水平观察且画面水平，进入 `HOME`，到位后进入 `IDLE`。前向由模型插件配置 `forward_az_deg` 指定，默认 0°，即 Gazebo 世界 `+X`。网络报文不能设置 `forward_az_deg`。若当前安装和机械限位不能达到该公共姿态，返回失败，不能将回执解释为已完成复位。

```json
{"type":"command_result","cmd":"reset","ok":true,"message":null,"seq":8}
```

回执仅确认接受，实际到位判断同 `home`。

### 8. `reset_axis`：指定公共姿态轴回零

偏航轴回零：

```json
{"cmd":"reset_axis","seq":9,"axis":"yaw"}
```

俯仰轴回零：

```json
{"cmd":"reset_axis","seq":10,"axis":"pitch"}
```

滚转轴回零：

```json
{"cmd":"reset_axis","seq":11,"axis":"roll"}
```

`axis` 为必填字符串，仅接受小写 `yaw/pitch/roll`。可达检查通过后取消原跟踪、手动或归位任务，清理跟踪故障和选框，模式进入 `HOME`，指定轴回 `attitude_deg.<axis>.pos=0`，其他两轴保持接收请求时最新采样对应的公共姿态角；各轴误差均小于 0.5°后进入 `IDLE`。其他轴在执行期间受闭环驱动，不是简单输出零速度；到位后不再保持位置闭环。

yaw 回零是朝静态参考前向的公共方位角，pitch 回零是水平观察，roll 回零是画面水平。插件求解完整公共目标对应的可达机械姿态，其他机械关节可能同时运动；不能按 `joints_deg` 是否归零判断成功，也不能把公共单轴复位当作机械单轴运动。若公共目标不可达则拒绝执行。

成功回执仅表示接受请求，实际到位需检查对应显示角、关节速度和模式；回执关联方式与 `home/reset` 相同。

```json
{"type":"command_result","cmd":"reset_axis","ok":true,"message":null,"seq":9}
```

`axis` 缺失或非法会拒绝执行；尚无关节采样、公共目标或保持路径不可达、奇异区不允许的 yaw/roll 复位也会拒绝，并保留原任务。可达检查包含目标端点和公共姿态插值路径。回执不包含 `axis`，客户端使用 `seq` 关联原请求，例如非法参数回执：

```json
{"type":"command_result","cmd":"reset_axis","ok":false,"message":"axis 必须为 yaw、pitch 或 roll","seq":15}
```

姿态不可执行时的失败说明为“公共轴零姿态或保持路径不可达，或yaw/roll处于Euler奇异区”。`reset` 不可达时的失败说明为“公共水平前向零姿态超出机械安全范围，请检查安装与光学标定”。

### 9. `reset_fault`：清跟踪故障

```json
{"cmd":"reset_fault","seq":12}
```

仅在跟踪状态为 `FAULT` 时执行 `Stop()`，转为 `IDLE` 并清理故障；其他状态下不改变任务，不回零、不重新建框。正常无成功回执，失败处理同 `manual`。可通过 `status` 或广播遥测检查 `tracking.fault` 与 `tracking.state`。

### 10. `bye`：注销客户端

```json
{"cmd":"bye","seq":13}
```

注销当前 UDP 地址，不再向它广播遥测，正常无回执。它不会停止吊舱任务；需要停止时，应先发送 `stop` 并确认。注销会取消该会话等待中的回执关联，短期保留去重记录。后续从相同地址发送已识别命令会再次登记。

## 序号、重传和错误处理

- `track/stop/home/reset/reset_axis` 需要等待 `command_result`。当前地面站每 0.2 秒重传同一个请求，最多发送 5 次；自行集成可采用相同策略。重传必须保持同一个 `seq` 和相同 JSON 内容，不能生成新序号后当作同一次重传。
- 对 `status` 和七条插件控制指令，网关以客户端地址与 `seq` 去重；对象字段顺序不影响比较，但字段值及有无字段影响比较。相同请求不重复执行，已有结果会重发；仍等待插件时不会马上生成新回执。`ping/bye` 在去重逻辑之前处理，不进入该结果缓存。
- 去重窗口按已接收最大序号保留最近 1024 个整数序号范围，不是“最近 1024 条发送次数”。同序号改报文返回“seq 已用于其他请求”，超出窗口返回“seq 已超出去重窗口，未执行”。低于最近控制序号的未见过控制命令返回“过时指令，未执行”；`status/ping` 不推进控制序号。
- `seq` 不提供时仍可执行，回执中为 `null`，但无法保证去重或阻止旧控制覆盖新控制。不要在集成中省略它。
- `manual/reset_fault` 不等待成功结果，不占等待队列。五条有回执控制指令最多等待 4096 条；30 秒未收到插件结果返回失败并说明“执行结果未知”，不自动再次执行。
- 会话过期或 `bye` 后，去重记录默认短期保留约 60 秒；复用同一本地地址应延续序号，新的客户端可使用新的本地端口。网关重启或缓存保留期结束后不保证旧请求去重。结果未知时应先查询实际遥测，不能直接认为“指令未执行”。
- 已识别指令可能先收到 `hello`，再收到参数错误；非法 JSON、非法 UTF-8 或未知命令不会登记新会话。空数据报被忽略，超过 8192 字节的数据报直接丢弃。

失败回执示例（公共角速度不是数字）：

```json
{"type":"command_result","cmd":"manual","seq":14,"ok":false,"message":"参数必须是数值"}
```

格式错误示例响应：

```json
{"type":"error","message":"无效 JSON","seq":null}
```

`message` 用于说明原因，客户端应优先按 `type/cmd/seq/ok` 判断；不同错误阶段的说明文本可能不同。网关内部的 `request_id` 用于关联插件结果，不需要客户端设置，也不会作为 UDP 回执字段返回。额外的未定义字段不会作为控制参数转发，但会影响同序号内容比较。

## 发送 JSON 字符串的完整示例

以下 Python 示例只依赖标准库，保持一个 socket，发送显式 JSON 字符串；先转动 1 秒，再发 `stop`，接收并区分广播与回执。将 `HOST` 改为网关主机地址即可；该示例会实际控制公共 yaw 角，运行前应处于非奇异且可达的姿态。

```python
import json
import socket
import time

HOST = "127.0.0.1"
PORT = 8765
server = (HOST, PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 0))
sock.settimeout(0.1)

def send_json_string(payload):
    sock.sendto(payload.encode("utf-8"), server)

send_json_string('{"cmd":"ping","seq":1,"t":0}')
deadline = time.monotonic() + 2.0
compatible = False
while time.monotonic() < deadline:
    try:
        data, source = sock.recvfrom(65535)
    except socket.timeout:
        continue
    if source != server or len(data) > 8192:
        continue
    msg = json.loads(data.decode("utf-8"))
    if msg.get("type") == "hello":
        compatible = (msg.get("version") == 2
                      and msg.get("angle_convention") == "px4_frd")
        break
if not compatible:
    sock.close()
    raise RuntimeError("未确认协议版本 2 与 px4_frd，停止发送控制指令")

seq = 2
started = time.monotonic()
while time.monotonic() - started < 1.0:
    # 每次续期使用新序号；公共 yaw 正向表示向参考右方转动。
    send_json_string('{"cmd":"manual","seq":%d,"yaw":10,"pitch":0,"roll":0}' % seq)
    seq += 1
    time.sleep(0.1)

stop_payload = '{"cmd":"stop","seq":%d}' % seq
received = False
for attempt in range(5):
    send_json_string(stop_payload)  # 重传保持同一个序号和报文内容。
    deadline = time.monotonic() + 0.2
    while time.monotonic() < deadline:
        try:
            data, source = sock.recvfrom(65535)
        except socket.timeout:
            continue
        if source != server or len(data) > 8192:
            continue
        msg = json.loads(data.decode("utf-8"))
        if (msg.get("type") == "command_result"
                and msg.get("cmd") == "stop" and msg.get("seq") == seq):
            print(msg)
            received = True
            break
    if received:
        break
if not received:
    print("未收到停止回执，请查看实际遥测确认执行状态")
send_json_string('{"cmd":"bye","seq":%d}' % (seq + 1))
sock.close()
```

真实客户端应持续接收遥测，而不是等操作完成才读接收队列；持续手动控制建议使用独立定时发送与接收循环。若需要将对象转换成 JSON 字符串，可用 `json.dumps(obj, ensure_ascii=False, allow_nan=False)`，再执行一次 UTF-8 编码发送。

## 直接接入 Gazebo 的区别

插件订阅 `/gimbal/command`，消息类型为 `gz.msgs.StringMsg`，其 `data` 字段保存 JSON 字符串。插件只执行 `track/stop/home/reset/reset_axis/manual/reset_fault`；`ping` 在插件端不产生响应，`status/bye` 属于 UDP 网关，不支持直接调用插件。

直接调用插件没有 UDP 的客户端登记、`seq` 去重、控制顺序保护或心跳响应。插件结果发布于 `/gimbal/command_result`；如需关联多个请求，可自行提供字符串 `request_id`，插件原样回传。`manual/reset_fault` 正常仍无回执。UDP 集成程序应接网关，不能混用两层的响应规则。
