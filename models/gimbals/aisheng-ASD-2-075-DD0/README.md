# ASD-2-075-DD0 两轴导引头仿真组件

本目录提供 ASD-2-075-DD0 两轴导引头的 Gazebo 模型、标准二进制 UDP 网关、单路 RTSP 视频和配套上位机。当前已实现：

- 复制 `two_axis_pod` 的壳体、球头和两轴机械结构，原 `two_axis_pod` 不动；
- 内部机械零位朝上、整机绕 Y 轴 +90° 安装后的前视两轴运动；
- 每轴 60°/s、yaw ±60°、pitch −80°～+30° 的运动范围和限位；
- 广角 38°×22°、长焦 12.7°×7° 双路 1920×1080 相机；同一时刻只订阅、编码和显示当前镜头；
- 标准 45 字节控制帧 / 104 字节遥测帧，UDP 100 Hz；
- 地面站从标准遥测解析全部 37 个转发字段，供专用 `aisheng_udp_bridge` 发布 `dyt_aisheng_telemetry`；
- 上位机单路 RTSP 视频、遥测显示、双击点选/最近目标锁定、自动锁定、取消锁定、电锁/解锁、归中、单轴归零、输入框指定框架角、艾晟报文转发和手动转动。

当前不实现目标检测/识别、视频 SEI、红外、激光测距、地理跟踪、OSD、存储、除雾、图像增强、IP 设置和陀螺校准；演示目标飞机默认静态，可用 `tests/move_target.py` 或 `run_demo.sh --move` 运动；数字变焦已在上位机显示端实现。

## 外观图

![零位与转动对比](docs/preview.png)

左侧为内部机械 0/0（整机安装后朝载机前方）；右侧为发送 yaw 正、pitch 负速度指令后的实际转动。

![广角画面](docs/wide.png)

![长焦画面](docs/tele.png)

![广角 4x 数字变焦](docs/zoom4.png)

![上位机广角跟踪画面](docs/gcs_tracking.png)

上位机正在用广角跟踪 `target_aircraft`：绿色框为跟踪框，目标已居中，左上角显示标准遥测转换后的框架角、角速度、失调角、波门和接收速率。

![上位机长焦跟踪画面](docs/gcs_tele_tracking.png)

上位机切换到长焦后的原生跟踪画面：`LENS TELE`、目标居中、跟踪框约 90×70 px。

![上位机操作界面](docs/gcs_ui.png)

视频下方为命令按钮和 yaw/pitch 虚拟摇杆；左上角显示中文遥测、脱靶量和波门状态。界面图由真实仿真视频帧与遥测离线重绘，速率显示为离线初始值。

## 模型来源与边界

- 壳体、球头、镜框和两轴结构来自 `models/gimbals/two_axis_pod` 的当前网格；本目录复制后修改，**原 `two_axis_pod` 完全不动**。
- 首版沿用两轴包络：约 Ø84 mm×91 mm、0.24 kg；产品规格为 Ø75 mm×84.5 mm、≤230 g，外观和包络差异已记录。
- 产品为后部法兰、前部球罩双镜头；本组件沿用两轴吊舱的圆环底座，并在原镜头下方增加了一个纯视觉长焦镜圈（`tele_lens_barrel_visual` / `tele_lens_window_visual`）。长焦相机传感器与第二镜圈对齐；镜圈无碰撞、无质量，不改变机械结构。第二镜圈的位置、尺寸和间距已确认。

## 坐标、零位与安装

内部机械约定沿用 `two_axis_pod`：

```text
R_mount_gimbal = Rx(yaw) Ry(-pitch)
los_mount = (-sin(pitch), -sin(yaw)cos(pitch), cos(yaw)cos(pitch))
```

- pod 内部零位：光轴沿本地 +Z，即吊舱自身“朝上”。
- pod 内部 pitch 负值转向本地 +X；pitch=-90° 时沿本地 +X。
- yaw 范围 ±60°，pitch 范围 −80°～+30°，每轴限速 60°/s。
- 组件整机绕 Y 轴 +90° 安装：pod 本地 +Z 对准载机 +X。因此**载机坐标中的前视对应内部机械 pitch=0°**；内部 pitch 正值朝载机上方，负值朝载机下方。产品框架角（前视 0/0、pitch 上正下负、yaw 右正左负）与内部机械角同号，协议直接使用。
- 这样安装后 yaw 轴在载机坐标中变为垂直轴，前视时 yaw 可以正常左右转；不会出现“内部 pitch=-90° 时光轴与 yaw 轴重合”的奇异。

## 双路相机

- `camera`：广角，1920×1080@30 Hz，水平视场 38°，垂直约 22°，位于原镜头位置。
- `camera_tele`：长焦，1920×1080@30 Hz，水平视场 12.7°，垂直约 7°，位于原镜头下方的第二镜圈位置，光轴与广角平行。
- 两路传感器都在 `camera_link` 上；控制器只订阅当前镜头图像，网关只编码当前镜头，经本地 RTP 送入 RTSP 服务；上位机通过固定 RTSP URL 用 NVDEC 解码显示。
- 图像话题：`/aisheng_seeker/camera`、`/aisheng_seeker/camera_tele`；外部观察相机话题 `/aisheng_seeker/overview`。
- 上位机可切换广角/长焦显示；网关同时通知插件切换活动相机。双击/自动锁定使用当前活动镜头的像素和真值候选，跟踪控制使用该镜头的内参和光轴，长焦可原生独立跟踪。
- 长焦锁定期间，1920×1080 原图跟踪框宽度严格大于 500 px **或**高度严格大于 300 px 时，控制器自动切换广角；等于阈值时保持长焦。切换时等待广角新帧并按焦距换算跟踪框：`sim_truth` 保留同一目标实体和深度基准，`csrt` 在重投影后的同一目标区域重建模板。网关同步遥测镜头位与同一 RTSP URL 的视频源，并把广角数字变焦恢复为 1×；上位机跟随镜头遥测重建视频接收。
- 变焦 +/- 更新遥测状态位；上位机按 1×/2×/4× 对广角画面做中心裁剪和缩放，RTSP 流本身仍为全视场。

## 标准二进制 UDP 与视频

网关 `gateway/main.py` 在 Gazebo 插件 JSON 话题和标准二进制 UDP 之间转换。上位机控制导引头使用《无人机到导引头控制协议》的 45 字节控制帧，经 UDP 发到网关；网关再转换为插件内部 JSON。上位机不直接写插件 JSON，遥测也按 104 字节标准帧回传。

| 项目 | 默认值 |
|---|---|
| 控制/遥测 UDP | 网关默认 `0.0.0.0:8001`；地面站一个临时 socket 双向收发 |
| RTSP URL | `rtsp://127.0.0.1:8554/aisheng_seeker`（广角/长焦共用） |
| 内部 RTP | `127.0.0.1:7001`（仅 RTSP 服务读取） |
| 地面站 JSON 转发 | 点击“发送艾晟报文”后发往指定 IP/端口，默认 `127.0.0.1:9000` |
| 控制帧 | 45 字节，帧头 `0xAA55`，长度 `0x2D` |
| 遥测帧 | 104 字节，帧头 `0x55AA`，长度 `0x68` |
| 遥测周期 | 100 Hz |
| 校验 | 字节 4 起求和取低 8 位 |

已实现控制命令：

- `0xE9` 框架电锁、`0x9E` 电锁解除；
- `0x90` 指定框架角度、`0x14` 指定角度并电锁、`0x91` 归中；
- `0xB7` 点选目标锁定（像素 x/y）；
- `0x5F` 自动锁定、`0xF5` 关闭自动锁定；
- `0x8E` 取消锁定；
- `0xB1` 框架移动（左右/上下 −100～+100，映射到 ±60°/s）；
- `0xB9/0x9B` 变焦状态；
- `0x53` 切换激活镜头（参数 1 广角、2 长焦，仿真映射）。

标准遥测中的陀螺角速度、视线角速度和失调角按协议字段范围限幅 ±20°/s；实际 60°/s 机械转速保留在 Gazebo 插件 JSON 遥测中，不塞进标准字段。

网关可用 `--image-topic`、`--tele-image-topic` 覆盖图像话题，默认 `<topic-prefix>/camera` 和 `<topic-prefix>/camera_tele`。网关只回传 104 字节标准遥测：校验合法控制/心跳报文后记录源 IP 和源端口，并从控制 socket 回传；2 s 无有效报文后停止回传。地面站使用同一个临时 socket 发出心跳并接收遥测，解析后供显示和 JSON 转发共用，字段与时间约定见[协议与接口](docs/protocol.md)。

自动锁定使用 Gazebo 插件发布的 `truth_candidates`（当前广角相机内已注册飞机投影中心），选择离画面中心最近的一架；点选锁定选择点击位置附近最近的候选。两种锁定都通过内部 `track` 框启动插件的 `sim_truth` 单目标跟踪，不使用 YOLO 或纹理跟踪。自动锁定在目标丢失后会重新选择当前最近中心飞机。

RTSP 使用 H.264，传输层为 RTP over UDP；RTSP 服务固定挂载 `/aisheng_seeker`，镜头切换时 URL 不变，内容切换到新镜头。接收端使用 `rtspsrc protocols=udp latency=0` 和 `nvh264dec`，面向局域网低延迟。完整帧表、命令参数和限制见[协议与接口](docs/protocol.md)。

## 运行

在仓库根目录执行：

```bash
# 首次或修改 controller 后构建
cmake -S models/gimbals/aisheng-ASD-2-075-DD0/controller \
  -B models/gimbals/aisheng-ASD-2-075-DD0/controller/build \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build models/gimbals/aisheng-ASD-2-075-DD0/controller/build -j4

# 完整演示：Gazebo GUI + 二进制网关 + 单路 RTSP + 上位机
bash models/gimbals/aisheng-ASD-2-075-DD0/tests/run_demo.sh

# 不打开 Gazebo GUI，只保留上位机视频/遥测
bash models/gimbals/aisheng-ASD-2-075-DD0/tests/run_demo.sh --headless

# 仅关闭地面站；仿真、网关与视频服务继续运行
bash models/gimbals/aisheng-ASD-2-075-DD0/ground_station/stop.sh

# 同时让 target_aircraft 按默认航线运动
bash models/gimbals/aisheng-ASD-2-075-DD0/tests/run_demo.sh --move

# 保存一张包含底部控制面板的上位机截图后继续运行
bash models/gimbals/aisheng-ASD-2-075-DD0/tests/run_demo.sh --headless \
  --snapshot /tmp/aisheng_gcs_ui.png

# 仅外观/转动预览，并自动做 30 s 两轴扫掠
bash models/gimbals/aisheng-ASD-2-075-DD0/tests/run_preview.sh

# 只看模型，不自动扫掠
bash models/gimbals/aisheng-ASD-2-075-DD0/tests/run_preview.sh --no-sweep

# 25°/s 视线角速度验收（自动启动动态目标世界和网关）
bash models/gimbals/aisheng-ASD-2-075-DD0/tests/run_los25.sh
```

`ground_station/config.ini` 的 `control_port=8001` 是地面站的控制目的端口，`run_demo.sh` 启动的网关跟随该值；地面站本地端口自动分配。`[forward]` 保存报文面板默认目的地址，也可用 `--forward-host`、`--forward-port` 指定，点击发送按钮后才转发。命令行参数优先于配置文件。02 场景按组合机型选择桥：`rocket_aisheng_seeker` 使用专用 `aisheng_udp_bridge`，`rocket_two_axis_pod` 使用自身 `dyt_udp_bridge`。

`ground_station/stop.sh` 仅匹配当前用户、本模型路径的 Python 地面站进程，支持 `--dry-run` 查看将停止的 PID。通过 `run_demo.sh` 启动时，关闭地面站后仿真、网关和视频服务继续运行；在原启动终端按 `Ctrl+C` 停止整套演示。

上位机界面参考两轴吊舱地面站：视频下方是命令按钮、yaw/pitch 输入框和虚拟摇杆，视频左上角显示中文遥测，右上角是艾晟报文转发面板。

- 底部按钮：`解锁`、`电锁`、`归中`、`取消锁定`、`自动锁定`、`关闭自动`、`发送指定角`、`广角`、`长焦`、`变倍+`、`变倍-`、`Yaw归零`、`Pitch归零`、`停止`；
- 底部虚拟摇杆：左负 yaw、右正 yaw，上正 pitch、下负 pitch，每轴最大 20°/s，按住保持转动，无需持续移动鼠标；松开鼠标立即停止；键盘 `W/A/S/D` 或方向键为 60°/s 点动；
- 双击视频画面：若当前电锁，先自动解锁；点击附近有目标时精确点选，附近没有目标时锁定画面内最近目标；
- 视频左上角分行显示四个角速度：陀螺仪俯仰/偏航角速度、俯仰/偏航视线角速度，单位 °/s，显示值按协议限幅 ±20°/s；
- 快捷键：`U` 解锁，`L` 电锁，`R`/`H` 归中，`C` 取消锁定，`O` 自动锁定，`P` 关闭自动锁定，`1` 广角，`2` 长焦，`Z` 变倍+，`Shift+Z` 变倍-，空格停止，`Q` 或 Esc 退出；
- 底部 `Yaw`/`Pitch` 输入框设置指定框架角，旁边显示限位 `−60.0～+60.0`、`−80.0～+30.0`；点击 `发送指定角` 才下发。输入框支持 `Enter` 发送、`Tab` 切换、`Esc` 取消。
- `Yaw归零`/`Pitch归零` 只把对应轴置零，另一轴保持当前机械角；电锁状态下会先自动解锁。
- 右上角“艾晟报文”面板：输入 IP/端口，点击 `发送艾晟报文`/`停止发送`；从收到的标准遥测解析 37 个字段，每个新采样最多转发一次、最高 50 Hz，超过 1 s 的旧样本不发送。`los_omega_n/e/d` 不在标准帧中，JSON 不发送这三项。

上位机视频显示 960×540（RTSP 源为 1920×1080），窗口下方为 300 px 控制面板。遥测使用固定标签/数值列，报文面板独占右上区域；按钮和输入框标签按字体包围盒对齐。锁定框、遥测和命令提示在独立图像副本上绘制，视频停帧时界面仍以 30 Hz 刷新。

无界面自检（需要网关和 Gazebo 已启动）：

```bash
.venv-linux/bin/python models/gimbals/aisheng-ASD-2-075-DD0/ground_station/main.py \
  --selftest 5
```

自检检查遥测频率、当前 RTSP 视频解码和解锁后的自由状态；低渲染性能机器会显示频率警告但仍按最低 40 Hz 判定。

上位机默认读取 `ground_station/config.ini`，命令行参数可覆盖。保存当前连接/视频/镜头配置：

```bash
.venv-linux/bin/python models/gimbals/aisheng-ASD-2-075-DD0/ground_station/main.py \
  --host 127.0.0.1 --control-port 8001 --save-config
```

启动脚本会打印 `GZ_PARTITION`。在另一个终端用同一分区手动发送指令或移动目标：

```bash
export GZ_PARTITION=<启动脚本打印的分区>

# 让 target_aircraft 按默认航线运动 30 s
.venv-linux/bin/python models/gimbals/aisheng-ASD-2-075-DD0/tests/move_target.py \
  --duration 30
```

手动速度/回零/停止：

```bash
export GZ_PARTITION=<启动脚本打印的分区>

# yaw 以 30°/s 转 2 s，pitch 0；松手自动停车
.venv-linux/bin/python models/gimbals/aisheng-ASD-2-075-DD0/tests/command.py \
  manual 30 0 --duration 2

# 两轴回机械零位（载机前视）
.venv-linux/bin/python models/gimbals/aisheng-ASD-2-075-DD0/tests/command.py home

# 立即停止
.venv-linux/bin/python models/gimbals/aisheng-ASD-2-075-DD0/tests/command.py stop
```

手动指令是**角速度**，单位 °/s；`yaw` 右正、`pitch` 上正。持续发送由 `command.py` 的 `--duration` 控制，插件在 0.5 s 无新指令时自动停车。

## 观察入口与定位标志

- 实体名：`AISHENG_SEEKER`。
- 定位标志：`AISHENG_SEEKER_LOCATION_MARKER`，位于模型左前方约 `(-0.12, -0.28, 1.55) m`，橙色箭头指向导引头，默认无碰撞体，便于在场景中快速定位。
- 浏览相机初始近裁剪距离 0.005 m，启动脚本会尝试把鼠标灵敏度设为 0.1。滚轮缩放参考鼠标指向的三维位置；近景检查请把鼠标放在模型实体表面，再用右键缓慢拖动微调，避免指向远处背景导致一步跳过小部件。
- 如果灵敏度服务未就绪，可在同一分区手动执行：

```bash
gz service --service /gui/camera/view_control/sensitivity \
  --reqtype gz.msgs.Double --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'data: 0.1'
```

## 演示世界

- `tests/preview.world.sdf` 由 `tools/generate_model.py` 生成，包含：
  - 固定在 1.46 m 台座上的导引头，整体绕 Y 轴 +90° 安装；
  - 前方 x=4 m 的 5×5 彩色靶墙和红色中心十字，用于观察广角/长焦视场和两轴转动；
  - 两架静态目标飞机 `target_aircraft`（橙白）和 `target_aircraft_2`（蓝白），位于靶墙前方，用于点选锁定和自动锁定最近中心飞机；
  - 橙色定位箭头 `AISHENG_SEEKER_LOCATION_MARKER`；
  - 外部观察相机 `/aisheng_seeker/overview`。
- 抓取一帧图像：

```bash
export GZ_PARTITION=<启动脚本打印的分区>
.venv-linux/bin/python models/gimbals/aisheng-ASD-2-075-DD0/tests/capture.py \
  --topic /aisheng_seeker/overview --output /tmp/aisheng_overview.png
```

## 文件

- `parameters.json`：包络、质量、机械限位、60°/s、广角/长焦相机参数、真值目标名和 LOS 观察点。
- `tools/generate_model.py`：从复制的 CAD 网格生成 `model.sdf` 和 `tests/preview.world.sdf`；修改参数后重新运行。
- `model.sdf`：生成的 Gazebo 模型，包含两轴、两个相机、真值目标配置和控制器。
- `controller/`：复制并改名的两轴控制器，库名 `gz-aisheng-seeker-controller`，机械速度硬上限 60°/s；遥测增加 `truth_candidates` 和图像宽高；原 `two_axis_pod` 控制器不变。
- `common/protocol.py`：标准 45/104 字节帧编解码、校验和、比例尺和限幅。
- `gateway/main.py`：二进制 UDP 网关、命令映射、点选/自动锁定、100 Hz 遥测。
- `gateway/video_worker.py`：单路相机独立编码进程；网关为广角/长焦各启动一个，避免两路 1080p 图像争用 Python GIL。
- `gateway/video_tx.py`：Gazebo 图像 → NVENC H.264 → 内部 RTP。
- `gateway/rtsp_server.py`：内部 RTP → 固定 RTSP URL（广角/长焦共用）。
- `ground_station/main.py`：单路 RTSP、中文遥测、双击锁定、自动锁定、命令按钮、虚拟摇杆、指定角输入框、艾晟报文转发和无界面自检。
- `ground_station/config.ini`：上位机网络、RTSP URL、JSON 遥测端口、启动镜头和镜头切换开关配置。
- `tests/run_demo.sh`：完整演示入口。
- `tests/run_preview.sh`：仅外观/转动预览入口。
- `tests/sweep.py`：自动两轴速度扫掠。
- `tests/command.py`：手动速度/回零/停止指令。
- `tests/capture.py`：保存 Gazebo 相机图像。
- `tests/move_target.py`：用 Gazebo `set_pose` 服务移动目标飞机，验证动态真值跟踪。
- `tests/check_binary.py`：二进制协议/运动回环检查。
- `tests/check_lock.py`：点选锁定/自动锁定/取消锁定/电锁回环检查。
- `tests/check_tele.py`：长焦原生点选锁定回环检查。
- `tests/check_limits.py`：两轴手动速度、限位和峰值速度回环检查。
- `tests/check_los25.py`：25°/s 视线角速度跟踪验收。
- `tests/run_los25.sh`：启动 `los25.world.sdf`、网关并执行 25°/s 验收。
- `tests/los25.world.sdf`：由生成器输出的动态目标世界，目标运动插件平滑移动 `target_aircraft`。
- `controller/src/target_motion.cc`：演示目标运动插件，按物理步设置目标位姿。
- `tests/test_protocol.py`：协议编解码单元测试。
- `tests/test_coordinates.py`：广角/长焦像素换算单元测试。
- `tests/test_gcs_ui.py`：按钮布局/命中、摇杆死区、命令映射、双击点选、单轴归零、解码帧隔离和叠加颜色测试。
- `tests/test_gcs_manual.py`：圆盘持续按住、松手停车、键盘点动超时和发送线程顺序测试。
- `tests/test_full_telemetry.py`：四目标标准帧、完整字段映射、同 socket 来源过滤、迟启动恢复和配置测试。
- `tests/test_udp_session.py`：网关同端口回传、合法对端更新和会话超时测试。
- `tests/test_gcs_lens_sync.py`：自动切广角的 UI 同步、手动切换确认和缺失遥测保护测试。
- `tests/test_lens_switch.py` / `tests/lens_switch_test.cc`：镜头阈值、真值锁定恢复、CSRT 模板重建、网关镜头确认和自动恢复 1× 测试。

## 当前验证

- 组件回归测试共 40 项通过，包含阈值边界、真值锁定恢复、CSRT 模板重建、镜头确认、自动恢复 1×、UI 绘制、圆盘持续控制、协议及 UDP 回环。
- 真实 Gazebo 回环中，圆盘持续按住 0.8 s 后仍以约 19.8°/s 转动，松开后角度保持；目标接近触发长焦转广角后仍绑定同一实体并恢复 TRACK，数字变焦从 4× 恢复 1×，已连接 RTSP 接收器重建后收到广角新帧。

- 控制器已构建，演示世界启动后插件发布 `/aisheng_seeker/telemetry`，遥测中 `mount.quat_world` 为绕 Y 轴 +90°，`los_mount` 在模型内部为 +Z，外部相机沿载机 +X 前视。
- 标准二进制网关回环：遥测 100.3 Hz、校验错误 0；解锁、指定角度、归中、手动框架移动均能改变机械角并回传。
- 点选锁定 `target_aircraft` 成功，跟踪框约 90×70 px；双击远离目标时改为锁定画面内最近目标，导引头 yaw/pitch 自动转到目标居中。
- 自动锁定选择离画面中心最近的 `target_aircraft`；取消锁定、关闭自动锁定和框架电锁均回传正确伺服状态。
- 用 `tests/move_target.py` 的默认航线（3.7 m 距离，y ±0.30 m、z ±0.12 m）移动目标时，129/129 个稳态采样保持 TRACK，峰值失调角 1.97°。
- `tests/los25.world.sdf` 的目标运动插件以约 26.7°/s 峰值平滑移动目标；25°/s 验收 100/100 稳态 TRACK，内部 `target_los` 峰值 26.21°/s，标准帧 LOS 按协议限幅到 20.00°/s，最大失调角 0.18°。跟踪使用目标 LOS 角速度前馈，前馈限幅 ±30°/s。
- 自动锁定跟踪中把目标移出视场，插件进入 LOST；目标移回后自动锁定重新选择并恢复 TRACK。
- 上位机 10 Hz 心跳；停止心跳 2 s 后网关停止跟踪并保持角度，控制超时保护已回环验证。
- 固定 RTSP URL `rtsp://127.0.0.1:8554/aisheng_seeker` 可解码；镜头切换时 URL 不变，内容切换到广角/长焦。
- 广角 1×/2×/4× 数字变焦已在上位机显示端验证，4× 画面为中心裁剪放大，`docs/zoom4.png` 为实际截图。
- 长焦原生点选锁定已验证：`camera.active=tele`、`tracking.state=TRACK`、跟踪框约 90×70 px，目标居中，失调角约 0。
- 框架最大转速 60°/s 已确认；yaw 正指令使机械角正向增加，pitch 负指令使机械角负向增加。`check_limits.py` 回环检查**框架角速度**峰值 60°/s，四个方向均未越过 yaw ±60°、pitch −80°～+30°。
- 规格书的“视线角速度 ±25°/s”与标准协议表的“±20°/s”存在字段范围差异：插件内部 `target_los` 保留实际值，标准遥测按协议字段限幅 ±20；25°/s 跟踪验收已通过，标准帧仍按协议发送 ±20。
- 无界面上位机自检可检查遥测、当前 RTSP 视频和解锁状态；当前 1920×1080 单路演示下遥测约 100 Hz、视频 29～30 fps。
- 上位机操作界面已按两轴吊舱地面站补充底部命令按钮、yaw/pitch 虚拟摇杆、单轴归零、输入框指定角、艾晟报文转发和中文遥测；界面绘制不修改视频解码缓存，文字与控件对齐，停帧时继续刷新；界面截图见 `docs/gcs_ui.png`。
- 37 字段完整链路已回环验证：104 字节标准帧 → 地面站解析与实际发送 → 专用 `aisheng_udp_bridge` → `listener dyt_aisheng_telemetry`；四目标数组、原始 int32 经纬度、设备 uint32 毫秒和缺失 LOS 向量标记均保持约定。
- 动态装配 `assembledmodels/rocket_aisheng_seeker/` 已验证 5.00 kg、airframe 4024、100 Hz 插件遥测、双相机话题和 PX4 机体生成；独立飞行验收见该装配目录。
- 尚未验证：更高机动/连续满速/极限限位碰撞、视频 SEI、多客户端控制。
