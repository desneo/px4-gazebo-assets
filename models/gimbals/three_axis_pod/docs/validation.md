# 吊舱功能自验证

## 结果范围

公共 PX4 FRD 姿态接口、控制保护和通信回归通过对应检查；立装公共单轴复位、立装/悬挂公共水平复位与朝天方向已在真实 Gazebo 插件中验证。移动目标长期指标、GUI 完整操作与跨机器弱网未验收。载机/PX4 联动、动态姿态补偿与真实飞行由其他项目验证，不属于本项目验收范围。

公共接口验收使用 `angle_convention=px4_frd`、`attitude_deg/attitude_quat_frd`，机械 `joints_deg/cmd_dps` 仅用于诊断。下表区分公共姿态、跟踪和视频的证据范围；跟踪或编码专项结果不替代公共角度方向、安装光学校准或动态载机补偿验收。

环境：Gazebo Harmonic 8.15.0、OpenCV C++ 4.6.0、Python 3.12。基本链路使用独立 `GZ_PARTITION`、UDP18765、RTSP18554、x264编码与avdec_h264解码，日志保存在 `runs/archive_check/` 和 `runs/plugin_watchdog/`（Git忽略）。系统Python缺少cv2，地面站联调通过调用方提供的 `/home/aeroarx/test/gambal_test/vendor` 补足Python依赖，仓库不隐式引用该路径。

| 检查 | 当前结果 |
|---|---|
| CMake构建与CTest | 插件构建成功，CTest 2/2 通过；包含公共 FRD 换算、两种安装、公共速度耦合、公共单轴保持与奇异/不可达保护，以及正常控制与watchdog硬断言 |
| 控制回归 | 0.5秒手动超时、续期恢复、0.5秒图像停转/2秒LOST、INIT与连续坏帧超时、动态上下限位、独立watchdog时钟及模式状态清理通过 |
| 2 秒间歇遮挡控制器回归 | 持续 30 fps 合成图像中，两次完整 2 秒遮挡后目标平移均自动重捕获；隔离 Gazebo 插件中两次 >2.05 秒墙钟空白图像后均恢复；遮挡期间停止输出，2.5 秒窗口到期要求重新选框；不替代真实场景几何遮挡验收 |
| Python协议/客户端与摇杆 | 地面站 49 项及控制器网关协议 8 项通过；含协议版本/公共字段门控、奇异保护、同名逆序回执、多客户端并发、重传、乱序、缓存窗口、超时重登记、参数错误、pending清理、来源/垃圾包、同机时延门控、摇杆与单轴按钮行为 |
| 虚拟摇杆 | 离线鼠标事件经实际客户端 JSON 编码产生连续 10 Hz manual 流，松手归零后无残留非零包；三轴行程/死区、失焦或窗口外释放状态、非 X11 租约、GUI 停顿到期、切换任务先归零、关闭幂等通过。焦点/物理左键输入在单元测试中模拟，未做真实桌面拖出窗口的人工验收 |
| 地面站界面 | 真实地理场景 UDP/RTSP 界面截图 runs/joystick_validation/public_angles_live.png：公共 yaw 约 +0.01°、pitch −0.50°、roll 0°，实际水平地平线、立装 skyward 标志和三轴复位按钮可见；Gazebo 与地面站同时运行，视频使用 NVENC。摇杆、单轴复位及协议门控另由离线 GUI 事件测试覆盖，不替代完整人工操作验收 |
| 控制报文资料 | 全部 10 条指令的 26 段 JSON 示例可解析，13 条请求（含三轴单独复位和零速度）经过实际网关 handler 分派校验，发送示例 Python 语法通过 |
| 公共单轴复位实测 | skyward 地理场景，经 UDP→网关→实际插件，公共 yaw 9.755°→0.494°、pitch −27.699°→−0.495°、roll 3.264°→0.499°；三条可靠回执成功，其他两公共轴执行全过程最大保持误差 0.136°；到位阈值 0.5°；证据 runs/axis_reset_validation/public_results.json |
| 真实插件隔离测试 | 因子0.2慢速仿真中，遥测观测手动停止0.637秒，断流停止0.562秒、LOST2.062秒；两次连续空白图像 >2 秒后自动重捕获通过。差值含控制与遥测周期，不把观察延迟作为新阈值；证据 runs/joystick_validation/public_watchdog.log |
| 插件回执/选框 | 实际插件原样回传两条同名请求的request_id，参数失败也关联原请求；有目标框时归一化框始终有效 |
| 实际UDP网关 | 双客户端同名home回执隔离、参数失败携seq、缓存重发和过时manual拒绝通过 |
| 地面站UDP客户端 | 登记、心跳、公共版本/字段门控与回执关联由客户端协议测试覆盖；实测公共单轴复位使用同一 UDP 网关链路 |
| 地理场景实体遮挡 | 公共接口运行的真实地理背景中，skyward 固定吊舱与动态小车静止状态，两轮遮挡板保持 2 秒墙钟，均 TEMP_LOST→TRACK、未进入 LOST，恢复后稳定 TRACK 1 秒；证据 runs/joystick_validation/public_occlusion.log、public_occlusion.jsonl |
| 地理场景 NVENC 硬编码视频 | RTX 4060，网关使用 nvcudah264enc；1920×1080，6 秒取得 181 帧，约 30.4 fps，181/181 帧序号有效；GPU 编码利用率观测 8%，客户端使用 avdec 软件解码 |
| RTSP解码 | 1920×1080，6秒取得165帧，约30.5fps，165/165帧序号识别成功，截图保存成功 |
| 立装移动小车 | 公共接口运行的 skyward 场景，0.5 m/s 前进/倒车/转弯/反向转弯，共 39.98 秒仿真时间；实际位移 18.386 米；2895/2895 条遥测有效 TRACK（100%）；中心角误差 P95=0.054°、最大 0.103°；独立车顶真值角差 P95=0.868°；证据 runs/joystick_validation/public_moving.log、public_moving.jsonl |
| 公共安装姿态与实际天空画面 | 地理 world 中 skyward：公共 reset 三轴误差均 <0.5°、仰角 0.046°，home 仰角 89.953°且 singular=true；downward：公共 reset 三轴误差均 <0.5°、仰角 −0.046°，home 仰角 43.854°。真实 IMU 还原参考后的光学姿态与世界遥测误差均 <0.0001°；证据 runs/mount_validation/skyward/results.json、downward/results.json 及 runs/joystick_validation/public_mount_*.log |
| 两种安装公共 pitch 正负方向 | skyward：水平复位实际仰角 0.046°，公共正 pitch 指令后实际/报告 6.046°，负 pitch 指令后回 0.046°；downward：复位 −0.046°，正 pitch 后实际/报告 6.074°，负 pitch 后回 0.074°。真实 IMU 与报告的差值均 <0.0001°，验证抬头正、低头负与实际光学方向一致；证据 runs/mount_validation/{skyward,downward}/results.json 中 manual_pitch 条目及 public_mount_*.log。该实测仅覆盖各安装的水平附近姿态 |

## 重复验证

在仓库根目录：

```bash
cmake -S models/gimbals/three_axis_pod/controller -B models/gimbals/three_axis_pod/controller/build
cmake --build models/gimbals/three_axis_pod/controller/build -j4
ctest --test-dir models/gimbals/three_axis_pod/controller/build -V
python3 -m unittest discover -s models/gimbals/three_axis_pod/ground_station/tests -v
python3 -m unittest discover -s models/gimbals/three_axis_pod/controller/tests -p 'test_*.py' -v
python3 models/gimbals/three_axis_pod/controller/tests/check_plugin.py
```

最后一条会自行启动独立 Gazebo 进程，临时测试世界保留真实 camera 传感器实体用于读取光学外参，将其输出设为低频独立话题，并向插件图像话题另行注入可控图像；结束时清理进程。测试文件与日志保存在 `runs/plugin_watchdog/`，需要 gz Python bindings 和 Gazebo 运行权限。仿真因子 0.2 用于证明超时使用真实单调时间；注入图像仅用于可控断流与遮挡，不替代真实视频测试。

协议单元测试用标准库AST加载实际类方法，不依赖cv2/gi；实际网关与插件运行测试用于补充真实进程与网络证据。

启动当前场景与网关后，单轴复位在线检查可运行 `python3 models/gimbals/three_axis_pod/tests/check_axis_reset.py`，使用 `--host/--port` 指定连接，`--output` 指定结果路径。该脚本会转动吊舱，逐轴验证公共姿态零位、可靠回执与其他公共轴在全过程的保持误差；结束后停转，不恢复测试前视线。

基本联调先按[组件使用说明](../scripts/README.md)启动仿真与网关，再运行：

```bash
bash models/gimbals/three_axis_pod/ground_station/run.sh --selftest 4 --send-home --send-manual --test-timeout 2.5
bash models/gimbals/three_axis_pod/ground_station/run.sh --decode-test 6 --decoder avdec --snapshot /tmp/gimbal.png
python3 models/gimbals/three_axis_pod/tests/check_commands.py
```

自定义测试端口时为前两条加 `--udp-port 18765 --rtsp-port 18554`，最后一条加 `--port 18765`。在线控制检查会转动吊舱，依赖默认棋盘世界和未被其他客户端干扰的场景，按遥测等待到位，最长15秒。

## 地理场景验证

环境入口、Gazebo 3D 窗口及遮挡操作见 [地理相机场景](../../../../scenarios/geographic_camera/README.md)。运行中的场景可执行：

```bash
GZ_PARTITION=geographic_camera python3 scenarios/geographic_camera/check_occlusion.py --mount-mode skyward
GZ_PARTITION=geographic_camera python3 scenarios/geographic_camera/check_moving_target.py --mount-mode skyward
```

安装验证自行启动并清理独立 Gazebo 实例，无需连接运行中的网关：

```bash
python3 models/gimbals/three_axis_pod/controller/tests/check_mount.py --mount-mode skyward --public-manual
python3 models/gimbals/three_axis_pod/controller/tests/check_mount.py --mount-mode downward --public-manual
python3 models/gimbals/three_axis_pod/controller/tests/check_mount.py --mount-mode downward --merged
```

`--public-manual` 在公共水平复位后补测正、负 pitch，使用真实 IMU 光学姿态核对公共方向；结果覆盖默认 `runs/mount_validation/<安装模式>/results.json`。合并装配命令可用于重复检查装配外参，但未列入上表当前两种标准安装的公共手动方向实测范围。

`check_occlusion.py` 复位相机并两次移动真实遮挡板，保存 UDP 遥测与事件。服务确认后每轮至少保持 2 秒墙钟；实体显现时间受仿真更新影响，视频显示受链路延迟影响。遮挡结果针对平台附近小车静止状态；`check_moving_target.py` 的移动验收另有约 40 秒速度轨迹及真值对比，不代表移动期间遮挡、长期跟踪或任意外观变化的验收。悬挂安装目前验收到位控制、静态姿态遥测和水平附近正负 pitch 方向，未完成该安装下移动目标与实体遮挡的同等验收。

## 未覆盖

- GUI鼠标框选、快捷键与中文字体的完整人工操作。
- 移动期间的 2 秒遮挡重捕获、长期遮挡、复杂外观变化及更高速度轨迹。
- NVIDIA 硬件解码；硬编码已在地理场景验证，客户端解码使用软件。
- 跨机器时钟同步、弱网压力与多操作员控制权仲裁。
- 真实地理环境中的移动目标长期跟踪指标。
- 公共姿态接口在任意装配角/机械边界下的完整可达域扫描、侧装正式模式，以及悬挂安装下移动与遮挡跟踪。

载机安装、PX4 联动和姿态补偿不属于本项目验收范围。

当前行为与边界见[系统说明](system.md)。
