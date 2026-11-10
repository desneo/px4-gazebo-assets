# 火箭穿越机与 A2416A 捷联导引头

`rocket_jielian_a2416a` 将 `models/vehicles/rocket_racing_quad` 与固定式 `models/gimbals/jielian-A2416A` 组合为独立资产。导引头底面位于机体 `z=+0.320 m`，镜头朝机体 `+Z`，最高点保持 `z=+0.380 m`；机尾保持 `z=-0.210 m`，组合总长 590 mm。

该组件是捷联式导引头，没有机械框架、活动关节或伺服角速度。相机与火箭刚性连接，目标失调角由固定相机光轴和飞机真值位置计算。

## 几何与物理

- 顶部 60 mm 机身视觉由 Ø60×60 mm 导引头占位外形替换。
- 过渡肩部为 `z=0.308～0.320 m`。
- 原火箭质量与惯量保持；被替换的机头外形不扣除质量，再增加 0.05 kg 导引头 link。
- 动态组合总质量 4.81 kg，导引头通过唯一的 `seeker_mount_joint` 固定到 `base_link`。
- PX4 airframe 编号为 4026，默认 Gazebo 话题前缀为 `/rocket_jielian_a2416a`。

## 目录

- `parameters.ini`：装配尺寸、基础组件引用、PX4 编号、浏览相机和飞行验收参数。
- `tools/generate_model.py`：生成静态造型、裁切机身网格和整体/近景世界。
- `tools/generate_dynamic_model.py`：生成继承原火箭动力、气动、传感器并带固定导引头的 PX4 动态资产。
- `tests/run_flight_check.sh`：独立执行完整飞行、真值跟踪和遥测验收并保留运行日志。
- `model.sdf`、`meshes/`：已生成的静态预览资产，不含碰撞、关节或控制插件。
- `tests/test_assembly.py`：源资产隔离、尺寸、质量、安装、话题、airframe 和资源闭包测试。

## 生成与检查

在本目录运行：

```bash
python3 tools/generate_model.py
python3 tools/generate_dynamic_model.py
python3 -m unittest tests/test_assembly.py -v
```

动态输出默认位于 `runtime/rocket_jielian_a2416a/`，包含 `model.sdf`、自有网格、PX4 airframe、运行参数快照和测试世界。运行参数覆盖示例：

```bash
python3 tools/generate_dynamic_model.py \
  --output-dir /tmp/rocket_jielian_a2416a \
  --airframe-id 4026 \
  --topic-prefix /rocket_jielian_a2416a
```

生成器拒绝覆盖基础模型目录、当前静态装配目录和非法话题前缀。静态与动态生成均读取基础模型当前文件，生成期间会检查源文件指纹不变。

## 预览

```bash
GZ_SIM_RESOURCE_PATH="$(pwd)/..${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" \
  gz sim -r tests/world.sdf
```

近景入口为 `tests/closeup.sdf`。实体树可搜索 `ROCKET_JIELIAN_A2416A` 和橙色无碰撞标志 `JIELIAN_A2416A_LOCATION_MARKER`。浏览相机近裁剪为 0.005 m；检查厘米级镜头时应把鼠标指向部件表面并用右键缓慢拖动。建议通过 `/gui/camera/view_control/sensitivity` 服务将灵敏度设为 0.1。

## 动态运行接口

| 话题 | 内容 |
| --- | --- |
| `<prefix>/command` | 网关发送给捷联插件的内部 JSON 控制 |
| `<prefix>/telemetry` | 50 Hz 真值检测、跟踪和失调角 JSON |
| `<prefix>/command_result` | 命令执行结果 |
| `<prefix>/image` | 1920×1080@25 Hz RGB 相机图像 |
| `<prefix>/camera_info` | 相机内参 |

动态生成器按 4.81 kg 组合质量重新派生 `MPC_THR_HOVER`、固定名义重心、`CA_ROTOR*_P*` 以及 `EKF2_IMU/GPS_POS_*`。原生电机、气动和传感器位置保持不变。生成成功只证明资产和 PX4 参数自洽；完整飞行与跟踪实测结果见 `docs/validation.md`。

## 完整飞行与跟踪验收

先构建 A2416A Gazebo 插件并保证 PX4 SITL 已构建，然后在仓库根目录执行：

```bash
cmake -S models/gimbals/jielian-A2416A \
  -B models/gimbals/jielian-A2416A/build -DBUILD_TESTING=ON
cmake --build models/gimbals/jielian-A2416A/build -j2
bash assembledmodels/rocket_jielian_a2416a/tests/run_flight_check.sh headless
```

启动器使用独立临时目录和 `GZ_PARTITION`，不会修改 PX4 源码或现有模型。动态测试世界保存一个位于初始光轴前方 500 m 的 `target_aircraft` 飞机真值夹具；验收期间先开启识别和自动锁定，再执行 30 s 悬停、200 km/h 持续 30 s、300 km/h 持续 10 s和降落停机。最终门禁同时要求飞行报告通过、50 Hz 捷联遥测有效、记录到真实目标 `TRACKING`，且所有平台专有字段保持零。

运行目录、飞行报告和 `tracking-validation.json` 路径会打印到终端；异常退出时仍保留生成资产和日志。可用 `PX4_ROOT`、`PX4_INSTANCE` 和第二个位置参数分别指定 PX4 路径、实例号和飞行结果文件。

## 02 场景

PX4 仓库的 02 场景可启动本装配并接入雷达、H.264 RTSP 和艾晟兼容遥测：

```bash
bash scenarios/02_rocket_two_axis_pod/start.sh \
  --aircraft-model rocket_jielian_a2416a --gui
```

场景仅使用 `sim_truth`，真实目标包含 `xiaomotuo`、`xiaobianfu` 和场景内的其他 A2416A 组合飞机。控制/二进制遥测从 UDP 8001 递增，艾晟兼容 JSON 从 UDP 9000 递增并由现有 `aisheng_udp_bridge` 发布到飞控，H.264 RTSP 从端口 8554 递增。启动器生成逐机地面站 INI，但必须由操作者点击“发送艾晟报文”后才开始向飞控转发。
