# 火箭穿越机与中科 D80

`rocket_zhongke_d80` 将 `models/vehicles/rocket_racing_quad` 与 `models/gimbals/zhongke-d80` 组合为独立资产。底环水平安装在火箭顶部，底环安装面在机体坐标 `(0,0,0.289) m`。保留原机尾、翼面、电机和传感器位置；总长 590 mm，机尾 `z=-0.210 m`、顶部 `z=0.380 m`，过渡肩部为 `z=0.277～0.289 m`。

## 安装与活动范围

组合使用云台内部运动链，安装 RPY 为 `(0,0,0)`。基础中科独立示例的模型级 `+Y90` 不属于此顶部水平安装；基础 `model.sdf` 保持其自身前视姿态。组合生成器只复制组件内部 link、joint 和 plugin，并在 link 位姿上加安装平移。

机械零位光轴沿机体 `+Z`。外轴 yaw 绕安装局部 `+X`，内轴 pitch 绕活动局部 `−Y`。`pitch=-80°` 时光轴朝机体 `+X` 并高于水平 10°；`pitch=-90°` 时正对 `+X`，外轴对视线的作用退化。默认范围 yaw `−90°～+90°`、pitch `−90°～+30°`，速度上限由中科 INI 继承，默认 `20°/s`。

直立载机时，位于相机下方的前方地面目标可能超出机构覆盖范围。场景锁定验证须先检查目标实际投影与机械可达性；端点处的有限输出和失锁状态不能等同于已具备额外转轴自由度。

## 目录与配置

- `parameters.ini`：装配尺寸、基础性能 INI 路径、airframe 编号、浏览相机和飞行验收阈值。
- `tools/generate_model.py`：静态造型生成器。
- `tools/generate_dynamic_model.py`：带 PX4 动力接口、真实关节及中科插件的动态生成器。
- `tools/pod_kinematics.py`：从 SDF 读取内部运动链，供预览和几何验收使用。
- `model.sdf`、`meshes/`：24 个 visual 的静态预览资产，无活动关节、控制插件和碰撞体。
- `tests/world.sdf`、`tests/closeup.sdf`：整体及近景造型预览。
- `tests/test_assembly.py`：源隔离、惯性、CG、推力、安装、话题和运行 CLI 契约。
- `tests/source_baseline.json`：基础模型及参考装配的文件指纹，供隔离检查。
- `runtime/`：本装配的动态生成目录。

中科性能从 `component.parameters_file` 指向的源 INI 读取，使用基础配置与模型生成 API。动态生成器和网关接收相同运行 CLI，在源性能配置上覆盖实体、飞机 ID、世界、目标和 UDP/RTSP 端点，经校验后重新计算 `config_hash`；不生成逐机 INI。独立调用仍支持 `--config` 指定另一份完整源性能 INI。

## 生成

以下命令均在本目录执行，不启动 Gazebo 或 PX4：

```bash
python3 tools/generate_model.py
python3 tools/generate_model.py --pitch-deg -80 --output-dir runtime/preview_forward
python3 tools/generate_dynamic_model.py
```

默认动态输出为 `runtime/rocket_zhongke_d80/`。多机或场景使用：

```bash
python3 tools/generate_dynamic_model.py \
  --output-dir /绝对路径/runtime/rocket_zhongke_d80 \
  --model rocket_zhongke_d80_1 --aircraft-id 1 \
  --world geo_32_217857_119_105878 \
  --target-name xiaomotuo --target-name xiaobianfu \
  --udp-bind 0.0.0.0 --udp-port 8880 \
  --rtsp-bind 0.0.0.0 --rtsp-port 8566 --rtsp-mount /zhongke_d80_1 \
  --topic-prefix /uav_1/pod
```

动态输出包含 `model.sdf`、`model.config`、自有 `meshes/`、`px4/4025_gz_rocket_zhongke_d80`、`manifest.json`、`parameters.json` 和 `tests/world.sdf`。`parameters.json` 是生成的质量、动力与 PX4 运行快照，不是配置入口；`manifest.json` 记录输入 hash、组件身份、名义重心、质量和话题。外层模型资源名固定为 `rocket_zhongke_d80`，场景可在载入时赋每机实体名；插件元数据采用 CLI 覆盖后的 `model.name`。

默认 airframe 为 `4025`，可用 `--airframe-id` 显式指定。生成器只输出 airframe 文件，不修改 PX4 的 ROMFS/CMake 注册。动态生成拒绝覆盖基础模型、静态装配及其他已有组合目录。

## 物理与 PX4

原生火箭机质量 `4.76 kg`、中科默认 `0.195 kg`，组合质量 `4.955 kg`。基础 `base_link` 的 `4.50 kg`、惯性中心和完整惯量张量保留；云台三个 link 的质量、惯性中心及完整张量由基础中科生成 API 按当前 INI 产生并保留。顶部视觉和机身圆柱碰撞截短不扣除原机头质量，肩部不额外加质量。

动态模型有 12 个 link、11 个 joint；固定 `pod_mount_joint` 将 `base_link` 连接到 `mount_link`，两个机械轴使用真实关节。动力、电机通道、翼面气动和基础传感器均继承原火箭模型。悬停推力按当前总质量、重力和原电机转速/推力曲线重新求解，四电机空间位置保持不变。

零关节名义重心同时用于云台 `los_observer_cg_model` 和 PX4 位置参考。生成器从原机重心与组合重心之差推导 12 个 `CA_ROTOR*_PXYZ` 和 6 个 `EKF2_IMU/GPS_POS_*` 参数，使用 FLU 到 FRD 的符号转换；不能手填覆盖这些派生参数。此固定参考不随实时云台关节角改变。飞行角速度参数和验收阈值可在装配 INI 中查看；生成完成本身不表示已通过飞行稳定性或锁定验收。

`mount_dynamic=true` 使中科插件每步更新真实安装姿态，优先采用安装 link 的实际世界角速度并转换到安装坐标；缺少角速度时使用姿态差分滤波。组件的控制库仍使用基础中科构建产物，不复制或修改其源码。

## 运行接口

Gazebo 资源路径应包含动态输出目录的上一级；系统插件路径应包含 `models/gimbals/zhongke-d80/controller/build/lib`，并保留本机已有的 Gazebo 系统路径。渲染世界需启用 `gz::sim::systems::Sensors`（Ogre2）。动态输出的 `tests/world.sdf` 是原生动力自验证世界，不启用视频渲染或提供锁定目标；02场景负责真实视频、目标实体与飞行集成。

默认话题前缀 `/rocket_zhongke_d80`，可用 `--topic-prefix /uav_<id>/pod` 改写：

| 后缀 | 内容 |
| --- | --- |
| `/command` | 中科网关发给插件的内部控制 JSON |
| `/telemetry` | 实际关节、光学和真值候选诊断 JSON |
| `/command_result` | 插件控制结果 JSON |
| `/image` | 单路真实 Gazebo 图像 |
| `/camera_info` | 实际相机内参 |
| `/diagnostics` | 中科网关发布的只读诊断 |

网关复用基础 `controller/service/gimbal_bridge.py`，传入与生成器相同的运行 CLI，以及 `--topic-prefix`、`--image-topic <前缀>/image`；使用自定义性能时两者还须传入相同的 `--config`。UDP 使用中科 16 字节控制和 32 字节遥测，保持同一实际客户端端点。地面站通过 `--udp-host`、`--udp-port`、`--rtsp-url`、`--window-title` 指定连接，不依赖场景会话清单，收到二进制遥测后再转发 15 字段 JSON。

基础组件独立运行的会话接口保持兼容。场景地面站显示实际二进制状态与测量；目标名称、候选和原因通过场景分区内 `/diagnostics` 验收。

## 预览与验证

```bash
GZ_SIM_RESOURCE_PATH="$(pwd)/..${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" \
  gz sim -r tests/world.sdf
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

实体树搜索 `ROCKET_ZHONGKE_D80` 或橙色无碰撞箭头 `ROCKET_ZHONGKE_D80_LOCATION_MARKER`。箭头指向顶部云台侧方，近景入口为 `tests/closeup.sdf`。浏览相机近裁剪默认 `0.005 m`；滚轮指向部件表面，右键缓慢拖动微调。`view.mouse_sensitivity` 默认 `0.1`，同分区 GUI 启动后可通过 `/gui/camera/view_control/sensitivity` 的 `gz.msgs.Double` 请求应用；该值同时影响缩放、平移和旋转，成像相机参数由中科 INI 单独控制。

静态与动态模型均可用 `gz sdf -k <model.sdf>` 校验。离线测试检查配置继承和结构；02场景待命载机上的实际图像、二进制遥测、自动/指点锁定、移动目标保持、人工接管和回零已验证；结果、证据和飞行/人工几何检查边界见[验证说明](docs/validation.md)。

## 02场景入口

在PX4仓库目录执行，默认两轴机型及原艾晟选择方式保持：

```bash
bash /home/aeroarx/zsh/PX4-Autopilot/scenarios/02_rocket_two_axis_pod/start.sh \
  --aircraft-model rocket_zhongke_d80 --gui
```

场景共用 `/tmp/px4-scenarios/02_rocket_two_axis_pod` 和分区 `px4_scenario_02_rocket_two_axis_pod`。首机 UDP 为 8880、RTSP 为 8566，第二机分别为 8881、8567；每机视频路径为 `/zhongke_d80_<id>`。启动器打印各机的完整地面站命令，首机示例：

```bash
/home/aeroarx/zsh/px4-gazebo-assets/models/gimbals/.venv-zhongke-d80/bin/python \
  /home/aeroarx/zsh/px4-gazebo-assets/models/gimbals/zhongke-d80/ground_station/ground_station.py \
  --udp-host 127.0.0.1 --udp-port 8880 \
  --rtsp-url rtsp://127.0.0.1:8566/zhongke_d80_1 --window-title '中科D80 飞机1'
```

场景不自动启动地面站。停止场景统一使用无参数命令：

```bash
bash /home/aeroarx/zsh/PX4-Autopilot/scenarios/02_rocket_two_axis_pod/stop.sh
```

停止脚本按现有全局进程搜索方式清理 Gazebo、PX4、网关与 RTSP 中继；运行模型、日志与 PID 文件由公共场景目录管理，不生成 `session.ini` 或地面站配置副本。
