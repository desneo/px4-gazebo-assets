# 两轴立装演示与自测

`world.sdf` 提供 1.5 m 高的水平台面、固定立装吊舱、上方纹理靶标和独立近景外观观察相机。Ø84 mm 底部圆环水平落在台面上，导引头整体向上高 91 mm。吊舱仅固定 `mount_link`，两根轴仍可活动。世界内直接包含生成的吊舱，网关模型名为 `two_axis_pod`，世界名为 `two_axis_pod_demo`；不启动 PX4。

`world_base.sdf` 是环境源文件。修改环境或模型参数后，运行 `python3 models/gimbals/two_axis_pod/tools/generate_model.py`，同步生成基本模型和演示世界。

## 离线验证

仓库根目录：

```bash
python3 models/gimbals/two_axis_pod/tests/test_model.py
cmake -S models/gimbals/two_axis_pod/controller -B models/gimbals/two_axis_pod/controller/build
cmake --build models/gimbals/two_axis_pod/controller/build -j4
ctest --test-dir models/gimbals/two_axis_pod/controller/build --output-on-failure
.venv-linux/bin/python -m unittest discover -s models/gimbals/two_axis_pod/ground_station/tests
gz sdf -k models/gimbals/two_axis_pod/model.sdf
gz sdf -k models/gimbals/two_axis_pod/tests/world.sdf
```

协议与频率配置测试只需标准库；完整地面站测试使用项目 `.venv-linux` 中的 OpenCV 等依赖，不启动 Gazebo；其中 `test_config.py` 验证带注释 INI 的类型读取和对 JSON 配置的拒绝，`test_windows.py` 验证 Windows 配置、视频帧处理、D3D11 硬解打开参数以及拒绝软件解码回退。真实 UDP 测试仅向本机回环地址发送数据。

`test_rates.py` 验证两秒真实时间窗口的接收/发送 Hz、预热、身份过滤、失败发送不计数、停发/断流归零、重启清零及界面 Hz 文案。

`test_telemetry.py` 验证统一 `hz` 配置、每份新姿态只发送一次、暂停/恢复、重复时间戳过滤、频率不一致时停止周期转发及有界队列。CTest 另验证不同仿真实时因子下的真实时间 100 Hz/50 Hz 遥测调度、暂停恢复、新物理样本门禁、物理更新不足告警及非法配置拒绝；控制调度单测覆盖4 ms物理步的8/12 ms交替、3 ms量化、大步长、暂停和仿真时间回退，保证长期频率准确且每步至多运行一次。控制测试同时覆盖Gazebo世界角速度到安装局部系的变换，以及35 Hz姿态差分后备和跳变拒绝。

`test_protocol.py` 另验证重复关闭视觉的停止、网关在 off/detect 下限频清理遗留 TRACK，且不影响人工转动和回零。

`test_lifecycle.py` 检查同仓库/端口/飞机 ID 匹配、远端网关隔离、PID 复用保护，以及真实后台进程忽略 SIGINT 时的有界 SIGTERM 停止。

`test_forward.py` 使用本机真实 UDP 检查艾晟仅十三字段输出、跟踪门宽高取最终像素框且无效时清零、关闭视觉/检测/搜索与残留 TRACK 矛盾时清零、自动/手动有效性切换、角度/速度原量、右下为正的脱靶量、非法值/旧协议拒绝及发送恢复、身份与来源过滤、停止/重启不重发旧状态、IP/端口校验、发送中点击编辑自动暂停且修改后仅发往新端口、INI 注释保留、输入焦点屏蔽云台快捷键和脱靶量过期显示。`test_client.py` 验证旧协议门禁，缺少遥测版本或使用旧版本 时不允许角度控制。CTest 验证真值框四边各扩展 5 px 与边界裁剪、负俯仰朝前、图像 Jacobian 与载机补偿、脱靶量中心/四象限、测量帧号、初始化/停止/遮挡/断帧超时失效及重捕获。

实窗自测需先启动独立两轴飞机靶标场景并将相机对准默认靶标，使用单独的 Station INI。该自测会切换到手动框选并发起一次跟踪，仅用于隔离验收；转发接收端须配置为本机回环地址：

```bash
.venv-linux/bin/python models/gimbals/two_axis_pod/ground_station/tests/check_forward_live.py \
  --config runs/two_axis_forward_acceptance/station.ini \
  --output runs/two_axis_forward_acceptance/forward_check
```

自测执行实际输入/点击发送、停止与重启，逐包比对艾晟转换结果，验证朝前角度为负、有效/无效输出及原始框中心和角偏差，结果与截图保存在输出目录。只允许回环目标地址，勿覆盖实际接收地址；需要时为自测另建 INI。

## 在线验证

```bash
export GZ_PARTITION=two_axis_validation
bash models/gimbals/two_axis_pod/scripts/run_plugin_sim.sh
python3 models/gimbals/two_axis_pod/tests/check_sim.py
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --encoder nvenc
bash models/gimbals/two_axis_pod/ground_station/run.sh --selftest 5 --send-home --send-manual
bash models/gimbals/two_axis_pod/ground_station/run.sh --decode-test 5 --decoder nvdec --snapshot runs/two_axis_pod/rtsp.png
bash models/gimbals/two_axis_pod/scripts/stop_bridge.sh
bash models/gimbals/two_axis_pod/scripts/stop_plugin_sim.sh
```

`check_sim.py` 依赖 gz Python 绑定与 Pillow，会运动两轴到四个限位、从限位回零、验证零位非奇异、外轴改变视线与实际活动link光学姿态、单轴复位、无续发超时停车、拒绝滚转、显式 `csrt` 跟踪与两路渲染。完成后两轴回零并停止。结果保存到 `runs/two_axis_pod/validation.json`，零位帧为 `camera_zero.png` 与 `overview_zero.png`。测试使用独立分区，不要同时用地面站或其他客户端下发控制命令。

RTSP 解码和地面站自检需具备 OpenCV Python 等依赖；`run_bridge.sh` 和地面站 `run.sh` 会自动配置共用的 `.venv-linux`，也可用 `GIMBAL_PYTHON` 指定已配置的独立环境。`--selftest` 会实际运动模型；应在关节在线检查完成后执行。测试目录中的离线网络/UI测试不启动图形窗口。

完整400块地理背景中的动态靶标与实体遮挡验收见[地理部件测试场景](../../../../scenarios/geographic_two_axis_pod/README.md)，使用同一两轴模型与协议。

目标检测/跟踪和网关环境测试使用带 OpenCV 的两轴环境，覆盖 YOLO 输出解析、连续 5 帧确认、误检过滤、ByteTrack 两阶段关联、稳定 ID、Hungarian 匹配、中心最近选择、RGB 红框、模型路径校验和 OpenCV DNN 实际加载归档 ONNX：

```bash
.venv-linux/bin/python -m unittest \
  models/gimbals/two_axis_pod/controller/service/tests/test_vision.py \
  models/gimbals/two_axis_pod/controller/service/tests/test_bridge_env.py
```

多机视频验证（无需 Gazebo 或 PX4，需要 NVIDIA NVENC/NVDEC、GStreamer 与 GstRtspServer）：

```bash
.venv-linux/bin/python \
  models/gimbals/two_axis_pod/controller/service/tests/test_rtsp_multi.py
```

测试使用临时 RTSP 端口，启动独立编码进程验证同端口不同 ID 后缀的视频隔离、重复路径拒绝、单路退出、租约过期和中继重启恢复。测试中继在全部视频退出后空闲约 30 秒自动退出。

机械转速限制为每轴 ±20°/s；`test_model.py` 核对参数、插件和两关节物理速度限制一致，装配 `test_dynamic_model.py` 检查原生继承。CTest 覆盖各模式、补偿叠加、运行中收紧、旧配置无法抬高上限，以及目标在视野内以 ±22° 水平角、±15° 垂直角快速跨四象限抖动时 `sim_truth` 逐帧保持 TRACK、显式框四边各扩展 5 px 且不产生无效帧。`test_rate_limit.py` 验证艾晟角速度限幅、有限值/float32 校验、13 字段格式、跟踪门宽高、边界、未跟踪状态、输入保留及真实回环 UDP 输出；摇杆测试验证手动流和方向输入最大 20°/s。

## yaw 左右方向验收

当前协议为 v8 / `motion_head_joint_right_positive`，yaw 正方向绕安装 +X。按镜头视线方向，鼠标右拖/右方向键对应正 yaw，左拖/左方向键对应负 yaw；静止背景与镜头转动方向相反。

```bash
.venv-linux/bin/python models/gimbals/two_axis_pod/tests/check_yaw_direction.py
```

脚本启动独立 Gazebo 分区和无渲染物理世界，调用生产鼠标输入映射，检查左右转动的实际 `camera_link` 姿态、机械位置/速度、生产艾晟 `gyro_yaw_rate` 转换发送线程及本机真实 UDP 收包，并确认未配置无人机重心/未绑定目标时 LOS 为 0/0。另检查旧 yaw=-X 模型被新插件明确禁用。`--plugin-dir` 可指定待验库目录，`--output` 指定日志与结果目录，默认使用 `controller/build/lib` 和 `runs/two_axis_yaw_direction`。只停止本脚本创建的服务器，不连接 PX4 或修改地面站配置。该检查不替代真实视频渲染或整机飞行验证。

C++ 测试覆盖图像右目标产生正 yaw、左目标产生负 yaw、实际 CSRT 左右目标、有限差分 Jacobian 和载机补偿；模型测试从 SDF 转轴与 sensor 外参独立验证整个工作范围的左右方向。组合/场景测试同步检查逆解、原生轴向继承及 v6/旧角度约定拒绝。

## 目标惯性 LOS

内部 `target_los` 计算目标模型原点（当前真值跟踪使用的目标参考点）相对零关节固定无人机名义重心的 NED 方位角/仰角变化率，右正/上正；机械速率由独立 `gyro_pitch_rate/gyro_yaw_rate` 映射。艾晟16字段中四个速率均限幅±20°/s；内部LOS不截断，明确无效时外发0/0，其它字段继续。界面沿用现有track状态，不增加独立LOS无效提示。

CTest `two-axis-target-los` 覆盖双方运动、径向运动、NED方向、跨±180°、零距离/竖直奇异、原始超20速率、名义重心随模型旋转的固定偏移、暂停/回退/长间隔/瞬移/换目标。地面站测试覆盖gyro与LOS来源独立、16字段、无效0/0与恢复、数据契约拒绝、旧协议拒绝及真实回环UDP。

```bash
.venv-linux/bin/python models/gimbals/two_axis_pod/tests/check_inertial_los.py \
  --plugin-dir models/gimbals/two_axis_pod/controller/build/lib
```

该入口使用独立Gazebo分区、简化载机与真实目标实体，具体物理用例和通过数据以输出报告为准；仅停止本脚本创建的服务器，不连接用户PX4实例或改地面站配置。不会把组件/合成图像验收声明为整机飞行或完整RTSP链路验收。固定台架无无人机名义重心声明时，目标LOS保持无效；要测机载LOS，使用派生CG的动态装配，`sim_truth`绑定实际实体，CSRT显式配置并验证`los_target_name`。

三维目标视线角速度通过`los_omega_n/e/d`输出，单位°/s，对应固定NED三轴，按`(r×v)/|r|²`计算，不含绕视线自转且不施加±20°/s限幅。原有方位角/仰角变化率与机械gyro字段保留现有限幅。内部`omega_valid/omega_reason`独立于两角度导数的`valid/reason`：竖直视线时方位角退化，向量仍可有效；未绑定等无效情况发送向量0/0/0。界面沿用track状态。
