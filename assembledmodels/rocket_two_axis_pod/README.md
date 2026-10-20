# 火箭穿越机与顶部两轴吊舱

总长590 mm的火箭机与顶部两轴吊舱组合，吊舱替换火箭机纵轴顶部91 mm，各同名轴对齐。当前7.94 kg组合体与7.70 kg基础单机均已通过各自完整无风组件世界飞行自验证，组合外形仍待人工确认。入库的 `model.sdf` 为静态造型评审模型；动态装配通过独立生成入口提供 PX4、飞行动力学、真实关节和相机。

## 外形与装配

由 `models/vehicles/rocket_racing_quad` 与 `models/gimbals/two_axis_pod` 的当前原生 SDF 与视觉资产组合。两轴吊舱坐在火箭机纵轴顶部，替换原机头上段。机体轴向总长保持 **590 mm**，沿用基础模型原点、四翼、电机和桨叶的几何及位置。

- 火箭机坐标原点、机尾 `z=-225 mm` 保持，组合最高点 `z=+365 mm`。`base_link` 原点不在机身几何轴向中点，重心和安装位置均按所属坐标系读取。
- 吊舱安装面中心位于 `(0, 0, 274) mm`，采用零安装旋转，吊舱 +X、+Y、+Z 分别对齐火箭机同名轴。机头和四桨推力参考方向均为 +Z。
- 吊舱总高 91 mm、底环直径 84 mm、相机球直径 48 mm，均沿用组件实际几何。
- 吊舱占据 `z=274～365 mm`。保留机身在 `z=262～274 mm` 设置12 mm长过渡肩部，接到底环半径42 mm；机身最大直径为90 mm。顶部替换不切入翼根。
- 生成器从基础模型实际机身 STL 裁切并肩接，在本装配目录生成 `meshes/fuselage_cut.stl`；其他显示几何复用原生组件。显示翼面与动力舱连续连接，机臂碰撞体保留基础模型的较短近似，当前不考察连接处接触精度。

## 两轴方向

外轴 yaw 绕组合模型 +Z，即火箭机纵轴，正值从 +X 转向 +Y；内轴 pitch 绕随 yaw 转动的局部 +Y，正值使镜头从 +Z 转向局部 +X。yaw 范围 ±90°，pitch 范围 −45°～+90°。

零位镜头朝机头 +Z，pitch=+90° 时朝 yaw 旋转后的 +X 侧方。镜头方向为 `(cos(yaw)sin(pitch), sin(yaw)sin(pitch), cos(pitch))`。pitch=0° 时镜头方向与外轴平行，yaw 无法改变视线方向。机体倾转后安装轴随机体变化，不等于世界偏航、俯仰轴。静态预览将机械姿态烘焙到视觉位姿，不提供实时关节操作；相机球心在吊舱坐标中为 `(15, 0, 56) mm`，零位在组合坐标中为 `(15, 0, 330) mm`。

## 造型预览

以下命令在仓库根目录执行，需要 Python 3、NumPy、Gazebo Sim；离线评审图还需要 Matplotlib。

```bash
# 整机；默认 yaw=0、pitch=0
sh assembledmodels/rocket_two_axis_pod/tests/run.sh overall

# 吊舱近景；镜头侧向 +X，方便检查支架与罩壳
sh assembledmodels/rocket_two_axis_pod/tests/run.sh close 0 90

# 组合姿态 yaw=45°、pitch=45°；或改为 close 0 -45 检查另一俯仰端点
sh assembledmodels/rocket_two_axis_pod/tests/run.sh close 45 45

# 重新生成入库的零位造型与评审图
python3 assembledmodels/rocket_two_axis_pod/tools/generate_model.py
python3 assembledmodels/rocket_two_axis_pod/tools/render_views.py --poses

# 原生资产继承、裁切网格与预览配置检查
python3 assembledmodels/rocket_two_axis_pod/tests/test_appearance.py
```

启动器每次生成独立 `/tmp/rocket-two-axis-pod.*` 运行副本和 Gazebo 分区。整机实体名为 `ROCKET_TWO_AXIS_POD`，位置 `(0,0,0.24) m`；底环安装面在世界 `z=0.514 m`，最高点 `z=0.605 m`。橙色定位箭头实体名为 `ROCKET_POD_LOCATION_MARKER`，位于整机 −X 侧，箭杆中心 `(-0.20,0,0.55) m`，指向顶部吊舱。箭头和预览模型均无碰撞体。可在实体树搜索整机并聚焦；`close` 入口直接配置吊舱近景。

浏览相机默认近裁剪距离为 **0.005 m**，鼠标灵敏度为 **0.1**。滚轮缩放时请把鼠标放到吊舱实体表面，避免指向远处背景；用右键缓慢拖动微调。启动器在 GUI 服务就绪后请求设置灵敏度，结果在运行目录 `camera.log`；仅在 GUI 正常加载且同一 Gazebo 分区中服务可达时生效。可在另一终端使用启动器输出的分区设置：

```bash
GZ_PARTITION=<启动器输出的分区> gz service \
  --service /gui/camera/view_control/sensitivity \
  --reqtype gz.msgs.Double --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'data: 0.1'
```

`parameters.json` 中 `view.mouse_sensitivity`、`view.near_clip_m`、`view.far_clip_m` 配置浏览相机，`view.camera_pose` 和 `view.close_camera_pose` 分别配置整机与吊舱近景入口，重启后生效。吊舱视频相机参数属于基础组件 `models/gimbals/two_axis_pod/parameters.json`，与浏览相机独立；本静态预览未加载视频传感器。

## 动态装配

`tools/generate_dynamic_model.py` 生成包含真实关节、相机、传感器与 PX4 动力接口的运行资产，默认写入本目录忽略入库的 `runtime/rocket_two_axis_pod/`；不会覆盖静态 `model.sdf`。默认 airframe ID 为 4023，模型为 `rocket_two_axis_pod`，话题前缀为 `/rocket_two_axis_pod`。

```bash
python3 assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py
python3 assembledmodels/rocket_two_axis_pod/tools/check_dynamic_model.py
```

动态装配从当前原生基础模型继承全部机身、动力舱和旋翼的质量、惯性、重心、位姿、动力与传感器，并加入原生吊舱的真实关节和相机。`base_link` 质量为7.44 kg，惯性姿态中的重心 z=-0.094 m及完整惯性张量保持原生值；基础机总质量7.70 kg加吊舱0.24 kg，组合总质量为 **7.94 kg**。顶部外形和碰撞截短不扣除机头质量，也不按截短圆柱重算惯量；这是一套显式仿真配置，不代表实物质量标定。

保留机身碰撞圆柱长 **0.499 m**，覆盖局部 `z=-0.225～+0.274 m`，碰撞中心 z=+0.0245 m；碰撞中心与惯性重心分别配置。动力舱、旋翼的全部位姿继承基础模型，吊舱以零旋转安装于 z=0.274 m。动态模型含12个link、11个joint，以固定关节连接 `base_link` 与 `mount_link`，吊舱活动关节及镜头姿态与组件结构一致。

PX4 参数由当前原生基础机体文件继承，模型名和airframe ID对应装配接口；悬停推力按组合总质量、世界重力、各电机原生推力系数及 PX4 怠速/上限映射自动计算。`THR_MDL_FAC=1` 时，PX4 先把推力比例开方，再将输出线性映射到 `[怠速, 最大转速]`；解锁怠速为50 rad/s，未解锁输出为0。当前自动计算配置已通过完整组件世界飞行验收，参数或工况变化后需对应复验。

装配参数 `dynamic.body_mass_kg=null` 表示完整继承原生机身质量；该参数和 `--body-mass-kg` 只能省略或与原生值相同，不支持独立覆盖质量。`dynamic.airframe_id`、`dynamic.topic_prefix` 配置运行接口。运行副本中的 `parameters.json` 保存实际各 link 质量、惯性与姿态、原生电机/气动插件、PX4参数、测试设置和世界关键配置。`manifest.json` 记录安装位置、保留长度、`body_mass_method=native_inertial_preserved`、组合总质量、怠速、计算悬停转速与推力比例及源资产 SHA-256；对应指纹也写入运行快照。生成器复制当前原生网格，仅读取基础组件。完整地理世界的飞行与跟踪入口见[场景说明](../../scenarios/geographic_rocket_two_axis_pod/README.md)，运行前应核对其实际模型与配置。

## 完整飞行自验证

以下入口生成独立动态资产，启动 Gazebo 与 PX4，等待 IMU 首帧后调用基础模型的严格飞行验收工具，自动完成起飞、30 s悬停、200 km/h连续30 s巡航、300 km/h连续10 s冲刺、减速与降落停机。速度、漂移和高度使用 Gazebo 位姿真值，判定与[基础自验证](../../models/vehicles/rocket_racing_quad/tests/README.md)一致。

```bash
# 默认 headless、PX4 instance0，结果写入本次运行目录的 flight.json
bash assembledmodels/rocket_two_axis_pod/tests/run_flight_check.sh

# 指定 PX4 instance1 和结果路径；带界面把 headless 改为 sitl
PX4_INSTANCE=1 \
  bash assembledmodels/rocket_two_axis_pod/tests/run_flight_check.sh \
  headless /tmp/rocket-pod-instance1-flight.json
```

需要已构建的两轴控制插件与兼容 PX4、`pymavlink`、NumPy和Gazebo Python绑定。`PX4_ROOT` 默认 `/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot`，`FLIGHT_PYTHON` 可指定解释器。`PX4_INSTANCE` 支持0～8，默认0，对应 MAVLink `udpin:0.0.0.0:(14540+实例号)`；同一实例的验收应顺序执行。入口检查相关端口并使用独立 Gazebo 分区、`etc` 和 rootfs。

运行目录为 `/tmp/rocket-two-axis-pod-flight.XXXXXX`，保留生成模型、参数快照、`manifest.json`、`execution.json`、`gz_partition.txt`、`result_path.txt`、Gazebo/PX4/飞行日志、IMU首帧和 `exit_code.txt`。`execution.json` 记录实际实例、MAVLink连接、模式、分区、配置及结果路径；退出码保留飞行验收的实际结果。

此入口验证组件世界中的飞行。该世界包含惯性导航等物理传感器系统，没有加载图像渲染的 `Sensors` 系统，未在此入口重验视频、地面站或目标跟踪。完整地理场景的飞行与视频跟踪需要独立证据。

## 多机接入

每架载机的吊舱分别配置飞机ID、UDP控制/遥测端口及RTSP URL；同机可共用RTSP 8555，通过 `/two_axis_pod_<aircraft_id>` 路径区分。控制端口由网关启动配置显式指定，不随飞机ID自动改变。逐机地址表、地面站连接示例和PX4实例隔离要求见[多机集成说明](../../scenarios/geographic_rocket_two_axis_pod/README.md#多机集成的地址与端口对应)。

动态生成器支持 `--topic-prefix` 配置插件与相机话题前缀，生成资产的模型名仍为 `rocket_two_axis_pod`。多机运行时的实体命名、每机话题及PX4绑定由场景启动器负责；现有地理场景入口仍为单机，不能仅改变 `--aircraft-id` 后双开。

## 验证范围

当前7.94 kg组合体于2026-10-20通过完整组件世界飞行自验证：悬停30 s最大漂移0.086579 m，200 km/h连续30.012 s巡航、300 km/h连续10.016 s冲刺，正常减速、落地与停机。Gazebo位姿真值和21项当前源资产指纹均经归档工具核对，证据见[完整飞行验证](docs/validation.md)。

当前外形仍待人工确认。静态模型含34个visual，5项专用静态检查覆盖原生几何继承、闭合裁切网格、零位轴向包络 `[-225,365] mm`、安装位置、浏览相机和定位标志；静态、动态、启动及归档共26项离线检查通过。组件世界飞行通过不代表完整地理场景往返、视频或目标跟踪已经复验。

- [整体三视图与斜视图](docs/model_views.png)、[头部尺寸与轴向](docs/head_detail.png)、[四组机械姿态对比](docs/mechanical_poses.png)：由实际 SDF 视觉几何生成的离线评审图，不是 Gazebo 截图。

以上图像需核对其所用资产与当前原生配置一致，静态图不作为飞行或跟踪验收依据。[组合体验收](../../scenarios/geographic_rocket_two_axis_pod/docs/validation.md)中1.377555556 kg等不同质量与几何配置下的结果，不能作为当前590 mm、7.94 kg组合的通过证据。当前性能结论必须对应实际运行快照、源资产指纹与完整飞行记录。
