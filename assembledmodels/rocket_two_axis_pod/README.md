# 火箭穿越机与顶部两轴吊舱

总长590 mm的火箭机与顶部两轴吊舱组合，吊舱替换机头顶部91 mm，各同名轴对齐。当前动态组合采用协议 v7、yaw 左负右正及每轴 ±20°/s 限速；真值与 CA/EKF 均参考固定零关节名义重心。当前整机飞行、悬停稳定性和机载CSRT跟踪已通过，飞行归档只证明其对应运行快照与源指纹。组合静态外形仍待人工确认；入库 `model.sdf`为静态造型评审模型。

## 外形与装配

由 `models/vehicles/rocket_racing_quad` 与 `models/gimbals/two_axis_pod` 的当前原生 SDF 与视觉资产组合。两轴吊舱坐在火箭机纵轴顶部，替换原机头上段。机体轴向总长保持 **590 mm**，沿用基础模型原点、四翼、电机和桨叶的几何及位置。

- 组合局部机尾z=-210 mm、最高点z=+380 mm，总高590 mm。`base_link`原点为局部z=0，基础机整机重心z=+30 mm，机身自身重心z≈+33.177778 mm，分别定义。
- 吊舱安装面中心 `(0,0,289) mm`，零安装旋转；吊舱+X/+Y/+Z与机体同名轴对齐，机头与推力沿+Z。
- 吊舱总高 91 mm、底环直径 84 mm、相机球直径 48 mm，均沿用组件实际几何。
- 吊舱占据z=289～380 mm，12 mm过渡肩部位于z=277～289 mm，连接底环半径42 mm；主身最大直径90 mm。裁切保留机身范围z=-210～289 mm。
- 生成器从基础模型实际机身 STL 裁切并肩接，在本装配目录生成 `meshes/fuselage_cut.stl`；其他显示几何复用原生组件。显示翼面与动力舱连续连接，机臂碰撞体保留基础模型的较短近似，当前不考察连接处接触精度。

## 两轴方向

外轴 yaw 绕组合模型水平 +X，外轴线经过吊舱局部 `(0,0,56) mm` 并穿过球心；内轴 pitch 绕随 yaw 转动的局部 −Y，负值使镜头从 +Z 转向局部 +X。yaw 范围 ±60°，pitch 范围 −80°～+30°；按镜头视线方向，左转为负、右转为正。底环水平、底座朝下，球头一侧为载机 +X；基础组件已将 CAD −X 转到此安装方向，装配不再额外旋转。

动态装配显式设置 `mount_dynamic=true`。基础吊舱在 `mount_link` 启用Gazebo速度检查，优先将实际世界角速度转换到当前安装局部系供TRACK前馈；仅速度数据不可用时使用35 Hz低通和跳变拒绝的姿态差分。默认100 Hz控制在4 ms物理步下保留周期余相，按8/12 ms交替运行，不因周期取整降为约83.3 Hz；每个物理状态至多运行一次。IDLE保持停车，不承担主动稳像。

零位沿用 CAD 原始 0°，镜头朝机头 +Z，运动链为 `Rx(yaw) Ry(-pitch)`，方向为 `(-sin(pitch),-sin(yaw)cos(pitch),cos(yaw)cos(pitch))`。pitch=0° 时 yaw 改变视线方向，球心不会绕底环中心公转。安装水平时光轴中心最低约高于水平面 5°；pitch=−80°、yaw=±60° 时水平视线角仅约±8.7°。机体倾转后安装轴随机体变化；模型、插件、网关和地面站均使用协议 v7、`motion_head_joint_right_positive`。静态生成器读取基础 SDF 的 link home pose、关节轴和枢轴，不对新 CAD 另套偏航公式。相机球心局部为 `(15,0,56) mm`，组合坐标为 `(15,0,345) mm`；视频相机沿球头 +Z 偏移 23.7 mm，零位图像右向 −Y、图像上向 −X，极端姿态下如实显示罩壳遮挡。

## 造型预览

以下命令在仓库根目录执行，需要 Python 3、NumPy、Gazebo Sim；离线评审图还需要 Matplotlib。

```bash
# 整机；默认 yaw=0、pitch=0
sh assembledmodels/rocket_two_axis_pod/tests/run.sh overall

# 吊舱近景；镜头朝 +X 上方 10°，方便检查支架与罩壳
sh assembledmodels/rocket_two_axis_pod/tests/run.sh close 0 -80

# 组合姿态 yaw=45°、pitch=-45°；或改为 close 0 30 检查另一俯仰端点
sh assembledmodels/rocket_two_axis_pod/tests/run.sh close 45 -45

# 重新生成入库的零位造型与评审图
python3 assembledmodels/rocket_two_axis_pod/tools/generate_model.py
python3 assembledmodels/rocket_two_axis_pod/tools/render_views.py --poses

# 原生资产继承、裁切网格、预览配置与机械角检查
python3 -m unittest discover -s assembledmodels/rocket_two_axis_pod/tests -p 'test_*.py'
```

启动器每次生成独立 `/tmp/rocket-two-axis-pod.*`运行副本和Gazebo分区。整机实体 `ROCKET_TWO_AXIS_POD`位于 `(0,0,0.225) m`，尾端离地15 mm；安装面世界z=0.514 m、最高点z=0.605 m，零位相机球心世界坐标 `(0.015,0,0.570) m`。橙色无碰撞定位箭头 `ROCKET_POD_LOCATION_MARKER`位于-X侧，箭杆中心 `(-0.20,0,0.55) m`。预览模型本身也无碰撞体；实体树聚焦或 `close`入口可观察吊舱。

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

动态生成器读取原生基础模型并加入吊舱。当前基础 `base_link`质量4.50 kg、重心z=+0.0331777777777778 m，完整惯性张量关于其自身重心定义；基础机质量4.76 kg，加0.24 kg吊舱得到组合质量5.00 kg。顶部显示和碰撞截短不按圆柱比例扣除基础质量或重算惯性。原生各link的质量、惯性位姿、惯性张量、传感器和电机物理位置均按源文件继承。

动态生成器的保留机身碰撞圆柱长0.499 m，范围z=-0.210～+0.289 m、中心z=+0.0395 m；惯性中心独立配置。动态结构含12个link、11个joint，固定安装关节连接 `base_link`和 `mount_link`，吊舱真实关节与组件结构对应。

基础机合成重心为 `(0,0,0.030) m` FLU，组合体在吊舱关节零位的固定名义重心为 `(-0.000176261396,-0.000000003426,0.044398504845) m` FLU。PX4位置参数从基础机重心参考平移至组合名义重心参考：`p组合=p基础+(+0.000176261396,-0.000000003426,+0.014398504845) m` FRD。生成器仅派生四旋翼的12个 `CA_ROTOR*_P{X,Y,Z}` 与IMU/GPS的6个 `EKF2_*_POS_{X,Y,Z}` 位置参数，物理部件不随参考点平移。

| 位置参数 | 基础机FRD值 | 组合体FRD值 |
| --- | --- | --- |
| 各旋翼PX | 原生值 | 原生值+0.000176261396 m |
| 各旋翼PY | 原生值 | 原生值−3.426×10⁻⁹ m |
| 各旋翼PZ | +0.095 m | +0.109398504845 m |
| IMU/GPS X | 0 | +0.000176261396 m |
| IMU/GPS Y | 0 | −3.426×10⁻⁹ m |
| IMU/GPS Z | +0.015 m | +0.029398504845 m |

其余PX4参数从原生机体文件继承，模型绑定和输出编号对应装配接口；5.00 kg悬停参数为0.1829004643346519。组合机体继承基础机的高机动限制：水平加速度15 m/s²、向上12 m/s²、向下6 m/s²，自动/最大jerk为30/40 m/s³；完整飞行验收的巡航和冲刺速度设定均采用8 s线性爬升。标准真值工具从快照计算固定零关节名义重心，组合偏移 `[-0.000176261396,-0.000000003426,0.044398504845]` m FLU；按模型姿态旋转偏移后转换ENU→NED并差分速度，不计算运动关节的瞬时重心。

装配参数 `dynamic.body_mass_kg=null` 表示完整继承原生机身质量；该参数和 `--body-mass-kg` 只能省略或与原生值相同，不支持独立覆盖质量。`dynamic.airframe_id`、`dynamic.topic_prefix` 配置运行接口。运行副本中的 `parameters.json` 保存实际各link质量、惯性与姿态、插件、PX4参数、测试设置和世界关键配置。`manifest.json`还保存安装位置、质量方法、悬停计算和源资产SHA-256。

`manifest.px4_reference`与运行快照的 `assembly.px4_reference` 保存同一参考点说明：`reference_pose=zero_joint_positions`、`base_cg_flu_m`、`assembly_nominal_cg_flu_m`、`offset_translation_frd_m`，以及18个原位置参数 `source_parameters` 和派生结果 `parameters`。这些字段用于核对生成机体文件的坐标来源。生成器仅读取基础组件、复制当前原生网格；地理场景入口见[场景说明](../../scenarios/geographic_rocket_two_axis_pod/README.md)。

## 完整飞行自验证

完整飞行入口使用实际运行快照、对应世界/实体与固定名义重心测量声明。当前5.00 kg组合归档完成30 s悬停、200 km/h连续30.000 s巡航、300 km/h连续10.000 s冲刺、减速、降落及停机，并通过ULog稳定性和实际镜头LOS门禁；49项源文件指纹标识运行快照，见[完整验证](docs/validation.md)。严格飞行门槛与[基础自验证](../../models/vehicles/rocket_racing_quad/tests/README.md)一致。

```bash
# 默认 headless、PX4 instance0，结果写入本次运行目录的 flight.json
bash assembledmodels/rocket_two_axis_pod/tests/run_flight_check.sh

# 指定 PX4 instance1 和结果路径；带界面把 headless 改为 sitl
PX4_INSTANCE=1 \
  bash assembledmodels/rocket_two_axis_pod/tests/run_flight_check.sh \
  headless /tmp/rocket-pod-instance1-flight.json
```

需要已构建的两轴控制插件与兼容 PX4、`pymavlink`、`pyulog`、NumPy和Gazebo Python绑定。`PX4_ROOT` 默认 `/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot`，`FLIGHT_PYTHON` 可指定解释器。`PX4_INSTANCE` 支持0～8，默认0，对应 MAVLink `udpin:0.0.0.0:(14540+实例号)`；同一实例的验收应顺序执行。入口检查相关端口并使用独立 Gazebo 分区、`etc` 和 rootfs。

运行目录为 `/tmp/rocket-two-axis-pod-flight.XXXXXX`，保留生成模型、参数快照、`manifest.json`、`execution.json`、`gz_partition.txt`、`result_path.txt`、Gazebo/PX4/飞行日志、ULog、IMU首帧、`camera-telemetry.jsonl`、`stability.json` 和 `exit_code.txt`。`execution.json` 记录实际实例、MAVLink连接、模式、分区、配置、飞行结果及稳定性结果路径；任一飞行或稳定性门禁失败都会成为入口退出码。

稳定性门禁只分析悬停阶段跳过前5秒后的正式窗口。它从ULog检查roll/pitch rate error的RMS、P95、最大值、5～35 Hz频带RMS及主峰，检查去趋势姿态、控制分配未实现、`unallocated_torque` 和四路电机触底。35 Hz上限为83.3 Hz ULog角速度记录率保留采样裕量，并覆盖已观察到的16～20 Hz振动。入口还必须收到吊舱插件实际发布的 `camera.quat_world`，并按运行相机内参计算LOS像素抖动；缺少该遥测会明确判定失败，不使用机体姿态或估算数据代替实际镜头姿态。门槛来自生成快照的 `stability_validation`，正式归档工具拒绝缺少该结果、放宽门槛或稳定性未通过的报告。

此入口验证组件世界中的飞行。该世界包含惯性导航等物理传感器系统，没有加载图像渲染的 `Sensors` 系统，因此 `camera.quat_world` 只验收实际光学姿态，不代表视频帧、地面站或目标跟踪通过。完整地理场景的飞行与视频跟踪需要独立证据。

配套地面站可发送仅含11个转换字段的艾晟UDP JSON，框架角直接采用上述机械角，朝前俯仰值为负，并包含最终像素跟踪框宽高 `tracking_gateW/H`；网关独立状态输出仍为完整原始遥测。发送、倍率及字段定义见[地面站说明](../../models/gimbals/two_axis_pod/ground_station/README.md)。

## 多机接入

每架载机的吊舱分别配置飞机ID、UDP控制/遥测端口及RTSP URL；同机可共用RTSP 8555，通过 `/two_axis_pod_<aircraft_id>` 路径区分。控制端口由网关启动配置显式指定，不随飞机ID自动改变。逐机地址表、地面站连接示例和PX4实例隔离要求见[多机集成说明](../../scenarios/geographic_rocket_two_axis_pod/README.md#多机集成的地址与端口对应)。

动态生成器支持 `--topic-prefix` 配置插件与相机话题前缀，生成资产的模型名仍为 `rocket_two_axis_pod`。多机运行时的实体命名、每机话题及PX4绑定由场景启动器负责；现有地理场景入口仍为单机，不能仅改变 `--aircraft-id` 后双开。

## 验证范围

当前静态装配为24个visual、无碰撞体，包络z=-0.210～+0.380 m，安装面z=0.289 m、肩部起点z=0.277 m。组合离线单测和动态检查覆盖原生继承、名义重心、CA/EKF平移及真值测量声明，并同步机械姿态 `(yaw,pitch)=(0,0)/(0,-80)/(45,-45)/(0,30)/(-60,-80)/(60,-80)` 的镜头方向与两轴运动学。实际通过数量以本次运行结果为准。

组合完整飞行归档：悬停30 s最大漂移0.050719 m，200 km/h巡航连续30.000 s、300 km/h冲刺连续10.016 s，落地停机确认；悬停角速度、5～35 Hz频谱、控制分配、电机触底和实际镜头LOS全部通过。归档包含固定名义重心声明、运行快照、49项当前源指纹及原始采样，已由正式报告工具核验，见[验证结果](docs/validation.md)。当前地理场景补充运行另通过一次往返、70仿真秒机载CSRT跟踪及降落，完整RTSP身份/帧号绑定仍按[地理场景说明](../../scenarios/geographic_rocket_two_axis_pod/docs/validation.md)单独验收。组合静态外形供人工评审；最高稳定速度对照仍需单独验证。

- [整体三视图与斜视图](docs/model_views.png)、[头部尺寸与轴向](docs/head_detail.png)、[四组机械姿态对比](docs/mechanical_poses.png)：由实际 SDF 视觉几何生成的离线评审图，不是 Gazebo 截图。

以上图像是静态评审资料，不替代人工确认或动态证据。所附组件完整飞行的摘要、曲线和原始记录见[验证结果](docs/validation.md)；[地理场景验收](../../scenarios/geographic_rocket_two_axis_pod/docs/validation.md)属于独立范围，不能据组件飞行归档推断视频/跟踪已经通过。

动态装配从基础吊舱继承两关节 ±20°/s 物理速度上限和控制器限速，无上层覆盖；该继承检查、整机飞行和70仿真秒快速机载CSRT目标跟踪均已通过。
