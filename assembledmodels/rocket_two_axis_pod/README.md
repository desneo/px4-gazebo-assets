# 火箭穿越机与顶部两轴吊舱

静态组合造型已通过人工确认：总长 450 mm，吊舱替换火箭机纵轴顶部的一部分，各同名轴对齐。当前入库的 `model.sdf` 为静态造型评审模型，不包含 PX4、飞行动力学、活动关节控制、视频或锁定功能。

## 外形与装配

由 `models/vehicles/rocket_racing_quad` 与 `models/gimbals/two_axis_pod` 的视觉资产组合。两轴吊舱坐在火箭机纵轴顶部，替换原机头上段。机体轴向总长保持 **450 mm**；原模型的四翼、电机、桨叶及位置保持原尺寸。

- 火箭机坐标原点、机尾 `z=-225 mm` 不变，组合最高点 `z=+225 mm`。
- 吊舱安装面中心位于 `(0, 0, 134) mm`，采用零安装旋转，吊舱 +X、+Y、+Z 分别对齐火箭机同名轴。机头和四桨推力参考方向均为 +Z。
- 吊舱总高 91 mm、底环直径 84 mm、相机球直径 48 mm，均沿用组件实际几何。
- 原机头 `z=134～225 mm` 段被替换。保留机身在 `z=122～134 mm` 设置 12 mm 长过渡肩部，接到底环半径 42 mm；机身最大直径为 90 mm。顶部替换不切入翼根。
- 生成器只读取基础模型，在本装配目录生成截短机身网格，其他网格通过原组件资源路径复用。

## 两轴方向

外轴 yaw 绕组合模型 +Z，即火箭机纵轴，正值从 +X 转向 +Y；内轴 pitch 绕随 yaw 转动的局部 +Y，正值使镜头从 +Z 转向局部 +X。yaw 范围 ±90°，pitch 范围 −45°～+90°。

零位镜头朝机头 +Z，pitch=+90° 时朝 yaw 旋转后的 +X 侧方。镜头方向为 `(cos(yaw)sin(pitch), sin(yaw)sin(pitch), cos(pitch))`。pitch=0° 时镜头方向与外轴平行，yaw 无法改变视线方向。机体倾转后安装轴随机体变化，不等于世界偏航、俯仰轴。当前预览将机械姿态烘焙到视觉位姿，不提供实时关节操作；零位球心为 `(15, 0, 190) mm`。

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
```

启动器每次生成独立 `/tmp/rocket-two-axis-pod.*` 运行副本和 Gazebo 分区。整机实体名为 `ROCKET_TWO_AXIS_POD`，位置 `(0,0,0.24) m`；底环在世界 `z=0.374 m`，最高点 `z=0.465 m`。橙色定位箭头实体名为 `ROCKET_POD_LOCATION_MARKER`，位于整机 −X 侧、世界 `z≈0.41 m`，指向顶部吊舱。箭头和预览模型均无碰撞体。可在实体树搜索整机并聚焦；`close` 入口直接配置吊舱近景。

浏览相机默认近裁剪距离为 **0.005 m**，鼠标灵敏度为 **0.1**。滚轮缩放时请把鼠标放到吊舱实体表面，避免指向远处背景；用右键缓慢拖动微调。启动器在 GUI 服务就绪后请求设置灵敏度，结果在运行目录 `camera.log`；仅在 GUI 正常加载且同一 Gazebo 分区中服务可达时生效。可在另一终端使用启动器输出的分区设置：

```bash
GZ_PARTITION=<启动器输出的分区> gz service \
  --service /gui/camera/view_control/sensitivity \
  --reqtype gz.msgs.Double --reptype gz.msgs.Boolean \
  --timeout 2000 --req 'data: 0.1'
```

`parameters.json` 中 `view.mouse_sensitivity`、`view.near_clip_m`、`view.far_clip_m` 和 `view.camera_pose` 配置浏览相机，重启后生效。吊舱视频相机参数属于基础组件 `models/gimbals/two_axis_pod/parameters.json`，与浏览相机独立；本静态预览未加载视频传感器。

## 动态装配

`tools/generate_dynamic_model.py` 生成包含真实关节、相机、传感器与 PX4 动力接口的运行资产，默认写入本目录忽略入库的 `runtime/rocket_two_axis_pod/`；不会覆盖静态 `model.sdf`。默认 airframe ID 为 4023，模型为 `rocket_two_axis_pod`，话题前缀为 `/rocket_two_axis_pod`。

```bash
python3 assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py
python3 assembledmodels/rocket_two_axis_pod/tools/check_dynamic_model.py
```

动态装配保留 12 个 link、11 个 joint，以固定关节连接 `base_link` 与 `mount_link`，吊舱活动关节、镜头姿态与组件结构一致。机身碰撞和均匀圆柱惯量按保留的 359 mm 长度截短，质心放在截短中心；默认机身质量按保留长度比例估计为 0.877555556 kg，整机质量为 1.377555556 kg。其余动力舱、旋翼、吊舱质量沿用组件估计值，不代表实物标定。解锁怠速默认为 50 rad/s，四桨总静推力为 0.19 N，约为重量的 1.4%；未解锁输出显式设为 0。`THR_MDL_FAC=1` 时，PX4 先把推力比例开方，再将输出线性映射到 `[怠速, 最大转速]`；生成器结合整机质量和该映射计算悬停推力比例，使实际悬停推力与重量匹配。

装配参数 `dynamic.body_mass_kg=null` 表示按保留长度比例估计，可用明确质量覆盖；解锁怠速继承基础火箭模型的 `motor.idle_velocity_rad_s`，悬停推力按组合总质量和电机输出映射计算；`dynamic.airframe_id`、`dynamic.topic_prefix` 配置运行接口。生成的 `manifest.json` 记录装配参数、怠速、实际悬停转速与推力比例、质量估计方法和源资产指纹。生成器仅读取基础组件，复制运行所需网格，不修改基础模型。完整地理世界的飞行与跟踪入口见[场景说明](../../scenarios/geographic_rocket_two_axis_pod/README.md)。

## 多机接入

每架载机的吊舱分别配置飞机ID、UDP控制/遥测端口及RTSP URL；同机可共用RTSP 8555，通过 `/two_axis_pod_<aircraft_id>` 路径区分。控制端口由网关启动配置显式指定，不随飞机ID自动改变。逐机地址表、地面站连接示例和PX4实例隔离要求见[多机集成说明](../../scenarios/geographic_rocket_two_axis_pod/README.md#多机集成的地址与端口对应)。

动态生成器支持 `--topic-prefix` 配置插件与相机话题前缀，生成资产的模型名仍为 `rocket_two_axis_pod`。多机运行时的实体命名、每机话题及PX4绑定由场景启动器负责；现有地理场景入口仍为单机，不能仅改变 `--aircraft-id` 后双开。

## 验证范围

模型、整机预览世界与近景世界通过 `gz sdf -k`。实际视觉几何含 34 个 visual，零位轴向包络为 `[-225, 225] mm`，长度 450 mm。六组机械姿态 `(yaw,pitch)=(0,0)/(0,90)/(45,45)/(0,-45)/(-90,90)/(90,90)` 的镜头实际旋转方向与两轴运动学一致，生成的默认 SDF 与源配置一致。

- [Gazebo 整机渲染](docs/gazebo_overall.png)与[Gazebo 机头渲染](docs/gazebo_head.png)：Gazebo 服务器相机实际渲染的静态模型。
- [整体三视图与斜视图](docs/model_views.png)、[头部尺寸与轴向](docs/head_detail.png)、[四组机械姿态对比](docs/mechanical_poses.png)：由实际 SDF 视觉几何生成的离线评审图，不是 Gazebo 截图。

以上静态图不作为飞行或跟踪验收依据。动态装配在完整地理场景中通过当前 80 m 高度、15 m/s、300 m 航段的两次完整往返、降落和 70 仿真秒真实图像跟踪自动检查，结果与限制见[组合体验收](../../scenarios/geographic_rocket_two_axis_pod/docs/validation.md)。
