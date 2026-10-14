# 火箭穿越机（rocket_racing_quad）

用于 PX4 SITL + Gazebo 的四旋翼火箭穿越机。机头朝上起飞，旋翼推力沿机身轴线；加速时整机倾转，使机头接近水平朝前。几何、质量、动力、气动、PX4 和飞行验收设置集中在 [parameters.json](parameters.json)，修改方法见[参数说明](docs/parameters.md)。

## 目标与当前尺寸

- 机头到机尾轴向长度 **L = 0.450 m**，四个动力舱呈 X 型分布，沿四个电机对角方位连接的长三角翼面。
- 对角电机轴距 **0.381 m（15 in）**，各电机 x/y 偏移约 ±0.1347 m；桨径 **0.254 m（10 in）**。
- 巡航水平速度达到 **200 km/h**；冲刺达到 **300 km/h 并连续保持至少 10 s**，均保持高度稳定。
- 默认验收还采用巡航连续 30 s、悬停 30 s 漂移 ≤1 m、高度容差 ±2 m。这些时长和容差可调，其中巡航时长与高度容差是测试设置。

| 当前配置 | 数值 | 依据 |
| --- | --- | --- |
| 机身长度 / 直径 | 0.450 / 0.090 m | 长度为已确认要求，直径为建模估计 |
| 动力舱长度 / 直径 | 0.090 / 0.080 m | 建模估计 |
| 机身 / 单动力舱 / 单桨质量 | 1.100 / 0.040 / 0.025 kg | 仿真估计 |
| 总质量 | 1.360 kg | 上述质量合计 |
| 最大桨转速 | 1250 rad/s | 仿真估计 |
| 解锁怠速 / 未解锁转速 | 50 / 0 rad/s | 正怠速保证四个电机通过ESC解锁转速检查，检查保持启用 |
| 单桨推力系数 | 1.9×10⁻⁵ N·s²/rad² | 仿真估计，推力 = 系数 × 转速² |
| 四桨静态峰值推力 | 118.75 N，推重比约 8.9 | 由配置计算，不是实测 |
| 阻力系数 / 参考面积 | 0.73 / 0.0177 m² | 仿真估计，等效 CdA = 0.012921 m² |

尚未指定实际电机、螺旋桨、电池或风洞数据。现有估计参数与简化动力学用于仿真调试，速度目标是否达到由飞行验收确定。当前没有电池放电或转速随电压下降的模型。

公开候选的布局、尺寸、许可证和接口适配依据见[复用评估](docs/reuse.md)。

## 模型与资产

```text
rocket_racing_quad/
├── parameters.json       # 参数源
├── model.sdf             # 生成的动力学、传感器及插件配置
├── model.config
├── meshes/               # 参数化生成的 STL
├── tools/                # 网格/模型生成器和世界模板
├── px4/                  # 生成的 PX4 机体启动文件
├── tests/                # 自验证世界、启动器、飞行验收
└── docs/                 # 参数说明、项目状态
```

机体使用 Gazebo FLU 坐标（x 前、y 左、z 上），机头指向 +z，四桨推力 +z。生成 PX4 机体文件时，将电机位置转换到 FRD 坐标（y/z 反号）。机身与动力舱惯量按圆柱近似；翼面和翼内连接臂的质量并入机身质量。碰撞体使用圆柱和旋翼旋转圆盘包络，外形网格用于显示。

桨盘中心 z=-0.185 m，即距机尾40 mm；电机整流罩中心 z=-0.120 m，位于桨盘上方。机体不包含参考图中的独立展示支架，机尾落地接触由机身圆柱碰撞体近似。

每桨使用 `MulticopterMotorModel`，气动使用 `AdvancedLiftDrag`，前向沿机身 +z、气动中心在机体原点，升力系数为零，采用常值阻力系数。该简化模型没有独立尾翼气动力、失速辨识或随速度变化的桨效率曲线。IMU、气压、磁力计、卫星定位传感器均位于 `base_link`。

PX4输出映射使用50 rad/s正解锁怠速，未解锁时四电机停转。悬停推力按当前质量和非零怠速映射计算，默认 `MPC_THR_HOVER≈0.0944385`；改变质量或动力配置时应同步重算，公式见[参数说明](docs/parameters.md)。装配模型继承基础动力设置。

## 生成、启动与测试

以下命令均从仓库根目录运行。需要 Python 3、NumPy、Gazebo Sim，以及已构建的兼容 PX4 SITL；飞行脚本还需要 `pymavlink`；单入口真值验收还需要 Gazebo Python transport/messages 模块。

```sh
# 一条命令完成启动、Gazebo真值验收与清理
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless

# 更新模型目录中的网格、SDF、世界和PX4机体文件
python3 models/vehicles/rocket_racing_quad/tools/generate_model.py

# 只查看外形，无需PX4
sh models/vehicles/rocket_racing_quad/tests/run.sh view

# 单入口自动启动、飞行验收、停止仿真，结果默认保存在运行目录
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless

# 分步启动PX4和Gazebo；带界面使用sitl
sh models/vehicles/rocket_racing_quad/tests/run.sh headless

# 另一终端：使用启动器实际输出的配置路径
python3 models/vehicles/rocket_racing_quad/tests/flight_check.py \
  --config /tmp/rocket-racing.XXXXXX/models/rocket_racing_quad/parameters.json \
  --log /tmp/rocket-flight-review.json
```

`XXXXXX` 是路径示例，必须替换成启动器输出的实际目录。启动器每次从配置生成独立运行资产，复制已构建 PX4 的 `etc`，使用独立 rootfs 和 Gazebo 通信分区；不修改 PX4 源码。默认 PX4 工作区为 `/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot`，可通过 `PX4_ROOT` 指定其他路径。生成的机体 ID 默认 4022，模型名为 `gz_rocket_racing_quad`。

按 Ctrl+C 停止启动器及仿真，日志保留在运行目录。调整模型、PX4 或仿真配置后应重启启动器；仅修改源配置不会改变已经启动的运行副本。完整操作与判定见[自验证说明](tests/README.md)。

```sh
# 生成一致性、几何、配置映射检查与飞行脚本回归
python3 models/vehicles/rocket_racing_quad/tests/check_model.py
python3 -m unittest discover -s models/vehicles/rocket_racing_quad/tests -p 'test_*.py' -v

# 导出机身网格多视图（需要matplotlib）
python3 models/vehicles/rocket_racing_quad/tools/render_views.py
```

[多视图预览](docs/model_views.png) 是由网格和配置绘制的静态预览，不是Gazebo运行截图，也不代表飞行或碰撞验收通过。

## 验证状态

当前模型采用[外形参考图](docs/reference.png)所示的低位电机、下置桨和长三角翼面布局。静态检查与SDF校验通过；80 mm 动力舱外形已通过人工验收，当前配置完整飞行验收待复测，结果见[验证说明](docs/validation.md)。当前状态见[项目状态](docs/status.md)。

## 参数与验证证据

- [参数说明](docs/parameters.md)：字段单位、依赖关系、修改与复测方法。
- [公开模型复用评估](docs/reuse.md)：三个候选仓库的来源、许可证与适配结论。
- [几何多视图](docs/model_views.png)：供人工外形确认，非Gazebo运行截图。
- [验证说明](docs/validation.md)：当前配置的验证状态。

`AdvancedLiftDrag` 的指定前向/上向平面用于简化气动计算，本模型不应解释为已验证的全姿态三维气动模型。当前高速验收覆盖无风平面世界中的直线航段；改变航迹、风场或气动参数后应复测。
