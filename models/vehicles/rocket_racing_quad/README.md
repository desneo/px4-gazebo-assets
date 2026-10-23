# 火箭穿越机（rocket_racing_quad）

用于 PX4 SITL + Gazebo 的四旋翼火箭穿越机。机头朝上起飞，旋翼推力沿机身轴线；加速时整机倾转，使机头接近水平朝前。基础模型标准启动入口以本目录的 [model.sdf](model.sdf)、`meshes/`、`px4/` 和 [tests/world.sdf](tests/world.sdf) 为准，复制为独立运行资产。质量、重心、惯性和动力直接维护在 SDF 中，PX4 参数直接维护在机体文件中；[parameters.json](parameters.json) 提供测试设置与启动等待期限，其物理字段不影响该入口或当前动态装配，仅供显式离线生成器使用。配置边界见[参数说明](docs/parameters.md)。

## 目标与当前配置

- 模型总高度（沿机身轴向）**L=0.590 m**，全部显示网格与碰撞体的轴向包络为 **z=-0.210～+0.380 m**。尾端为直径68 mm平底，尾部54 mm沿轴向平滑过渡至直径90 mm主身。
- `base_link`原点为局部z=0，机身几何轴向中点和碰撞圆柱中心为z=+0.085 m；碰撞圆柱长0.590 m、直径0.090 m。整机重心为z=+0.030 m，`base_link`自身重心为z≈+0.033177778 m。
- 当前 SDF 的电机 x/y 偏移为 **±0.2086 m**，对角电机轴距约 **0.590010 m**；桨径 **0.254 m（10 in）**。
- 四片主翼沿45°/135°/225°/315°布置，厚4 mm，翼端径向位置约0.295005 m、z=-0.020～+0.020 m，与动力舱连续连接。另有四片独立小尾翼，沿0°/90°/180°/270°布置，厚4 mm、径向最大100 mm，z=-0.210～-0.105 m。机臂碰撞体仍保留径向末端0.1905 m的较短近似。
- 目标为巡航水平速度 **200 km/h**、冲刺 **300 km/h 连续至少 10 s**，均保持高度稳定。默认测试还包括巡航连续 30 s、悬停 30 s 漂移 ≤1 m、高度容差 ±2 m，时长和容差可调。

| 当前原生配置 | 数值 | 配置位置或含义 |
| --- | --- | --- |
| 机身长度 / 直径 | 0.590 / 0.090 m | SDF 机身碰撞体 |
| 动力舱长度 / 直径 | 0.090 / 0.080 m | SDF 动力舱碰撞体 |
| 机身 / 单动力舱 / 单桨质量 | 4.500 / 0.040 / 0.025 kg | 各 link 的 `inertial/mass` |
| 整机总质量 | 4.760 kg | 所有 link 质量合计 |
| 机身重心 | `(0, 0, +0.033177778) m` | `base_link/inertial/pose`，相对link原点 |
| 整机重心 | `(0, 0, +0.030) m` | 由全部link质量和位置合成 |
| 机身惯性主对角项 | `(0.13985490706, 0.139243371, 0.19049315186) kg·m²` | 关于机身重心；三个积惯量为 0 |
| 四桨最大转速 / PX4 四路上限 | 800 / 800 rad/s | SDF 电机插件 / `SIM_GZ_EC_MAX1`～`MAX4` |
| 解锁怠速 / 未解锁转速 | 50 / 0 rad/s | PX4 电机输出映射 |
| 单桨推力系数 / 反扭矩系数 | 8.9119×10⁻⁵ N·s²/rad² / 0.02863386 m | SDF 电机插件，`T=kω²` |
| 四桨静态峰值推力 | 约 228.14 N，推重比约 4.89 | 由 SDF 计算，不是实测 |
| 阻力系数 / 参考面积 | 0.73 / 0.0177 m² | 等效 CdA = 0.012921 m² |

四个动力舱中心z=0、桨盘z=-0.065 m，x/y幅值0.2086 m。PX4控制分配参考整机重心，FRD旋翼PZ=+0.095 m，IMU/GPS偏置 `(0,0,+0.015) m`。参数文件配置 `MPC_THR_HOVER=0.1727721546023802`，对应4.76 kg整机约361.744 rad/s的静态悬停转速；该值高于PX4内部0.05悬停下限。

当前高机动限制为 `MPC_ACC_HOR=15 m/s²`、`MPC_ACC_HOR_MAX=15 m/s²`、`MPC_ACC_UP_MAX=12 m/s²`、`MPC_ACC_DOWN_MAX=6 m/s²`，`MPC_JERK_AUTO=30 m/s³`、`MPC_JERK_MAX=40 m/s³`。完整飞行验收的巡航和冲刺速度设定均采用8 s线性爬升。

尚未指定实际电机、螺旋桨、电池或风洞数据。现有参数与简化动力学用于仿真调试，速度目标是否达到由当前配置的飞行验收确定。当前没有电池放电或转速随电压下降的模型。公开候选的布局、尺寸、许可证和接口适配依据见[复用评估](docs/reuse.md)。

## 模型与资产

```text
rocket_racing_quad/
├── parameters.json       # 测试与启动设置；物理字段供离线生成器使用
├── model.sdf             # 当前动力学、重心、惯性、传感器及插件配置
├── model.config
├── meshes/               # 当前显示网格
├── tools/                # 运行副本准备工具、离线生成器和世界模板
├── px4/                  # 当前 PX4 机体启动文件
├── tests/                # 当前自验证世界、启动器、飞行验收
└── docs/                 # 参数说明、项目状态
```

机体使用Gazebo FLU坐标（x前、y左、z上），机头和四桨推力沿+z；PX4使用FRD坐标（y/z反号）。几何轴向中点、link原点、各link重心和整机重心分别定义。机身质量4.50 kg加四个动力舱与旋翼0.26 kg，总质量4.76 kg；机身惯性张量关于其自身重心定义，显示几何、质量、惯性与气动为独立配置，不由网格自动重算。

桨盘中心z=-0.065 m，距平底尾端145 mm；动力舱中心z=0，位于桨盘上方。机体没有独立展示支架；碰撞仍使用圆柱与圆盘等简化包络，不按平底和小尾翼显示网格生成精确碰撞。测试世界初始 `base_link`原点z=0.215 m，尾端离地5 mm、机头顶部离地0.595 m。

每桨使用 `MulticopterMotorModel`，气动使用 `AdvancedLiftDrag`，前向沿机身+z，气动力作用点 `cp=(0,0,+0.015) m`，升力系数为零、阻力系数为常值。IMU、气压、磁力计与卫星定位传感器同在 `base_link`局部z=+0.015 m。当前没有独立尾翼气动力、失速辨识或随速度变化的桨效率曲线。

## 启动与测试

以下命令均从仓库根目录运行。需要 Python 3、NumPy、Gazebo Sim，以及已构建的兼容 PX4 SITL；飞行脚本还需要 `pymavlink`；单入口真值验收还需要 Gazebo Python transport/messages 模块。

```sh
# 只查看当前原生资产，无需 PX4
sh models/vehicles/rocket_racing_quad/tests/run.sh view

# 启动 PX4 和 Gazebo；带界面使用 sitl
sh models/vehicles/rocket_racing_quad/tests/run.sh headless

# 自动启动、Gazebo 真值飞行验收与清理
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless

# 另一终端：使用启动器实际输出的配置快照路径
python3 models/vehicles/rocket_racing_quad/tests/flight_check.py \
  --config /tmp/rocket-racing.XXXXXX/models/rocket_racing_quad/parameters.json \
  --log /tmp/rocket-flight-review.json
```

`XXXXXX` 必须替换成启动器输出的实际目录。`tools/prepare_runtime.py` 复制当前原生资产，提取运行参数、各 link 的质量/重心/惯性和文件指纹，写入运行目录的 `parameters.json`。启动器复制已构建 PX4 的 `etc`，装入本次复制的机体文件，并使用独立 rootfs 和 Gazebo 通信分区。默认 PX4 工作区为 `/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot`，可通过 `PX4_ROOT` 指定其他路径；当前机体 ID 为 4022，模型名为 `gz_rocket_racing_quad`。

启动器的第二个可选 JSON 参数仅替换 `flight_test` 和 `simulation.imu_wait_timeout_s`；不会用其中的质量、几何、动力、PX4 或世界字段覆盖原生文件。按 Ctrl+C 停止启动器及仿真，日志保留在运行目录。原生文件变化后须重启，已有运行副本不会随源文件变化。完整操作与判定见[自验证说明](tests/README.md)。

```sh
# 原生资产静态检查与离线回归
python3 models/vehicles/rocket_racing_quad/tests/check_model.py
python3 -m unittest discover -s models/vehicles/rocket_racing_quad/tests -p 'test_*.py' -v

# 显式离线生成到独立目录；输出基于 JSON，不代表当前手调 SDF
python3 models/vehicles/rocket_racing_quad/tools/generate_model.py \
  --output-dir /tmp/rocket-export/rocket_racing_quad

# 离线生成器配置的多视图预览（需要 matplotlib）
python3 models/vehicles/rocket_racing_quad/tools/render_views.py
```

离线生成器可导出参数化网格、SDF、世界和 PX4 文件；默认 JSON 的物理字段与当前原生文件不同。`--output-dir` 必填，且禁止输出到当前模型目录或其父/子路径，以保护手调资产。`render_views.py` 同样按 JSON 生成几何，[多视图预览](docs/model_views.png) 不能用作当前外移电机 SDF 的外形或碰撞验收证据。

## 验证状态

当前590 mm平底、四主翼与四小尾翼基础外形已获人工确认。4.76 kg配置已通过完整无风组件世界自验证：悬停30 s最大漂移0.083901 m，200 km/h连续30.000 s巡航、300 km/h连续10.000 s冲刺，完成减速、降落与停机。真值测量点为快照质量加权得到的固定零关节名义重心 `(0,0,+0.030) m`，不是运动关节的瞬时重心。完整记录和15项源指纹见[验证说明](docs/validation.md)。惯性仍是未实物标定的显式仿真值；组合使用其自身重心与报告独立验收。

`AdvancedLiftDrag` 的指定前向/上向平面用于简化气动计算，本模型不应解释为已验证的全姿态三维气动模型。改变航迹、风场或气动参数后应复测。
