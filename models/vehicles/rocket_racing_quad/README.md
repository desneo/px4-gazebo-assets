# 火箭穿越机（rocket_racing_quad）

用于 PX4 SITL + Gazebo 的四旋翼火箭穿越机。机头朝上起飞，旋翼推力沿机身轴线；加速时整机倾转，使机头接近水平朝前。基础模型标准启动入口以本目录的 [model.sdf](model.sdf)、`meshes/`、`px4/` 和 [tests/world.sdf](tests/world.sdf) 为准，复制为独立运行资产。质量、重心、惯性和动力直接维护在 SDF 中，PX4 参数直接维护在机体文件中；[parameters.json](parameters.json) 提供测试设置与启动等待期限，其物理字段不影响该入口或当前动态装配，仅供显式离线生成器使用。配置边界见[参数说明](docs/parameters.md)。

## 目标与当前配置

- 模型总高度（沿机身轴向）**L = 0.590 m**，全部显示网格与碰撞体的 z 包络均在 **-0.225～+0.365 m**；四个动力舱呈 X 型分布。
- `base_link` 原点位于模型局部 z=0，机身几何轴向中点为 z=+0.070 m；机身碰撞圆柱长度0.590 m、中心 z=+0.070 m。重心和惯性坐标继续相对 `base_link` 定义。
- 当前 SDF 的电机 x/y 偏移为 **±0.2086 m**，对角电机轴距约 **0.590010 m**；桨径 **0.254 m（10 in）**。
- 四片显示翼面延伸至动力舱轴线的径向位置约0.295005 m，并与动力舱连续连接；翼端高度 z=-0.140～-0.100 m、厚度0.004 m。机臂碰撞体仍使用径向末端0.1905 m的近似，当前不考察连接处的接触精度；显示网格不改变显式配置的质量、惯性与简化气动力。
- 目标为巡航水平速度 **200 km/h**、冲刺 **300 km/h 连续至少 10 s**，均保持高度稳定。默认测试还包括巡航连续 30 s、悬停 30 s 漂移 ≤1 m、高度容差 ±2 m，时长和容差可调。

| 当前原生配置 | 数值 | 配置位置或含义 |
| --- | --- | --- |
| 机身长度 / 直径 | 0.590 / 0.090 m | SDF 机身碰撞体 |
| 动力舱长度 / 直径 | 0.090 / 0.080 m | SDF 动力舱碰撞体 |
| 机身 / 单动力舱 / 单桨质量 | 7.440 / 0.040 / 0.025 kg | 各 link 的 `inertial/mass` |
| 整机总质量 | 7.700 kg | 所有 link 质量合计 |
| 机身重心 | `(0, 0, -0.094) m` | `base_link/inertial/pose`，相对机身坐标原点 |
| 整机重心 | 约 `(0, 0, -0.095722) m` | 由各 link 质量及位置合成 |
| 机身惯性主对角项 | `(0.12959890706, 0.129623371, 0.17530315186) kg·m²` | 关于机身重心；三个积惯量为 0 |
| 四桨最大转速 / PX4 四路上限 | 1570 / 1570 rad/s | SDF 电机插件 / `SIM_GZ_EC_MAX1`～`MAX4` |
| 解锁怠速 / 未解锁转速 | 50 / 0 rad/s | PX4 电机输出映射 |
| 单桨推力系数 / 反扭矩系数 | 8.9119×10⁻⁵ N·s²/rad² / 0.02863386 m | SDF 电机插件，`T=kω²` |
| 四桨静态峰值推力 | 约 878.68 N，推重比约 11.64 | 由 SDF 计算，不是实测 |
| 阻力系数 / 参考面积 | 0.73 / 0.0177 m² | 等效 CdA = 0.012921 m² |

PX4 的电机位置、反扭矩系数和四路最大转速与当前 SDF 配套：位置按 FLU→FRD 转换，`CA_ROTOR*_PZ=0.185 m` 保持模型原点参考；`KM` 的0/1路为+0.02863386 m，2/3路为-0.02863386 m。当前配置 `MPC_THR_HOVER=0.0755504`；按7.70 kg总质量和当前映射静态计算的理论值约 **0.0727901**，该配置已通过当前无风组件世界的完整飞行自验证，静态理论值与实测配置应分别记录。其他工况或参数需对应验证。

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

机体使用 Gazebo FLU 坐标（x 前、y 左、z 上），机头指向 +z，四桨推力 +z。PX4 机体文件使用 FRD 坐标（y/z 反号）。`base_link` 原点与机身几何轴向中点相差0.070 m，位置与重心应按各自参考系读取。SDF 中的质量、惯性、惯性坐标系和气动参数独立配置，鼻锥外形长度不会自动重算这些数值。惯性矩必须对应所属 link 的重心与惯性坐标轴；当前机身7.44 kg加四个动力舱与四个旋翼的0.26 kg，整机为7.70 kg，几何尺寸不构成这些物理参数已按实物验证的依据。

桨盘中心 z=-0.185 m，即距机尾40 mm；电机整流罩中心 z=-0.120 m，位于桨盘上方。机体不包含独立展示支架，机尾落地接触由机身圆柱碰撞体近似。测试世界初始 `base_link` 原点 z=0.230 m，机尾离地0.005 m、顶部离地0.595 m；离地位置不改变模型0.590 m的本体高度。

每桨使用 `MulticopterMotorModel`，气动使用 `AdvancedLiftDrag`，前向沿机身 +z、气动中心在机体原点，升力系数为零，采用常值阻力系数。该简化模型没有独立尾翼气动力、失速辨识或随速度变化的桨效率曲线。IMU、气压、磁力计、卫星定位传感器均位于 `base_link`。

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

当前590 mm、7.70 kg配置于2026-10-20通过完整SITL自验证：悬停30 s最大漂移0.122833 m，200 km/h巡航连续30 s、300 km/h冲刺连续10 s，完成减速、降落与停机。测量使用Gazebo位姿真值，结果适用于当前无风组件世界；外形仍待人工确认，机臂连接处接触精度不在验收范围内。实际结果、原始记录及资产指纹见[验证说明](docs/validation.md)，当前状态见[项目状态](docs/status.md)。

`AdvancedLiftDrag` 的指定前向/上向平面用于简化气动计算，本模型不应解释为已验证的全姿态三维气动模型。改变航迹、风场或气动参数后应复测。
