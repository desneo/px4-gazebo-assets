# 参数与上游来源

`parameters.json` 是生成器输入。质量为整机总质量，生成器将基体设为 49 kg，桨/舵面/轮子等共 1 kg。惯量 `[18,32,42] kg·m²` 为仿真估计。`length_m` 与 `span_m` 对实际几何和关节位置按标称尺寸线性缩放；气动面积、平均弦长、惯量、桨径和推进器参数独立配置，改变尺寸时需同步重新评估这些数据。

气动有效参考面积 1.625 m²、参考弦长 0.65 m 为通用仿真参数，独立于三角翼美术网格的几何面积和轮廓；不能将其解释为外观参考实机的气动数据。`aero` 覆盖 AdvancedLiftDrag 的升力、阻力、俯仰稳定导数；其余无量纲导数保留上游估计。空气密度 1.2041 kg/m³，测试世界无风；PX4 模拟空速由估计器机体前向速度构造差压，不读取风场，因此本验收同时使用 Gazebo 运动真值。

推力遵循原生 MulticopterMotorModel 的 `motor_constant × omega²`，`motor_max_rad_s` 同步写入电机插件与 PX4 `SIM_GZ_EC_MAX1`。`propeller_diameter_m=0.70` 为通用推进器的标称参考尺寸，外观桨为近似美术几何。通用推进器轴指向机体前方 +X，物理链接位于机体 `(-0.72,0,0.10) m`；外观尾桨通过纯视觉变换沿同一转轴向机尾偏移，不改变推进器物理位置或推力参数。满油门参数用于近目标终端速度；不是直接写入速度状态或强制限速。推进器未包含真实桨随前进比衰减的完整台架曲线。

机体系为 Gazebo FLU：+X 机头、+Y 左翼、+Z 上方。PX4 采用原生 `gz_bridge` 坐标变换。电机通道 101；舵面通道 201/202 左右副翼、203 升降舵、204 方向舵；205/206 保留未使用襟翼接口。控制机构、传感器噪声、插件名称来自已有 PX4 资产。

上游：本机 `../PX4-Autopilot/Tools/simulation/gz/models/advanced_plane/model.sdf` 和 `ROMFS/px4fmu_common/init.d-posix/airframes/4008_gz_advanced_plane`。PX4 提交 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，Gazebo 资产提交 `b6127f4ec20de867e215fb5f78ae88b80f371909`。保留模板 `tools/px4_advanced_plane.sdf`，许可证分别保存在 `tools/PX4_LICENSE`（PX4 主仓）和 `tools/PX4_GAZEBO_LICENSE`（Gazebo 资产子模块）。未复用上游第三方几何网格。

`wheel_axle_friction_nm` 为三个轮轴各自的恒定摩擦力矩（N·m），当前每轮1.5 N·m，轮胎半径0.15 m，对应总等效地面滚动/轻制动阻力约30 N；它同时增加起飞滑跑阻力并缩短落地滑跑。该值为轮轴/滚阻/制动的合并仿真估计，不是实测轮胎数据。

`px4` 字典覆盖生成机体配置：`FW_LND_USETER=0` 在无测距仪的平面世界中按任务海拔降落，`SIM_BAT_MIN_PCT=100` 保持模拟电池容量，`FW_BAT_SCALE_EN=0` 禁用电压油门缩放。测试不会依赖电量消耗增加或削弱推力。

美术网格为白色融合三角翼、圆润中央壳体、翼尖竖翼、装饰面板线与黑色小窗。参考图片仅作外形美术参考，静态美术源保存在 `models/props/delta_wing_display/`，运行模型保存独立网格副本与 `tools/artwork.sdf`；美术参考不用于确定实际结构、动力参数或型号性能。

通用着陆控制配置采用 `FW_LND_FL_TIME=4 s`、`FW_LND_FLALT=15 m`、`FW_LND_FL_PMIN=6°`：按下降速度和最小高度提前进入拉平，并为时间渐变的俯仰与下降率控制留出高度裕量；最小拉平俯仰参考值为6°，通用俯仰偏置由PX4原生控制器处理。它们只调整通用 PX4 导航/控制行为，不改变用户确认的可视外形或机体动力学。
