# 参数配置与复测

[parameters.json](../parameters.json) 是本模型工具读取的自定义 JSON 配置，**不是 PX4 或 Gazebo 原生配置格式**。`tools/generate_model.py` 将其转换为 STL、`model.sdf`、`tests/world.sdf` 和 `px4/<机体编号>_gz_rocket_racing_quad`；飞行脚本读取 `flight_test` 节点。世界的非参数化内容保存在 `tools/world_template.sdf`。

所有数值应为有限数，JSON 不支持注释；参数用途维护于本文。单位见字段后缀；桨转速单位为 rad/s，换算 rpm 时乘以 `60/(2π)`。下列数值为默认配置的说明，实际值以所使用的 JSON 为准。

## 常用几何与质量

机身中心为坐标原点，机头沿 +z，机尾沿 -z。`body_length_m` 是 L，取机头到机尾的轴向长度。改变尺寸不会自动改变质量、桨的动力系数或气动参考面积，应按实验目的同步调整。

| 字段（geometry） | 默认值与单位 | 修改影响 |
| --- | --- | --- |
| `body_length_m` | 0.450 m | 机身长度、翼根位置、惯量和生成高度 |
| `body_radius_m` | 0.045 m | 机身外形、碰撞体、惯量及桨盘净空 |
| `diagonal_wheelbase_m` | 0.381 m | 对角电机轴距；PX4控制分配位置随之更新 |
| `propeller_diameter_m` | 0.254 m | 桨网格、旋转碰撞圆盘与桨惯量 |
| `nacelle_length_m` / `nacelle_radius_m` | 0.090 / 0.040 m | 动力舱外形、碰撞体及惯量 |
| `nacelle_z_m` / `rotor_z_m` | -0.120 / -0.185 m | 动力舱中心/桨盘沿机身轴线的位置 |
| `fin_span_m` / `fin_thickness_m` / `fin_root_length_m` | 0.1455 / 0.004 / 0.300 m | 翼面从机身半径向外的展长、厚度与轴向翼根长度 |
| `fin_tip_z_m` / `fin_tip_chord_m` | -0.120 m / 0.040 m | 外翼端中心高度与轴向弦长，控制三角翼与电机罩连接 |
| `arm_radius_m` | 0.008 m | 翼内连接臂碰撞近似的半径，不单独显示圆杆 |

当前为下置桨：电机整流罩底部 z=-0.165 m，桨毂顶部 z=-0.171 m，净隙6 mm由同轴轴段连接。翼面外端沿四个电机对角方位布置；外露翼面下缘位于桨盘上方，靠机身的根部收尖不得进入桨盘。

电机 x/y 偏移 = 对角轴距 / `(2√2)`；相邻桨盘净空 = 对角轴距 / `√2` − 桨径，当前约 15.41 mm；桨盘与机身径向净空 = 对角轴距 / `2` − 桨半径 − 机身半径，当前 18.5 mm。生成器检查桨盘干涉、尾翼与桨平面间隔、轴向包络等约束；改变轴向位置时仍应查看动力舱与桨毂连接关系。

`mass.body_kg=1.10`、`nacelle_kg=0.040`、`propeller_kg=0.025` 分别是机身、单动力舱、单桨质量，单位 kg；总质量为 `body_kg + 4×(nacelle_kg + propeller_kg)`。机身质量包含翼面和翼内连接臂。质量及惯量均为估计，改变后需要重新检查悬停油门、控制响应和速度表现。

## 动力与气动

| 字段（motor） | 默认值与单位 | 修改影响 |
| --- | --- | --- |
| `max_velocity_rad_s` / `idle_velocity_rad_s` | 1250 / 50 rad/s | 最大/解锁怠速；未解锁输出为0，解锁后最低转速为50 rad/s |
| `thrust_constant` | 1.9×10⁻⁵ N·s²/rad² | 静态单桨推力 `T=kω²`，增大后相同转速推力增大 |
| `moment_constant_m` | 0.02 m | 反扭矩/推力比；同步PX4转子力矩系数及旋转方向符号 |
| `time_constant_up_s` / `time_constant_down_s` | 0.0125 / 0.025 s | 转速上升/下降响应时间 |
| `rotor_drag_coefficient` | 1.2×10⁻⁴ | Gazebo桨侧向阻力模型系数，按插件关系作用于转速和横向速度 |
| `rolling_moment_coefficient` | 1×10⁻⁶ | Gazebo桨横向流动滚动力矩系数 |
| `visual_slowdown` | 10，无量纲 | 仿真桨关节转速减速倍率，插件用其还原用于力计算的转速 |

`aerodynamics.air_density_kg_m3=1.225`、`drag_coefficient=0.73`、`reference_area_m2=0.0177` 分别为空气密度、阻力系数和参考面积。其乘积决定简化阻力量级：`D≈0.5ρCdAv²`，速度单位 m/s。参考面积是独立估计参数，不随机身尺寸自动重算；转子附加阻力另由电机插件计算。当前气动模型没有实测标定，几何和动力参数变化后必须重新飞行验收。

## PX4参数

`px4.airframe_id=4022` 决定生成机体文件编号。`px4.parameters` 内是 PX4 原生参数名称到数值的映射，生成器写成 `param set-default`；参数名称是否合法且是否受当前 PX4 版本支持，应查看启动日志。旋翼位置、数量、反扭矩、输出编号及转速映射由几何/动力配置生成，不需重复手填。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `MPC_THR_HOVER` | 约0.0944385 | 归一化悬停推力，按当前质量、正怠速与PX4输出映射计算 |
| `MPC_THR_MAX` | 0.9 | 最大归一化推力，保留推力裕量 |
| `MPC_THR_MIN` | 0.03 | 最小归一化推力，必须低于悬停推力，保留下降控制余量 |
| `MPC_THR_MAX` | 0.9 | 最大归一化推力，须高于悬停推力，保留爬升与高速高度控制余量 |
| `THR_MDL_FAC` | 1 | 归一化推力经开方映射为转速命令，与Gazebo平方推力关系匹配 |
| `MPC_USE_HTE` | 0 | 使用上述配置悬停推力，不启用悬停推力估计器 |
| `MPC_TILTMAX_AIR` | 85° | 最大飞行倾角；高速保持高度需要足够垂直推力 |
| `MPC_XY_VEL_MAX` | 95 m/s | 水平速度限制，必须覆盖验收速度与控制余量 |
| `MPC_XY_CRUISE` | 15 m/s | PX4任务模式巡航设置；本项目200 km/h目标由OFFBOARD验收设置给定 |
| `MPC_VEL_MANUAL` | 20 m/s | 手动位置模式速度设置 |
| `EKF2_HGT_REF` | 0 | 平面试验使用气压高度参考，避免长距离地理坐标曲率影响离地高度 |
| `NAV_DLL_ACT` | 2 | 数据链丢失动作枚举，含义以当前PX4版本为准 |
| `EKF2_BCOEF_X` / `EKF2_BCOEF_Y` / `EKF2_MCOEF` | 0 / 0 / 0.12 | PX4估计器阻力模型设置，不是Gazebo阻力系数 |

当前Gazebo推力满足 `T=kω²`；`idle_velocity_rad_s=50` 保证解锁后四个电机即使分配命令为零也具有正转速，通过ESC解锁转速检查；`SIM_GZ_EC_DIS1` 至 `DIS4` 为0，未解锁时停转。ESC检查保持启用。`THR_MDL_FAC=1` 时，PX4先将归一化推力开方，再将输出映射到 `[ωidle, ωmax]`。因此实际转速为 `ω=ωidle+(ωmax−ωidle)√u`，悬停应使用 `MPC_THR_HOVER=((√(mg/(4k))−ωidle)/(ωmax−ωidle))²`，其中 `g=9.8 m/s²`。当前总质量1.36 kg，对应悬停转速约418.770 rad/s；50 rad/s解锁怠速的四桨静态总推力为0.19 N，远小于重力。

`MPC_THR_MIN` 必须满足 `0 ≤ MPC_THR_MIN < MPC_THR_HOVER < MPC_THR_MAX ≤ 1`，当前上下限显式配置为0.03和0.9。质量、推力系数、最大转速或解锁怠速变化时，应同步重算悬停推力、检查控制余量并重新试飞。

原生参数只改变飞控行为；例如提高 `MPC_XY_VEL_MAX` 不会增加电机推力。反过来，修改Gazebo动力也不会自动整定PX4。`param set-default` 不会覆盖同一rootfs中的已保存参数；启动器每次创建新rootfs以隔离实验。飞行期间经控制台临时改参不会写回JSON，应在复测前同步配置并重启。

## 传感器与仿真

`sensors` 中 IMU/气压/磁力计/卫星定位频率默认 250/50/100/30 Hz。气压计使用 1 Pa 标准差、磁力计三轴使用 0.0001 T 标准差的高斯噪声，模拟当前小型高速四旋翼的传感器波动，并避免静止时理想恒值被飞控判定为传感器停滞。`simulation.step_size_s=0.004` 是物理步长；IMU频率不能超过 `1/step_size_s`。`real_time_factor=1` 为目标实时倍率，实际倍率受机器性能限制。步长、传感器频率或噪声变动可能影响控制与估计器稳定性。

`ground_size_m=50000` 是方形地面边长，单位m，显示平面与实体碰撞地面同步，用于长距离高速测试；应覆盖完整加速、保持、减速与降落轨迹。

`spawn_clearance_m=0.005` 为机尾离地间隙；生成高度为机身半长加该间隙。地理原点由 `latitude_deg`、`longitude_deg`、`elevation_m` 给出，单位为度、度、米，影响磁场和导航参考。`imu_wait_timeout_s=90` 是启动器等待IMU首帧的墙钟期限，不代表估计器已完成初始化。

## 飞行验收与参数对比

`flight_test` 包含连接地址、速度门槛、持续时间、高度容差、加减速及超时；字段和判定见[自验证说明](../tests/README.md)。巡航/冲刺速度单位为 km/h，PX4速度参数单位为 m/s，两者相差3.6倍。默认设定值余量15 km/h只帮助越过门槛，不降低验收门槛。

以下命令从仓库根目录执行：

```sh
# 修改配置后更新仓库模型资产
python3 models/vehicles/rocket_racing_quad/tools/generate_model.py

# 参数对比：复制JSON后编辑，实验文件放在/tmp
cp models/vehicles/rocket_racing_quad/parameters.json /tmp/rocket-test.json
sh models/vehicles/rocket_racing_quad/tests/run.sh headless /tmp/rocket-test.json

# 另一终端：把XXXXXX替换成启动输出中的实际目录
python3 models/vehicles/rocket_racing_quad/tests/flight_check.py \
  --config /tmp/rocket-racing.XXXXXX/models/rocket_racing_quad/parameters.json \
  --log /tmp/rocket-test-result.json
```

启动器自动重生成运行资产；不必为每次试验改写仓库资产。模型、动力、气动、PX4、传感器或仿真参数变化后，停止旧仿真并重新启动。飞行脚本的 `--config` 应指向实际启动模型的运行副本，不能指向后来又修改过的源文件，否则验收中的限速检查和模型参数会不一致。命令行覆盖仅用于本次测试，不改变模型或PX4；提高目标速度时须先确认实际飞控限速已允许目标加余量。

需要单独导出资产时可使用 `generate_model.py --config /tmp/rocket-test.json --output-dir /tmp/rocket-export/rocket_racing_quad`。输出目录包含该配置快照，但不包含测试Python脚本和启动器，应继续使用仓库内脚本。
