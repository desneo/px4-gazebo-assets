# 参数配置与复测

标准启动入口以模型目录中的原生文件为准，不从 JSON 重生成物理资产：

| 配置内容 | 生效文件 |
| --- | --- |
| 各 link 的质量、重心、惯性、位置、碰撞、传感器、动力与气动 | [model.sdf](../model.sdf) |
| 显示外形 | `meshes/`，由 SDF 引用 |
| PX4 电机映射、控制分配与飞控参数 | [px4/4022_gz_rocket_racing_quad](../px4/4022_gz_rocket_racing_quad) |
| 世界物理步长、地理原点、地面、初始位置和 GUI | [tests/world.sdf](../tests/world.sdf) |
| 飞行验收设置与 IMU 启动等待期限 | [parameters.json](../parameters.json) 的 `flight_test`、`simulation.imu_wait_timeout_s` |

`tools/prepare_runtime.py` 将上述原生资产复制到独立目录，提取实际运行数据并写出运行副本的 `parameters.json`。其中 `model` 保存各 link 的质量、惯性与姿态、电机插件和气动配置；`px4` 保存复制的机体文件参数；`simulation` 保存世界名称、物理引擎设置、重力、生成位置和启动等待期限；`runtime.source_sha256` 保存原生模型、完整世界文件、PX4、网格及默认 JSON 的文件指纹。飞行日志应使用这个运行快照，不能把源 JSON 的旧物理字段当成实际模型数据。

`run.sh` 和 `run_check.sh` 的可选 JSON 参数只提供 `flight_test` 与 `simulation.imu_wait_timeout_s`。其中的 `geometry`、`mass`、`motor`、`aerodynamics`、`sensors`、PX4 参数及其他世界字段不会覆盖原生文件。改变物理模型或飞控设置时，应修改表中对应文件并重启。

## 质量、重心与惯性

模型局部坐标以 `base_link`原点为参考，机头沿+z、机尾沿-z。机身端点z=-0.210 m与+0.380 m，总高0.590 m；几何轴向中点和机身碰撞圆柱中心z=+0.085 m，均不等于link原点或整机重心。SDF每个 `<link>`的 `<inertial>`独立描述质量、重心和关于该重心的惯性：

```xml
<inertial>
  <pose>0 0 0.0331777777777778 0 0 0</pose>
  <mass>4.5</mass>
  <inertia>
    <ixx>0.139854907060</ixx>
    <iyy>0.139243371000</iyy>
    <izz>0.190493151860</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>
</inertial>
```

`pose`依次是 `x y z roll pitch yaw`，前三项单位m、后三项单位rad，描述惯性坐标系相对于所属link的姿态。当前 `base_link`质量4.50 kg，重心 `(0,0,+0.0331777777777778) m`；四个动力舱各0.040 kg、四个旋翼各0.025 kg，总质量4.76 kg，合成整机重心 `(0,0,+0.030) m`。所列惯性张量关于 `base_link`自身重心，积惯量为0，不能当作关于link原点的惯性直接使用。

四动力舱中心x/y为±0.2086 m、z=0；四桨中心x/y相同、z=-0.065 m，对角轴距约0.590010 m。主翼厚4 mm，45°/135°/225°/315°布置，翼端r≈0.295005 m、z=-0.020～+0.020 m；小尾翼独立保存在 `meshes/tail_fins.stl`，0°/90°/180°/270°布置，厚4 mm、r≤0.100 m，z=-0.210～-0.105 m。尾端平底直径68 mm，54 mm长过渡段以smoothstep连接直径90 mm主身。机臂碰撞体仍保留径向末端0.1905 m、z=0的近似，不覆盖全部显示翼面。

## 动力、气动与 PX4 配套

| 配置项 | 当前 SDF | 当前 PX4 |
| --- | --- | --- |
| 电机 x/y 位置幅值 | 0.2086 m | 0.2086 m；x保留、y反号 |
| 桨盘z位置 | -0.065 m，参考 `base_link`原点（FLU） | `CA_ROTOR*_PZ=+0.095 m`，参考整机重心（FRD） |
| 反扭矩/推力比幅值 | 0.02863386 m | `CA_ROTOR*_KM` 的0/1路为+0.02863386 m，2/3路为-0.02863386 m |
| 最大转速 | 800 rad/s | `SIM_GZ_EC_MAX1`～`MAX4` 均为 800 rad/s |
| 解锁怠速 / 未解锁转速 | 由 PX4 映射输入 | 50 / 0 rad/s |
| 单桨推力系数 | 8.9119×10⁻⁵ N·s²/rad² | 通过转速映射使用 SDF 推力 |
| 悬停推力 | 4.76 kg静态理论值约0.1727721546 | `MPC_THR_HOVER=0.1727721546023802` |

Gazebo FLU转为PX4 FRD时y/z反号。控制分配位置以整机重心为参考，桨盘相对重心为z=-0.095 m FLU，故 `CA_ROTOR*_PZ=+0.095 m` FRD；x/y参考点没有偏移。传感器在link局部z=+0.015 m，比整机重心低0.015 m，对应 `EKF2_IMU_POS_X/Y=0`、`EKF2_GPS_POS_X/Y=0`及两者Z=+0.015 m FRD。启动器复制这些原生配置，不自动改写参考点。

当前推力满足 `T=kω²`。当 `THR_MDL_FAC=1` 时，PX4 将归一化推力开方，再按解锁怠速和最大转速进行映射：

```text
ω = ωidle + (ωmax − ωidle) √u
uhover = ((√(mg/(4k)) − ωidle) / (ωmax − ωidle))²
```

按4.76 kg、`g=9.8 m/s²`、四个相同电机及50/800 rad/s映射，静态悬停转速约361.744025 rad/s，公式值 `uhover≈0.1727721546`，机体文件配置 `MPC_THR_HOVER=0.1727721546023802`。上限调整不改变推力系数、电机时间常数或实际悬停转速，只缩减未使用的峰值动力并扩大归一化低侧控制余量。`MPC_USE_HTE=0`；`MPC_THR_MIN=0.03`、`MPC_THR_MAX=0.9`。

`MPC_XY_VEL_MAX=95 m/s` 是水平控制速度限制；`MPC_XY_CRUISE=15 m/s` 是 PX4 任务模式设置，本项目 OFFBOARD 的200 km/h巡航目标来自 `flight_test`。`MPC_TILTMAX_AIR=85°`，`MPC_VEL_MANUAL=20 m/s`。高机动限制为 `MPC_ACC_HOR=15 m/s²`、`MPC_ACC_HOR_MAX=15 m/s²`、`MPC_ACC_UP_MAX=12 m/s²`、`MPC_ACC_DOWN_MAX=6 m/s²`，`MPC_JERK_AUTO=30 m/s³`、`MPC_JERK_MAX=40 m/s³`。飞行验收的 `ramp_seconds=8`，表示巡航和冲刺速度设定各用8 s线性爬升。`EKF2_BCOEF_X` / `EKF2_BCOEF_Y` / `EKF2_MCOEF` 为0 / 0 / 0.12，是估计器设置，不是 Gazebo 气动系数。

SDF 的空气密度1.225 kg/m³、阻力系数0.73、参考面积0.0177 m²，用于简化阻力关系 `D≈0.5ρCdAv²`。参考面积独立配置，不随尾翼网格或机臂长度重算；转子附加阻力另由电机插件计算。当前没有独立尾翼气动力、实测标定或电压下降模型。四个电机的 `timeConstantUp` / `timeConstantDown` 均为0.0125 / 0.025 s。

当前总高0.590 m的平底、四主翼与四小尾翼基础外形已获人工确认，4.76 kg配置通过完整无风组件世界飞行自验证。当前惯性矩正定并满足惯性三角关系和几何包络约束，实际网格非负质量矩匹配表明数学可行，见[验证说明](validation.md)。惯性未作实物材料或CAD标定；气动和动力仍为独立显式配置，仿真通过不代表真实硬件性能。

`AdvancedLiftDrag`的 `cp=0 0 0.015`表示气动力作用点在link局部z=+0.015 m，比整机重心低0.015 m。它与传感器位置、重心和几何轴向中点分别配置，横向气动力会产生相应力矩。

`param set-default` 不会覆盖已有 rootfs 中保存的参数；启动器每次创建新 rootfs。飞行期间通过控制台改参不会写回仓库机体文件，正式复测前应同步原生文件并重启。修改 Gazebo 动力也不会自动整定 PX4。

## 传感器与世界

当前IMU、气压、磁力计与卫星定位传感器都在link局部z=+0.015 m，频率为250/50/100/30 Hz。气压计使用1 Pa标准差、磁力计三轴使用0.0001 T标准差的高斯噪声。

当前世界物理步长0.004 s、目标实时倍率1，IMU频率不能超过步长倒数。地面边长50 km、初始 `base_link`原点z=0.215 m，尾端离地5 mm、顶部离地0.595 m；本体总高0.590 m。地理原点 `(47.397971057728974,8.546163739800146)`、海拔0 m，均由 `tests/world.sdf`生效。源JSON的同名物理字段仅供离线生成器使用。

`simulation.imu_wait_timeout_s=90` 是启动器等待 IMU 首帧的墙钟期限，不代表估计器已初始化。步长、传感器频率、噪声或世界变化后需检查控制与估计器稳定性。

## 飞行验收设置

`flight_test` 包含连接地址、速度门槛、持续时间、高度容差、加减速及超时，字段和判定见[自验证说明](../tests/README.md)。速度字段以名称单位为准；巡航/冲刺使用km/h，PX4速度参数使用m/s，两者相差3.6倍。默认设定值余量15 km/h不会降低验收门槛。

从仓库根目录执行：

```sh
# 复制后仅调整 flight_test 或 simulation.imu_wait_timeout_s
cp models/vehicles/rocket_racing_quad/parameters.json /tmp/rocket-test.json
sh models/vehicles/rocket_racing_quad/tests/run.sh headless /tmp/rocket-test.json

# 把 XXXXXX 替换成启动器输出中的实际目录
python3 models/vehicles/rocket_racing_quad/tests/flight_check.py \
  --config /tmp/rocket-racing.XXXXXX/models/rocket_racing_quad/parameters.json \
  --log /tmp/rocket-test-result.json
```

飞行脚本的 `--config`应指向实际运行快照。`--ground-truth`从 `model.links`质量与惯性位姿计算固定零关节名义重心，单机为 `(0,0,0.030) m`、当前组合为 `(-0.000176261396,-0.000000003426,0.044398504845) m` FLU；每条模型姿态旋转该固定偏移，再换算ENU→NED并由参考点差分求速度。报告 `measurement_reference`记录 `kind=nominal_cg`、`body_offset_flu_m`、`reference_pose=zero_joint_positions`，归档时核对与快照一致。它不表示运动关节的瞬时重心；离线JSON不能替代含有效 `model.links`的运行快照。

## 显式离线生成

`tools/generate_model.py` 仍支持从完整 JSON 生成参数化网格、模型、世界和 PX4 机体文件。源 JSON 的物理字段是该生成器的输入，**不是基础模型标准启动入口的实际物理配置**。当前顶部两轴吊舱动态装配从原生 SDF 和 PX4 文件继承基础机物理配置，源 JSON 只提供测试与 IMU 启动等待设置。当前默认 JSON 保留0.450 m机身长度、1.1 kg机身、0.381 m轴距、1250 rad/s上限及1.9×10⁻⁵推力系数，与当前原生模型不同。

```sh
# 输出到独立目录，避免覆盖手调 SDF、网格、世界和 PX4 文件
python3 models/vehicles/rocket_racing_quad/tools/generate_model.py \
  --config models/vehicles/rocket_racing_quad/parameters.json \
  --output-dir /tmp/rocket-export/rocket_racing_quad
```

`--output-dir` 必填，且禁止写入当前模型目录或其父/子路径。生成器目前按圆柱近似计算机身/动力舱惯性，并不保留原生 SDF 的自定义重心和惯性。

离线 JSON 的 `geometry` 使用m，包含机身长度/半径、对角轴距、桨径、动力舱尺寸与z位置、翼面尺寸和机臂半径；`mass` 使用kg，包含机身、单动力舱和单桨质量。`motor` 的转速单位rad/s、时间常数单位s，推力系数满足 `T=kω²`，反扭矩/推力比单位m。`aerodynamics`、`sensors`、`simulation` 和 `px4.parameters` 决定生成资产的对应设置；这些字段仅在显式生成时生效。

生成器检查净空和输入约束，但几何、质量、动力或飞控参数变化后仍需重新验收。导出目录没有测试Python脚本和启动器；需要采用导出资产时，应明确更新相应原生文件后再使用标准入口。`render_views.py` 从 JSON 重建几何，其输出不能代替当前原生 SDF 的外形检查。
