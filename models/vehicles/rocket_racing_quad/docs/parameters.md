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

模型局部坐标以 `base_link` 原点为参考，机头沿 +z，机尾沿 -z。机身轴向端点为 z=-0.225 m与+0.365 m，几何轴向中点位于 z=+0.070 m，不与 `base_link` 原点重合。机身碰撞圆柱长0.590 m、中心 z=+0.070 m。SDF 中每个 `<link>` 的 `<inertial>` 独立描述质量、重心和惯性：

```xml
<inertial>
  <pose>0 0 -0.094 0 0 0</pose>
  <mass>7.44</mass>
  <inertia>
    <ixx>0.129598907060</ixx>
    <iyy>0.129623371000</iyy>
    <izz>0.175303151860</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>
</inertial>
```

`pose` 依次是 `x y z roll pitch yaw`，前三项单位 m，后三项单位 rad；描述重心位置和惯性坐标轴相对于所属 link 的姿态。惯性项单位 kg·m²，必须对应此重心与坐标轴。当前机身重心为 `(0, 0, -0.094) m`，各 link 合计质量 **7.70 kg**，合成重心约为 `(0, 0, -0.095722) m`。机身质量为7.44 kg，四个动力舱与四个旋翼合计0.26 kg。这些质量、重心及惯性使用独立显式值，不由0.590 m机身几何自动推算。

当前动力舱和桨中心 x/y 为 ±0.2086 m，对角轴距约 **0.590010 m**。全部显示网格与碰撞体的 z 包络为 **-0.225～+0.365 m**，模型总高度为 **0.590 m**，无部件超出该高度。四片显示翼面外端径向位置约0.295005 m，与动力舱连续连接；翼端 z=-0.140～-0.100 m、厚度0.004 m。机臂碰撞体保留径向末端0.1905 m的近似，不覆盖完整显示翼面，当前不考察连接处碰撞精度。显示几何不会自动改变质量、惯性或插件参考面积，无接触自由飞行仍使用这些显式参数。

## 动力、气动与 PX4 配套

| 配置项 | 当前 SDF | 当前 PX4 |
| --- | --- | --- |
| 电机 x/y 位置幅值 | 0.2086 m | 0.2086 m；x保留、y反号 |
| 桨盘 z 位置 | -0.185 m（FLU） | +0.185 m（FRD） |
| 反扭矩/推力比幅值 | 0.02863386 m | `CA_ROTOR*_KM` 的0/1路为+0.02863386 m，2/3路为-0.02863386 m |
| 最大转速 | 1570 rad/s | `SIM_GZ_EC_MAX1`～`MAX4` 均为 1570 rad/s |
| 解锁怠速 / 未解锁转速 | 由 PX4 映射输入 | 50 / 0 rad/s |
| 单桨推力系数 | 8.9119×10⁻⁵ N·s²/rad² | 通过转速映射使用 SDF 推力 |
| 悬停推力 | 7.70 kg静态理论值约 0.0727901 | `MPC_THR_HOVER=0.0755504`，用于当前完整飞行通过配置 |

Gazebo FLU 转为 PX4 FRD 时，y/z 反号；旋转方向还决定反扭矩系数的符号。当前 PX4 电机位置与 SDF 配套，`CA_ROTOR*_PZ=0.185 m` 保持模型原点参考，没有按机身或整机重心重新平移控制分配坐标。启动器直接复制这些配置，不自动重写参数。

当前推力满足 `T=kω²`。当 `THR_MDL_FAC=1` 时，PX4 将归一化推力开方，再按解锁怠速和最大转速进行映射：

```text
ω = ωidle + (ωmax − ωidle) √u
uhover = ((√(mg/(4k)) − ωidle) / (ωmax − ωidle))²
```

按当前总质量7.70 kg、`g=9.8 m/s²`、四个相同电机、怠速50 rad/s及上限1570 rad/s，悬停转速约460.090486 rad/s，理论 `uhover≈0.0727901`。当前采用的 `MPC_THR_HOVER=0.0755504` 为配置值，已通过当前无风组件世界的完整悬停、巡航、冲刺及降落自验证，见[验证说明](validation.md)。静态计算值与实际配置分别记录，参数或工况变化后需重新验证。`MPC_USE_HTE=0`，当前不启用悬停推力估计器；`MPC_THR_MIN=0.03`、`MPC_THR_MAX=0.9`，应满足 `0 ≤ MPC_THR_MIN < MPC_THR_HOVER < MPC_THR_MAX ≤ 1`。

`MPC_XY_VEL_MAX=95 m/s` 是水平控制速度限制；`MPC_XY_CRUISE=15 m/s` 是 PX4 任务模式设置，本项目 OFFBOARD 的200 km/h巡航目标来自 `flight_test`。`MPC_TILTMAX_AIR=85°`，`MPC_VEL_MANUAL=20 m/s`。`EKF2_BCOEF_X` / `EKF2_BCOEF_Y` / `EKF2_MCOEF` 为0 / 0 / 0.12，是估计器设置，不是 Gazebo 气动系数。

SDF 的空气密度1.225 kg/m³、阻力系数0.73、参考面积0.0177 m²，用于简化阻力关系 `D≈0.5ρCdAv²`。参考面积独立配置，不随尾翼网格或机臂长度重算；转子附加阻力另由电机插件计算。当前没有独立尾翼气动力、实测标定或电压下降模型。四个电机的 `timeConstantUp` / `timeConstantDown` 均为0.0125 / 0.025 s。

当前总高0.590 m的机身外形尚未获得人工确认，当前7.70 kg配置已通过无风组件世界的完整飞行自验证。质量、惯性、动力、阻力系数、参考面积和压力中心均为独立参数，不能由鼻锥长度自动推导；几何尺寸符合要求不构成真实硬件性能验证。

`AdvancedLiftDrag` 的 `cp=0 0 0` 表示气动力作用点在 `base_link` 原点，与当前重心不同；横向气动力会产生相应力矩。压力中心应按气动模型确定，启动器不会将其自动移到重心。

`param set-default` 不会覆盖已有 rootfs 中保存的参数；启动器每次创建新 rootfs。飞行期间通过控制台改参不会写回仓库机体文件，正式复测前应同步原生文件并重启。修改 Gazebo 动力也不会自动整定 PX4。

## 传感器与世界

当前 SDF 的 IMU/气压/磁力计/卫星定位频率为250/50/100/30 Hz。气压计使用1 Pa标准差、磁力计三轴使用0.0001 T标准差的高斯噪声，避免理想静止恒值被飞控判为传感器停滞。

当前世界物理步长0.004 s、目标实时倍率1，IMU频率不能超过步长倒数。地面边长50 km、初始 `base_link` 原点 z=0.230 m，机尾离地5 mm、顶部离地0.595 m；模型本体总高度仍为0.590 m。地理原点为 `(47.397971057728974, 8.546163739800146)`，海拔0 m。这些值均由 `tests/world.sdf` 生效，源 JSON 的同名字段仅供离线生成器使用。

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

飞行脚本的 `--config` 应指向实际启动模型的运行快照，使验收限速检查与复制的 PX4 参数一致。命令行覆盖仅用于本次验收，不改变模型或PX4；提高目标速度时须先确认实际飞控限速允许目标加余量。

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
