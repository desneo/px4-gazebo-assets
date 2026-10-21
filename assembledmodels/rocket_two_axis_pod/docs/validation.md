# 组合组件飞行证据与当前验证范围

归档的590 mm、5.00 kg顶部两轴吊舱组合配置于2026-10-20 15:52:59（Asia/Shanghai）通过完整无风组件世界飞行自验证：悬停、200 km/h巡航、300 km/h冲刺、减速、降落及停机均通过，无验收或降落恢复错误。下列原始数据及源指纹保持不变。

当前组件使用两轴协议 v5，俯仰轴为随动 −Y、范围 −90°～+45°、−90° 朝安装 +X。外形、质量及固定名义重心沿用590 mm装配配置，但两轴参数、模型及装配生成代码的源指纹已与归档快照不同；该飞行记录不能作为当前全部源文件已通过完整飞行的证据。当前角定义的装配离线检查通过，完整组件飞行及地理场景往返、视频/跟踪仍待当前源指纹下复测。组合外形仍待人工确认。

## 实际结果

| 检查 | 判定条件 | 实测结果 |
| --- | --- | --- |
| 悬停 | 30 s，漂移≤1 m，高度误差≤2 m | 最大漂移0.115301 m，高度误差0.633234 m，通过 |
| 巡航 | ≥200 km/h连续30 s，高度误差≤2 m | 连续30.000 s，峰值216.952345 km/h，高度误差0.421964 m，通过 |
| 冲刺 | ≥300 km/h连续10 s，高度误差≤2 m | 连续10.008 s，峰值316.484826 km/h，高度误差1.003532 m，通过 |
| 减速、降落与停机 | 完成减速，确认落地并解除解锁 | 通过 |

峰值是对应航段瞬态最高速度，不替代连续保持。原始真值独立复核的连续窗口为巡航30.000 s、冲刺10.008 s。顶层 `passed=true`、`landing_confirmed=true`，测量来源 `gazebo_pose`。

世界 `rocket_two_axis_pod_test`、实体 `rocket_two_axis_pod`，起飞目标高度30 m，无风；PX4 instance0、MAVLink `udpin:0.0.0.0:14540`。墙钟耗时210.255 s，记录6181条遥测和10307条真值样本。PX4构建标签 `v1.17.0`、构建提交 `d6f12ad1c4f7`，源码工作区描述 `v1.17.0-dirty`，Gazebo Sim8.15.0。

- [归档摘要、运行配置与22项源资产指纹](validation.json)
- [完整原始飞行记录](../tests/results/flight_result.json.gz)
- [归档速度与高度曲线](flight_validation.png)

实际运行目录 `/tmp/rocket-current-combo-check/rocket-two-axis-pod-flight.eRfCt3`，报告 `/tmp/rocket-current-combo-check/flight.json`。`execution.json`保存实例、MAVLink连接、配置及结果路径；归档按其运行快照、22项源指纹及原始连续达标样本核对，新运行仍须通过来源一致性检查。

## 物理输入与控制参考点

基础机质量4.76 kg、重心 `(0,0,0.030) m` FLU；另加0.24 kg吊舱，组合质量5.00 kg。基础 `base_link`质量4.50 kg、自身重心z=+0.0331777777777778 m，关于自身重心的惯性张量及各link物理位姿从原生文件继承，不按截短圆柱重算。吊舱零旋转安装面z=0.289 m，肩部起点z=0.277 m，整体包络z=-0.210～+0.380 m。

组合关节零位的固定名义重心为 `(0.000204,0,0.043856) m` FLU。18个CA/EKF位置参数从基础参考平移到组合参考，FRD平移量 `(-0.000204,0,+0.013856) m`：旋翼PX均减0.000204 m、PY不变、PZ=+0.108856 m；IMU/GPS X=-0.000204 m、Y=0、Z=+0.028856 m。`manifest.px4_reference`与运行快照 `assembly.px4_reference`记录零关节位姿、两种重心、平移量及原/派生参数。

本次组合悬停参数文件值0.04452973995335638；基础文件值0.04206385775789426。所用PX4位置控制器内部下限0.05，文件值不能等同于内部实际值；通过结果对应当前文件和该控制器实现。

## 真值测量声明

报告 `measurement_reference`为 `kind=nominal_cg`、`body_offset_flu_m=[0.000204,0,0.043856]`、`reference_pose=zero_joint_positions`。真值工具从快照各link质量和惯性位姿加权得到固定名义重心，每条模型位姿旋转该机体系偏移后转换ENU→NED，速度由该参考点位置差分计算。归档核对声明与快照一致。

这一固定点不表示云台运动时按实时关节状态变化的瞬时重心。当前组合35项离线回归及动态检查通过，包含负俯仰姿态、源关节继承与名义重心/CA/EKF派生不变；离线结果与实际飞行证据分别记录。

## 验证边界与入口

当前静态组合含35个visual、无碰撞体，组合外形尚未人工确认。归档飞行结果适用于所附快照的显式质量、惯性、动力、简化气动和无风组件世界；不代表真实硬件性能，也不覆盖连接处精确碰撞或任意风场、航迹。

组件世界没有加载图像渲染的 `Sensors`系统，本次不重验视频、地面站或目标跟踪。完整地理场景往返、视频/跟踪及[最高稳定速度对照](../../../scenarios/rocket_speed_comparison/README.md)仍属独立验收；本次瞬态峰值不能替代最高稳定速度结论。基础单机另有[独立完整通过记录](../../../models/vehicles/rocket_racing_quad/docs/validation.md)。

```bash
bash assembledmodels/rocket_two_axis_pod/tests/run_flight_check.sh headless
# 替换为启动器输出的实际报告路径
python3 assembledmodels/rocket_two_axis_pod/tools/report_validation.py \
  /tmp/rocket-two-axis-pod-flight.XXXXXX/flight.json
```

生成、启动、PX4实例、日志和人工观察说明见[装配说明](../README.md)。资产、参数或验收设置变化后，需要对应运行快照与源指纹的新结果。
