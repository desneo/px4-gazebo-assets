# 组合组件验证状态

当前吊舱继承基础模型的 yaw 左负右正约定、协议 v7 和每轴 ±20°/s 机械速度限制。装配继承、离线方向、完整整机飞行、悬停稳定性及运动载机 CSRT 跟踪均已通过当前配置验证；本文所附组件飞行归档需与其运行快照和源指纹一起阅读。

当前正式完整飞行入口增加悬停稳定性门禁：跳过hover前5秒，从ULog检查roll/pitch rate error、5～35 Hz频谱、去趋势姿态、控制分配未实现、`unallocated_torque` 和电机触底；同时必须收到吊舱实际 `camera.quat_world`，并检查按运行相机内参换算的LOS像素抖动。缺少镜头遥测时明确判定失败，不能用机体姿态代替。35 Hz上限为83.3 Hz ULog角速度记录率保留采样裕量，并覆盖已观察到的16～20 Hz振动。当前门槛要求各轴rate error RMS≤2°/s、P95≤4°/s、最大值≤10°/s、5～35 Hz RMS≤0.5°/s，去趋势姿态P95≤0.1°且峰峰值≤0.5°；控制分配未实现比例≤1%、最长连续≤0.1秒，`unallocated_torque` P99≤0.002，单电机触底比例≤1%；实际LOS径向P95≤6 px、最大值≤15 px。

当前组合采用新 CAD 两轴组件，590 mm 机体包络 z=-0.210～+0.380 m，底环水平、底座朝下，球头一侧朝载机 +X；安装面 z=0.289 m、肩部起点 z=0.277 m。外轴绕水平 +X，内轴绕随动 −Y，零位沿用 CAD 原始 0°、镜头朝 +Z；运动链为 `Rx(yaw) Ry(-pitch)`，yaw±60°、pitch−80°～+30°，协议 v7、`motion_head_joint_right_positive`。

装配显式启用 `mount_dynamic=true`，继承基础吊舱的100 Hz余相控制调度及动态安装角速度路径：4 ms物理步按8/12 ms交替控制，优先使用Gazebo实际世界角速度并转换到安装局部系，数据不可用时才使用35 Hz低通和跳变拒绝的姿态差分。纯调度/运动学测试、独立固定组件Gazebo验收、完整飞行实际镜头LOS及运动载机动态跟踪均已通过。

静态装配包含24个visual、无碰撞体；动态装配包含12个link、11个joint、总质量5.00 kg。装配离线检查验证原生几何和动力学继承、SDF真实枢轴与转轴、球心固定、相机外参、机械限位、生成确定性及CA/EKF参考点派生。离线运动学和结构检查不代替完整飞行、真实视频或跟踪验收。组合外形仍待人工确认。

## 当前质量与控制参考点

基础机质量4.76 kg、合成重心 `(0,0,0.030) m` FLU；另加0.24 kg吊舱。基础机各link的质量、惯性位姿及惯性张量均继承原生文件，不按显示裁切扣减机头质量。吊舱三组质量、重心及惯量继承基础组件按实际 CAD 闭合实体体积分配的结果。

组合固定零关节名义重心为 `(-0.000176261396,-0.000000003426,0.044398504845) m` FLU。18个CA/EKF位置参数按基础重心到组合名义重心的差值派生，FRD平移量为 `(+0.000176261396,-0.000000003426,+0.014398504845) m`。旋翼PZ=+0.109398504845 m；IMU/GPS X=+0.000176261396 m、Y≈−3.426×10⁻⁹ m、Z=+0.029398504845 m。Y方向微小非零量来自CAD实体积分，未人为归零；实际完整精度以生成的 `manifest.px4_reference` 和运行快照 `assembly.px4_reference` 为准。

组合悬停参数文件值为0.1829004643346519；基础文件值为0.1727721546023802。两者均按50～800 rad/s电机映射和各自质量自动计算。

组合机体继承基础机原生PX4高机动限制：水平加速度15 m/s²、向上12 m/s²、向下6 m/s²，自动/最大jerk为30/40 m/s³；完整飞行验收的巡航和冲刺速度设定均采用8 s线性爬升。

真值工具由运行快照的各link质量和惯性位姿求得同一固定名义重心，将该机体系偏移随模型姿态旋转，再转换ENU→NED并差分速度。`measurement_reference` 应使用 `kind=nominal_cg`、`reference_pose=zero_joint_positions` 及该运行的 `body_offset_flu_m`。这个固定参考点不表示运动关节的瞬时重心。

## 完整组件飞行结果

归档组合于 **2026-10-23 21:25:34（Asia/Shanghai）** 通过当前运行快照的完整无风组件世界飞行，正常减速、降落并解除解锁，无验收、稳定性或降落恢复错误。归档覆盖飞行门槛、固定名义重心声明、ULog稳定性、实际镜头LOS、原始连续达标样本及全部 **49 项当前源文件指纹**；未缩短悬停、巡航或冲刺的正式时长。

| 检查 | 正式判定条件 | 实测结果 |
| --- | --- | --- |
| 悬停 | 30 s，漂移≤1 m，高度误差≤2 m | 最大漂移0.050719 m、高度误差0.416199 m，通过 |
| 巡航 | ≥200 km/h连续30 s，高度误差≤2 m | 连续30.000 s、峰值224.252178 km/h、高度误差0.287606 m，通过 |
| 冲刺 | ≥300 km/h连续10 s，高度误差≤2 m | 连续10.016 s、峰值319.135478 km/h、高度误差0.207032 m，通过 |
| 减速、降落与停机 | 完成减速，确认落地并解除解锁 | 通过 |

悬停正式窗口中，横滚/俯仰角速度误差 RMS 为0.063255/0.155130°/s，P95为0.128263/0.313680°/s；5～35 Hz频带 RMS 为0.039824/0.043278°/s。控制分配未实现比例和四路电机触底比例均为0。实际 `camera.quat_world` 共1251个有效样本，LOS径向P95为0.425626 px、最大值0.725524 px，水平/垂直峰峰值为0.841152/1.149161 px，稳定性门禁通过。

峰值为对应航段瞬态最高速度，不替代连续保持，也不代表最高稳定速度对照已通过。测量来源为 `gazebo_pose`，世界 `rocket_two_axis_pod_test`、实体 `rocket_two_axis_pod`，目标高度30 m。该次运行使用 PX4 instance0、MAVLink `udpin:0.0.0.0:14540`，墙钟耗时168.184 s，保存4921条遥测和8201条Gazebo真值采样。

- [飞行结果、运行快照与30项源指纹](validation.json)
- [完整原始飞行记录](../tests/results/flight_result.json.gz)
- [实际速度与高度曲线](flight_validation.png)

原始报告 SHA-256：`a17eb5f885412399c73c0d32b0fb66a1a0f369a9aca4e1960e493698416ee641`；运行快照 SHA-256：`959d919bfc4540ec496f6f3c8b2296e1510c73f5de98ceea5726e42c9f5232e8`。运行目录 `/tmp/rocket-two-axis-pod-flight.6YJun9`，原始报告 `/tmp/rocket-pod-accel8-flight.json`；入库压缩记录和摘要包含复核需要的完整快照、指纹、原始采样及稳定性结论，不依赖临时目录保留。

报告必须由实际生成资产、原始采样、固定名义重心声明和源文件指纹共同验证；来源不一致时归档工具拒绝标记为当前通过。复现与正式归档入口：

```bash
python3 -m unittest discover -s assembledmodels/rocket_two_axis_pod/tests -p 'test_*.py'
python3 assembledmodels/rocket_two_axis_pod/tools/check_dynamic_model.py
bash assembledmodels/rocket_two_axis_pod/tests/run_flight_check.sh headless
# 使用该次启动器输出的实际报告路径
python3 assembledmodels/rocket_two_axis_pod/tools/report_validation.py \
  /tmp/rocket-two-axis-pod-flight.XXXXXX/flight.json
```

## 视频与场景验收边界

组件世界没有加载视频渲染的 `Sensors` 系统，因此组件归档只验收实际光学姿态，不覆盖视频编码或RTSP。当前地理世界补充运行完成一次往返、降落和70仿真秒显式CSRT跟踪：2120/2120帧有效、丢失0帧、独立真值确认率100%，载机姿态变化18.414°，目标射线误差P95为0.472°、中心误差P95为0.824°；完整RTSP身份链仍须由[完整地理场景](../../../scenarios/geographic_rocket_two_axis_pod/docs/validation.md)单独归档。[最高稳定速度对照](../../../scenarios/rocket_speed_comparison/docs/validation.md)也仍需各自证据，瞬态速度峰值不能替代连续稳定窗口。基础单机结果见[独立完整飞行验证](../../../models/vehicles/rocket_racing_quad/docs/validation.md)。

生成、启动、PX4实例、日志和人工观察说明见[装配说明](../README.md)。
