# 组合组件验证状态

当前吊舱继承基础模型的 yaw 左负右正约定、协议 v7 和每轴 ±20°/s 机械速度限制。装配继承及离线方向检查已通过；本文所附整机飞行/跟踪归档需与其源指纹一起阅读，不能作为当前配置的验收。当前配置尚未复验整机飞行和快速目标跟踪。

当前组合采用新 CAD 两轴组件，590 mm 机体包络 z=-0.210～+0.380 m，底环水平、底座朝下，球头一侧朝载机 +X；安装面 z=0.289 m、肩部起点 z=0.277 m。外轴绕水平 +X，内轴绕随动 −Y，零位沿用 CAD 原始 0°、镜头朝 +Z；运动链为 `Rx(yaw) Ry(-pitch)`，yaw±60°、pitch−80°～+30°，协议 v7、`motion_head_joint_right_positive`。

静态装配包含24个visual、无碰撞体；动态装配包含12个link、11个joint、总质量5.00 kg。装配离线检查验证原生几何和动力学继承、SDF真实枢轴与转轴、球心固定、相机外参、机械限位、生成确定性及CA/EKF参考点派生。离线运动学和结构检查不代替完整飞行、真实视频或跟踪验收。组合外形仍待人工确认。

## 当前质量与控制参考点

基础机质量4.76 kg、合成重心 `(0,0,0.030) m` FLU；另加0.24 kg吊舱。基础机各link的质量、惯性位姿及惯性张量均继承原生文件，不按显示裁切扣减机头质量。吊舱三组质量、重心及惯量继承基础组件按实际 CAD 闭合实体体积分配的结果。

组合固定零关节名义重心为 `(-0.000176261396,-0.000000003426,0.044398504845) m` FLU。18个CA/EKF位置参数按基础重心到组合名义重心的差值派生，FRD平移量为 `(+0.000176261396,-0.000000003426,+0.014398504845) m`。旋翼PZ=+0.109398504845 m；IMU/GPS X=+0.000176261396 m、Y≈−3.426×10⁻⁹ m、Z=+0.029398504845 m。Y方向微小非零量来自CAD实体积分，未人为归零；实际完整精度以生成的 `manifest.px4_reference` 和运行快照 `assembly.px4_reference` 为准。

组合悬停参数文件值约0.04452973995335638；基础文件值0.04206385775789426。PX4位置控制器内部悬停下限0.05，文件值不等同于内部实际值。

真值工具由运行快照的各link质量和惯性位姿求得同一固定名义重心，将该机体系偏移随模型姿态旋转，再转换ENU→NED并差分速度。`measurement_reference` 应使用 `kind=nominal_cg`、`reference_pose=zero_joint_positions` 及该运行的 `body_offset_flu_m`。这个固定参考点不表示运动关节的瞬时重心。

## 完整组件飞行结果

归档组合于 **2026-10-21 20:08:15（Asia/Shanghai）** 通过完整无风组件世界飞行，正常减速、降落并解除解锁，无验收或降落恢复错误。正式报告工具检查了默认完整验收门槛、固定名义重心声明、原始连续达标样本及全部 **30 项该次运行源文件指纹**；未缩短悬停、巡航或冲刺的正式时长。

| 检查 | 正式判定条件 | 实测结果 |
| --- | --- | --- |
| 悬停 | 30 s，漂移≤1 m，高度误差≤2 m | 最大漂移0.050177 m、高度误差0.352756 m，通过 |
| 巡航 | ≥200 km/h连续30 s，高度误差≤2 m | 连续30.016 s、峰值216.918984 km/h、高度误差0.185758 m，通过 |
| 冲刺 | ≥300 km/h连续10 s，高度误差≤2 m | 连续10.016 s、峰值316.507183 km/h、高度误差0.754334 m，通过 |
| 减速、降落与停机 | 完成减速，确认落地并解除解锁 | 通过 |

峰值为对应航段瞬态最高速度，不替代连续保持，也不代表最高稳定速度对照已通过。测量来源为 `gazebo_pose`，世界 `rocket_two_axis_pod_test`、实体 `rocket_two_axis_pod`，目标高度30 m。该次运行使用 PX4 instance3、MAVLink `udpin:0.0.0.0:14543`，墙钟耗时211.379 s，保存6181条遥测和10372条Gazebo真值采样。

- [飞行结果、运行快照与30项源指纹](validation.json)
- [完整原始飞行记录](../tests/results/flight_result.json.gz)
- [实际速度与高度曲线](flight_validation.png)

原始报告 SHA-256：`83c9b1cdce24263f405983b62f8c11a00500cf1abe12a5d160bc1acc9e040374`；运行快照 SHA-256：`2eedc398dfb48b93101bf912cf10b761a065a29e1a5d31ae830736039c5f7a3c`。运行目录 `/tmp/rocket-two-axis-pod-flight.MFaaL3`，原始报告 `/tmp/motion-head-combo-flight.json`；入库压缩记录和摘要包含复核需要的完整快照、指纹和原始采样，不依赖临时目录保留。

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

组件世界没有加载视频渲染的 `Sensors` 系统，本次通过不覆盖视频、地面站或目标跟踪。完整地理世界往返、机载视频和图像跟踪仍待单独复验。[完整地理场景](../../../scenarios/geographic_rocket_two_axis_pod/docs/validation.md)和[最高稳定速度对照](../../../scenarios/rocket_speed_comparison/docs/validation.md)仍需各自证据，瞬态速度峰值不能替代连续稳定窗口。基础单机的结果见[独立完整飞行验证](../../../models/vehicles/rocket_racing_quad/docs/validation.md)。

生成、启动、PX4实例、日志和人工观察说明见[装配说明](../README.md)。
