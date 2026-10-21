# 当前组合体验收状态

当前静态装配总高590 mm、包络z=-0.210～+0.380 m，吊舱零旋转安装面z=0.289 m、肩部起点z=0.277 m。基础机质量4.76 kg、重心z=+0.030 m，另加0.24 kg吊舱得到组合质量5.00 kg；动态CA/EKF位置和组件自验证真值均使用组合零关节固定名义重心参考。

组合CA/EKF位置已从基础机重心平移到固定零关节名义重心 `(0.000204,0,0.043856) m` FLU，组件自验证的真值测量支持也已实现。4.76 kg基础单机当前完整飞行通过，5.00 kg组合保留归档配置的完整组件飞行证据；当前采用两轴协议v5，俯仰绕随动−Y、−90°朝安装+X、0°朝安装+Z、范围−90°～+45°，运动链为 `Rz(yaw) Ry(-pitch)`。当前合并源指纹的组合组件完整飞行，以及本地理场景往返、视频和目标跟踪尚未重新验证，见[装配状态](../../../assembledmodels/rocket_two_axis_pod/docs/validation.md)。

配套Station支持9字段艾晟JSON转换输出，框架角使用当前机械角，脱靶量保持右、下正；该能力的组件验证与本地理飞行/视频验收分别记录，不能互相替代。字段定义见[地面站说明](../../../models/gimbals/two_axis_pod/ground_station/README.md)。

## 判定与复测范围

本场景使用 PX4 实际位置反馈执行飞行，Gazebo 世界位姿独立检查端点、高度、横向偏差与速度。默认相对起飞高度80 m、航点间距300 m、中部速度15 m/s；飞行容差为高度±2 m、横向偏差5 m、到达航点误差3 m，中部速度±1.5 m/s的样本比例至少95%。

框选跟踪要求可达期间 TRACK 与独立目标确认比例均至少95%，角误差P95不超过3°，并要求真实双向位移与载机姿态变化。实际 CSRT 选框必须与独立目标方向交叉确认；目标坐标只允许用于显式请求的一次初始化，不参与持续图像跟踪控制。

两轴机械范围、视线退化和转速限制仍适用；两轴不提供独立画面滚转稳定，不能由单次可达目标的结果推断任意姿态均可锁定。RTSP身份、视频帧号与网关帧表对应由独立绑定检查验收，控制器图像跟踪通过不代表视频路径身份已经通过。各项容差是当前检查设置，不是实物性能指标。

## 现有归档的适用性

[归档摘要](validation.json)及其关联原始记录对应450 mm、约1.377556 kg的不同装配配置。其往返、落地、跟踪和RTSP检查结果不作为当前590 mm、5.00 kg配置的通过证据；归档中的模型、源资产指纹和运行设置应与实际复测记录逐项核对。

以下文件保留供来源核查，不代表当前配置已复验：

- [组合体原始飞行报告](../tests/results/combined_flight.json.gz)、[逐步飞行样本](../tests/results/combined_samples.jsonl.gz)
- [逐帧跟踪记录](../tests/results/tracking_samples.jsonl.gz)、[目标跟踪快照](tracking_snapshot.png)、[RTSP视频帧](pod_video.png)、[飞行与跟踪曲线](flight_tracking.png)
- [RTSP身份与帧表检查摘要](../tests/results/rtsp_binding_summary.json.gz)、[网关遥测和视频帧号记录](../tests/results/rtsp_binding_samples.jsonl.gz)
- [低速单机参考摘要](baseline_validation.json)、[低速原始记录](../tests/results/baseline_flight.json.gz)：其质量和解锁怠速等配置与当前基础机不同，也不能作为当前通过证据。

当前4.76 kg基础单机已通过30 s悬停、200 km/h连续30.012 s巡航、300 km/h连续10.000 s冲刺及落地停机，见[基础结果](../../../models/vehicles/rocket_racing_quad/docs/validation.md)。最高稳定速度另由[速度场景](../../rocket_speed_comparison/README.md)专门验收；组件飞行不替代地理往返或视频跟踪复测。

当前角度定义的离线回归入口：`python3 -m unittest discover -s scenarios/geographic_rocket_two_axis_pod -p 'test_*.py'`。离线运动学、目标可达性和投影检查不构成完整PX4在线往返与图像跟踪通过证据。

## 复现与证据要求

启动、人工框选、有限往返及RTSP绑定检查命令见[场景说明](../README.md)。只有实际飞行、降落、独立图像跟踪和绑定检查全部满足对应条件，才能按该运行生成通过摘要。

本次运行应保留实际 `world.sdf`、动态模型、配置快照、源资产指纹、服务日志、PX4 ULog、逐步飞行与逐帧跟踪数据。归档摘要必须对应当前运行资产；完整地形中的实时帧率受渲染、跟踪及主机负载影响，不能把一次解码采样视为固定性能保证。
