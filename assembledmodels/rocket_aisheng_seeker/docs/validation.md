# 导引头组合装配验证

本文件记录 `rocket_aisheng_seeker` 当前运行快照的验证结果。静态外形检查、动态装配生成检查、02 场景接口检查与完整 PX4 飞行验收分别独立执行；任一单项结果不代表其他快照。

## 静态装配

- 10 项检查通过：原生几何继承、机身裁切闭合、安装面高度、两轴枢轴/限位、相机视觉旋转、预览世界定位标志和生成确定性。
- 静态模型含 26 个 visual、无碰撞体、无关节、无插件；安装面 `z=0.289 m`，组合最高点 `z=0.380 m`。
- 原 `rocket_racing_quad`、`rocket_two_axis_pod` 和导引头基础模型的 `model.sdf` 哈希在生成前后不变。

## 动态装配

- 生成质量 5.00 kg，PX4 airframe 4024；`base_link`、`mount_link`、`yaw_arm_link`、`camera_link` 和固定 `pod_mount_joint` 结构检查通过。
- 控制器插件 `gz-aisheng-seeker-controller`，`mount_dynamic=true`；广角/长焦相机话题分别为 `image` 和 `image_tele`。
- 场景话题前缀 `/uav_<id>/pod` 生成检查通过；命令、遥测、结果和两路图像话题全部随前缀切换。
- 生成模型在 Gazebo headless 中加载后，插件遥测 100 Hz，`/image` 与 `/image_tele` 两路相机话题均存在。
- 5 项动态装配单元测试通过，包含源模型哈希、质量/安装面、话题、PX4 机体和网格完整性。

## 02 场景导引头路径

- `start.sh --aircraft-model rocket_aisheng_seeker` 使用导引头生成器、控制器、二进制 UDP/RTP 网关和导引头上位机；默认 `rocket_two_axis_pod` 路径保持原命令和 RTSP 接口。
- 首机控制与遥测共用 UDP 8001，RTSP 8554、内部 RTP 7001；`interfaces.tsv` 保存控制端点、专用桥接收端口和视频 URL。
- 网关从 `/uav_1/pod/telemetry` 接收插件遥测，并把 104 字节标准帧回传到地面站控制/心跳源端点。地面站报文面板解析并转发 37 字段 JSON 到 PX4 9000，`aisheng_udp_bridge` 发布 `dyt_aisheng_telemetry`。
- 生产编解码、地面站实际转发和专用 PX4 实例的 `listener` 回环通过，包含四目标数组、原始 int32 经纬度、设备毫秒和缺失 LOS 向量 `NaN`。场景启动接线由组合机型自动选择。

## 完整飞行

`tests/run_flight_check.sh headless` 在独立运行目录生成模型，启动 Gazebo 与 PX4 SITL，执行起飞、悬停、200 km/h 巡航、300 km/h 冲刺、减速、降落和稳定性分析。

| 项目 | 结果 |
|---|---|
| 起飞/降落 | 起飞检测、降落检测和自动上锁均通过 |
| 悬停 | 最大漂移 0.104 m，最大高度误差 0.245 m，通过 |
| 200 km/h 巡航 | 连续 30.008 s，峰值 224.25 km/h，最大高度误差 0.530 m，通过 |
| 300 km/h 冲刺 | 连续 10.016 s，峰值 319.21 km/h，最大高度误差 0.350 m，通过 |
| 悬停稳定性 | roll/pitch rate error、5～35 Hz 频谱、姿态、控制分配、电机触底和镜头 LOS 全部通过 |
| 镜头 LOS | 广角相机 1920×1080、38° HFOV；P95 径向 0.67 px，最大 1.58 px，限值 6/15 px |

完整飞行报告与稳定性结果归档在 `tests/results/flight_result.json.gz` 和 `flight_stability.json`；`analyze_flight_stability.py` 对重复 ULog 时间戳按最后一条去重，并从双相机模型中选择 `camera` 广角传感器计算 LOS。

## 未覆盖

- 更高机动、连续满速、极限限位碰撞；
- 视频 SEI、多客户端控制、真实数据链；
- 目标检测/识别、红外、激光测距、地理跟踪和 OSD。
