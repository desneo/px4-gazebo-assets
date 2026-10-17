# 两轴立装吊舱地理背景测试

本场景使用 `worlds/geo_32_217857_119_105878/world.sdf` 的完整运行副本，保留全部 400 块地形和影像。`prepare_world.py` 将环境资产路径解析为绝对路径，再内联 `models/gimbals/two_axis_pod/model.sdf`。原环境和组件源模型均不由本场景改写。

吊舱底部圆环在 `(0, 0, 50)` 固定于世界，Ø84 mm 圆环与地面平行，导引头整体向上高 91 mm。圆环下方的蓝色水平台面 `pod_mount_fixture` 是独立观察夹具，顶面高度 50 m，不属于吊舱外形和尺寸。安装姿态为单位旋转，使用已确认的立装运动链 `Rz(yaw) Ry(pitch)`。俯仰零位朝天，正值朝前，限位 −45°～+90°；偏航绕竖直 +Z，限位 ±90°。该安装与限位下镜头无法指向安装水平以下，因此测试对象为空中夹具，不使用地面小车，也不通过转动安装方向改变可视范围。

动态飞机靶标 `tracking_target` 的初始中心为 `(30, 0, 70)`，使用仓库内小蝙蝠固定翼外形放大到约 5 m 翼展，关闭重力。它由 Gazebo VelocityControl 驱动，只作为视觉测试夹具，不代表飞行动力学。速度话题为 `/model/tracking_target/cmd_vel`，里程计话题为 `/model/tracking_target/odometry`。`detection_aircraft_left` 与 `detection_aircraft_right` 是无碰撞静态飞机夹具，和动态目标共同验证多目标稳定编号及中心最近自动锁定。

实体遮挡板名为 `occlusion_panel`，尺寸 `0.2 × 20 × 20 m`；隐藏位置为 `(100, 100, 80)`，遮挡位置为 `(15, 0, 60)`，横跨初始吊舱与靶标视线。

## 启动与观察

在仓库根目录运行，需先按组件说明构建专用控制插件：

```bash
bash scenarios/geographic_two_axis_pod/run.sh --gui
GZ_PARTITION=geographic_two_axis_pod python3 scenarios/geographic_two_axis_pod/check_target.py --prepare-only
bash models/gimbals/two_axis_pod/ground_station/run.sh
```

启动后机械零位朝天；先执行 `check_target.py --prepare-only`，将相机指向靶标并完成测试初始化，再打开地面站观察运动与视频。启动器不会自动调用测试初始化。测试无需 PX4，也无需载机装配。

默认 partition 为 `geographic_two_axis_pod`，运行目录为 `runs/geographic_two_axis_pod/`，UDP 为 `8766`，RTSP 为 `8555`，内部 RTP 自动分配，RTSP 挂载点为 `/two_axis_pod_1`（`--aircraft-id` 可配置）。视频强制使用 NVENC/NVDEC；不可用时网关或地面站明确报错退出。`GZ_PARTITION` 和 `GIMBAL_RUNDIR` 环境变量可覆盖默认值；其余参数传给组件网关。多机可共用 RTSP 8555，必须传入不同 `--aircraft-id` 并隔离控制端口和 Gazebo 话题；地面站配置对应 ID 与控制端口。

额外 UDP JSON 状态副本可在组件 `controller/service/status_udp.json` 中启用，也可传入 `--status-ip 192.168.1.100 --status-port 9000 --status-hz 20`。此输出使用独立 socket 和发送循环，无需接收端登记，不改变原 station 服务，见[网关配置](../../models/gimbals/two_axis_pod/controller/service/README.md#额外-udp-json-状态上报)。

```bash
# 完整视觉检查；不要同时运行其他 UDP 控制客户端。
GZ_PARTITION=geographic_two_axis_pod python3 scenarios/geographic_two_axis_pod/check_target.py
# 飞机检测、处理速率、三目标自动锁定与锁定后不换目标。
python3 scenarios/geographic_two_axis_pod/check_vision.py
# 保留检测模式后，用 NVDEC 保存带服务端红框和编号的 RTSP 帧。
python3 scenarios/geographic_two_axis_pod/check_vision.py --leave-mode detect
bash models/gimbals/two_axis_pod/ground_station/run.sh \
  --decode-test 5 --snapshot runs/geographic_two_axis_pod/vision_nvdec.png
# 停止场景服务（沿用本次启动的运行目录）。
GIMBAL_RUNDIR="$PWD/runs/geographic_two_axis_pod" bash models/gimbals/two_axis_pod/scripts/stop_bridge.sh
GIMBAL_RUNDIR="$PWD/runs/geographic_two_axis_pod" bash models/gimbals/two_axis_pod/scripts/stop_plugin_sim.sh
```

定位标志 `POD_LOCATION_MARKER` 位于吊舱旁：橙黑相间高杆、洋红色发光球和黄色十字牌，顶部黄色箭头指向 `(0,0,50)` 附近的吊舱。标志仅用于观察，没有碰撞体；灯球及箭头在镜头下方。可在实体树中查找 `POD_LOCATION_MARKER` 定位附近区域，再选择 `two_axis_pod` 聚焦查看水平底环及其上方的罩壳、支架和相机。

模型内联在运行世界中；修改外形后须停止本运行目录的 Gazebo 服务端和网关，再重新运行启动命令。只重新打开 GUI 不会更新服务端已加载的模型。启动器发现已有服务时，在生成世界之前退出，避免磁盘文件与运行模型不一致。

Gazebo GUI 默认从 `(-15, -35, 65)` 观察吊舱与靶标全景；近距离检查吊舱外形可在 GUI 中选择 `two_axis_pod` 并聚焦。停止服务后可手动关闭 Gazebo GUI 和地面站窗口。日志与生成的 `world.sdf` 均位于运行目录。

## 验收与地面站实窗检查

```bash
# 40 仿真秒：前进、倒退、转弯、反向转弯各10秒；随后两轮实体遮挡。
python3 scenarios/geographic_two_axis_pod/check_target.py --test all
# 单独查看实际地面站运动跟踪，10秒后自动保存截图并关闭地面站。
python3 scenarios/geographic_two_axis_pod/check_target.py \
  --observe-seconds 10 --output runs/geographic_two_axis_pod/observe.jsonl
```

测试脚本默认连接该场景分区。移动测试以 0.5 m/s 完成四段轨迹，转弯角速度为 ±0.06 rad/s。验收要求：有效 TRACK 比例至少95%、中心角误差与独立靶标中心真值角差 P95 均不超过3°、累计移动距离至少10米、无重力靶标高度变化小于0.1米。里程计仅用于验收，不参与图像跟踪控制。遮挡测试每轮保持实体遮挡2秒墙钟，要求出现 TEMP_LOST、不进入终止 LOST、移除遮挡后恢复TRACK并持续稳定1秒。`check_vision.py` 要求网关上报全部视觉能力、至少识别一个飞机目标、视觉输出速率不低于相机输入的 80%，自动锁定进入 TRACK 且两秒内不换轨迹。

记录保存到 `runs/geographic_two_axis_pod/target.jsonl` 和 `target.summary.json`；实窗观察使用独立 `observe.jsonl`，避免覆盖验收结果。地面站截图为 `ground_station.png`。观察脚本调用同一个 `run.sh`；入口自动配置 `.venv-linux`，不依赖仓库外的固定虚拟环境路径。

单独测试可使用 `--test moving --output runs/geographic_two_axis_pod/moving.jsonl` 和 `--test occlusion --output runs/geographic_two_axis_pod/occlusion.jsonl`，各自生成同名 `.summary.json`。实体遮挡计时期间避免并行导出完整世界等耗时服务请求，以免延迟遮挡板移除，超过默认 2.5 秒恢复窗口。

实测结果及地面站截图见[两轴组件验证资料](../../models/gimbals/two_axis_pod/docs/validation.md#完整地理背景部件测试)。

## 小尺寸吊舱近景查看

浏览相机滚轮缩放根据鼠标指向的三维位置确定参考距离。近距离查看时把鼠标放在吊舱实体表面；指向远处地形或天空会增大步长。右键按住上下缓慢拖动比滚轮逐格缩放更适合微调，中键拖动可绕鼠标按下时的参考点旋转。

场景GUI启动后将鼠标灵敏度设为 `0.1`（原生默认值为 `1`），同时影响缩放、平移和旋转。可用 `GZ_VIEW_SENSITIVITY` 改写，例如 `GZ_VIEW_SENSITIVITY=0.05 bash scenarios/geographic_two_axis_pod/run.sh --gui`。当前窗口可随时调整，无需重启：

```bash
GZ_PARTITION=geographic_two_axis_pod gz service \
  -s /gui/camera/view_control/sensitivity \
  --reqtype gz.msgs.Double --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: 0.1'
```

`data: 1` 恢复原生速度；`data: 0.05` 进一步减慢。浏览相机近裁剪距离为 `0.005 m`，便于观察直径 48 mm 的相机球体；该设置在加载生成的world后生效，与吊舱视频相机的裁剪参数独立。修改源配置不改变已经运行窗口的裁剪面。
