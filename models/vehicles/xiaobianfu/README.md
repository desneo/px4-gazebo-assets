# 小蝙蝠固定翼

白绿后掠无尾飞翼，翼尖竖翼、中央座舱和一副两叶尾推桨。外形以用户提供的参考图为依据，飞行系统优先复用本机 PX4 `advanced_plane` 的 AdvancedLiftDrag、电机模型及传感器结构，机体配置由 `4008_gz_advanced_plane` 调整为双升降副翼混控。来源许可证保存在 `LICENSE-PX4`，Gazebo资产许可证另存为 `LICENSE-PX4-GZ`。来源版本：PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`，gz子模块 `b6127f4ec20de867e215fb5f78ae88b80f371909`。基础 SDF 位于 `tools/px4_advanced_plane.sdf`。

整机长度（包含尾桨）0.40 m、翼展1.00 m、总起飞质量1.00 kg。目标最大持续平飞空速100 km/h（27.778 m/s）；不模拟电池容量和电压衰减。动力、气动、惯量和接地摩擦均为仿真估计，不代表真实飞机测量或适航数据。没有云台。

## 使用

依赖本机 Gazebo Harmonic、PX4 SITL 构建、Python 3 和编译插件用的 `gz-sim8` 开发包。

```sh
python3 models/vehicles/xiaobianfu/tools/generate_model.py
python3 models/vehicles/xiaobianfu/tests/check_model.py
sh models/vehicles/xiaobianfu/tests/run.sh view
sh models/vehicles/xiaobianfu/tests/run.sh headless
```

启动器在 `/tmp/xiaobianfu.*` 建立独立模型、插件、PX4配置与日志，打印 `GZ_PARTITION`。飞行控制客户端与手抛脚本必须继承该分区。另一个终端执行启动器打印的 `flight_check.py --config ...` 命令，会自动进行起飞、最大持续平飞与降落；测试脚本与配置均使用生成副本，结果默认保存在该副本的 `tests/flight_result.json`。启动前保证无其他 PX4 实例占用 MAVLink 端口。`PX4_ROOT` 可覆盖本机 PX4 路径。

## 手抛与机腹着陆

飞机在草地上静置初始化导航；进入自动起飞模式且解锁后执行：

```sh
export GZ_PARTITION=<启动器打印的分区>
sh models/vehicles/xiaobianfu/tests/hand_launch.sh
```

脚本暂停世界、按 `launch.height_m`（默认1.6m）举高，再恢复物理。HandLaunch 插件沿机体前向/上向施加0.14s外力脉冲，默认产生约14m/s前向和2m/s向上初速，用连续加速度触发PX4发射检测。脉冲结束后完全依赖 Gazebo 物理与PX4闭环控制，每次启动世界只允许触发一次，重新试验须重新启动。气动阻力和重力意味着实际抛出速度稍有偏差。

无起落架，中央机腹与翼面碰撞体接地；草地使用10km×10km、厚0.1m的大平面碰撞箱体，表面高度z=0，采用摩擦近似，不模拟草叶、地形起伏、结构损伤或桨叶破损。自动降落任务必须提供足够长、无障碍的进近路线。平面世界无测距仪，`FW_LND_USETER=0` 使用任务给定的地面地理高度，验收器根据Gazebo真值和切平面曲率计算该高度。

## 参数与人工观察

`parameters.json` 集中管理尺寸、质量、惯量、气动、推进与手抛条件，生成器重建SDF和演示世界。电机转速上限同步生成到 PX4 执行器配置。PX4 airframe 的控制器和速度范围单独维护，修改机体动力参数后需同步调整并重新验证。`docs/model_views.png` 是参数外形示意图；`docs/reference.jpg` 是参考图。

演示世界的粉色定位柱与黄色指向条为 `XIAOBIANFU_LOCATION_MARKER`，柱位于 `(0,-2,1)m`，指向原点飞机，不设置碰撞。Gazebo实体列表搜索 `xiaobianfu` 可聚焦飞机；近景建议鼠标放在翼面上，使用右键缓慢拖动微调。浏览相机建议敏感度0.1、近裁剪0.005m；灵敏度可通过 `/gui/camera/view_control/sensitivity` (`gz.msgs.Double`, `data: 0.1`) 调整，需要GUI的相机控制插件已加载；默认近景相机位置 `(1.5,-1.8,1)m`，近裁剪0.005m；在 `tools/world_template.sdf` 的GUI MinimalScene配置修改并重新生成。吊舱视频相机未配置。

## 验证状态

当前外形已由用户在Gazebo中人工确认，确认记录及外形指纹见 `docs/appearance.json`。多视图PNG仅用于辅助查看。

当前参数的模型质量/结构、SDF合法性、手抛插件编译与PX4完整自动飞行验收已通过。无风测试中完成手抛自动起飞、满油门最大持续平飞、正常机腹着陆、停稳并自动停机；详细结果见 `docs/validation.md`；真值速度、高度、接地前姿态/下降速度及配置哈希见 `docs/validation.json`，原始遥测保存在 `tests/results/flight_result.json.gz`，过程图见 `docs/flight_validation.png`。

验收采用30s连续仿真时间窗口、高度±2m、真值速度位于[目标,目标×1.02]并保持满油门；这些是可调整的仿真测量设置。首次距地0.15m前记录接地状态，要求下降速度≤1.8m/s、bank≤15°、pitch在[-2°,20°]；接地后要求真值速度低于0.3m/s、姿态正常并停机连续3秒。有限时长仿真不能证明真实飞机的无限续航或真实性能。
