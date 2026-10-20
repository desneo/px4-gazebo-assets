# 项目状态与继续入口

当前原生模型为机头朝上起飞、低位电机和下置桨的火箭穿越机。标准入口复制 `model.sdf`、`model.config`、`meshes/`、`px4/` 和 `tests/world.sdf` 运行，保留原生文件中的质量、重心、惯性及动力配置；测试与启动等待设置来自 `parameters.json`。配置边界见[参数说明](parameters.md)。

## 当前配置与目标

- 模型总高度0.590 m，全部显示网格与碰撞体的 z 包络为-0.225～+0.365 m；动力舱长度0.090 m及直径0.080 m、四桨直径0.254 m。
- 机身几何轴向中点为 z=+0.070 m，`base_link` 原点为 z=0；机身碰撞圆柱长度0.590 m、中心 z=+0.070 m。局部坐标和重心均以各自所属 link 为参考。
- SDF 电机 x/y=±0.2086 m，对角轴距约0.590010 m；动力舱中心 z=-0.120 m、桨盘 z=-0.185 m。
- 机身质量7.44 kg、重心 z=-0.094 m；整机质量7.70 kg、合成重心 z≈-0.095722 m。惯性矩使用 SDF 显式值。
- 四桨推力系数8.9119×10⁻⁵ N·s²/rad²、反扭矩系数0.02863386 m、最大转速1570 rad/s。PX4 四路最大转速均为1570 rad/s。
- 目标为200 km/h巡航、300 km/h冲刺连续至少10 s；默认巡航保持30 s、悬停30 s及高度容差±2 m为测试设置。

## 配套设置与验证边界

- PX4 电机x/y位置按当前SDF的±0.2086 m进行FLU→FRD转换；`CA_ROTOR*_PZ=0.185 m`保持模型原点参考。`KM`的0/1路为+0.02863386 m，2/3路为-0.02863386 m，与当前旋转方向配套。
- `MPC_THR_HOVER=0.0755504`；当前7.70 kg模型按静态推力和转速映射计算的理论值约0.0727901，悬停推力估计器关闭；该配置用于当前完整飞行通过记录。
- 四片显示翼面与动力舱连续连接，外端径向位置约0.295005 m、z=-0.140～-0.100 m、厚度0.004 m。机臂碰撞体保留径向末端0.1905 m的近似，当前不考察连接处碰撞精度，该限制不阻断无接触自由飞行的性能验证。
- 测试世界初始 `base_link` 原点 z=0.230 m，机尾离地0.005 m、顶部离地0.595 m；模型本体高度为0.590 m。
- 源 `parameters.json` 的物理字段供显式离线生成使用，不能作为当前原生基础模型的物理参数；`render_views.py` 的 JSON 几何预览也不能代表当前 SDF。
- 顶部两轴吊舱动态装配完整继承当前原生基础模型的质量、重心、惯性和动力，另加0.24 kg吊舱，组合总质量7.94 kg；单机与组合体均已完成各自当前配置的完整组件世界飞行自验证，组合证据见[独立验证说明](../../../../assembledmodels/rocket_two_axis_pod/docs/validation.md)。

当前590 mm、7.70 kg配置于2026-10-20通过完整无风组件世界飞行自验证：悬停30 s最大漂移0.122833 m，巡航≥200 km/h连续30 s，冲刺≥300 km/h连续10 s，完成减速、降落与停机。Gazebo位姿真值结果、归档原始记录及当前资产指纹见[验证说明](validation.md)。当前外形尚未获得人工确认；质量、重心、惯性、动力与气动均使用独立参数，结果不代表真实硬件性能。当前气动仍是无独立尾翼气动力的简化仿真，其他配置、风场或航迹需重新验证。

## 操作入口

从仓库根目录运行：

```sh
python3 models/vehicles/rocket_racing_quad/tests/check_model.py
python3 -m unittest discover -s models/vehicles/rocket_racing_quad/tests -p 'test_*.py' -v
sh models/vehicles/rocket_racing_quad/tests/run.sh view
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless

# 仅在需要参数化导出时显式运行，保留原生手调资产
python3 models/vehicles/rocket_racing_quad/tools/generate_model.py \
  --output-dir /tmp/rocket-export/rocket_racing_quad
```

启动器使用独立资产、PX4 etc/rootfs与Gazebo分区。默认 PX4 工作区为 `/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot`，可用 `PX4_ROOT` 覆盖；需要兼容的 PX4 SITL 构建及 Gazebo Sim。Gazebo 分区不自动隔离 PX4 实例和 MAVLink 端口，避免同时启动争用相同实例/端口的 SITL。

运行目录中的 `parameters.json` 保存测试设置、世界关键配置与 PX4 参数、各 link 质量/惯性/姿态及源文件指纹，完整世界通过 `world.sdf` 及其指纹保留。修改原生文件后需重新启动；只有对应当前配置的完整验收才能支持性能结论。
