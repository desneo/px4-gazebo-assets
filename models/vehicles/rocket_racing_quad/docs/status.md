# 项目状态与继续入口

当前模型为低位电机、下置桨、四片长三角翼面的火箭穿越机，动力舱直径80 mm。[多视图](model_views.png)供外形检查；当前外形已通过人工验收，完整飞行验收待复测。结果与入口见[验证说明](validation.md)。

## 已确认的尺寸与目标

- 机头朝上起飞，推力沿机身轴线；高速时整机倾转，机头接近前进方向。
- 机头到机尾长度 L=0.450 m、对角电机轴距0.381 m、四桨直径0.254 m。
- 电机整流罩中心 z=-0.120 m，长度0.090 m、直径0.080 m；桨盘 z=-0.185 m，距机尾40 mm。电机罩在桨盘上方，四片长三角翼连接机身与电机座。
- 巡航达到200 km/h、冲刺300 km/h连续至少10 s；巡航保持30 s、高度容差±2 m为可调测试设置。
- 参数集中在 `parameters.json`，用途与依赖关系见[参数说明](parameters.md)。当前动力与气动仍是仿真估计。

## 验收状态

静态检查与离线回归通过。80 mm 动力舱外形已通过人工验收；尺寸变化影响碰撞体与惯量，悬停、巡航、冲刺、高度保持、减速与落地需使用当前配置复测。

当前基础模型使用50 rad/s正解锁怠速、未解锁输出0及相应悬停推力映射；在完整地理场景通过80 m起飞、95 m/s控制上限容差内的连续10秒稳定平飞、减速与降落。单机及组合体速度数据见[对照结果](../../../../scenarios/rocket_speed_comparison/docs/validation.md)。默认平面世界中的完整悬停/巡航/冲刺自验证未由此对照替代。

## 操作入口

从仓库根目录运行：

```sh
python3 models/vehicles/rocket_racing_quad/tools/generate_model.py
python3 models/vehicles/rocket_racing_quad/tests/check_model.py
python3 -m unittest discover -s models/vehicles/rocket_racing_quad/tests -p 'test_*.py' -v
python3 models/vehicles/rocket_racing_quad/tools/render_views.py
sh models/vehicles/rocket_racing_quad/tests/run.sh view
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless
```

启动器使用独立资产、PX4 etc/rootfs与Gazebo分区，不修改相邻PX4源码。默认PX4工作区为 `/home/aeroarx/test/codex_gimbal_test/PX4-Autopilot`，当前兼容PX4 v1.17 SITL和Gazebo Sim 8.15.0，可用 `PX4_ROOT` 覆盖。Gazebo分区不自动隔离PX4实例和MAVLink端口，避免同时启动争用相同实例/端口的SITL。

修改配置后需重新启动；只有当前配置的完整验收才能支持性能结论。当前外形已通过人工确认。外部代码放项目上一级，spec-kit产物忽略且不提交，其他项目的未提交修改不应覆盖。
