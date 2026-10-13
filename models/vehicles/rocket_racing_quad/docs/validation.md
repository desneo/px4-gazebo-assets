# 当前配置验证结果

当前模型采用低位电机、下置桨和长三角翼面。机身轴向长度0.450 m，对角轴距0.381 m，桨径0.254 m，动力舱长度0.090 m、直径0.080 m。

## 验收状态

- 参数驱动资产生成与5项静态检查通过。
- 14项离线回归通过；SDF模型与世界校验通过。
- [模型多视图](model_views.png)为当前资产的几何预览，不是Gazebo运行截图；外形已通过人工确认。
- 当前配置完整飞行验收待复测。动力舱尺寸影响碰撞体和惯量，不能沿用不同配置的性能结论。

`validation.json`、`flight_validation.png`及`tests/results/flight_result.json.gz`的配置与当前参数不同，不作为当前配置的通过证据。仅在当前配置完整飞行验收完成后重新生成这些文件。

## 复测入口

从仓库根目录运行：

```sh
sh models/vehicles/rocket_racing_quad/tests/run_check.sh headless
# 替换为启动器输出的实际报告路径
python3 models/vehicles/rocket_racing_quad/tools/report_validation.py /tmp/rocket-racing.XXXXXX/flight_result.json
```

质量、惯量、动力和气动仍为仿真估计，不代表真实硬件性能。飞行验收覆盖无风平面世界中的直线航段。巡航目标200 km/h连续30 s，冲刺目标300 km/h连续10 s，高度容差±2 m；巡航时长和高度容差可调。模型不包含独立展示支架，机尾接地采用机身圆柱碰撞近似。
