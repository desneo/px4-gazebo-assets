# 三轴吊舱演示环境

`world.sdf` 世界名 `three_axis_pod_test`，含地面平面、棋盘纹理 `ground_tex`、固定吊舱装配 `three_axis_pod_stand`（实例名 `three_axis_pod`）和小车 `test_car`（初始位置 0,60,0.55 m）。物理步长 0.004 s，不启动 PX4。

资源：`models/gimbals/three_axis_pod`、`assemblies/three_axis_pod_stand`、`models/props/ground_tex` 与 `models/props/test_car`；棋盘网格的纹理随模型保存。小车包含 Gazebo DiffDrive 插件，命令话题见其 model.sdf；演示世界未提供自动运动轨迹。

启动和网关见 [组件使用说明](../scripts/README.md)。该演示环境与真实地理背景环境分别维护。
