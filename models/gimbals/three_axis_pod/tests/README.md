# 三轴吊舱演示环境

`world.sdf` 世界名 `three_axis_pod_test`，含地面平面、棋盘纹理 `ground_tex`、固定吊舱装配 `three_axis_pod_stand`（实例名 `three_axis_pod`）和小车 `test_car`（初始位置 0,60,0.55 m）。物理步长 0.004 s，不启动 PX4。

这里的支架装配为动态模型，通过重力和底座摩擦落地，未用世界固定关节锁定。吊舱采用立装 `skyward`；机械零位朝天，执行 `reset` 后公共三轴为 0°，指向静态参考水平前向且画面水平。`check_commands.py` 使用 `attitude_deg` 公共角度断言，目标纹理与正天到位条件仅适用于默认立装棋盘世界，不适用于悬挂或地理场景。

资源：`models/gimbals/three_axis_pod`、`assemblies/three_axis_pod_stand`、`models/props/ground_tex` 与 `models/props/test_car`；棋盘网格的纹理随模型保存。小车包含 Gazebo VelocityControl 插件，轮子仅用于显示，命令话题见其 model.sdf；演示世界未提供自动运动轨迹。

启动和网关见 [组件使用说明](../scripts/README.md)。该演示环境与真实地理背景环境分别维护。

单轴复位在线验证：启动包含当前控制插件和网关的场景后，运行 `python3 models/gimbals/three_axis_pod/tests/check_axis_reset.py`；远端或独立端口使用 `--host`、`--port`。脚本会先偏移公共三轴姿态，再逐轴发送 `reset_axis`，检查回执、公共零位及其他两公共姿态角保持，默认结果保存到 `runs/axis_reset_validation/results.json`。测试会转动吊舱，结束后停转，不恢复原视线；该检查不依赖棋盘目标纹理。
