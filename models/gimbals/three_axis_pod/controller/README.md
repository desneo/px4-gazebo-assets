# 三轴吊舱控制器

本控制器专用于 `three_axis_pod` 中的一组三轴关节与相机，吊舱与 gimbal 一一对应。

`gz-gimbal-controller` 在 Gazebo 进程内完成 CSRT 跟踪、视觉伺服、状态机和关节速度驱动。配套 `service/` 将 gz-transport 桥接为 UDP 与 RTSP。

协议版本 2 使用 `angle_convention=px4_frd`：控制输入 `manual.yaw/pitch/roll` 为公共 Euler 角速度（°/s），不是目标角度；`attitude_deg` 为公共姿态及其角速度/加速度。公共零位为水平朝前、画面水平，yaw 右转为正、pitch 抬头为正、roll 沿视线顺时针为正。插件根据实际静态安装与 SDF 传感器光学姿态换算机械三轴，并处理耦合、限位和 Euler 奇异。

`joints_deg` 和 `cmd_dps` 为机械诊断：pitch/roll 的 `pos` 是机械位置，yaw 的 `pos_raw` 为连续关节角，`pos` 为 `wrap(pos_raw−180°)`。公共 `reset_axis` 不以这些诊断值为回零目标。静态前向由 `forward_az_deg` 指定；没有动态载机姿态补偿或 PX4 联动。天顶/天底附近不可独立定义 yaw 与 roll，集成端需检查 `attitude_deg.singular`，具体边界见[控制协议](../docs/control-protocol.md)。

构建（仓库根目录）：

```bash
cmake -S models/gimbals/three_axis_pod/controller -B models/gimbals/three_axis_pod/controller/build
cmake --build models/gimbals/three_axis_pod/controller/build -j4
```

产物：`build/lib/libgz-gimbal-controller.so`。模型位于 `models/gimbals/three_axis_pod/`，装配位于 `assemblies/three_axis_pod_stand/`，演示世界位于 `models/gimbals/three_axis_pod/tests/`，启动脚本位于 `models/gimbals/three_axis_pod/scripts/`。

功能、接口、参数单位、依赖、运行方法和当前限制见 [系统说明](../docs/system.md)，检查结果见 [自验证说明](../docs/validation.md)。
