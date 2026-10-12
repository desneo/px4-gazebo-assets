# 三轴吊舱控制器

本控制器专用于 `three_axis_pod` 中的一组三轴关节与相机，吊舱与 gimbal 一一对应。

`gz-gimbal-controller` 在 Gazebo 进程内完成 CSRT 跟踪、视觉伺服、状态机和关节速度驱动。配套 `service/` 将 gz-transport 桥接为 UDP 与 RTSP。

构建（仓库根目录）：

```bash
cmake -S models/gimbals/three_axis_pod/controller -B models/gimbals/three_axis_pod/controller/build
cmake --build models/gimbals/three_axis_pod/controller/build -j4
```

产物：`build/lib/libgz-gimbal-controller.so`。模型位于 `models/gimbals/three_axis_pod/`，装配位于 `assemblies/three_axis_pod_stand/`，演示世界位于 `models/gimbals/three_axis_pod/tests/`，启动脚本位于 `models/gimbals/three_axis_pod/scripts/`。

功能、接口、参数单位、依赖、运行方法和当前限制见 [系统说明](../docs/system.md)，检查结果见 [自验证说明](../docs/validation.md)。
