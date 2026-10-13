# 测试小车

带车顶棋盘纹理的动态刚体目标，由 Gazebo `VelocityControl` 插件驱动。轮子为视觉模型，没有轮关节或 DiffDrive 动力学。速度命令话题为 `/model/test_car/cmd_vel`，消息类型为 `gz.msgs.Twist`。

组件演示世界中的初始位置为 `(0,60,0.55)` 米，见[演示世界](../../gimbals/three_axis_pod/tests/README.md)。[地理相机场景](../../../scenarios/geographic_camera/README.md)将小车实例命名为 `tracking_target`，提供碰撞平台、实际位移遥测和前进/倒车/转弯的跟踪验收脚本。
