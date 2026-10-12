# 三轴吊舱

`three_axis_pod` 是包含相机、三轴云台、控制插件、网络网关及地面站的完整组件。吊舱与内置云台一一对应。

相机与网格随模型保存；零位朝天，yaw/pitch/roll 关节由 `gz-gimbal-controller` 直接速度驱动。相机为 1920×1080@30Hz、HFOV 60°。pitch 范围 −135°～45°，roll 范围 −45°～45°，yaw 连续旋转。

- [controller/](controller/README.md)：C++ 控制插件、Python 网关、模型生成工具与控制单元测试。
- [ground_station/](ground_station/README.md)：配套 UDP/RTSP 桌面地面站。
- [scripts/](scripts/README.md)：组件启动、停止及运行配置。
- [tests/](tests/README.md)：组件演示世界和在线控制自测。
- [docs/system.md](docs/system.md)：功能、协议、依赖与当前限制。
- [docs/validation.md](docs/validation.md)：功能自验证结果与复现命令。

运行前先构建控制插件，再按使用说明启动仿真、网关和地面站。固定支架装配模型见 [three_axis_pod_stand](../../../assemblies/three_axis_pod_stand/README.md)。
