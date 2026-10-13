# 三轴吊舱

`three_axis_pod` 是包含相机、三轴云台、控制插件、网络网关及地面站的完整组件。吊舱与内置云台一一对应。

**集成前必须由人工显式确认安装方式。** Agent 将本组件集成到新模型、载机或项目时，须确认使用立装（底座朝下，`skyward`）还是吊装（底座朝上、悬挂，`downward`）。已有明确确认可沿用；未确认时必须询问，不得自行采用默认值、根据“倒装”等称呼推断，或以自动感知代替人工确认。确认后同步配置物理安装 pose、`mount_mode` 和相机光学姿态，具体见[安装确认与配置](docs/mounting.md#集成前安装确认)。

相机与网格随模型保存；默认 `mount_mode=skyward`（底座朝下、机械关节全零时朝天），也支持 `downward` 悬挂安装（对应物理安装下，机械关节全零时朝地）。集成时应同时设置模型安装 pose，参数本身不会旋转模型。yaw/pitch/roll 关节由 `gz-gimbal-controller` 直接速度驱动。相机为 1920×1080@30Hz、HFOV 60°。pitch 机械范围 −135°～45°，roll 机械范围 −45°～45°，yaw 连续旋转。

UDP 协议版本为 2，`angle_convention=px4_frd`。`manual` 输入与 `attitude_deg` 遥测使用公共姿态：零位水平朝前、画面水平，yaw 向右为正、pitch 抬头为正、roll 沿视线顺时针为正。静态水平前向由 `forward_az_deg` 指定，不随载机运动更新。`reset` 回公共三轴 0°，`reset_axis` 回指定公共角 0°并保持其他公共角，`home` 朝可达的世界上方。`joints_deg/cmd_dps` 保留机械诊断定义，不可作为公共姿态使用。天顶/天底附近 Euler 奇异与可达边界见[控制协议](docs/control-protocol.md)。

- [controller/](controller/README.md)：C++ 控制插件、Python 网关、模型生成工具与控制单元测试。
- [ground_station/](ground_station/README.md)：配套 UDP/RTSP 桌面地面站。
- [scripts/](scripts/README.md)：组件启动、停止及运行配置。
- [tests/](tests/README.md)：组件演示世界和在线控制自测。
- [docs/system.md](docs/system.md)：功能、协议、依赖与当前限制。
- [docs/control-protocol.md](docs/control-protocol.md)：全部 10 条控制指令、JSON 字符串报文示例与会话/回执规则，含 yaw/pitch/roll 独立回公共姿态零位。
- [docs/validation.md](docs/validation.md)：功能自验证结果与复现命令。
- [docs/mounting.md](docs/mounting.md)：安装示意图与 PX4 对外角度约定，包含当前实现边界。

运行前先构建控制插件，再按使用说明启动仿真、网关和地面站。固定支架装配模型见 [three_axis_pod_stand](../../../assemblies/three_axis_pod_stand/README.md)。
