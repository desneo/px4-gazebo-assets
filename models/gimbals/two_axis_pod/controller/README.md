# 两轴立装吊舱控制器

专用 Gazebo Harmonic C++ 插件直接驱动 `yaw_joint`、`pitch_joint`，类名为 `two_axis::GimbalController`，库名为 `gz-two-axis-pod-controller`。相机位于 `camera_link`，传感器名为 `camera`，安装 link 为 `mount_link`。

```bash
cmake -S models/gimbals/two_axis_pod/controller -B models/gimbals/two_axis_pod/controller/build -DCMAKE_BUILD_TYPE=Release
cmake --build models/gimbals/two_axis_pod/controller/build -j4
ctest --test-dir models/gimbals/two_axis_pod/controller/build --output-on-failure
export GZ_SIM_SYSTEM_PLUGIN_PATH=models/gimbals/two_axis_pod/controller/build/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}
```

依赖 Gazebo Sim 8、Transport 13、Msgs 10、Plugin 2 及 OpenCV（core/imgproc/video/tracking）。JSON、CSRT/模板重捕获及基础矩阵运算源码通过相对路径复用三轴控制器，必须保留仓库中的 `three_axis_pod/controller`；两轴控制逻辑独立。

## 机械约定

水平安装面法线为 +Z，+X 为参考前方、+Y 朝左。外轴 `yaw` 绕 +Z 旋转，内轴 `pitch` 绕随动 +Y 旋转，旋转链为 `Rz(yaw) Ry(pitch)`。两轴零位时镜头朝 +Z 天空，pitch 正值朝前，+90° 朝 +X；pitch −45° 指向后上方。yaw 正值把前向视线从 +X 转向 +Y。

硬限位 yaw ∈ [−90°, +90°]、pitch ∈ [−45°, +90°]。`mount_mode` 必须为 `upright`。姿态显示、手动输入、回零全部采用机械角约定；home/reset 回到 (0°, 0°)。reset_axis 只复位选定机械轴，另一轴保持执行指令时的采样角度。

## 控制与保护

手动命令单位为 °/s。速度默认上限 100°/s，加速度默认上限 160°/s²。加速度平滑之后按剩余行程限制向外速度，保留 0.2° 软件端点缓冲及两倍控制间隔的前瞻余量，避免离散速度指令撞击物理硬限位；允许反向退出。停止与超时保护立即清零速度。手动指令超过 0.5 秒未续发即停止。

跟踪采用 OpenCV CSRT，短时遮挡采用模板匹配重捕获。像素误差转换为图像右/下方向角误差，经比例增益（默认 4）及阻尼最小二乘 Jacobian 转换为关节速度，再施加速度、加速度和机械限位约束；不使用目标仿真真值。图像断帧超过 0.5 秒立即停止，超过默认 2.5 秒恢复窗口进入 LOST，需要重新选框。连续 1 秒中心误差小于 1° 标记稳定，误差大于 2° 标记不稳定。无效关节采样进入 FAULT，恢复采样后可用 reset_fault 清除。

`pitch = 0°` 时视线方向与 yaw 轴平行，yaw 不能改变朝天方向；球心随支架移动，画面方向随偏航旋转。此时不具备完整的二维视线方向控制能力。控制器通过阻尼抑制退化附近的 yaw 速度，遥测在 `abs(sin(pitch)) < 0.05` 时报告 `singular: true`。机械手动控制与回零仍有效。

## Gazebo Transport

- 命令：`/two_axis_pod/command`，`gz.msgs.StringMsg` JSON。
- 回执：`/two_axis_pod/command_result`，包含 `type/cmd/ok/message/request_id`，request_id 在请求携带时原样返回。
- 遥测：`/two_axis_pod/telemetry`，默认 50 Hz；控制默认 100 Hz。

命令支持 track（归一化 `box:[x,y,w,h]`）、manual（`yaw/pitch` 机械速度）、stop、home、reset、reset_axis（axis 为 yaw/pitch）、reset_fault、ping。任何 manual 的 roll 字段均拒绝，包括零值。

遥测 `protocol_version:4`、`angle_convention:"upright_joint"`；`attitude_deg`、`joints_deg` 只有 yaw/pitch 的 pos/vel/acc，`cmd_dps` 只有 yaw/pitch。`limits` 包含 yaw/pitch/saturated，`los_mount` 是安装系单位视线，顶层 `singular` 表示视线退化。`camera` 包含相机传感器相对安装系/世界的四元数与欧拉姿态及帧序号，不将机械轴角误称为世界偏航/俯仰。tracking、target、stability、frame 保留配套网关所需信息。网络接口见 [网关说明](service/README.md)。

测试覆盖旋转链、有限差分 Jacobian、退化、手动超时、双轴限位、单轴回零、CSRT 跟踪及断帧丢失保护。
