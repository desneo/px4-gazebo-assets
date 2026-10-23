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

水平安装面法线为 +Z，+X 为载机参考前方、+Y 朝左。外轴 `yaw` 绕安装 −X 旋转，经过 `(0, 0, 0.056) m`；内轴 `pitch` 绕随动 −Y 旋转，零位轴心为 `(0.015, 0, 0.056) m`。旋转链为 `Rx(-yaw) Ry(-pitch)`，轴名表示机械关节，不表示世界 Euler 偏航和俯仰。两轴零位时光轴朝安装 +Z，pitch 负值朝前，正值朝后。yaw 正值使零位光轴从 +Z 向 +Y 转动。

确认的机械范围为 yaw ∈ [−60°, +60°]、pitch ∈ [−80°, +30°]。插件默认范围与组件一致，也可从 SDF 读取有限、包含机械零位的有效区间；物理关节与控制配置必须保持一致。`mount_mode` 必须为 `upright`，表示底座水平向下。姿态显示、手动输入、回零全部采用机械角约定；home/reset 回到 (0°, 0°)。reset_axis 只复位选定机械轴，另一轴保持执行指令时的采样角度。

相机 sensor 相对 `camera_link` 的 RPY 为 `(0, −π/2, 0)`，sensor +X 是视线、+Y 是图像左、+Z 是图像上。零位时图像左为安装 +Y、图像上为安装 −X，因此图像右/下分别为随动 −Y/+X。插件读取实际 sensor 局部旋转，与机械旋转各组合一次，得到相机光学姿态和视线遥测；不能另在 link 或插件中重复应用 CAD 到安装系的转换。图像伺服仅支持该 CAD 固定光学基底；实际旋转矩阵与预期各元素偏差超过 `1e-8` 或存在非有限值时，插件明确报错并禁用。

## 控制与保护

手动命令单位为 °/s。两轴速度上限均为 20°/s，加速度默认上限保持 160°/s²。该机械轴速度限制作用于手动、回零、单轴复位和自动跟踪，包括载机补偿叠加后的最终控制输出；配置可收紧，不能提高 20°/s 的硬上限。运行中降低配置会立即约束已有速度指令，旧场景配置大于 20 时插件明确告警并按 20 执行。加速度平滑之后按剩余行程限制向外速度，保留 0.2° 软件端点缓冲及两倍控制间隔的前瞻余量，避免离散速度指令撞击物理硬限位；允许反向退出。停止与超时保护立即清零速度。非零手动指令超过 0.5 秒未续发即以“控制输入超时”安全停止；显式双轴零速是正常松手停车，立即退出手动模式且不产生超时提示。

跟踪采用 OpenCV CSRT，短时遮挡采用模板匹配重捕获。`track` 可携带 `frame_seq`，插件在最近 4 个已处理图像中按序号精确选帧，使网关自动检测框和 CSRT 初始化使用同一图像；手动画框不携带序号时仍使用最新帧。像素误差转换为图像右/下方向角误差，经比例增益（默认 4）及阻尼最小二乘 Jacobian 转换为关节速度，再施加速度、加速度和机械限位约束；不使用目标仿真真值。图像断帧超过 0.5 秒立即停止，超过默认 2.5 秒恢复窗口进入 LOST，需要重新选框。连续 1 秒中心误差小于 1° 标记稳定，误差大于 2° 标记不稳定。无效关节采样进入 FAULT，恢复采样后可用 reset_fault 清除。

`mount_dynamic` 默认为 false，沿用固定安装参考。机载装配设置为 true 时，每步读取 `mount_link` 世界姿态，更新安装四元数和相机世界姿态；TRACK 中把载机角速度对视线的影响投影到实际两轴 Jacobian，加入反向前馈。目标平移和载机平移仍由图像误差闭环修正，不读取目标世界坐标。大于 0.5 rad 的单步姿态跳变不转换为角速度前馈。进入机械限位、输出速度受限或视线退化时明确报告约束，不能将 CSRT 仍识别选框解释为稳定锁定。

视线方向为 `(-sin(pitch), sin(yaw)cos(pitch), cos(yaw)cos(pitch))`。图像右/下方向的关节 Jacobian 为 `diag(-cos(pitch), -1)`，yaw 使用阻尼最小二乘逆。两轴零位不退化；当视线平行外轴、`abs(cos(pitch)) < 0.05` 时，yaw 无法独立改变视线方向，遥测报告 `singular: true`。确认的机械范围不包含该退化姿态。机械手动控制与回零仍有效。

## Gazebo Transport

- 命令：`/two_axis_pod/command`，`gz.msgs.StringMsg` JSON。
- 回执：`/two_axis_pod/command_result`，包含 `type/cmd/ok/message/request_id`，request_id 在请求携带时原样返回。
- 遥测：`/two_axis_pod/telemetry`，与网关共用 `service/telemetry.json` 的 `hz`，默认 50 Hz（单调真实时间）；控制默认 50 Hz（仿真时间）。

插件从 SDF `telemetry_config` 指向的 JSON 读取 `hz`，默认 `model://two_axis_pod/controller/service/telemetry.json`。配置缺失或非法时明确报错并禁用插件；修改后需重启仿真及网关。遥测调度使用 `steady_clock`，不由 `UpdateInfo.dt` 或仿真实时因子决定发送目标。只有新的物理状态可发布，单步最多产生一份新姿态；暂停不发布，恢复和仿真时间重置不补发历史周期。物理更新的真实频率不足配置 `hz` 时限频告警。完整配置与发送行为见[网关说明](service/README.md)。

三个话题可用 SDF 的 `command_topic/telemetry_topic/command_result_topic` 配置。图像默认订阅由世界、模型、link、sensor 名拼接的完整路径，可用 `image_topic` 对应相机显式 topic。

命令支持 track（归一化 `box:[x,y,w,h]`）、manual（`yaw/pitch` 机械速度）、stop、home、reset、reset_axis（axis 为 yaw/pitch）、reset_fault、ping。任何 manual 的 roll 字段均拒绝，包括零值。

遥测 `protocol_version:6`、`angle_convention:"motion_head_joint"`；`attitude_deg`、`joints_deg` 只有 yaw/pitch 的 pos/vel/acc，`cmd_dps` 只有 yaw/pitch。`limits` 包含 yaw/pitch/saturated，`los_mount` 是安装系单位视线，顶层 `singular` 表示视线退化。`camera` 包含相机传感器相对安装系/世界的四元数与欧拉姿态及帧序号，不将机械轴角误称为世界偏航/俯仰。tracking、target、stability、frame 保留配套网关所需信息。模型、插件、网关及地面站须共同使用协议 v6 和同一机械角约定。网络接口见 [网关说明](service/README.md)。

测试覆盖旋转链、有限差分 Jacobian、退化、动态安装角速度估计与独立积分后的世界视线漂移、手动超时、双轴限位、单轴回零、CSRT 跟踪及断帧丢失保护。
