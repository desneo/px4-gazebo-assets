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

水平安装面法线为 +Z，+X 为载机参考前方、+Y 朝左。外轴 `yaw` 绕安装 +X 旋转，经过 `(0, 0, 0.056) m`；内轴 `pitch` 绕随动 −Y 旋转，零位轴心为 `(0.015, 0, 0.056) m`。旋转链为 `Rx(yaw) Ry(-pitch)`，轴名表示机械关节，不表示世界 Euler 偏航和俯仰。两轴零位时光轴朝安装 +Z，pitch 负值朝前，正值朝后。yaw 正值使零位光轴从 +Z 向图像右侧 −Y 转动，负值向图像左侧 +Y 转动；左右按从吊舱后方沿镜头视线定义。插件启动时校验实际关节局部轴向为 yaw=+X、pitch=−Y，拒绝旧轴向模型。

确认的机械范围为 yaw ∈ [−60°, +60°]、pitch ∈ [−80°, +30°]。插件默认范围与组件一致，也可从 SDF 读取有限、包含机械零位的有效区间；物理关节与控制配置必须保持一致。`mount_mode` 必须为 `upright`，表示底座水平向下。姿态显示、手动输入、回零全部采用机械角约定；home/reset 回到 (0°, 0°)。reset_axis 只复位选定机械轴，另一轴保持执行指令时的采样角度。

相机 sensor 相对 `camera_link` 的 RPY 为 `(0, −π/2, 0)`，sensor +X 是视线、+Y 是图像左、+Z 是图像上。零位时图像左为安装 +Y、图像上为安装 −X，因此图像右/下分别为随动 −Y/+X。插件读取实际 sensor 局部旋转，与机械旋转各组合一次，得到相机光学姿态和视线遥测；不能另在 link 或插件中重复应用 CAD 到安装系的转换。图像伺服仅支持该 CAD 固定光学基底；实际旋转矩阵与预期各元素偏差超过 `1e-8` 或存在非有限值时，插件明确报错并禁用。

## 控制与保护

手动命令单位为 °/s。两轴速度上限均为 20°/s，加速度默认上限保持 160°/s²。该机械轴速度限制作用于手动、回零、单轴复位和自动跟踪，包括载机补偿叠加后的最终控制输出；配置可收紧，不能提高 20°/s 的硬上限。运行中降低配置会立即约束已有速度指令，旧场景配置大于 20 时插件明确告警并按 20 执行。加速度平滑之后按剩余行程限制向外速度，保留 0.2° 软件端点缓冲及两倍控制间隔的前瞻余量，避免离散速度指令撞击物理硬限位；允许反向退出。停止与超时保护立即清零速度。非零手动指令超过 0.5 秒未续发即以“控制输入超时”安全停止；显式双轴零速是正常松手停车，立即退出手动模式且不产生超时提示。

控制调度使用仿真时间并保留未满一个周期的余相。默认100 Hz控制在250 Hz物理步下按8/12 ms交替运行，长期次数与配置频率一致；大物理步丢弃无法补跑的周期但保留余相，每个新物理状态至多运行一次。暂停、恢复及仿真时间回退会重新建立基准，不补跑旧周期。该调度不改变IDLE、stop、手动看门狗和跟踪断帧语义，也不引入独立STABILIZE模式。

`tracking_backend` 默认为 `sim_truth`，只有显式配置 `csrt` 才启用 OpenCV CSRT 和模板重捕获。`track` 可携带 `frame_seq`，插件在最近 4 个已处理图像中按序号精确选帧：真值模式在该帧选框内绑定离框中心最近的已登记飞机实体，CSRT 模式则用该帧初始化纹理跟踪。绑定后真值模式只投影同一实体，不因其他飞机经过而换锁；其距离缩放框上下左右各扩展 5 px，再裁剪到图像边界，作为显式 `target.box_px` 跟踪框。实体在相机裁剪范围且该框与画面相交时持续输出，物理遮挡不会造成失败。像素误差经比例增益（默认 4）及阻尼最小二乘 Jacobian 转换为关节速度，再施加速度、加速度和机械限位约束。图像断帧超过 0.5 秒立即停止；绑定实体离开视野、删除或持续断帧超过默认 2.5 秒恢复窗口后进入 LOST，需要重新选框。连续 1 秒中心误差小于 1° 标记稳定，误差大于 2° 标记不稳定。无效关节采样进入 FAULT，恢复采样后可用 reset_fault 清除。

`sim_truth_target_name` 可重复配置，列出本场景允许绑定的飞机顶层模型名；未登记的地形、标志物和其他模型不会参与绑定。`mount_dynamic` 默认为 false，沿用固定安装参考。机载装配设置为 true 时，插件为 `mount_link` 启用Gazebo速度检查，每步读取世界姿态并优先使用 `Link::WorldAngularVelocity`；世界系角速度按当前world→mount旋转转换为安装局部值。只有速度数据不可用时才使用姿态差分后备，后备值经过35 Hz一阶低通并拒绝大于0.5 rad的单步姿态跳变。TRACK 中把该载机角速度对视线的影响投影到实际两轴 Jacobian，加入反向前馈；IDLE不会主动稳像。`sim_truth` 使用与图像时间戳匹配的相机和目标世界位置生成像素误差，`csrt` 使用图像框误差。进入机械限位、输出速度受限或视线退化时明确报告约束，不能将后端仍输出目标框解释为稳定锁定。

视线方向为 `(-sin(pitch), -sin(yaw)cos(pitch), cos(yaw)cos(pitch))`。图像右/下方向的关节 Jacobian 为 `diag(cos(pitch), -1)`，yaw 使用阻尼最小二乘逆。两轴零位不退化；当视线平行外轴、`abs(cos(pitch)) < 0.05` 时，yaw 无法独立改变视线方向，遥测报告 `singular: true`。确认的机械范围不包含该退化姿态。机械手动控制与回零仍有效。

## Gazebo Transport

- 命令：`/two_axis_pod/command`，`gz.msgs.StringMsg` JSON。
- 回执：`/two_axis_pod/command_result`，包含 `type/cmd/ok/message/request_id`，request_id 在请求携带时原样返回。
- 遥测：`/two_axis_pod/telemetry`，与网关共用 `service/telemetry.json` 的 `hz`，默认 50 Hz（单调真实时间）；控制默认 100 Hz（仿真时间）。

插件从 SDF `telemetry_config` 指向的 JSON 读取 `hz`，默认 `model://two_axis_pod/controller/service/telemetry.json`。配置缺失或非法时明确报错并禁用插件；修改后需重启仿真及网关。遥测调度使用 `steady_clock`，不由 `UpdateInfo.dt` 或仿真实时因子决定发送目标。只有新的物理状态可发布，单步最多产生一份新姿态；暂停不发布，恢复和仿真时间重置不补发历史周期。物理更新的真实频率不足配置 `hz` 时限频告警。完整配置与发送行为见[网关说明](service/README.md)。

三个话题可用 SDF 的 `command_topic/telemetry_topic/command_result_topic` 配置。图像默认订阅由世界、模型、link、sensor 名拼接的完整路径，可用 `image_topic` 对应相机显式 topic。

命令支持 track（归一化 `box:[x,y,w,h]`）、manual（`yaw/pitch` 机械速度）、stop、home、reset、reset_axis（axis 为 yaw/pitch）、reset_fault、ping。任何 manual 的 roll 字段均拒绝，包括零值。

遥测 `protocol_version:8`、`angle_convention:"motion_head_joint_right_positive"`；`attitude_deg`、`joints_deg` 只有 yaw/pitch 的 pos/vel/acc，`cmd_dps` 只有 yaw/pitch。`limits` 包含 yaw/pitch/saturated，`los_mount` 是安装系单位视线，顶层 `singular` 表示视线退化。`tracking.backend` 报告 `sim_truth` 或 `csrt`；真值绑定期间 `target.entity_id/entity_name` 报告固定实体，CSRT 或未绑定时为 `null`。`camera` 包含相机传感器相对安装系/世界的四元数与欧拉姿态及帧序号，不将机械轴角误称为世界偏航/俯仰。模型、插件、网关及地面站须共同使用协议 v8 和同一机械角约定。网络接口见 [网关说明](service/README.md)。

测试覆盖目标相对固定名义重心的NED三维LOS角速度向量、各分量方向、一般姿态叉乘等价性、与LOS正交、径向零值、大坐标归一化、不限幅及竖直独立有效性，以及NED方位/仰角导数、径向与双方相对运动、方位跨±180°、上下符号、垂直奇异、固定重心刚体旋转、首样本/暂停/回退/目标切换/瞬移保护，以及旋转链、有限差分 Jacobian、退化、Gazebo世界角速度坐标变换、动态安装姿态差分低通/跳变拒绝与独立积分后的世界视线漂移、4/3 ms及大步长控制余相、暂停/时间回退、手动超时、双轴限位、单轴回零、真值投影/选框/同步保护、真值框四边各扩展 5 px、边界裁剪及框外可见性保持、CSRT 跟踪及断帧丢失保护。


## 目标惯性视线角速度

`target_los` 与 `los_mount` 含义独立：前者使用目标位置和载机固定名义重心位置，后者仅表示镜头光轴。SDF `los_observer_cg_model` 指定零关节名义重心在插件所附模型坐标中的固定三维偏移（m）；动态装配由当前各link质量和零关节惯性位姿派生。插件将该固定偏移随模型姿态旋转，不能以移动关节的瞬时组合重心或相机光心替代。缺失或非法偏移仅让目标LOS无效，原有控制继续可用。

`PostUpdate` 在同一Gazebo物理状态采样观察点和目标世界位置，并用相同仿真时间差分得到相对速度。带球坐标配置的world通过Gazebo `LOCAL2→GLOBAL` 向量变换应用heading后转换ENU→NED；未配置球坐标的局部world采用标准ENU。目标相对位置 `(N,E,D)` 的方位角为 `atan2(E,N)`，仰角为 `atan2(-D,hypot(N,E))`；输出是两角的解析导数，单位°/s，北向东右转为正、抬升为正，不是角速度向量的两个分量。内部结果不作±20°/s限幅，外部艾晟映射负责报文限幅；机械转速始终仍由 `joints_deg.*.vel` 提供。

`sim_truth` 只使用当前跟踪绑定的固定实体ID。`csrt` 需显式SDF `los_target_name`，并在选框初始化时用同帧同步真值投影核对该实体中心位于框内，之后固定同一实体ID；不匹配仅让LOS无效，不能改变CSRT纹理跟踪控制，也不能按名字自动替换删除的实体。未TRACK、未配置目标、未匹配身份均不猜测目标。实体空间位置是目标顶层模型原点，不表示目标视觉框中心或额外目标重心。

`target_los` 包含 `valid`、`frame:"NED"`、`observer_reference:"aircraft_nominal_cg"`、`target_name`、`sim_stamp`、`yaw_rate_dps/pitch_rate_dps`、`relative_position_ned_m/relative_velocity_ned_mps`、`reason`，以及协议v8的三维向量扩展 `omega_ned_dps`、`omega_valid`、`omega_reason`。`valid/reason` 仅表示方位角和仰角两个导数的有效性；`omega_valid/omega_reason` 独立表示三维向量有效性。其 `sim_stamp` 指双方位置采样的仿真时刻；在现有PreUpdate遥测中携带最新PostUpdate结果，通常相差一个物理步。对应表示无效时角速度为 `null`，不以零伪装有效测量；艾晟外部接口的无效值由地面站单独处理。

三维向量为 `omega = (r × v) / |r|²`，其中 `r` 是目标减无人机名义重心的NED相对位置，`v` 是同一相对位置的时间导数。`omega_ned_dps:[N,E,D]` 按NED右手系表达，单位°/s，与 `r` 正交，表示不包含沿视线自转分量的最小LOS角速度向量。实现先归一化再叉乘，避免位置叉乘或距离平方溢出。该向量的N/E/D分量不能当作方位角/仰角导数；其真实值不作±20°/s限幅，外部对应 `los_omega_n/e/d`。原有双角度导数的报文限幅保持±20°/s。

首样本、重新绑定、暂停/重复时刻、时间回退及超过0.5 s的样本间隔须建立新的差分基准。单方位置变化超过 `max(1 m,1000 m/s×dt)` 视为瞬移，当前样本无效并重新建基准；这仅是测量连续性检查。距离小于1 µm时两种表示均无效；水平距离小于1 µm或总距离的百万分之一时，两角度导数无效，但三维向量仍可有效，典型为 `valid:false`、`reason:"vertical_singularity"`、`omega_valid:true`、`omega_reason:"ok"`。未绑定、首样本、时间/位置连续性异常会让两种表示均无效。`reason` 可为 `ok`、`observer_cg_unconfigured`、`observer_missing`、`target_unbound`、`target_unconfigured`、`target_unsynchronized`、`target_selection_mismatch`、`target_missing`、`first_sample`、`target_changed`、`time_paused`、`time_reset`、`time_gap`、`position_jump`、`zero_range`、`vertical_singularity`、`nonfinite_sample` 或 `nonfinite_rate`。
