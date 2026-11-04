# INI 参数

`parameters.ini` 是产品资料、机械、光学、跟踪和运行资源的唯一人工配置源。模型生成器、控制服务和地面站读取同一份校验结果；修改后停止并重启组件，不热加载。启动器在会话生成目录重新生成 SDF 并记录配置 hash，避免新配置与旧模型混用。`generated.json`、SDF 和 `runs/<会话>/session.ini` 是生成数据，不替代性能 INI。

配置采用 UTF-8。未知/缺失节或键、重复键、非有限数值、非法区间、协议无法表达的数值和冲突的光学参数会报中文错误，不回退到其他云台配置。角度单位为 °，速度为 °/s；表中“正”均指正有限数。

## 实际模型与机械

| Section.key | 默认值 | 单位、有效范围与作用 |
| --- | --- | --- |
| `model.mass_kg` | 0.195 | kg，正；按 CAD 体积分配总质量及惯性，不缩放外形 |
| `model.name` | zhongke_d80 | 非空英文标识符；模型实体及话题/诊断名称 |
| `model.aircraft_id` | 1 | 非负整数；本机运行清单身份，不加入厂商二进制帧；RTSP 路径由 `video.rtsp_mount` 显式配置 |
| `mechanics.yaw_min_deg` | −90 | °，和上界有序、包含 0，均在 −327.68～327.67 内；实际方位硬限位 |
| `mechanics.yaw_max_deg` | 90 | °，同上 |
| `mechanics.pitch_min_deg` | −90 | °，和上界有序、包含 0，均在 −327.68～327.67 内；实际俯仰硬限位 |
| `mechanics.pitch_max_deg` | 30 | °，同上 |
| `mechanics.max_rate_dps` | 20 | °/s，(0,327.67]；人工、指定角、回零和跟踪共用的实际转速上限 |
| `mechanics.max_acc_dps2` | 160 | °/s²，正；转动加速度约束 |
| `mechanics.kp` | 4 | s⁻¹，正；姿态误差的控制增益，输出仍受限速和限位约束 |
| `mechanics.stop_margin_deg` | 0.2 | °，非负且两倍缓冲小于每轴行程；软件停车缓冲，不移动硬限位 |

质量分配假设见 [资产说明](assets.md)。改变转速上限同时作用于 GUI 人工输入、控制层和物理约束；遥测转换不把测量截断到 20°/s。软件停车区始终包含机械零位，零位位于硬行程端点时仍可连续回零并结束HOME。

## 光学与成像

| Section.key | 默认值 | 单位、有效范围与作用 |
| --- | --- | --- |
| `optics.width_px` | 1920 | px，正偶数；真实原图宽度、编码和指点坐标共用 |
| `optics.height_px` | 1080 | px，正偶数；真实原图高度 |
| `optics.fps` | 30 | Hz，正；相机采样频率 |
| `optics.focal_length_mm` | 50 | mm，正；当前焦距，实际联动水平/垂直视场角 |
| `optics.reference_focal_length_mm` | 50 | mm，正；固定感光面的标定焦距 |
| `optics.reference_hfov_deg` | 7.03 | °，(0,180)；标定水平视场角，不是运行覆盖值 |
| `optics.reference_vfov_deg` | 3.95 | °，(0,180)；标定垂直视场角，不强制 `fy=fx` |
| `optics.near_clip_m` | 0.005 | m，0<near<far；视频渲染及真值投影的近裁剪 |
| `optics.far_clip_m` | 30000 | m，far>near；视频渲染及真值投影的远裁剪，独立于发现距离 |

设标定焦距为 `f0`、当前焦距为 `f`、标定角为 `H0/V0`，固定等效感光面为：

```text
sensor_width  = 2*f0*tan(H0/2)
sensor_height = 2*f0*tan(V0/2)
actual_HFOV   = 2*atan(sensor_width /(2*f))
actual_VFOV   = 2*atan(sensor_height/(2*f))
fx = width_px*f/sensor_width
fy = height_px*f/sensor_height
cx = width_px/2
cy = height_px/2
```

三角函数计算使用 rad。只把 `focal_length_mm` 从 50 改为 100 mm 时，感光面保持不变，两个实际视场角变窄；修改标定角或标定焦距则重新定义感光面。不能增加另一组独立运行 HFOV/VFOV 覆盖同一成像结果。

派生感光面、内参和实际视场也必须为可表达的正有限数；数值上溢、下溢为零或退化到0°/180°时，在读取INI时中文报错，不能将NaN/Infinity写入SDF。

SDF `lens/intrinsics`、CameraInfo和真实位置投影共用派生 `fx/fy/cx/cy`。地面站光学文字显示标定焦距与标定双向视场角。SDF 的 `horizontal_fov` 兼容字段不替代运行内参。协议 `zoom` 是 `f/f0`，按 0.1 倍量化，不能超过 U12 的 409.5 倍表达上限；不提供在线镜头变焦命令。

## 真值目标

| Section.key | 默认值 | 单位、有效范围与作用 |
| --- | --- | --- |
| `tracking.target_names` | target_a,target_b,target_c | 逗号分隔精确模型名、去空格、无重复；不存在实体不进入候选 |
| `tracking.discovery_range_m` | 300 | m，正；相机光学中心到目标原点的三维距离，≤上限可发现 |
| `tracking.point_fallback_radius_px` | 80 | 原图 px，非负；超过时仍选离点击位置最近的合格候选并提示 |
| `tracking.reacquire_period_s` | 0.2 | s，正；自动等待/失效时重新选择的周期 |
| `tracking.lost_timeout_s` | 2.5 | s，正；短暂丢失到最终 LOST 的计时，不延迟停止旧输出 |

发现条件包括注册、有效帧/位置、相机前方及投影中心在原图内。距离只限制初选和重新选取，已锁目标越过上限而仍可观测时继续跟踪。出画面、实体或数据失效等条件仍触发失效处理。远裁剪也影响投影可见性，不能设置成 300 m 来代替发现距离。

## 产品资料

以下均为资料项，不产生电气、功耗耦合或随机指向误差模型，也不调整复制几何。

| Section.key | 默认值 | 单位与有效范围 |
| --- | --- | --- |
| `product.mass_limit_g` | 195 | g，正，标称质量上限；实际质量由 `model.mass_kg` 决定 |
| `product.supply_voltage_v` | 12 | V，正，DC 供电资料 |
| `product.average_power_limit_w` | 7 | W，非负且≤峰值，标称平均功耗上限 |
| `product.peak_power_limit_w` | 12.7 | W，非负且≥平均，标称峰值功耗上限 |
| `product.diameter_limit_mm` | 49 | mm，正，标称直径上限 |
| `product.height_limit_mm` | 80 | mm，正，标称高度上限 |
| `product.pointing_accuracy_deg` | 0.2 | °，非负，指定范围内的标称指向精度 |
| `product.accuracy_yaw_min_deg` | −30 | °，有限且≤上界，精度适用方位下界 |
| `product.accuracy_yaw_max_deg` | 30 | °，有限且≥下界，精度适用方位上界 |
| `product.accuracy_pitch_min_deg` | −30 | °，有限且≤上界，精度适用俯仰下界 |
| `product.accuracy_pitch_max_deg` | 30 | °，有限且≥下界，精度适用俯仰上界 |
| `product.fov_tolerance_percent` | 5 | %，非负，标称视场容差，不作为随机抖动 |

## 网络、视频与演示

| Section.key | 默认值 | 单位、有效范围与作用 |
| --- | --- | --- |
| `network.udp_bind` | 127.0.0.1 | IP 字面量；中科标准 UDP 监听地址 |
| `network.udp_port` | 8870 | 整数 1～65535；独立控制/遥测端口 |
| `network.telemetry_hz` | 60 | Hz，正；真实时间的新物理测量目标频率，源暂停不重复缓存补频 |
| `network.peer_lease_s` | 2 | s，正；单客户端控制对端租约 |
| `network.manual_timeout_s` | 0.5 | s，正；当前非零 MANUAL 速度超时，空指令不刷新 |
| `network.telemetry_max_age_s` | 1 | s，正；有效接收和转发队列的新鲜性上限 |
| `video.rtsp_bind` | 127.0.0.1 | IP 字面量；本组件 RTSP 监听地址 |
| `video.rtsp_port` | 8556 | 整数 1～65535；独立 RTSP 端口 |
| `video.rtsp_mount` | /zhongke_d80_1 | 非空合法路径；显式指定单路视频路径 |
| `video.bitrate_kbps` | 8000 | kb/s，正整数；NVENC 编码码率 |
| `viewer.near_clip_m` | 0.005 | m，正；Gazebo 浏览相机近裁剪，不改变视频镜头 |
| `viewer.sensitivity` | 0.1 | 正数；GUI 服务使用的鼠标缩放/平移/旋转灵敏度 |
| `demo.world_name` | zhongke_d80_demo | 非空英文标识符；独立演示世界名 |
| `demo.marker_name` | ZHONGKE_D80_LOCATION_MARKER | 非空英文标识符；保存的无碰撞定位标志名 |

端口已占用时拒绝启动。会话清单记录实际分区、端点、配置路径和 hash，GUI 与工具按清单连接，不猜测其他组件端口。浏览相机设置的应用条件见 [安装与观察](mounting.md)。

## 地面站连接与偏好

`ground_station/config.ini` 仅含以下项目。保存接收地址保留原注释，不写入“启用发送”开关；该文件不参与性能 hash。

无 `--session` 时从此文件独立连接，不要求仿真清单。显式 `--session` 使用清单中的性能路径、远端UDP及RTSP，本地接收端点和界面偏好仍从此文件读取。`parameters_file` 的相对路径以地面站配置文件所在目录为基准。

| Section.key | 默认值 | 作用 |
| --- | --- | --- |
| `connection.udp_host` | 127.0.0.1 | 中科设备/网关的IP字面量，控制目的地址，也是遥测来源地址 |
| `connection.udp_port` | 8870 | 整数1～65535；设备控制/遥测共用UDP端口 |
| `connection.local_bind` | 0.0.0.0 | 地面站本地绑定IP；须与远端地址族一致 |
| `connection.local_port` | 0 | 整数0～65535；0自动分配，其余固定本地收发端口；启动输出实际端点 |
| `connection.rtsp_url` | rtsp://127.0.0.1:8556/zhongke_d80_1 | 已运行视频服务的完整RTSP地址 |
| `connection.parameters_file` | ../parameters.ini | 性能INI路径；不另存机械/光学参数，显式会话或 `--config` 可选择性能源 |
| `video.display_scale` | 0.5 | 无量纲，0.05～2.0；显示缩放，不改变原图像素 |
| `video.decoder` | nvdec | 当前只支持 NVIDIA 硬解 |
| `video.window_title` | 中科 D80 地面站 | 窗口标题及焦点检查身份 |
| `video.font_path` | /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc | 中文字体路径，优先加载该字体 |
| `forward.ip` | 127.0.0.1 | 合法接收 IP，不能与本组件控制端点相同 |
| `forward.port` | 9000 | 整数 1～65535；15 字段 JSON 接收端口，默认不发送 |
