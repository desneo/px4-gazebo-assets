# px4-gazebo-assets

用于 PX4 + Gazebo 的仿真资产仓库，管理基础模型、装配模型、完整环境和仿真实验。相机及其参数直接维护在对应云台模型中。

## 目录结构

```text
px4-gazebo-assets/
├── README.md                  # 仓库用途、目录归属规则和使用方法
├── AGENTS.md                  # AI 工作规则与硬规则
├── models/                    # 可复用的基础模型
│   ├── vehicles/              # 无人机本体，包含机载传感器参数
│   │   ├── rocket_racing_quad/ # 流线型火箭机身高速穿越机，含网格、生成脚本与自验证
│   │   ├── xiaomotuo/         # 小摩托：平滑三角翼外观、两叶尾桨、起落架通用固定翼
│   │   └── xiaobianfu/        # 小蝙蝠：白绿飞翼、手抛起飞、机腹着陆固定翼
│   ├── gimbals/               # 云台，包含相机、控制器及其参数
│   │   ├── two_axis_pod/      # 两轴立装吊舱，含飞机检测、自动锁定、控制、网关和地面站
│   │   └── three_axis_pod/
│   │       ├── controller/    # 专用 C++ 控制插件、网关、工具和单元测试
│   │       ├── ground_station/ # 吊舱配套地面站
│   │       ├── scripts/       # 组件启动、停止和配置
│   │       ├── tests/         # 组件自测与演示世界
│   │       └── docs/          # 组件功能、接口与验证说明
│   ├── mounts/                # 独立安装座、支架
│   └── props/                 # 建筑、树木、障碍物与静态美术展示
│       └── delta_wing_display/ # 小摩托外观所用的平滑三角翼独立美术展示
├── assembledmodels/           # 由基础 model 组合的复杂模型
│   └── rocket_two_axis_pod/   # 590 mm 火箭机与顶部两轴吊舱的造型与动态装配
├── assemblies/                # 已有固定支架三轴吊舱装配
├── worlds/                    # 静态环境定义、专用资源和说明
└── scenarios/                 # 大型综合测试场景的配置、启动和辅助脚本(组件自己用的测试跟model放一起)
    ├── geographic_two_axis_pod/ # 完整地理背景中的两轴立装部件测试
    ├── geographic_rocket_two_axis_pod/ # 火箭机机载两轴吊舱的往返飞行与空中目标跟踪
    ├── rocket_speed_comparison/ # 当前单机与组合体的最高稳定平飞速度对照
    └── common/                # 多个场景共用的辅助脚本
```

## 文件归属规则

- 基础模型按类别放入 `models/`，每个模型使用独立目录，其网格、材质、纹理和专用说明随模型保存。
- 多个 model 组合完成的新增复杂模型放入 `assembledmodels/`，每个装配模型使用独立目录；已有固定支架三轴吊舱保留在 `assemblies/three_axis_pod_stand/`。
- 可复用的静态环境使用独立的 `worlds/<环境名称>/` 目录，静态环境定义保存为 `world.sdf`，环境专用的网格、纹理等资源放入该目录下的 `assets/`，环境专用的原生 GUI 插件源码放入该目录下的 `gui/`，参数、数据来源和使用方法维护在该目录的 `README.md` 中。
- `scenarios/` 只放置大型综合测试场景，每个场景使用独立的 `scenarios/<场景名称>/` 目录，其 `README.md` 明确描述使用哪个 world。实验专用配置、启动和辅助脚本按需放入该目录；环境自身的内容归属对应的 world。
- 场景可以使用 `run.sh` 作为启动入口；需要配置文件时，由对应脚本读取。自定义配置文件不视为 PX4 或 Gazebo 原生支持的格式。
- 组件的使用说明、启动脚本、配置、自测试和小型演示世界均随对应模型保存，不单独放入 `scenarios/`。组件专用控制插件与网关放入模型的 `controller/`，配套地面站放入 `ground_station/`，使用脚本放入 `scripts/`，组件自测和演示世界放入 `tests/`，功能与验证说明放入 `docs/`。
- 三轴吊舱归入 `models/gimbals/three_axis_pod/`，吊舱与其内置云台一一对应，地面站是该组件的一部分。固定支架与吊舱的组合模型归入 `assemblies/three_axis_pod_stand/`。
- 两轴立装吊舱归入 `models/gimbals/two_axis_pod/`，模型、专用控制器、网关、地面站、生成工具与立装演示均随组件保存。该组件的机械角定义与三轴公共姿态协议不同。
- 多个场景共用的辅助脚本放入 `scenarios/common/`。
- 目录与文件使用有意义的英文名称。目录结构和归属规则统一维护在本文件中，模型或场景的专用说明维护在各自目录中。

## 使用状态

仓库包含静态真实地理背景环境、四旋翼火箭穿越机、小摩托与小蝙蝠固定翼模型，以及两轴立装吊舱、固定支架三轴吊舱、演示世界、控制插件、网络网关和地面站。

固定翼资产优先基于 PX4 已有 `advanced_plane` 模型修改，分别维护几何、质量、动力、气动与机体配置。小摩托整机总长 3.3 m、翼展 2.5 m、总质量 50 kg，目标最大可持续平飞空速 200 km/h；小蝙蝠整机总长 0.40 m、翼展 1 m、总质量 1 kg，目标最大可持续平飞空速 100 km/h。两款不考虑电池容量限制，估计参数与实际验证结果随模型记录。模型和运行入口见[小摩托说明](models/vehicles/xiaomotuo/README.md)与[小蝙蝠说明](models/vehicles/xiaobianfu/README.md)。

两款外形均已在 Gazebo 中获人工确认。小摩托采用平滑融合三角翼美术外观，动力学与碰撞使用独立的通用仿真近似；纯视觉资源另提供[静态展示入口](models/props/delta_wing_display/README.md)。两款均通过当前无风平面测试世界的自动起飞、连续 30 仿真秒最大油门平飞、自动着陆和停稳停机验收；测试设置、当前配置指纹与实际数据见[小摩托验证说明](models/vehicles/xiaomotuo/docs/validation.md)和[小蝙蝠验证说明](models/vehicles/xiaobianfu/docs/validation.md)。

- 环境：`worlds/geo_32_217857_119_105878/world.sdf`，以 WGS84 坐标 `(32.21785737811048, 119.1058778440883)` 为中心，提供包含半径 5 km 区域的真实高程、地表影像和静态天空；配套 GUI 插件提供“回到地图中心”按钮，构建和安装方法见环境说明。
- 启动：在仓库根目录运行 `gz sim -r worlds/geo_32_217857_119_105878/world.sdf`。运行条件、数据来源和精度限制见 [环境说明](worlds/geo_32_217857_119_105878/README.md)。
- 场景：运行 `bash scenarios/geographic_camera/run.sh --gui`，使用上述真实地理 world 的运行副本验证立装/悬挂三轴吊舱、移动小车跟踪、NVENC 视频与 2 秒实体遮挡重捕获；不启动 PX4。目标、遮挡操作及安装方向见[场景说明](scenarios/geographic_camera/README.md)。
- 三轴吊舱：见 [组件说明](models/gimbals/three_axis_pod/README.md)和[使用说明](models/gimbals/three_axis_pod/scripts/README.md)；功能、协议和当前限制见 [系统说明](models/gimbals/three_axis_pod/docs/system.md)，自验证结果见 [验证说明](models/gimbals/three_axis_pod/docs/validation.md)。
- 两轴地理测试：运行 `bash scenarios/geographic_two_axis_pod/run.sh --gui`，在完整地理环境中测试标准立装两轴吊舱、三个飞机夹具、多目标检测、自动锁定、动态目标跟踪、实体遮挡与 UDP/RTSP 地面站；不加载载机或 PX4，见[测试场景说明](scenarios/geographic_two_axis_pod/README.md)。
- 火箭机机载两轴场景：`scenarios/geographic_rocket_two_axis_pod/`保留完整地理world、PX4往返飞行和空中纹理靶标入口。当前590 mm组合的固定名义重心与CA/EKF位置派生已实现，组件自验证工具支持同一重心真值；基础单机当前完整飞行通过，组合保留归档配置的完整组件飞行证据。两轴采用俯仰负值朝前、协议v5；该源指纹下的组合组件完整飞行及地理往返、视频与跟踪尚未复测，见[场景说明](scenarios/geographic_rocket_two_axis_pod/README.md)与[验证状态](scenarios/geographic_rocket_two_axis_pod/docs/validation.md)。
- 火箭机速度对照：`scenarios/rocket_speed_comparison/`保留80 m高度、连续10 s最高稳定速度比较入口。当前单机4.76 kg、组合5.00 kg，组件自验证参考点已统一为各自固定零关节名义重心；该速度场景尚未复测，不沿用不同配置的归档结论，见[场景说明](scenarios/rocket_speed_comparison/README.md)与[验证状态](scenarios/rocket_speed_comparison/docs/validation.md)。
- 两轴立装吊舱：见[组件说明](models/gimbals/two_axis_pod/README.md)、[安装与运动学](models/gimbals/two_axis_pod/docs/mounting.md)和[验证说明](models/gimbals/two_axis_pod/docs/validation.md)。Ø84 mm 底部圆环与地面平行，导引头整体向上高 91 mm。偏航绕竖直 +Z 轴转动，范围 ±90°；俯仰绕随动水平 −Y 轴转动，范围 −90°～+45°，机械零位朝天，−90° 朝前，运动链为 `Rz(yaw) Ry(-pitch)`，控制协议为 v5。组件使用单类 YOLO11 飞机模型和多目标轨迹确认；检测/自动搜索显示服务端红框与编号，自动选中中心最近目标后转入 CSRT 单目标跟踪并显示绿色框。关闭视觉、检测、自动锁定和手动框选互斥。视频强制硬件编解码，Linux 地面站使用 NVDEC，Windows 地面站显式请求并校验 D3D11 硬件解码，不可用时均直接报错。网关支持多机共用 RTSP 8555，按 `/two_axis_pod_<aircraft_id>` 路径末尾的飞机 ID 区分；网关与地面站配置相同 ID，默认 1。网关可配置额外的 UDP JSON 状态接收地址和频率，默认 20 Hz，与 station 会话独立；地面站另提供仅含 9 个转换字段的艾晟 JSON 发送。Windows 专用入口使用隔离 Python 虚拟环境，安装与启动见[地面站说明](models/gimbals/two_axis_pod/ground_station/README.md#windows-专用版本)。
- 火箭穿越机：见[模型说明](models/vehicles/rocket_racing_quad/README.md)与[测试说明](models/vehicles/rocket_racing_quad/tests/README.md)。当前平底、四主翼与四小尾翼外形高0.590 m，整机质量4.76 kg、固定名义重心z=+0.030 m，外形已获人工确认。当前配置已通过30 s悬停、200 km/h连续30.012 s巡航、300 km/h连续10.000 s冲刺及落地停机，采用固定名义重心Gazebo真值；质量/惯性几何核验表明数学可行，未作实物标定。原始记录和15项源指纹见[验证结果](models/vehicles/rocket_racing_quad/docs/validation.md)。
- 火箭机与顶部两轴吊舱：`assembledmodels/rocket_two_axis_pod/`静态装配高590 mm，吊舱安装面z=0.289 m、肩部起点z=0.277 m、最高点z=+0.380 m，静态SDF含35个visual、无碰撞体，组合外形待人工确认。动态组合质量5.00 kg，固定零关节名义重心 `(0.000204,0,0.043856) m` FLU，CA/EKF派生与真值测量支持已实现，所附5.00 kg组件飞行记录通过30 s悬停、200 km/h连续30.000 s巡航、300 km/h连续10.008 s冲刺及落地停机，不代替当前合并源指纹的完整飞行复测；入口与结果见[装配说明](assembledmodels/rocket_two_axis_pod/README.md)。

空目录中的 `.gitkeep` 用于让 Git 保存目录；添加实际文件后可以移除。

## 多机吊舱集成入口

每架飞机的吊舱使用独立UDP控制/遥测端口和独立RTSP URL。RTSP服务端口可共享8555，URL以 `/two_axis_pod_<aircraft_id>` 区分；UDP控制端口由启动配置分别指定，不随飞机ID自动变化。网关和地面站须使用对应同一架飞机的ID、主机、UDP端口和视频路径配置。

其他Agent集成前请阅读[逐机地址与端口对应表](scenarios/geographic_rocket_two_axis_pod/README.md#多机集成的地址与端口对应)及[网关共享RTSP说明](models/gimbals/two_axis_pod/controller/service/README.md#多机共享-rtsp)。组合场景当前仍使用单个PX4实例；多机接入还需分别配置Gazebo实体/话题、PX4实例、SYSID及MAVLink端口，不能只修改视频ID后双开现有启动器。

三轴吊舱的安装资料见[安装方式与对外角度约定](models/gimbals/three_axis_pod/docs/mounting.md)，其中附有可独立浏览的安装示意图。
