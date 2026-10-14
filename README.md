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
│   │   ├── two_axis_pod/      # 两轴立装吊舱，包含专用控制、网关、地面站和演示
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
│   └── rocket_two_axis_pod/   # 450 mm 火箭机与顶部两轴吊舱的造型与动态装配
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
- 两轴地理测试：运行 `bash scenarios/geographic_two_axis_pod/run.sh --gui`，在完整地理环境中测试标准立装两轴吊舱、空中动态棋盘靶标、实体遮挡与 UDP/RTSP 地面站；不加载载机或 PX4，见[测试场景说明](scenarios/geographic_two_axis_pod/README.md)。
- 火箭机机载两轴场景：`scenarios/geographic_rocket_two_axis_pod/` 使用现有地理 world、动态组合模型和 PX4，提供 80 m 相对起飞高度、15 m/s、300 m 航段的往返飞行入口及空中纹理靶标；当前配置通过两次完整往返、降落及 70 仿真秒真实图像跟踪自动检查，场景效果已通过人工确认，见[场景说明](scenarios/geographic_rocket_two_axis_pod/README.md)与[验收结果](scenarios/geographic_rocket_two_axis_pod/docs/validation.md)。
- 火箭机速度对照：`scenarios/rocket_speed_comparison/` 在同一无风地理世界顺序运行当前单机和组合体，比较80 m高度下连续10秒的最高稳定平飞速度。实测单机340.53 km/h、组合体340.44 km/h，均在95 m/s控制上限的设定容差内并正常降落；当前相同简化阻力配置下组合体达到单机速度水平，见[场景说明](scenarios/rocket_speed_comparison/README.md)和[实测结果](scenarios/rocket_speed_comparison/docs/validation.md)。
- 两轴立装吊舱：见[组件说明](models/gimbals/two_axis_pod/README.md)、[安装与运动学](models/gimbals/two_axis_pod/docs/mounting.md)和[验证说明](models/gimbals/two_axis_pod/docs/validation.md)。Ø84 mm 底部圆环与地面平行，导引头整体向上高 91 mm。偏航绕竖直 +Z 轴转动，范围 ±90°；俯仰绕随动水平 +Y 轴转动，范围 −45°～+90°，机械零位朝天，正值朝前。网关可配置额外的 UDP JSON 状态接收地址和频率，默认 20 Hz，与 station 会话独立。
- 火箭穿越机：见 [模型说明](models/vehicles/rocket_racing_quad/README.md)和[自验证说明](models/vehicles/rocket_racing_quad/tests/README.md)；采用机头朝上起飞、整机倾转加速的布局，机身轴向长度 0.45 m、对角轴距 15 英寸、桨径 10 英寸，目标为 200 km/h 巡航及 300 km/h 连续冲刺 10 秒。参数集中在模型的 `parameters.json`，修改与复测见[参数说明](models/vehicles/rocket_racing_quad/docs/parameters.md)；启动器生成独立 Gazebo 资产、PX4 机体配置和运行目录，当前验收进度见[项目状态](models/vehicles/rocket_racing_quad/docs/status.md)。
- 火箭机与顶部两轴吊舱：组合资产位于 `assembledmodels/rocket_two_axis_pod/`，以 91 mm 高的吊舱替换原机头顶部，整体轴向长 450 mm，吊舱各轴与机体同名轴对齐。静态组合造型已通过人工确认；整机及近景入口见[装配说明](assembledmodels/rocket_two_axis_pod/README.md)。

空目录中的 `.gitkeep` 用于让 Git 保存目录；添加实际文件后可以移除。

三轴吊舱的安装资料见[安装方式与对外角度约定](models/gimbals/three_axis_pod/docs/mounting.md)，其中附有可独立浏览的安装示意图。
