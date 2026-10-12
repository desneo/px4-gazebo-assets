# px4-gazebo-assets

用于 PX4 + Gazebo 的仿真资产仓库，管理基础模型、装配模型、完整环境和仿真实验。相机及其参数直接维护在对应云台模型中。

## 目录结构

```text
px4-gazebo-assets/
├── README.md                  # 仓库用途、目录归属规则和使用方法
├── AGENTS.md                  # AI 工作规则与硬规则
├── models/                    # 可复用的基础模型
│   ├── vehicles/              # 无人机本体，包含机载传感器参数
│   ├── gimbals/               # 云台，包含相机、控制器及其参数
│   │   └── three_axis_pod/
│   │       ├── controller/    # 专用 C++ 控制插件、网关、工具和单元测试
│   │       ├── ground_station/ # 吊舱配套地面站
│   │       ├── scripts/       # 组件启动、停止和配置
│   │       ├── tests/         # 组件自测与演示世界
│   │       └── docs/          # 组件功能、接口与验证说明
│   ├── mounts/                # 独立安装座、支架
│   └── props/                 # 建筑、树木、障碍物等场景物体
├── assemblies/                # 多个 model 组合完成的复杂模型
├── worlds/                    # 静态环境定义、专用资源和说明
└── scenarios/                 # 大型综合测试场景的配置、启动和辅助脚本(组件自己用的测试跟model放一起)
    └── common/                # 多个场景共用的辅助脚本
```

## 文件归属规则

- 基础模型按类别放入 `models/`，每个模型使用独立目录，其网格、材质、纹理和专用说明随模型保存。
- 多个 model 组合完成的复杂模型放入 `assemblies/`，每个装配模型使用独立目录。
- 可复用的静态环境使用独立的 `worlds/<环境名称>/` 目录，静态环境定义保存为 `world.sdf`，环境专用的网格、纹理等资源放入该目录下的 `assets/`，参数、数据来源和使用方法维护在该目录的 `README.md` 中。
- `scenarios/` 只放置大型综合测试场景，每个场景使用独立的 `scenarios/<场景名称>/` 目录，其 `README.md` 明确描述使用哪个 world。实验专用配置、启动和辅助脚本按需放入该目录；环境自身的内容归属对应的 world。
- 场景可以使用 `run.sh` 作为启动入口；需要配置文件时，由对应脚本读取。自定义配置文件不视为 PX4 或 Gazebo 原生支持的格式。
- 组件的使用说明、启动脚本、配置、自测试和小型演示世界均随对应模型保存，不单独放入 `scenarios/`。组件专用控制插件与网关放入模型的 `controller/`，配套地面站放入 `ground_station/`，使用脚本放入 `scripts/`，组件自测和演示世界放入 `tests/`，功能与验证说明放入 `docs/`。
- 三轴吊舱归入 `models/gimbals/three_axis_pod/`，吊舱与其内置云台一一对应，地面站是该组件的一部分。固定支架与吊舱的组合模型归入 `assemblies/three_axis_pod_stand/`。
- 多个场景共用的辅助脚本放入 `scenarios/common/`。
- 目录与文件使用有意义的英文名称。目录结构和归属规则统一维护在本文件中，模型或场景的专用说明维护在各自目录中。

## 使用状态

仓库包含静态真实地理背景环境，以及固定支架三轴吊舱、演示世界、控制插件、网络网关和地面站；尚未添加无人机模型。

- 环境：`worlds/geo_32_217857_119_105878/world.sdf`，以 WGS84 坐标 `(32.21785737811048, 119.1058778440883)` 为中心，提供包含半径 5 km 区域的真实高程、地表影像和静态天空。
- 启动：在仓库根目录运行 `gz sim -r worlds/geo_32_217857_119_105878/world.sdf`。运行条件、数据来源和精度限制见 [环境说明](worlds/geo_32_217857_119_105878/README.md)。
- 场景：`scenarios/geographic_camera/README.md` 描述云台相机与视觉测试使用的 world。
- 三轴吊舱：见 [组件说明](models/gimbals/three_axis_pod/README.md)和[使用说明](models/gimbals/three_axis_pod/scripts/README.md)；功能、协议和当前限制见 [系统说明](models/gimbals/three_axis_pod/docs/system.md)，自验证结果见 [验证说明](models/gimbals/three_axis_pod/docs/validation.md)。

空目录中的 `.gitkeep` 用于让 Git 保存目录；添加实际文件后可以移除。
