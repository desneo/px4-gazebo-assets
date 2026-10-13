# 两轴侧装吊舱

`two_axis_pod` 是独立的两轴侧装相机组件，包含 Gazebo 模型、图像跟踪控制插件、UDP/RTSP 网关、地面站与固定侧装演示。模型采用侧面安装板、单侧支臂和球形相机的近似外形；尺寸沿用三轴吊舱量级，不代表实物测量结果。三轴组件保持独立。

![两轴侧装吊舱零位实景](docs/preview.png)

## 安装与角度

本组件安装方式已确认为侧装：安装板竖直，模型 +X 为板法线朝外的参考前向，+Y 朝左，+Z 朝天。模型原点位于安装板外轴中心；装配时固定 `mount_link`，不能把整个模型设为静态，否则活动关节也会被冻结。标准侧装不需要额外旋转模型，位置与绕世界竖直方向的安装朝向可按装配设置。

- 偏航为外轴，绕安装板法线 +X 转动，范围 −90°～+90°；从吊舱看向安装板时顺时针为正。
- 俯仰为内轴，绕外轴带动的局部 +Y 转动，范围 −45°～+90°；俯仰 0° 朝天，正值转向前方，+90° 朝前，−45° 朝后上方（均指偏航为零时）。
- 默认两轴均为 0°，镜头朝天。`reset`、`home` 回两轴机械零位；`reset_axis` 回指定轴零位并保持另一轴的采样位置。
- 无滚转关节、滚转控制或主动画面水平稳定。偏航是机械轴名称，不是绕世界竖直轴的航向角。俯仰到 +90° 时视线与外轴平行，外轴转动表现为画面旋转，无法通过该轴左右调整视线。

运动学、示意与边界见[安装说明](docs/mounting.md)。控制协议使用 `protocol_version=3`、`angle_convention=side_mount_joint`，与三轴组件的 PX4 FRD 角度定义不同。

## 构建与运行

在仓库根目录执行：

```bash
cmake -S models/gimbals/two_axis_pod/controller -B models/gimbals/two_axis_pod/controller/build
cmake --build models/gimbals/two_axis_pod/controller/build -j4
ctest --test-dir models/gimbals/two_axis_pod/controller/build --output-on-failure
bash models/gimbals/two_axis_pod/scripts/run_plugin_sim.sh
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh
bash models/gimbals/two_axis_pod/ground_station/run.sh
```

运行依赖与网络地址见[启动说明](scripts/README.md)、[网关说明](controller/service/README.md)和[地面站说明](ground_station/README.md)。演示世界不启动 PX4。相机为 1920×1080@30Hz、水平视场 60°。

## 参数与资料

- `parameters.json`：尺寸、机械限位、最大速度/加速度与相机参数。
- `tools/generate_model.py`：生成 `model.sdf` 与 `tests/world.sdf`。演示环境底图来自 `tests/world_base.sdf`，生成时加入固定侧装吊舱和朝天纹理靶标。
- [控制协议](docs/control-protocol.md)：机械角、命令与遥测约定。
- [验证说明](docs/validation.md)：验证命令、结果及限制。
- [演示与自测](tests/README.md)：演示世界与在线检查。
- [完整地理背景测试](../../../scenarios/geographic_two_axis_pod/README.md)：空中动态靶标、实体遮挡和地面站验证，不加载载机。

修改参数后在仓库根目录运行 `python3 models/gimbals/two_axis_pod/tools/generate_model.py` 并重新验证。控制插件直接读取模型中的限位参数；相机姿态和实际关节轴必须与安装说明一致。
