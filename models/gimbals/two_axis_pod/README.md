# 两轴立装吊舱

`two_axis_pod` 是独立的两轴立装相机组件，包含 Gazebo 模型、飞机目标检测与多目标跟踪、图像跟踪控制插件、UDP/RTSP 网关、地面站与固定立装演示。导引头以水平圆环为底部，后部弧形罩壳向上延伸，球形相机位于罩壳前方的双侧俯仰支架内。圆环直径 84 mm，整机高度 91 mm、底部以上主体高度 83 mm、底部圆面厚度 8 mm、相机球直径 48 mm，并表现 6×M3 孔位。三轴组件保持独立。

![两轴立装吊舱零位实景](docs/preview.png)

## 安装与角度

本组件安装方式已确认为立装：Ø84 mm 底部圆环与地面平行，导引头整体位于圆环上方，不另设矩形底座。模型 +X 朝前、+Y 朝左、+Z 朝天，原点位于底部安装面中心；沿 +Z 的整机高度为 91 mm。装配时固定 `mount_link`，不能把整个模型设为静态，否则活动关节也会被冻结。罩壳和俯仰支架随偏航整体旋转，球形相机在支架内俯仰。

- 偏航为外轴，绕竖直 +Z 转动，范围 −90°～+90°；按右手定则，正值把 +X 前向转向 +Y 左侧。
- 俯仰为内轴，绕外轴带动的局部 +Y 转动，范围 −45°～+90°；俯仰 0° 朝天，正值转向前方，+90° 朝前，−45° 朝后上方（均指偏航为零时）。
- 默认两轴均为 0°，镜头朝天。`reset`、`home` 回两轴机械零位；`reset_axis` 回指定轴零位并保持另一轴的采样位置。
- 无滚转关节、滚转控制或主动画面水平稳定。零位朝天时视线方向与偏航轴平行；偏航转动保持朝天方向，但球心随支架移动，画面方向也随之旋转。俯仰离开零位后，偏航可左右调整视线方向。

运动学、示意与边界见[安装说明](docs/mounting.md)。控制协议使用 `protocol_version=4`、`angle_convention=upright_joint`，与三轴组件的 PX4 FRD 角度定义不同。

机载装配可通过插件 `mount_dynamic=true` 启用实际安装姿态更新和跟踪中的载机转动补偿；机械轴随载机变化，零位仍沿安装 +Z。组合模型的参数、质量估计与运行入口属于对应装配和场景目录，不改变本组件的固定立装演示。

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

Linux 地面站启动时自动检查并安装缺失的运行依赖，使用仓库根目录 `.venv-linux` 并读取带注释的 `ground_station/config.ini`，无需预先执行安装命令，也无需重复传入端口、飞机 ID 和解码器。运行依赖与网络地址见[启动说明](scripts/README.md)、[网关说明](controller/service/README.md)和[地面站说明](ground_station/README.md)。演示世界不启动 PX4。相机为 1920×1080@30Hz、水平视场 60°。

地面站提供关闭视觉、目标检测、自动锁定和手动框选四个互斥模式。目标检测使用 `models/yolo11/aircraft_yolo11n/model.onnx` 的单类飞机模型；多目标轨迹默认连续命中 5 帧后确认，检测及自动搜索阶段的 RTSP 画面包含红框与稳定轨迹号。自动锁定首次选择离画面中心最近的已确认目标，然后停止多目标叠加并把该框交给 CSRT，地面站只显示一个绿色跟踪框；CSRT 最终丢失后恢复检测搜索。视频只允许 NVENC 编码和 NVDEC 解码，不提供软件回退。

网关支持向配置的 IP/端口额外发送 UDP JSON 状态副本，默认关闭、默认频率 20 Hz。配置文件为 `controller/service/status_udp.json`；该独立输出不改变 station 原有遥测和控制，见[主动上报配置](controller/service/README.md#额外-udp-json-状态上报)。

## 参数与资料

- `parameters.json`：尺寸、机械限位、最大速度/加速度与相机参数。
- `models/yolo11/aircraft_yolo11n/`：单类飞机 ONNX、类别、来源、许可、元数据与校验值。
- `controller/service/vision.json`：检测置信度、NMS、多目标确认/丢失参数和推理线程数。
- `tools/generate_model.py`：生成 `model.sdf`、罩壳网格与 `tests/world.sdf`。演示环境底图来自 `tests/world_base.sdf`，生成时加入固定立装吊舱和朝天纹理靶标。
- [控制协议](docs/control-protocol.md)：机械角、命令与遥测约定。
- [验证说明](docs/validation.md)：验证命令、结果及限制。
- [演示与自测](tests/README.md)：演示世界与在线检查。
- [完整地理背景测试](../../../scenarios/geographic_two_axis_pod/README.md)：空中动态靶标、实体遮挡和地面站验证，不加载载机。

修改参数后在仓库根目录运行 `python3 models/gimbals/two_axis_pod/tools/generate_model.py` 并重新验证。控制插件直接读取模型中的限位参数；相机姿态和实际关节轴必须与安装说明一致。

外廓尺寸按设计资料设置，48 mm 相机球采用提供的实物尺寸。罩壳曲面、支架、球心位置、孔位分布和碰撞体为外观与运动仿真近似，质量和惯量未做实物标定。

视频网关支持多机共用 RTSP 8555，URL 为 `rtsp://<host>:8555/two_axis_pod_<aircraft_id>`。网关与地面站使用相同 `--aircraft-id`，默认 1；多路隔离、运行方式与测试见[网关说明](controller/service/README.md#多机共享-rtsp)。
