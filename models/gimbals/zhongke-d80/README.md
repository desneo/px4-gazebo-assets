# 中科 D80 两轴云台仿真

本组件在本机 1.56 上提供独立的前视两轴云台、单路可见光视频、中文地面站和中科二进制 UDP 接口。外形和内部运动链复制自 `two_axis_pod`，前视安装及真值指点/自动选取交互参考艾晟组件；所有资产、控制器和配套程序保存在本目录，不修改原模型、已有装配、场景或 PX4。

默认方位 −90°～+90°、俯仰 −90°～+30°，每轴最大实际转速 20°/s，总质量 195 g。相机默认 1920×1080@30 Hz、50 mm、7.03°×3.95°；修改焦距按固定等效感光面联动视场角。默认 300 m 仅限制目标发现与重新选取，已锁目标超距后继续跟踪。

复制外形约为 φ84 mm×91 mm，保持原尺寸；产品表中的 φ49 mm×80 mm 是资料参数，不用于缩放外形。供电、功耗和标称指向精度不产生电气或随机误差模型。组件范围是云台观察仿真及自身演示，不包含载机飞行控制。

## 运行

在本组件目录执行：

```bash
cd /home/aeroarx/zsh/px4-gazebo-assets/models/gimbals/zhongke-d80
bash scripts/setup.sh
bash scripts/build.sh
bash scripts/run_demo.sh --session demo --gui
```

依赖 Gazebo Harmonic（Sim 8、Transport 13、Msgs 10）、C++17/CMake、Python 3.12，以及 NVIDIA GStreamer NVENC/NVDEC。`setup.sh` 使用模型上一级的独立 `.venv-zhongke-d80` 和 `.cache-zhongke-d80`；不配置目标检测网络。视频不回退为软件编解码。

`run_demo.sh` 保持前台运行，启动独立 Gazebo 分区、世界和网关，实际端点及进程身份写入 `runs/demo/session.ini`。需要无浏览窗口时使用 `--headless`；指定另一份性能配置使用 `--config /绝对路径/parameters.ini`。UDP 或 RTSP 端口占用时报错，不接管已有服务。

在另一个终端打开地面站：

```bash
./ground_station/run.sh --session demo
```

地面站也可独立连接设备或仿真网关，在 `ground_station/` 目录执行 `./run.sh view config.ini`，或直接 `./run.sh` 使用默认配置。独立模式从地面站INI的 `[connection]` 读取设备IP、UDP端口和RTSP地址，不需要本机仿真会话清单。`--session` 是显式读取本机仿真实际端点和诊断的启动方式，详见[地面站说明](ground_station/README.md)。

方向键和摇杆用于手动转动；释放输入即发送零速。`R` 回零、`O` 自动锁定、空格停止、`Esc` 退出。单轴回零以另一轴的新鲜实测角保持其当前位置。双击视频选取画面内离点击位置最近的合格真值目标；点击附近没有目标时仍回退选取最近候选并提示。自动锁定选择最靠近画面中心的合格候选，正常跟踪保持同一实体，无候选时等待。

“发送中科报文”把实际收到且校验通过的 32 字节二进制遥测转换成 15 字段 JSON；默认关闭。编辑目的地址先停发并清队列，Enter/Esc 只结束编辑，点击发送才恢复。配置位置与操作详见 [地面站说明](ground_station/README.md)。

停止地面站或整个演示：

```bash
./ground_station/stop.sh --session demo
bash scripts/stop_demo.sh --session demo
```

停止入口只处理本组件对应清单中 PID 和启动身份匹配的进程，不使用全局 `pkill`。关闭地面站也会停止其人工输入、控制租约续期和 JSON 转发。
无参数 `ground_station/stop.sh` 会查找本中科组件登记的全部活跃地面站，并核对实际程序路径，只关闭客户端；Gazebo、PX4、网关及其他模型保持运行。带 `--station-config` 或 `--session` 时只停止指定客户端。

## 参数与接口

性能配置为 [parameters.ini](parameters.ini)，每项有中文说明，修改后重启组件。地面站连接、显示偏好和JSON接收地址在 [ground_station/config.ini](ground_station/config.ini)；`connection.parameters_file` 指向性能INI，不保存第二套机械或光学参数。

| 默认资源 | 值 |
| --- | --- |
| 模型实体 | `zhongke_d80` |
| 控制/遥测 UDP | `127.0.0.1:8870`，收 16 字节、回 32 字节 |
| 地面站本地UDP | `0.0.0.0:0`，0表示分配临时端口；同一socket发控制、收遥测，可在连接配置固定 |
| RTSP | `rtsp://127.0.0.1:8556/zhongke_d80_1` |
| 本机只读诊断 | `/zhongke_d80/diagnostics`，需使用会话清单中的 Gazebo 分区 |
| 遥测目标频率 | 60 Hz，来源暂停或失效时不重播旧测量 |
| JSON 默认目的地址 | `127.0.0.1:9000`，默认不发送 |

装配生成器和网关共用运行覆盖接口：`--model`、`--aircraft-id`、`--world`、可重复的 `--target-name`、`--udp-bind`、`--udp-port`、`--rtsp-bind`、`--rtsp-port`、`--rtsp-mount`。源性能INI先读取校验，再应用显式运行字段并重新校验、计算配置指纹，不生成逐机INI。生成器和网关必须接收相同字段以保持指纹一致；机械与光学参数继续由源INI维护。基础组件独立演示仍使用原有会话入口。

标准端口只使用中科固定二进制帧，不添加 JSON hello、seq、ACK 或设备 ID。一个有效客户端持有默认 2 s 租约；GUI、命令工具和实时验收工具应依次运行，避免相互争用控制。命令发送成功只代表交给本机 socket，执行状态依赖后续遥测与诊断。

目标名单使用精确实体名称和 Gazebo 真实位置，不运行 YOLO 或拖框纹理跟踪，也不判断实体遮挡。目标身份、失锁原因和限位通过本机诊断显示；诊断不替代二进制测量，不进入外发 JSON。完整约定见 [协议说明](docs/protocol.md)。

## 检查入口

```bash
../.venv-zhongke-d80/bin/python -m unittest discover -s tests -p 'test_*.py'
../.venv-zhongke-d80/bin/python -m unittest discover -s ground_station/tests
../.venv-zhongke-d80/bin/python -m unittest discover -s controller/service/tests
ctest --test-dir controller/build --output-on-failure
../.venv-zhongke-d80/bin/python tools/check_telemetry.py --offline
../.venv-zhongke-d80/bin/python tools/check_telemetry.py --session demo --duration 10 --output runs/demo/telemetry_validation.json
../.venv-zhongke-d80/bin/python tools/check_physics.py --session demo --output runs/demo/physics
```

实时工具使用现有会话，不自行启动世界；运行前关闭 GUI 控制客户端并等待租约释放。物理检查会操纵云台检查人工运动、回零及机械端点，保存原始采样、CameraInfo 和实际渲染图；几何遮挡/穿插仍需人工审阅。

无地面站窗口检查真实 UDP 和视频并截图：

```bash
../.venv-zhongke-d80/bin/python ground_station/ground_station.py --session demo --selftest --duration 8 --snapshot /tmp/zhongke-d80-station.png
```

`--decode-test` 仅检查实际 RTSP/NVDEC，不建立控制租约。检查结果和人工复核边界记录在 [验收说明](docs/validation.md)，不能由单元测试或布局夹具截图推断实际世界已通过验收。

## 目录索引

| 路径 | 内容 |
| --- | --- |
| `cad/`、`meshes/` | 复制资产、CAD 质量分配元信息及 SHA256，见 [资产说明](docs/assets.md) |
| `model.sdf`、`model.config`、`generated.json` | 由性能配置生成的模型和参数快照 |
| `common/` | INI 校验、光学派生、中科协议及接收类型 |
| `controller/` | 独立 C++ 插件、复制源码依赖及二进制网关/视频服务 |
| `ground_station/` | GUI、实际二进制客户端和 JSON 转发 |
| `scripts/`、`tools/`、`tests/` | 环境、构建、会话、生成、命令和验收入口 |
| `runs/<会话>/` | 生成世界、会话清单、日志和验收数据 |
| [安装说明](docs/mounting.md) | 一次 Y90 前视、轴方向、定位标志与浏览近景 |
| [参数说明](docs/parameters.md) | 全部配置项、单位、有效范围和作用 |

产品资料来源为本目录《中科DYT性能参数说明.txt》《中科友成-DYT控制协议.docx》；JSON 字段对应 `/home/aeroarx/zsh/PX4-Autopilot/msg/DytZhongkeTelemetry.msg`。本组件未附加厂商未提供的硬件能力或许可声明。
