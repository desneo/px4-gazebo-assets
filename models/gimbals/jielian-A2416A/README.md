# A2416A 捷联式导引头

`jielian-A2416A` 是固定光轴的单可见光捷联式导引头基础模型。模型只有一个 `seeker_link`，不含机械框架、转动关节或伺服控制；相机、检测与跟踪、二进制 UDP 网关、硬件 H.264 视频和地面站均随组件保存。

当前机械外形是 Ø60×60 mm、0.05 kg 的占位圆柱，光轴沿本体 `+Z`。产品光学数据、协议适用边界和验收准则见 [开发规格](docs/development-spec.md)。

## 目录与配置

- `parameters.ini`：模型、光学、跟踪、协议、网络、视频、浏览相机和演示目标的唯一人工配置源。
- `scripts/generate_model.py`：确定性生成 `model.sdf` 和 `worlds/preview.sdf`。
- `src/`：50 Hz 真值投影与单目标跟踪 Gazebo 插件。
- `protocol/`、`gateway/`：45/104 字节二进制协议、100 Hz UDP 遥测和单客户端会话。
- `video/`：NVENC H.264 编码、RTP/RTSP 发布、OSD 与 CLAHE。
- `ground_station/`：视频、点选/框选、识别、自动锁定、变倍及艾晟桥兼容 JSON 转发界面。
- `tests/`：模型、投影状态机、协议、网关、地面站和视频前置条件测试。

## 生成、构建与自测

在本目录执行：

```bash
python3 scripts/generate_model.py
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s tests -p '*.py' -v
```

预览世界：

```bash
GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" \
  python3 scripts/run_preview.py
```

世界内可搜索 `JIELIAN_A2416A`、`JIELIAN_A2416A_LOCATION_MARKER` 和四个 `target_aircraft*` 目标。定位标志没有碰撞体；近景相机近裁剪为 0.005 m，启动脚本将浏览灵敏度设置为 0.1。

## 网关、视频与地面站

预览世界运行后，另开终端启动 RTSP 与 UDP 网关：

```bash
bash scripts/run_services.sh
```

再启动地面站：

```bash
bash ground_station/run.sh
```

地面站支持多实例，并按配置绝对路径登记独立进程。点击窗口右上角关闭按钮、按 `Q` 或按 `Esc` 都会退出当前实例并释放视频、遥测和 JSON 转发资源。也可使用专用停止入口；停止地面站不影响 Gazebo、PX4、导引头网关或 RTSP 服务：

```bash
# 停止当前用户的全部A2416A地面站
bash ground_station/stop.sh

# 只停止使用指定配置的实例；配置必须是绝对路径
bash ground_station/stop.sh --config /绝对路径/aircraft_1.ini

# 仅预览匹配的进程
bash ground_station/stop.sh --dry-run
```

停止前会核对当前用户、PID启动时间、Python解释器、程序模块、工作目录和配置路径；不会使用全局 `pkill`，也不会向整个进程组发送信号。

默认控制与遥测端口为 `127.0.0.1:8001`，RTSP 地址为 `rtsp://127.0.0.1:8554/jielian_a2416a`。网关和地面站启动前都会验证 NVIDIA H.264 编解码能力；没有可加载的 NVENC/`nvh264dec` 时会给出诊断并退出，不使用软件回退。编码输入强制为 NV12，输出为 H.264 High profile、byte-stream、无 B 帧的低延迟流。RTSP 服务通过常驻 RTP 接收和共享内存媒体源支持首次连接及断线重连，运行环境还需提供 GStreamer `shmsink/shmsrc`。

地面站以 10 Hz 保持控制会话，离散命令连续发送三次。点击“发送艾晟报文”后，每个有效 100 Hz 二进制遥测帧都会转换为 `aisheng_udp_bridge` 可直接接收的顶层 37 字段 JSON，并发送到面板中可编辑的 IP 和 Port；再次点击“停止发送”结束转发。默认目的地址为 `127.0.0.1:9000`，启动时可用 `--forward-host`、`--forward-port` 覆盖 INI 默认值。

“自动锁定”按钮会先开启识别，再开启自动锁定，初始状态下可直接使用。地面站界面不提供图像增强或 OSD 控制按钮；对应二进制协议命令和视频处理能力仍保留，供其他兼容控制端使用。

## 火箭装配

固定安装到火箭穿越机的派生资产位于 [`assembledmodels/rocket_jielian_a2416a`](../../../assembledmodels/rocket_jielian_a2416a/README.md)。装配生成器读取本基础模型，不回写本目录或现有火箭资产。

PX4 仓库的 02 场景支持直接启动该装配：

```bash
bash scenarios/02_rocket_two_axis_pod/start.sh \
  --aircraft-model rocket_jielian_a2416a --gui
```

场景逐机生成地面站 INI，复用 `aisheng_udp_bridge` 接收“发送艾晟报文”输出，并把 A2416A 组合飞机加入 3 km 雷达航迹；真值目标包含场景飞机、`xiaomotuo` 和 `xiaobianfu`。当前验证结果见 [验证说明](docs/validation.md)。
