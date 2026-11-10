# 验证状态

验证对象为 `jielian-A2416A` 基础模型、50 Hz 真值跟踪插件、45/104 字节协议、100 Hz UDP 网关、硬件 H.264 链路和配套地面站。

## 已通过

- `parameters.ini` 确定性生成 SDF；模型只有 `seeker_link`，无关节，圆柱尺寸、质量、惯量、光轴、1920×1080@25 Hz、37°×21°和显式内参均通过契约检查。
- C++ 跟踪状态机覆盖距离边界、排序、锁定槽位、人工不换目标、`TEMP_LOST` 2.5 s 重捕获、失调角符号和限幅。
- Gazebo 实际加载插件并以约 50 Hz 发布遥测；四个真值目标可检测，`xiaomotuo` 网格目标的真实包围盒投影和地理位置转换可用。
- 协议测试覆盖黄金帧、小端、校验、错误头、截断/粘连/错误长度、坐标边界、离散命令去重、单客户端切换、2 s 超时、支持命令及明确拒绝命令。
- 网关测试验证 50 Hz 测量可被相邻 100 Hz 遥测帧复用，同时每帧更新时间戳和报文序号；地面站按每个有效遥测帧逐帧转发顶层 37 字段艾晟 JSON，字段集合与 `aisheng_udp_bridge` 输入契约一致且不夹带诊断字段。
- 地面站契约覆盖视频区域点选/框选、识别、自动锁定、取消跟踪、变倍、增强、OSD、“发送艾晟报文”/“停止发送”、可编辑 IP/Port、INI 默认值和命令行逐实例覆盖，不含云台角度或角速度控件。
- 地面站窗口生命周期覆盖关闭按钮后立即退出且不被下一次 `imshow` 重建；进程生命周期覆盖按绝对配置隔离多实例、定向/全部停止、仅预览、PID复用和错误身份拒绝。停止入口只向精确匹配的当前用户 A2416A 地面站发送信号，不影响 Gazebo、PX4、网关或 RTSP。
- 基础模型预览世界、定位标志、箭头、近景相机、0.005 m 近裁剪和 0.1 浏览灵敏度入口可用。
- 02场景实测 H.264 编码端为25.0 fps且无丢帧；生产 `VideoReceiver` 经 TCP RTSP 和 `nvh264dec` 接收960×540 BGR画面，连续25秒收到601帧，进入稳态后每秒25帧。断开后重连首秒收到23帧，随后保持25帧/秒。

复验命令：

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s tests -p '*.py' -v
gz sdf -k model.sdf
SDF_PATH="$(cd .. && pwd)" gz sdf -k worlds/preview.sdf
```

## 视频硬件

视频链路只允许 NVIDIA NVENC H.264 编码和 `nvh264dec` 解码，不提供软件回退。当前主机的 NVIDIA 驱动和 GStreamer `nvcodec` 可用，硬件预检可加载 `nvcudah264enc` 与 `nvh264dec`。编码管线强制 NV12 输入、H.264 High profile、byte-stream、25 帧 GOP、无 B 帧和默认 2 Mbps CBR；接收端通过 TCP RTSP 使用 NVDEC。RTSP服务常驻绑定RTP端口，经 `shmsink/shmsrc` 为共享媒体源提供完整access unit，避免客户端连接前丢包。

复验入口：

```bash
bash scripts/run_services.sh
bash ground_station/run.sh --preflight
```

若驱动或 NVCODEC 条件失效，预检会给出具体缺失元素并终止视频链路，不会触发软件编解码回退。
