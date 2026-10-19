# 两轴吊舱启动与停止

在仓库根目录执行以下命令。演示世界 `tests/world.sdf` 使用固定立装两轴吊舱和测试目标，世界名为 `two_axis_pod_demo`。

```bash
cmake -S models/gimbals/two_axis_pod/controller -B models/gimbals/two_axis_pod/controller/build
cmake --build models/gimbals/two_axis_pod/controller/build -j4
export GZ_PARTITION=two_axis_pod_demo
bash models/gimbals/two_axis_pod/scripts/run_plugin_sim.sh
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --encoder nvenc
source models/gimbals/two_axis_pod/scripts/env.sh
gz sim -g -v2
```

另开同一分区的终端运行地面站：

```bash
export GZ_PARTITION=two_axis_pod_demo
bash models/gimbals/two_axis_pod/ground_station/run.sh
```

地面站启动入口自动检查并安装缺失依赖，使用仓库根目录 `.venv-linux` 并读取带注释的 `ground_station/config.ini`，无需单独运行安装命令，也无需在命令行重复填写端口、飞机 ID 和解码器。

仿真脚本启动无窗口服务端；`gz sim -g` 打开 Gazebo 3D 窗口。视频强制使用 NVIDIA NVENC 编码和 NVDEC 解码，不提供软件回退；缺少 GPU、驱动、`nvcudah264enc` 或 `nvh264dec` 时会显示中文错误并退出。

`run_bridge.sh` 会先检查 Gazebo/GStreamer Python 绑定、OpenCV DNN、`nvcudah264enc` 和 `controller/service/vision.json` 指定的 ONNX 模型。默认使用仓库根目录 `.venv-linux`；环境不存在或依赖不完整时自动调用 `setup_bridge.sh` 安装系统包和 Python 包，并实际用 OpenCV DNN 加载归档的 ONNX。NVIDIA 驱动或 NVENC 不可用时明确报错，不回退软编码。`GIMBAL_PYTHON` 仅用于显式指定已完整配置的高级环境；指定后启动器不会修改该解释器。

默认 UDP 8766、共享 RTSP 8555 `/two_axis_pod_1`，内部 RTP 默认自动分配，运行日志和 PID 存于 `runs/two_axis_pod/`。`GIMBAL_RUNDIR` 可改写运行目录；并行实例使用独立运行目录、UDP 控制端口和飞机 ID。同一 Gazebo 世界需要独立模型名、插件话题前缀和图像话题；不同仿真可使用独立 `GZ_PARTITION`。`env.sh` 设置仓库资源路径与两轴 `controller/build/lib` 插件路径。自定义世界作为仿真脚本首参，并相应设置网关 `--world`、`--gimbal-model`；相机参数为 `--camera-link`、`--camera-sensor`。

额外主动 UDP JSON 上报通过 `controller/service/status_udp.json` 配置，默认关闭、启用后默认 20 Hz。将 `enabled` 设为 `true` 并配置接收端 IP/端口后，重新启动网关；也可直接指定参数：

```bash
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --encoder nvenc \
  --status-ip 192.168.1.100 --status-port 9000 --status-hz 20
```

这是独立发送的原有遥测副本，不需要接收端登记，不改变 station 的 UDP 8766 与默认 50 Hz 广播。其他配置文件可通过 `--status-config` 指定；字段和时效行为见[网关说明](../controller/service/README.md#额外-udp-json-状态上报)。

```bash
bash models/gimbals/two_axis_pod/scripts/stop_bridge.sh
bash models/gimbals/two_axis_pod/scripts/stop_plugin_sim.sh
```

停止脚本只处理同一运行目录记录的服务进程组。Gazebo GUI 和地面站窗口另行关闭。启动脚本仅检查进程存活，遥测与图像就绪使用地面站在线自检验证，见[地面站说明](../ground_station/README.md)。立装机械角度和运动链见[安装说明](../docs/mounting.md)。

多机共用 RTSP 端口，通过 `--aircraft-id` 生成 `/two_axis_pod_<ID>`。地面站传入相同 ID；运行目录和控制 UDP 端口各自独立。完整示例与中继生命周期见[多机视频说明](../controller/service/README.md#多机共享-rtsp)。
