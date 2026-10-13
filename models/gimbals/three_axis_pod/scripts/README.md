# 三轴吊舱使用说明

使用 [models/gimbals/three_axis_pod/tests/world.sdf](../tests/world.sdf)，在固定支架上运行三轴吊舱、相机和测试小车，不启动 PX4。

在仓库根目录执行：

```bash
cmake -S models/gimbals/three_axis_pod/controller -B models/gimbals/three_axis_pod/controller/build
cmake --build models/gimbals/three_axis_pod/controller/build -j4
export GZ_PARTITION=three_axis_pod_test
bash models/gimbals/three_axis_pod/scripts/run_plugin_sim.sh
bash models/gimbals/three_axis_pod/scripts/run_bridge.sh --encoder nvenc
source models/gimbals/three_axis_pod/scripts/env.sh
gz sim -g -v2
```

最后一条打开 Gazebo 窗口，关闭窗口后服务仍可运行。另开终端运行地面站：

```bash
bash models/gimbals/three_axis_pod/ground_station/run.sh --decoder avdec
```

网关默认使用 NVIDIA `nvcudah264enc` 硬编码，需要可用 GPU、驱动与 GStreamer 插件；失败时不会自动软件编码回退。`--decoder avdec` 只指定地面站软件解码。组件脚本默认只启动无窗口服务端，展示 Gazebo 需要上述 `gz sim -g` 或地理场景的 `--gui`。

`env.sh` 配置本仓库模型搜索路径、插件路径和组件的 `server.config`；不依赖源工程的 PX4、scripts 或 vendor 目录。`GIMBAL_RUNDIR` 可改写日志/PID目录，默认 `runs/three_axis_pod/`。停止时用同一运行目录下的 `stop_bridge.sh/stop_plugin_sim.sh`，脚本只处理记录的服务进程组。

仿真启动脚本检查进程存活；图像与遥测是否就绪需通过日志或客户端自检确认。自定义世界可作为 `run_plugin_sim.sh` 的首个参数；使用不同世界/模型名时须给网关配置 `--world`、`--gimbal-model`，相机名通过 `--camera-link`、`--camera-sensor` 指定。并行实例须分别配置 `GZ_PARTITION`、`GIMBAL_RUNDIR` 及网关 `--udp-port`、`--rtsp-port`、`--rtp-port`，默认端口为 8765、8554、5600；地面站连接对应 UDP/RTSP 端口。

依赖、功能、接口及已知问题见 [系统说明](../docs/system.md)。

真实地理背景与 2 秒实体遮挡验证使用 [geographic_camera 场景](../../../../scenarios/geographic_camera/README.md)，人工验证入口为 `bash scenarios/geographic_camera/run.sh --gui`，同时打开 Gazebo 3D 窗口；场景默认使用 NVENC 硬编码。

该入口初始化吊舱对准小车，地面站仍须另行启动。默认安装 `skyward`，悬挂安装用 `--mount-mode downward`。`manual` 和 `attitude_deg` 使用[安装图中的 PX4 对外姿态约定](../docs/mounting.md)，`joints_deg/cmd_dps` 保留机械诊断；客户端须识别协议版本 2 和 `px4_frd`。停止服务使用相同 `GIMBAL_RUNDIR`；`stop_plugin_sim.sh` 不管理另行启动的 Gazebo GUI，GUI 与地面站窗口需分别关闭。
