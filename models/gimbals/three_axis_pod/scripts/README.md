# 三轴吊舱使用说明

使用 [models/gimbals/three_axis_pod/tests/world.sdf](../tests/world.sdf)，在固定支架上运行三轴吊舱、相机和测试小车，不启动 PX4。

仓库根目录：先构建插件，再设置 `GZ_PARTITION=three_axis_pod_test`，运行 `bash models/gimbals/three_axis_pod/scripts/run_plugin_sim.sh` 与 `bash models/gimbals/three_axis_pod/scripts/run_bridge.sh --encoder x264`；客户端用 `bash models/gimbals/three_axis_pod/ground_station/run.sh --decoder avdec`。

`env.sh` 配置本仓库模型搜索路径、插件路径和组件的 `server.config`；不依赖源工程的 PX4、scripts 或 vendor 目录。`GIMBAL_RUNDIR` 可改写日志/PID目录，默认 `runs/three_axis_pod/`。停止时用同一运行目录下的 `stop_bridge.sh/stop_plugin_sim.sh`，脚本只处理记录的服务进程组。

仿真启动脚本检查进程存活；图像与遥测是否就绪需通过日志或客户端自检确认。自定义世界可作为 run_plugin_sim.sh 的首个参数；使用不同世界/模型名时须给网关配置对应参数。

依赖、功能、接口及已知问题见 [系统说明](../docs/system.md)。
