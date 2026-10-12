# 吊舱功能自验证

## 结果范围

控制保护与通信回归通过，基本控制、遥测、静态纹理跟踪和视频链路可运行。真实飞行、移动目标长期指标、GUI完整操作与跨机器弱网未验收。

环境：Gazebo Harmonic 8.15.0、OpenCV C++ 4.6.0、Python 3.12。基本链路使用独立 `GZ_PARTITION`、UDP18765、RTSP18554、x264编码与avdec_h264解码，日志保存在 `runs/archive_check/` 和 `runs/plugin_watchdog/`（Git忽略）。系统Python缺少cv2，地面站联调通过调用方提供的 `/home/aeroarx/test/gambal_test/vendor` 补足Python依赖，仓库不隐式引用该路径。

| 检查 | 当前结果 |
|---|---|
| CMake构建与CTest | 插件构建成功，正常控制与watchdog两组硬断言测试通过 |
| 控制回归 | 0.5秒手动超时、续期恢复、0.5秒图像停转/2秒LOST、INIT与连续坏帧超时、动态上下限位、独立watchdog时钟及模式状态清理通过 |
| Python协议/客户端 | 31项全部通过，没有expectedFailure；含同名逆序回执、多客户端并发、重传、乱序、缓存窗口、超时重登记、参数错误、pending清理、来源/垃圾包及同机时延门控 |
| 真实插件隔离测试 | 因子0.2慢速仿真中，遥测观测手动停止0.578秒，断流停止0.555秒、LOST2.056秒；差值含控制与遥测周期，不把观察延迟作为新阈值 |
| 插件回执/选框 | 实际插件原样回传两条同名请求的request_id，参数失败也关联原请求；有目标框时归一化框始终有效 |
| 实际UDP网关 | 双客户端同名home回执隔离、参数失败携seq、缓存重发和过时manual拒绝通过 |
| 地面站UDP自检 | 4秒199条遥测，约49.6Hz；登记/心跳、home回执、manual生效和超时重登记通过；待回执为0 |
| RTSP解码 | 1920×1080，6秒取得165帧，约30.5fps，165/165帧序号识别成功，截图保存成功 |
| 静态纹理与到位控制 | 棋盘选框TRACK、目标有效；stop归零，reset水平前向、home朝天回零通过在线控制检查 |

## 重复验证

在仓库根目录：

```bash
cmake -S models/gimbals/three_axis_pod/controller -B models/gimbals/three_axis_pod/controller/build
cmake --build models/gimbals/three_axis_pod/controller/build -j4
ctest --test-dir models/gimbals/three_axis_pod/controller/build -V
python3 -m unittest discover -s models/gimbals/three_axis_pod/ground_station/tests -v
python3 models/gimbals/three_axis_pod/controller/tests/check_plugin.py
```

最后一条会自行启动独立Gazebo进程，生成没有相机传感器的临时测试世界，注入可控图像，并在结束时清理进程。测试文件与日志保存在 `runs/plugin_watchdog/`，需要gz Python bindings和Gazebo运行权限。仿真因子0.2用于证明超时使用真实单调时间；模拟图像仅用于可控断流，不替代真实视频测试。

协议单元测试用标准库AST加载实际类方法，不依赖cv2/gi；实际网关与插件运行测试用于补充真实进程与网络证据。

基本联调先按[组件使用说明](../scripts/README.md)启动仿真与网关，再运行：

```bash
bash models/gimbals/three_axis_pod/ground_station/run.sh --selftest 4 --send-home --send-manual --test-timeout 2.5
bash models/gimbals/three_axis_pod/ground_station/run.sh --decode-test 6 --decoder avdec --snapshot /tmp/gimbal.png
python3 models/gimbals/three_axis_pod/tests/check_commands.py
```

自定义测试端口时为前两条加 `--udp-port 18765 --rtsp-port 18554`，最后一条加 `--port 18765`。在线控制检查会转动吊舱，依赖默认棋盘世界和未被其他客户端干扰的场景，按遥测等待到位，最长15秒。

## 未覆盖

- GUI鼠标框选、快捷键与中文字体的完整人工操作。
- 移动小车轨迹、长期遮挡重捕获、跟踪精度与稳定性验收指标。
- NVIDIA硬编硬解，当前实际视频验证使用软编软解。
- 跨机器时钟同步、弱网压力与多操作员控制权仲裁。
- 载机安装、PX4联动、姿态补偿与真实地理环境中的目标跟踪。

当前行为与边界见[系统说明](system.md)。
