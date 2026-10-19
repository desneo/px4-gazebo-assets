# 两轴立装演示与自测

`world.sdf` 提供 1.5 m 高的水平台面、固定立装吊舱、上方纹理靶标和独立近景外观观察相机。Ø84 mm 底部圆环水平落在台面上，导引头整体向上高 91 mm。吊舱仅固定 `mount_link`，两根轴仍可活动。世界内直接包含生成的吊舱，网关模型名为 `two_axis_pod`，世界名为 `two_axis_pod_demo`；不启动 PX4。

`world_base.sdf` 是环境源文件。修改环境或模型参数后，运行 `python3 models/gimbals/two_axis_pod/tools/generate_model.py`，同步生成基本模型和演示世界。

## 离线验证

仓库根目录：

```bash
python3 models/gimbals/two_axis_pod/tests/test_model.py
cmake -S models/gimbals/two_axis_pod/controller -B models/gimbals/two_axis_pod/controller/build
cmake --build models/gimbals/two_axis_pod/controller/build -j4
ctest --test-dir models/gimbals/two_axis_pod/controller/build --output-on-failure
python3 -m unittest discover -s models/gimbals/two_axis_pod/ground_station/tests
gz sdf -k models/gimbals/two_axis_pod/model.sdf
gz sdf -k models/gimbals/two_axis_pod/tests/world.sdf
```

网络与地面站测试包含 95 项。其中 `ground_station/tests/test_config.py` 验证带注释 INI 的类型读取和对 JSON 配置的拒绝；`test_status_udp.py` 使用本机 UDP 回环验证额外状态输出：配置读取和覆盖、默认关闭、20 Hz 独立发送、原 station 50 Hz 遥测与 ping、字段完整保留、遥测过期暂停、发送异常恢复、关闭额外输出后 station 继续工作；`test_windows.py` 验证 Windows 配置、视频帧处理、D3D11 硬解打开参数以及拒绝软件解码回退。不会向外部 IP 发送测试数据。

`test_lifecycle.py` 检查同仓库/端口/飞机 ID 匹配、远端网关隔离、PID 复用保护，以及真实后台进程忽略 SIGINT 时的有界 SIGTERM 停止。

## 在线验证

```bash
export GZ_PARTITION=two_axis_validation
bash models/gimbals/two_axis_pod/scripts/run_plugin_sim.sh
python3 models/gimbals/two_axis_pod/tests/check_sim.py
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --encoder nvenc
bash models/gimbals/two_axis_pod/ground_station/run.sh --selftest 5 --send-home --send-manual
bash models/gimbals/two_axis_pod/ground_station/run.sh --decode-test 5 --decoder nvdec --snapshot runs/two_axis_pod/rtsp.png
bash models/gimbals/two_axis_pod/scripts/stop_bridge.sh
bash models/gimbals/two_axis_pod/scripts/stop_plugin_sim.sh
```

`check_sim.py` 依赖 gz Python 绑定与 Pillow，会运动两轴到四个限位、从限位回零、验证朝天退化处机械偏航、单轴复位、无续发超时停车、拒绝滚转、CSRT 跟踪与两路渲染。完成后两轴回零并停止。结果保存到 `runs/two_axis_pod/validation.json`，零位帧为 `camera_zero.png` 与 `overview_zero.png`。测试使用独立分区，不要同时用地面站或其他客户端下发控制命令。

RTSP 解码和地面站自检需具备 OpenCV Python 等依赖；`run_bridge.sh` 和地面站 `run.sh` 会自动配置共用的 `.venv-linux`，也可用 `GIMBAL_PYTHON` 指定已配置的独立环境。`--selftest` 会实际运动模型；应在关节在线检查完成后执行。测试目录中的离线网络/UI测试不启动图形窗口。

完整400块地理背景中的动态靶标与实体遮挡验收见[地理部件测试场景](../../../../scenarios/geographic_two_axis_pod/README.md)，使用同一两轴模型与协议。

目标检测/跟踪和网关环境测试使用带 OpenCV 的两轴环境，覆盖 YOLO 输出解析、连续 5 帧确认、误检过滤、ByteTrack 两阶段关联、稳定 ID、Hungarian 匹配、中心最近选择、RGB 红框、模型路径校验和 OpenCV DNN 实际加载归档 ONNX：

```bash
.venv-linux/bin/python -m unittest \
  models/gimbals/two_axis_pod/controller/service/tests/test_vision.py \
  models/gimbals/two_axis_pod/controller/service/tests/test_bridge_env.py
```

多机视频验证（无需 Gazebo 或 PX4，需要 NVIDIA NVENC/NVDEC、GStreamer 与 GstRtspServer）：

```bash
.venv-linux/bin/python \
  models/gimbals/two_axis_pod/controller/service/tests/test_rtsp_multi.py
```

测试使用临时 RTSP 端口，启动独立编码进程验证同端口不同 ID 后缀的视频隔离、重复路径拒绝、单路退出、租约过期和中继重启恢复。测试中继在全部视频退出后空闲约 30 秒自动退出。
