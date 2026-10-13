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

## 在线验证

```bash
export GZ_PARTITION=two_axis_validation
bash models/gimbals/two_axis_pod/scripts/run_plugin_sim.sh
python3 models/gimbals/two_axis_pod/tests/check_sim.py
bash models/gimbals/two_axis_pod/scripts/run_bridge.sh --encoder x264
bash models/gimbals/two_axis_pod/ground_station/run.sh --selftest 5 --send-home --send-manual
bash models/gimbals/two_axis_pod/ground_station/run.sh --decode-test 5 --decoder avdec --snapshot runs/two_axis_pod/rtsp.png
bash models/gimbals/two_axis_pod/scripts/stop_bridge.sh
bash models/gimbals/two_axis_pod/scripts/stop_plugin_sim.sh
```

`check_sim.py` 依赖 gz Python 绑定与 Pillow，会运动两轴到四个限位、从限位回零、验证朝天退化处机械偏航、单轴复位、无续发超时停车、拒绝滚转、CSRT 跟踪与两路渲染。完成后两轴回零并停止。结果保存到 `runs/two_axis_pod/validation.json`，零位帧为 `camera_zero.png` 与 `overview_zero.png`。测试使用独立分区，不要同时用地面站或其他客户端下发控制命令。

RTSP 解码和地面站自检需具备 OpenCV Python 等依赖；可用 `GIMBAL_PYTHON` 指定独立环境。`--selftest` 会实际运动模型；应在关节在线检查完成后执行。测试目录中的离线网络/UI测试不启动图形窗口。

完整400块地理背景中的动态靶标与实体遮挡验收见[地理部件测试场景](../../../../scenarios/geographic_two_axis_pod/README.md)，使用同一两轴模型与协议。
