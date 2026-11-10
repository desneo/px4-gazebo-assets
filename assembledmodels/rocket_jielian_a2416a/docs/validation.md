# 验证状态

## 自动验证

当前静态和动态生成测试覆盖：

- 基础火箭与 A2416A 文件在生成前后指纹一致。
- 静态装配无碰撞、关节或插件，整体和近景世界包含无碰撞定位标志。
- 导引头安装面 `z=0.320 m`、总长 0.590 m、导引头质量 0.05 kg、组合质量 4.81 kg。
- 动态装配只有固定的 `seeker_mount_joint` 与导引头关联，图像和内部控制话题使用配置前缀。
- airframe 4026、悬停推力和 PX4 名义重心参考由当前资产重新派生。
- 动态输出的所有模型网格均在输出目录闭包内。
- 动态验收世界含 500 m 正前方 `target_aircraft` 真值夹具；飞行入口会开启识别与自动锁定，并独立采集 50 Hz 捷联遥测。

执行入口：

```bash
python3 -m unittest tests/test_assembly.py -v
```

## 环境相关验证

H.264 视频要求可用的 NVIDIA 驱动、NVENC、NVDEC、GStreamer `nvcodec` 和 `shm` 插件，不允许软件编解码回退。当前主机预检可加载 `nvcudah264enc`、`nvh264dec`、`shmsink`和`shmsrc`；缺少这些条件时视频预检明确失败。

PX4 飞行验收采用 30 s 悬停、200 km/h 持续 30 s、300 km/h 持续 10 s、降落停机，并同时检查真值目标跟踪和遥测链路。执行入口：

```bash
bash assembledmodels/rocket_jielian_a2416a/tests/run_flight_check.sh headless
```

`tools/validate_flight_tracking.py` 要求飞行报告为通过状态、采集频率在 40～60 Hz、`measurement_seq` 递增、至少出现一次真实目标 `TRACKING`，并验证平台框架角、平台陀螺角速度、LOS角速度和导引头姿态始终为零。

当前动态资产已在 PX4 SITL 和 Gazebo 中完成联合验收，结果如下：

- 30 s 悬停通过：最大水平漂移 0.0894 m，最大高度误差 0.2797 m。
- 200 km/h 连续 30.004 s 通过：峰值 224.271 km/h，最大高度误差 0.5441 m。
- 300 km/h 连续 10.000 s 通过：峰值 319.199 km/h，最大高度误差 0.2221 m。
- 降落与解锁通过。
- 共保存 8,530 帧内部捷联遥测，实测 50.0001 Hz，`measurement_seq` 从 11 递增到 8,540；其中 2,783 帧为真实目标 `TRACKING`。
- 平台框架角、平台陀螺角速度、LOS角速度和导引头姿态在全部采样中为零。

结构化摘要保存在 `docs/flight-validation.json`。二进制 UDP 遥测的 100 Hz、独立帧时间戳和序号由网关回环集成测试验证；H.264 视频使用 NV12、High profile、1920×1080@25 Hz、默认2 Mbps和NVIDIA硬件编解码。02场景实测编码25.0 fps且无丢帧，生产接收器稳态25 fps，断线重连后继续保持25 fps。
