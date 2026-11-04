# 中科 D80 当前验证状态

验证环境为本机 1.56、Ubuntu 24.04、Gazebo Sim 8、NVIDIA RTX 4060。运行对象是独立云台演示，不启动 PX4 或载机。默认配置指纹为 `5aad5d46c82ac53651225b269904ed132d05b9407bd9ab6a5012056a7efbf5f3`。

## 已通过的实际检查

地面站60项回归通过，其中无显示环境跳过的真实窗口项已在桌面DISPLAY=:1单独执行通过；组件生命周期4项回归通过。对应测试日志、源码指纹与实际停止隔离结果保存在 `runs/station_controls/`。组件Python43项、网关服务33项和C++ CTest6项的验收结果与源码指纹见 `runs/acceptance/offline.json`。

| 检查 | 实际结果 | 证据入口 |
| --- | --- | --- |
| 默认物理运动 | 人工四向、停止、连续回零及四个端点通过；原始速度峰值约20°/s，独立位置差分峰值20.000000000012°/s，在数值容差0.0001°/s内 | `runs/validation/physics/result.json`、`samples.csv` |
| CameraInfo和真实渲染 | 独立fx=15628.7130636179、fy=15659.4763500848像素；50mm视场7.03°×3.95°；实际标靶像素与独立投影误差小于1像素 | `runs/validation/optics/result.json`、`actual_render.png` |
| 运行中的质量与惯性 | 从Gazebo运行服务导出的当前模型总质量0.195kg；三link质量/惯性/质心、关节约束和相机内参均匹配INI | `runs/validation/runtime_config.json`、`runtime_world.sdf` |
| 实际视频与标准遥测 | 8秒自检收到510个二进制包，接收显示60Hz；实际硬解195帧1920×1080视频 | `runs/acceptance/video_selftest.json`、`runs/validation/station.png` |
| UDP转JSON | 10.0149秒实际收到600个二进制包与600个JSON包，59.9109Hz；JSON恰好15字段，时间为发送时Unix毫秒 | `runs/validation/telemetry_validation.json` |
| 视频与JSON并行 | 12秒硬解364帧，JSON约10秒601包，60.0134Hz | `runs/validation/telemetry_with_video.json` |
| 暂停与断源 | 真实Gazebo暂停及本会话网关暂停后，标准遥测和JSON均停止新增；失效标记生效，恢复后继续新样本，停止发送不回播 | `runs/validation/source_pause.json` |
| 配置重启矩阵 | 7组全部通过：10/20/30°/s、0.240kg质量、改行程、1280×720、100mm焦距；质量/惯性取自运行世界服务 | `runs/configuration_matrix/runtime_results.json` |
| 改参实际限速 | 10档位置差分峰值约10°/s，30档约30°/s；有效TRACK的yaw也分别触及10/20/30°/s，全部在硬边界内 | `runs/matrix_rate_10/`、`runs/matrix_rate_30/`、`runs/validation/tracking/` |
| 100mm实际成像 | 固定感光面仍为6.142540310851201×3.448391171758914mm；CameraInfo与真实标靶像素共同验证视场3.5183103922°×1.9755868472° | `runs/matrix_focal_100mm/optics/result.json` |
| 真值跟踪 | 默认17项通过，包括POINT/AUTO、多目标身份保持、299/300/301m、斜向range、已锁超距持续及失锁重选限制、人工接管；不把投影可见性当实体遮挡检测 | `runs/validation/tracking/result.json`、`samples.jsonl` |
| 生命周期 | 重复启动拒绝、地面站单独退出/定向停止、故障启动回收、端口释放和重复停止通过；PID启动身份防误杀及并发清单测试通过 | `runs/lifecycle/` |
| 无参数地面站停止与控制面板 | 无参数stop.sh实际关闭02中科地面站；Gazebo、PX4、网关、中继、靶机、雷达和GUI的7组PID/启动身份保持。重启客户端后实际窗口为双击选取、自动锁定、停止三个底部按钮，二进制接收60.0 Hz，NVDEC视频正常 | `runs/station_controls/result.json`、`stop_isolation.json`、`window.png` |
| 端点零位 | yaw[0,90]、pitch[-90,0]、0.2°缓冲下，两组指定角后HOME均进入IDLE；位置误差小于0.018°，原始/差分速率均不超20°/s | `runs/zero_boundary/zero_boundary/result.json` |
| 真实桌面窗口 | 通过用户run.sh入口连续运行8秒并正常退出；中文标题、鼠标回调和UTF-8焦点识别正常，保留失焦停车；用户Gazebo/网关原PID存活 | `runs/manual/gui_window_result.json`、`gui_wrapper_fixed.png`、`gui_wrapper.log` |
| 地面站独立连接 | `view config.ini`真实GUI退出码0；独立UDP测试端收到22个有效控制帧并发出207个遥测测试帧，收发共用配置的固定本地端口；不依赖世界清单，退出和定向重复停止通过。此项UDP是协议测试报文，不代替物理测量验收 | `runs/standalone_gui_check/result.json`、`launch.log`、`station.png` |
| 已有资产隔离 | 394个参考文件与开始时内容基线一致，保留原two_axis地面站的既有配置修改 | `runs/baseline/reference_hashes.json`、`tests/reference_baseline.py` |

`runs/`属于本地运行产物，不提交。缺少本地证据时可按组件README的工具重新执行，不能把缺失记录视为通过。

## 尚未完成的检查

- 真实桌面GUI人工操作，以及新增行程端点和组合角的外部穿插/遮挡检查。

物理工具保存了四个端点的实际视频图像。它们用于检查视频视线效果，不能单独证明外部网格无穿插；后者仍需要Gazebo近景人工审查。无窗口自检、真实窗口自动运行和截图均不等同于完整人工交互验收。

## 复验命令

```bash
bash scripts/run_demo.sh --headless --session validation
../.venv-zhongke-d80/bin/python tests/check_sim.py --session validation
../.venv-zhongke-d80/bin/python tests/check_optics.py --session validation
../.venv-zhongke-d80/bin/python tests/check_tracking.py --session validation
../.venv-zhongke-d80/bin/python tests/check_telemetry.py --session validation --duration 10
../.venv-zhongke-d80/bin/python tools/check_source_pause.py --session validation
bash scripts/stop_demo.sh --session validation
```

运行控制测试时只保留一个中科UDP客户端。自动工具退出后，下一个客户端等待旧租约到期；视频专用`--decode-test`不建立控制租约。人工检查按[安装说明](mounting.md)定位部件、设置浏览相机并依次检查四向、松手、失焦、回零、双击、自动、接管和发送编辑/停止。
