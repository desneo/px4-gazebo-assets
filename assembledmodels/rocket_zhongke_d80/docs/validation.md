# 中科D80火箭组合验证

验证对象为 `rocket_zhongke_d80`，安装底环水平、机械零位朝机体+Z，名义质量4.955 kg、PX4 airframe 4025。基础火箭机、中科组件、原两轴/艾晟组合保持原源文件。

## 装配与旧入口

- 新组合8项离线测试通过：顶部安装、质量与惯量继承、名义重心/控制分配/悬停推力派生、单相机话题、逐机INI和hash、网格闭包、源文件隔离。
- 静态及动态 `gz sdf -k` 通过；原生传感器 `gz_frame_id` 作为Gazebo扩展保留。
- 原两轴动态生成11项、艾晟动态生成5项通过。
- 02原世界生成14项、原验收数学20项通过。原 `start.sh` 从全局进程检查之后、`stop.sh` 从原运行目录定义之后的主体代码保持逐字相同；新中科在前面显式分派。
- 新中科启动器20项回归通过：逐机INI/hash、独立目录、端口占用、PID身份、重复启动、日志就绪、默认解释器及依赖前检；GUI的CollisionGeometry与TargetControl两项CTest通过。
- `TargetControl` 新实体前缀支持独立编译通过，旧配置默认前缀仍为 `rocket_two_axis_pod_`。

## 真实02世界锁定

实际入口为 `/home/aeroarx/zsh/PX4-Autopilot/scenarios/02_rocket_two_axis_pod/start.sh --aircraft-model rocket_zhongke_d80`。验证运行目录 `/tmp/zhongke-scenario02-lock-validation`，分区 `zhongke_scenario02_lock_validation`；使用完整 `geo_32_217857_119_105878` 地理世界。PX4实际连接 `rocket_zhongke_d80_1`、完成初始化并保持未解锁待命。

单机使用UDP8880、RTSP8566和 `/uav_1/pod/*`。逐机INI中的实体名、config_hash、目标名单与插件、网关、清单一致。控制为中科16字节二进制，遥测为32字节二进制；目标身份另由同分区只读插件/网关诊断确认，未将诊断当作外发测量。

`tools/check_scenario02_lock.py` 的7项实际检查通过：

1. 同名同hash插件、实际图像及新鲜二进制遥测到达；机械角与插件测量一致。
2. 内轴实际转到−70°前向观察角。
3. 0F自动锁定150 m处的已注册视觉夹具。
4. 夹具移动后保持同一目标、实际关节角改变，角偏差收敛到0.35°以内。
5. 0D指点锁定同一真实实体，二进制有效跟踪位为真。
6. 24手动接管撤销目标绑定与有效跟踪位。
7. 2B回零，两轴回到0.1°以内并进入IDLE。

采样642个实际遥测帧，整体约59.56 Hz；最大实际轴速20.00°/s，符合默认限制。工具只创建并移动自己的无碰撞 `xiaobianfu` 视觉夹具，用唯一link标记确认所有权后删除；没有移动载机、原场景实体或用户已有目标。实际1920×1080原始相机图像已保存并复核，画面中的目标与投影方向一致。

地面站按外部独立连接INI自检通过：实际收到401个二进制遥测包，显示接收60.0 Hz；通过NVDEC解码178帧1920×1080实际RTSP视频。独立模式不显示仿真目标名称与原因，标准二进制测量和控制不受影响；目标身份验证使用场景只读诊断。

## 生命周期与隔离

重复启动正在运行的中科目录返回失败，原清单ready状态与所有登记PID/start_ticks保持。显式中科停止入口随后停止其全部4个登记进程：Gazebo、PX4、网关、中继；原有手动Gazebo、网关、中继及独立地面站仍保持原PID/start_ticks。不额外设置Python环境变量的正式入口启动/停止烟测也已通过。420个基础/原组合源文件的前后SHA256一致。用户原有未提交修改保留。

## 证据与复测

本机原始证据位于 `runtime/scenario02_lock/`，包括 `result.json`、`samples.jsonl.gz`、`auto_lock.png`、`ground_station.png`、逐机INI、运行清单、PX4/网关日志及生命周期/源隔离结果。`runtime/` 为可重新生成的本机运行产物，不提交入库。

已有独立中科02运行副本时，在本装配目录执行（参数需取该副本的清单）：

```bash
../../models/gimbals/.venv-zhongke-d80/bin/python tools/check_scenario02_lock.py \
  --config /tmp/zhongke-scenario02-lock-validation/config/aircraft_1.ini \
  --partition zhongke_scenario02_lock_validation \
  --output runtime/scenario02_lock
```

复测前关闭连接此机的地面站并等待2秒租约释放；已有 `xiaobianfu` 时工具拒绝移动或删除它。默认300 m只限制发现与重新选取，靶机界面的默认2000 m不适合初锁；目标还须处在真实视场和机械可达范围内。

本结果覆盖待命载机上的云台锁定、真实关节、相机、UDP和硬件视频链路。尚未覆盖载机飞行中的目标跟踪、飞行稳定性、人工作业界面的完整操作，以及装配极限角几何遮挡/穿插；这些不由本次地面锁定结果推断通过。
