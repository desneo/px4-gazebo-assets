# 中科 D80 地面站

在地面站目录可直接连接已运行的中科设备或仿真网关：

```bash
./run.sh
```

`./run.sh` 使用本目录默认 `config.ini`，默认连接 02 场景第一架飞机（UDP `8880`、RTSP `8566/zhongke_d80_1`），也可用 `./run.sh --station-config /绝对路径/config.ini` 指定配置。独立启动不需要仿真 `session.ini`，不会启动或接管仿真服务。

场景可直接通过命令行指定每架飞机的连接，不生成地面站INI或读取会话清单：

```bash
../.venv-zhongke-d80/bin/python ground_station/ground_station.py \
  --udp-host 127.0.0.1 --udp-port 8880 \
  --rtsp-url rtsp://127.0.0.1:8566/zhongke_d80_1 --window-title '中科 D80 1'
```

上述命令在组件目录执行；专属虚拟环境位于组件上一级。`--udp-host`、`--udp-port`、`--rtsp-url` 和 `--window-title` 优先于地面站INI及显式会话的对应值，覆盖后统一校验。直接执行 Python 并使用这些选项时不创建会话清单；未显式指定 `--station-config` 时，JSON目的地址的界面编辑只在本进程生效，不改写默认 `config.ini`。`run.sh` 和组件演示原有会话管理接口继续可用。

`config.ini` 的 `[connection]` 配置导引头IP、UDP端口、本地接收地址/端口、RTSP地址及性能INI路径。默认控制发往 `127.0.0.1:8880`，32字节遥测从同一设备端口返回地面站收发共用的socket。`local_port=0` 表示系统分配临时接收端口，启动日志输出实际端口；指定1～65535可固定本地端口。默认视频地址为 `rtsp://127.0.0.1:8566/zhongke_d80_1`。`[forward] port=9000` 是JSON外发目的端口。

无参数 `./stop.sh` 停止本中科组件登记的所有活跃地面站，包含独立配置和显式会话启动的客户端。停止前同时核对PID、启动时刻及实际中科程序路径，仅向地面站PID发送信号；Gazebo、PX4、网关和其他模型的地面站继续运行。需要只停止一个客户端时，使用 `./stop.sh --station-config /绝对路径/config.ini` 或 `./stop.sh --session 会话名`。独立客户端的进程身份和故障日志保存在按配置绝对路径生成的 `runs/station_<hash>/`。

需要从本机仿真清单获取实际连接和只读诊断时，显式执行 `./run.sh --session demo`；此模式的性能路径、远端UDP和RTSP默认使用会话清单；显式连接CLI覆盖对应端点。本地接收地址/端口及窗口、发送偏好仍使用地面站配置。`./stop.sh --session demo` 只停止该清单登记且身份匹配的地面站进程。
启动器使用模型上一级的 `.venv-zhongke-d80`，可用 `ZHONGKE_D80_PYTHON` 指定同等依赖的解释器。

双击视频发送一次指点帧；方向键和摇杆手动转动，释放即发送零速；`R` 回零、`O` 自动锁定、空格停止、`Esc` 退出。
底部控制按钮为“双击选取”“自动锁定”“停止”，手动转动直接使用方向键和摇杆。“停止”取消跟踪并发送零速，中断手动或回零运动；保留视频、遥测和JSON转发，不关闭窗口、不回零。
单轴回零仅在收到新鲜二进制姿态时可用，会保留另一轴当前实测角。
“已发送”表示命令交给本机 UDP socket，执行状态依赖后续遥测和只读诊断。

`config.ini` 保存连接、窗口偏好及JSON接收地址。性能参数由 `connection.parameters_file` 指向，默认 `../parameters.ini`；相对路径以该 `config.ini` 所在目录解析，不复制第二套机械和光学参数。
“发送中科报文”将实际接收并校验的 32 字节遥测转换为 15 字段 JSON；每次启动默认关闭。
编辑地址先停发并清队列，Enter / Esc 只结束编辑，点击发送才恢复。
目标名称、状态原因和限位来自显式会话模式的Gazebo只读诊断；独立模式默认不订阅本机诊断，目标名和原因显示未知。诊断缺失或过期不影响标准二进制测量，也不作为外发测量来源。

支持使用专属解释器直接执行 `ground_station/ground_station.py --session demo --selftest --duration 8 --snapshot /tmp/zhongke-d80.png`，无窗口检查实际 UDP 和 NVDEC 视频并保存画面。
`--decode-test` 只检查 RTSP 硬解，不建立控制租约；不支持软件解码回退。
