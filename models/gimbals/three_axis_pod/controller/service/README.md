# 吊舱网络网关

`gimbal_bridge.py` 订阅插件 JSON 遥测、回执与相机图像，转发 UDP 指令并提供 H.264 RTSP。依赖模块保存在本目录：`gz_source.py/video.py/stamp.py/attitude.py/kinematics.py`。

在仓库根目录运行 `bash models/gimbals/three_axis_pod/scripts/run_bridge.sh --encoder x264`，先启动同一 GZ_PARTITION 下的仿真。默认 UDP 8765、RTSP 8554、挂载点 `/gimbal`；用 `python3 models/gimbals/three_axis_pod/controller/service/gimbal_bridge.py --help` 查看参数。

接口、去重窗口、回执关联与会话限制见 [系统说明](../../docs/system.md)。
