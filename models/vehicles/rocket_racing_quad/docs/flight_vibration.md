# 悬停振动实验配置

`config/flight_hover.json` 为可选的实飞悬停扰动配置。默认原生 `model.sdf`、动力、惯性、传感器和 PX4 参数均不加载此配置；必须由运行副本工具或调用方显式启用。该配置是单份悬停日志的经验扰动模型，不代表机体结构参数的辨识结果，也不等价于全飞行包线的实物标定。

## 激励与坐标

- `force_components` 为作用于整机的低频体轴外力，单位 N；`torque_components` 为低频体轴外力矩，单位 N·m。每项为频率、三轴峰值振幅、三轴初相位。体轴采用 Gazebo FLU（x前、y左、z上），插件按当前姿态转换成世界坐标后作用于 `base_link`。同频不同相位的分量不等于独立随机噪声。
- `gyro_components` 和 `accel_components` 为飞控安装处的局部传感器谐波，单位分别 rad/s 和 m/s²，振幅是峰值而非 RMS。插件保持原 IMU 消息的时间戳、姿态及其他字段，只叠加角速度和比力；这些高频加速度不施加给整机或云台。该分离用于避免将局部机械振动错误地转换为整机大位移。
- 谐波基频为 `fundamental_hz * clamp(mean_rotor_speed / rotor_reference_rad_s, frequency_scale_min, frequency_scale_max)`。四个旋翼关节的平均绝对转速乘 `rotor_velocity_slowdown` 还原电机模型的真实角速度，谐波相位由原 IMU 消息的仿真时间积分。
- 各架飞机的首次 `base_link` 世界高度单独记录为起始高度。只有相对高度大于 `airborne_height_m` 且平均真实旋翼角速度大于 `minimum_rotor_rad_s` 时才激励，以 `ramp_time_s` 平滑增加幅度；落回门限以下或电机停转立即归零。地理场景地形绝对高度不影响门控，但在低于起始地形的航段中，激励可能关闭。

未引入机架柔性网格、真实软安装刚度或从实物测得的转速到振动传递函数。噪声原始标准差保持原生值，叠加项受上述配置控制。PX4 接收消息后仍按自身坐标、积分、滤波链路处理，因此日志振幅不等于配置峰值。

## 运行副本与场景集成

```sh
cmake -S models/vehicles/rocket_racing_quad/controller \
  -B models/vehicles/rocket_racing_quad/controller/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build models/vehicles/rocket_racing_quad/controller/build -j4
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/models/vehicles/rocket_racing_quad/controller/build/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
python3 models/vehicles/rocket_racing_quad/tools/configure_vibration.py \
  --model /tmp/example/models/rocket_racing_quad/model.sdf \
  --world /tmp/example/tests/world.sdf
```

运行副本工具拒绝编辑基础模型目录内的文件，不改变原生入口。CLI 可使用 `--profile PATH` 指定另一份 JSON；若模型实际实体名不同，使用 `--model-name NAME`，该参数只影响插件话题，不修改 SDF 的模型名称。省略 `--world` 时必须提供 `--world-name NAME`。

内嵌模型场景可调用 `tools/configure_vibration.py` 的 `configure_model(model, profile_path, world_name)` 和 `configure_world(world, profile_path)`，须先将模型命名为最终实体名，再配置。工具将原 IMU 改到该架飞机独立的 `_vibration_raw` 话题；插件把叠加后的消息发回 PX4 原 canonical 话题。输入与输出不同，不存在自订阅回环。重复配置同一模型会报错，避免叠加两次。所有原云台、GPS、气压、磁力计与控制接口保留。

`px4_parameters` 仅允许 `IMU_GYRO_CUTOFF` 和 `IMU_DGYRO_CUTOFF`；当前配置分别为25/20Hz，用于对齐实飞控制链滤波。`configure_airframe(path, profile_path)` 或 CLI `--airframe PATH --profile PATH` 将这些默认值追加到运行副本的机体文件，拒绝修改基础机体文件及重复应用。CLI可只传 `--airframe`。该功能不调整PID、推力、几何或其他参数，关闭模式时不调用。插件在输出IMU头部增加 `rocket_flight_vibration=1` 标记，PX4仅针对该标记采用源仿真采样时间；原输入时间戳保持不变。

启用时将机载 IMU 更新率设为 1000 Hz，世界时间步不大于 1 ms，以表示 68/135 Hz 及随转速移动的谐波。世界中已有更小步长会保留；若存在 `real_time_update_rate`，按新步长同步更新。对照测试应明确记录这项采样链路变化；更多飞机和完整相机场景的实时运行速度可能降低。未启用时不加载插件、不重定向话题、不修改世界时间步或传感器更新率。

## 验证

```sh
ctest --test-dir models/vehicles/rocket_racing_quad/controller/build --output-on-failure
python3 -m unittest discover -s models/vehicles/rocket_racing_quad/tests -p test_vibration.py -v
```

信号测试验证已知谐波的均方值、轴隔离、零门控和时间斜坡；Python 测试验证原生资产不变、仅指定运行模型 IMU 与插件变化、多机话题隔离、重复配置拒绝、环境参数保留以及非法配置/混叠频率拒绝。实际飞行比较还需检查真值姿态、位置、IMU 频谱、电机输出、起飞降落和传感器时间戳；只比较 IMU 标准差不能证明运动一致。

## 当前悬停校准范围

当前经验配置在02默认两轴组合机、约3m高度的60秒定点悬停中完成起飞与落地。45秒稳定窗口的滚转/俯仰标准差为0.570°/1.225°，实飞参考为0.682°/1.473°；物理真值为0.564°/1.215°。积分IMU陀螺三轴波动幅度偏差约2–6%，加速度约1–3%。偏航估计与实飞仍不匹配，不能将此配置视为完整三轴或高速动力学标定。完整吊舱UDP接口验收受到当前本机网络环境限制：启用和关闭时均发生大于1472字节的UDP报文丢失，独立网络探测也能复现；视频硬解码正常，未修改网关或网络设置。
