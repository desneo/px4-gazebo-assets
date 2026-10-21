# 火箭单机与组合体最高稳定速度对照

使用 `worlds/geo_32_217857_119_105878/world.sdf` 的完整地理环境，顺序试飞基础火箭穿越机与 `assembledmodels/rocket_two_axis_pod/` 的590 mm动态组合体。两次运行均无风、相对起飞高度80 m、沿北向加速，并使用独立Gazebo分区、全新PX4参数存储。结果来自实际PX4飞行和Gazebo真值，不使用预设轨迹。

单机复制当前原生SDF、网格和PX4配置；组合生成器读取相同基础资产并加入吊舱。当前布局均高590 mm，单机4.76 kg、固定名义重心 `(0,0,0.030) m`，组合5.00 kg、零关节名义重心 `(-0.000176261396,-0.000000003426,0.044398504845) m` FLU。组合 CA/EKF 已派生，组件自验证工具使用同一固定名义重心测量；基础单机与当前新 CAD 吊舱组合均已通过独立完整组件飞行。组合完成 30 s 悬停、200 km/h 连续 30.016 s 巡航、300 km/h 连续 10.016 s 冲刺及落地停机。本速度场景尚未复测，不沿用不同配置的归档结论。

## 启动

在仓库根目录执行：

```sh
# 单机完成并清理后，自动试飞组合体；两个结果都保存后生成comparison.json
python3 scenarios/rocket_speed_comparison/run_comparison.py \
  --run-dir runs/rocket_speed_comparison/my-comparison

# 也可以分别运行；同一时间只运行一架
bash scenarios/rocket_speed_comparison/run.sh --variant standalone \
  --run-dir runs/rocket_speed_comparison/my-standalone --gui
bash scenarios/rocket_speed_comparison/run.sh --variant assembled \
  --run-dir runs/rocket_speed_comparison/my-assembled --gui

# 请求降落并停止指定的一次运行
bash scenarios/rocket_speed_comparison/stop.sh runs/rocket_speed_comparison/my-assembled
```

需要已构建的相邻 `../PX4-Autopilot`、Gazebo Sim 8、Python的pymavlink/numpy/Gazebo传输绑定，以及已构建的两轴控制插件。可用 `PX4_ROOT`、`FLIGHT_PYTHON` 覆盖路径。端口占用检查和文件锁防止本场景重复启动PX4 instance0；其他PX4场景也应先停止。

`--config PATH` 读取自定义场景JSON，实际设置保存到运行目录的 `config.json`。`--target-speed` 只改变飞行指令，不能超过模型当前 `MPC_XY_VEL_MAX` 和场景速度上限。运行目录必须为空，防止混用旧rootfs或证据。飞行正常完成后自动减速、LAND并确认落地停机；中断时先请求有界降落，再按PID、启动时间和独立进程组清理所属服务。

## 测量和结论

默认目标95 m/s（342 km/h），是当前PX4控制上限。25仿真秒线性提高速度指令，之后最多观察25.3仿真秒，寻找最高的连续10秒稳定窗口；稳定窗口必须同时满足：

- 独立Gazebo位姿差分的水平速度标准差不超过0.5 m/s，最大最小差不超过2 m/s。
- 速度线性拟合加速度和首尾平均加速度绝对值都不超过0.2 m/s²。
- 每个样本相对目标高度误差不超过2 m，相邻真值间隔不超过0.15仿真秒。

瞬态峰值另列，不用它替代稳定速度。目标达到容差为1 m/s或目标的1%，取较大值。以组合体稳定速度不低于单机稳定速度减1 m/s作为当前对照的数值通过条件，同时报告实际差值。达到控制上限时标记 `control_limited`，不能称为动力极限。无连续有效窗口、未正常降落、角色或场景不一致时不能给出通过结论；未达到目标的较低稳定窗口只报告观测值。

4500 m航程边界以内，根据实际速度的 `v²/(2a)` 加500 m余量提前减速，避免高速冲出地形范围；减速率配置为4 m/s²。提前结束原因和最高稳定窗口分别记录。时间判定使用仿真时间，墙钟只用于通信与服务超时。

原生电机映射为50 rad/s解锁怠速、未解锁0、最大1570 rad/s，推力系数8.9119×10⁻⁵ N·s²/rad²。单机动力舱中心z=0、桨盘z=-0.065 m，CA旋翼PZ=+0.095 m FRD；组合按名义重心派生PZ=+0.109398504845 m，IMU/GPS X=+0.000176261396 m、Y≈−3.426×10⁻⁹ m、Z=+0.029398504845 m FRD。基础悬停参数0.04206385775789426、组合0.04452973995335638，所用PX4内部下限0.05。组件自验证的固定重心测量不代表云台运动时瞬时重心，也不替代本场景专用稳定速度比较，见[基础结果](../../models/vehicles/rocket_racing_quad/docs/validation.md)与[组合状态](../../assembledmodels/rocket_two_axis_pod/docs/validation.md)。

**气动范围：** 当前组合体继承基础火箭模型的阻力系数0.73和参考面积0.0177 m²，转子阻力参数也相同。尚未独立建模顶部吊舱改变后的额外气动阻力。比较仅适用于当前仿真配置，不能据此推断真实硬件装上吊舱后没有速度损失。此类模型调整需另行讨论确认。

## 观察入口

`--gui` 打开起飞点整体视角；`--close` 打开机体与吊舱近景。起飞点旁的 `SPEED_TEST_TAKEOFF_MARKER` 是颜色鲜明的无碰撞定位标志，位于原点西南约2 m，箭头指向飞机。可在Gazebo实体树搜索飞机名 `rocket_racing_quad` 或 `rocket_two_axis_pod` 并聚焦；高速飞行时使用实体跟随观察。

浏览相机近裁剪距离默认0.005 m，灵敏度0.1，经 `/gui/camera/view_control/sensitivity` 在GUI启动后设置；`view_near_clip_m` 和 `view_sensitivity` 可调整。近景缩放时将鼠标放在机体实体表面，右键缓慢拖动微调。浏览相机设置独立于吊舱视频相机；视频相机参数保留在基础两轴组件内。

## 证据与复现检查

每次试飞保存 `world.sdf`、实际运行模型、`asset_manifest.json`、`trial.json`、`trial.samples.jsonl` 和PX4 ULog。单机与组合体均保存实际运行参数和源文件指纹。清单记录源与运行资产SHA256和共同环境指纹，比较器检查实际机型、环境、控制上限和场景飞行设置一致性；数值通过不表示真实硬件性能相同，解释对照结果前须核对模型配置与简化气动范围。

```sh
python3 -m unittest discover -s scenarios/rocket_speed_comparison -p 'test_*.py' -v
python3 scenarios/rocket_speed_comparison/report_comparison.py \
  --standalone runs/rocket_speed_comparison/my-comparison/standalone \
  --assembled runs/rocket_speed_comparison/my-comparison/assembled \
  --output runs/rocket_speed_comparison/my-comparison/comparison.json
```

报告命令增加 `--publish` 可在源资产指纹仍匹配时，将实际飞行报告、逐步真值记录和清单压缩保存到 `tests/results/`，生成 `docs/validation.json` 和速度/高度曲线。具体实测结果见[验证说明](docs/validation.md)。
