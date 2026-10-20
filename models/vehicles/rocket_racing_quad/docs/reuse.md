# 公开模型复用评估

本模型采用自建参数化网格与SDF，当前原生模型的机身轴向长度为0.590 m、对角电机轴距约0.590010 m、桨径0.254 m，机头朝上起飞，当前7.70 kg原生配置已通过30 s悬停、200 km/h连续30 s巡航、300 km/h连续10 s冲刺及正常降落停机，真值结果与配置指纹见[验证说明](validation.md)。以下评估提供可追溯的候选依据，范围为三个公开仓库中的代表模型，不代表穷尽所有公开资源。

来源核验日期：2026-10-13。更新日期取浏览到的默认分支提交页首条记录；网页可能有缓存，因此它表示可核实的提交日期，不保证是实时最新提交。模型尺寸按链接中的源码字段读取或计算，不以名称推断。以下来源未提供本项目相同条件下的300 km/h连续10 s验收证据，不能据此推断候选一定达不到该速度。

| 仓库 / 代表模型 | 许可证依据 | 可核实的更新日期 | 布局与尺寸差距 | 当前采用方式 |
| --- | --- | --- | --- | --- |
| [PX4/PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) / x500_base | 仓库[LICENSE](https://github.com/PX4/PX4-gazebo-models/blob/main/LICENSE)：BSD-3-Clause | 默认分支提交页可见 [2026-08-09 / 5577035](https://github.com/PX4/PX4-gazebo-models/commits/main/) | 常规平展四旋翼；电机x/y为±0.174 m，计算对角轴距约0.492 m，小于当前约0.590010 m；不是0.590 m轴向火箭机身 | 参考当前Gazebo模型组织、传感器和接口约定；不直接复制资产 |
| [PX4/PX4-SITL_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic) / iris | [package.xml](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/package.xml) 声明BSD，未据此推断所有独立资产的许可证细则 | 默认分支提交页可见 [2026-05-12 / 00ac441](https://github.com/PX4/PX4-SITL_gazebo-classic/commits/main/) | 常规四旋翼；机身碰撞包络0.47×0.47×0.11 m，桨盘碰撞半径0.128 m；非轴向火箭外形，且使用Gazebo Classic插件接口 | 仅对照转子与动力模型字段；不直接复制资产或Classic插件 |
| [ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator) / Hummingbird | [描述包package.xml](https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_description/package.xml) 声明ASL 2.0；[模型文件头](https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_description/urdf/hummingbird.xacro) 明确Apache License 2.0 | 默认分支提交页可见 [2021-07-06 / cd813b7](https://github.com/ethz-asl/rotors_simulator/commits/master/) | “+”型四旋翼，臂长0.17 m、相对电机间距0.34 m、桨半径0.10 m；轴距和桨径均不同，使用ROS/xacro及Gazebo插件体系 | 仅对照多旋翼描述与动力模型概念；不直接复制网格、代码或配置 |

## 可核对的几何与接口

- x500_base的[原生SDF](https://github.com/PX4/PX4-gazebo-models/blob/main/models/x500_base/model.sdf) 给出电机位置、竖直桨轴和传感器；对角轴距由 `2×√(0.174²+0.174²)` 得到。它适合作为PX4与当前Gazebo集成的接口对照，现有外形与尺寸仍需重建。
- iris的[SDF模板](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/models/iris/iris.sdf.jinja) 给出机身碰撞包络、桨盘半径及Classic电机插件；例如 `rotor_0` 位置为 `(0.13,-0.22,0.023)` m、`rotor_1` 为 `(-0.13,0.20,0.023)` m。即使部分桨径接近，机体布局、尺寸及仿真接口也不满足直接替换条件。
- Hummingbird的[xacro模型](https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_description/urdf/hummingbird.xacro) 明确臂长、桨半径及四个轴向电机位置；0.34 m相对电机间距由两倍臂长得到。其[仓库说明](https://github.com/ethz-asl/rotors_simulator) 提供多旋翼、传感器及控制示例，未给出本项目要求的高速验收结果。

## 当前建模选择

这些代表模型可用于接口与建模概念对照，但均需重新处理火箭机身、机头沿推力轴线的外形、目标尺寸、质量/惯量、动力/气动参数及高速飞行验证。本项目采用自建网格及直接维护的原生SDF/PX4配置，不导入上述仓库的网格、纹理、模型文件或插件源码；运行时使用已安装Gazebo提供的系统插件。参考模型的更新记录或可起飞状态不作为本模型高速性能的证明，本模型性能以独立验收结果为准。
