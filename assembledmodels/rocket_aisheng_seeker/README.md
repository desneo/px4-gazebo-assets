# 火箭穿越机与顶部艾晟导引头（静态预览）

`rocket_aisheng_seeker` 是独立静态装配模型：590 mm 火箭穿越机顶部安装 ASD-2-075-DD0 两轴导引头。导引头以**底座平面朝下**坐在机头顶部替换切口，底环水平，零位光轴朝上；前视靠 `pitch=-80°`，约高于水平面 10°。本模型只用于结构评审，不包含 PX4、飞行动力学、活动关节、相机传感器或控制插件。

## 当前状态

- 由 `models/vehicles/rocket_racing_quad` 与 `models/gimbals/aisheng-ASD-2-075-DD0` 的当前原生视觉资产组合。
- 原 `rocket_racing_quad`、`rocket_two_axis_pod` 和导引头基础模型均未修改；本装配独立生成。
- 火箭总长保持 590 mm；导引头底环安装面 `z=289 mm`，导引头高 91 mm，组合最高点 `z=380 mm`。
- 导引头保留原镜头和下方第二镜圈；底环 Ø84 mm、球头 Ø48 mm、机身最大 Ø90 mm。
- 静态 `model.sdf` 含 26 个 visual、无碰撞体、无关节、无插件；机身裁切网格保存在 `meshes/fuselage_cut.stl`。
- 顶部为底座平面安装，因此零位视线朝上，前视时 yaw 水平效果受限；这是按现有 `rocket_two_axis_pod` 装配方式确认的静态结构，不是独立前视 +90° 安装。

## 预览图

![整机静态预览](docs/preview.png)

![导引头近景](docs/closeup.png)

![整机与近景对比](docs/compare.png)

导引头旁橙色箭头为 `ROCKET_AISHENG_SEEKER_LOCATION_MARKER`，无碰撞体。

## 运行

在仓库根目录执行：

```bash
# 整机
sh assembledmodels/rocket_aisheng_seeker/tests/run.sh overall

# 导引头近景
sh assembledmodels/rocket_aisheng_seeker/tests/run.sh close

# 指定 yaw/pitch 姿态（仅静态视觉，不加载控制）
sh assembledmodels/rocket_aisheng_seeker/tests/run.sh close 45 -45
```

启动器每次生成独立 `/tmp/rocket-aisheng-seeker.*` 运行副本和 Gazebo 分区。整机实体 `ROCKET_AISHENG_SEEKER` 位于 `(0,0,0.225) m`，尾端离地 15 mm；安装面世界 `z=0.514 m`、最高点 `z=0.605 m`。浏览相机近裁剪 0.005 m，鼠标灵敏度 0.1；近景缩放请把鼠标放在实体表面，右键缓慢拖动微调。

## 重新生成与检查

```bash
python3 assembledmodels/rocket_aisheng_seeker/tools/generate_model.py
python3 -m unittest discover -s assembledmodels/rocket_aisheng_seeker/tests -p 'test_*.py'
```

生成器读取火箭和导引头的当前 SDF/网格，裁切机身顶部并放置导引头；不要手工修改 `model.sdf`、`model.config`、`tests/world.sdf`、`tests/closeup.sdf` 或 `meshes/fuselage_cut.stl`。

## 静态预览要确认的内容

- 导引头底环是否与机头顶部切口贴合，整体高度和肩部过渡是否可接受；
- 两个镜圈、球头和罩壳在机头位置是否有干涉或遮挡；
- 导引头外形与火箭比例是否可接受；
- 底座朝下、零位朝上的安装方式是否符合后续使用预期。

本静态预览确认后再进行动态装配、PX4 飞行场景和 02 场景启动参数开发。02 场景启动文件将增加组合火箭型号参数，默认保持现有 `rocket_two_axis_pod`，不影响已有场景功能；该改动尚未实施。

## 文件

- `model.sdf`、`model.config`：生成的静态装配模型。
- `meshes/fuselage_cut.stl`：从火箭基础机身实际网格裁切得到的机身段。
- `parameters.json`：总长、肩部、初始 yaw/pitch 和浏览相机参数。
- `tools/generate_model.py`：静态装配生成器。
- `tools/pod_kinematics.py`：从导引头 SDF 读取两轴运动链，用于静态姿态和检查。
- `tools/render_views.py`：离线评审图工具（需要 Matplotlib；当前预览图由 Gazebo overview 相机生成）。
- `tests/run.sh`：Gazebo GUI 静态预览入口。
- `tests/test_appearance.py`：原生几何继承、裁切闭合、安装面、两轴机械角和再生成一致性检查。
