# 资产与质量模型

`cad/` 和 `meshes/` 是 `../two_axis_pod` 的独立副本，保留复制资产字节；运行与构建不导入原组件 Python 文件，也不编译其源码。整机前视通过模型姿态实现，不修改网格顶点或内部关节轴向。

| 文件 | 用途 |
| --- | --- |
| `cad/motion_head.step`、`cad/motion_head.step.json` | 源 CAD 和伴随元数据 |
| `cad/properties.json` | 零件归属、体积、质心、单位质量惯性、网格 SHA256 |
| `cad/source.ini` | 复制来源、质量分配假设与资产 SHA256 清单 |
| `meshes/*.dae` | 外形显示与碰撞几何，SDF 使用本组件的 `model://zhongke-d80/` URI |

刚体分为固定底座 `mount_link`、外轴部件 `yaw_arm_link` 和相机运动部件 `camera_link`。实际总质量由 `model.mass_kg` 指定，默认 0.195 kg；按 CAD 实体体积分配质量，以组内 CAD 质心和单位质量惯性计算刚体惯性。该分配采用均匀等效密度假设，不表示实物各材料的真实密度。

复制外形约 φ84 mm×91 mm，包含底座、外壳、支架、俯仰球壳、镜片与安装结构。`product.diameter_limit_mm=49`、`height_limit_mm=80` 是产品表资料，不重新缩放资产；修改质量也不改变外形尺寸。

控制器所需的共享基础源码独立放在 `controller/vendor/gimbal_core/`，中科控制器使用本目录头文件和源码。Python 配置、网关、地面站以及视频辅助文件也保存在本组件内。Gazebo、OpenCV 和 GStreamer 是系统运行依赖，不由引用其他云台目录提供。

模型生成器核对 CAD/网格的既定 SHA256，不匹配时拒绝生成。参考资产、原模型、已有装配和场景以及 PX4 消息文件的只读基线检查入口为 `tests/reference_baseline.py verify`；现有基线保留用户原有工作区状态，不以当前文件重新覆盖基线。
`tests/test_isolation.py` 是可在新检出目录运行的生成隔离回归，比较生成前后的参考文件内容及工作区状态；它不读取或更新本机验收基线。原始验收证据仍由上述只读基线工具独立核验。

复制的 CAD、网格及其来源目录未发现独立许可证文件。`cad/source.ini` 只记录来源与校验，不新增、推定或替代原资产的许可条款。
