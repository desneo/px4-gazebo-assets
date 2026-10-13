# 三角翼静态美术展示

白色圆润中央外壳与融合三角翼、翼尖竖翼、黑色小窗和纯视觉尾桨。近似展示长度3.3 m、宽度2.5 m，用于 Gazebo 中的外形审阅；网格为艺术近似，不是实物 CAD、制造几何、结构资料或型号性能复现。美术参考见 `docs/art_reference.png`。

模型显式设置 `static=true`，只有视觉网格和材质，无碰撞、惯量、气动、推进、舵机、传感器或 PX4 接口。面板线、黑窗和尾桨均为装饰。同组美术网格已随小摩托整机在 Gazebo 中获得用户确认；独立静态展示不参与飞行验收。

```bash
sh models/props/delta_wing_display/tests/view.sh
python3 models/props/delta_wing_display/tools/generate_display.py
```

在实体树选择 `delta_wing_display`，使用 Camera Tracking 移动/跟随入口近景审阅。初始相机为 `(4.5,-5,4.3) m`，近裁剪0.005 m，参数保存在 `tests/world.sdf`。鼠标指向实体表面滚轮缩放，右键缓慢拖动微调；可通过 `/gui/camera/view_control/sensitivity` 服务调整灵敏度。展示模型不会参与碰撞或飞行物理。
