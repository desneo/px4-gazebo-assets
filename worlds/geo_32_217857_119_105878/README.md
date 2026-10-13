# 真实地理背景相机环境

用于云台相机与视觉测试的 Gazebo Sim 8 环境。包含真实高程、地表影像、静态云彩天空和太阳方向光，不包含无人机、云台或视觉算法。

## 环境参数

- 中心：WGS84 纬度 `32.21785737811048`，经度 `119.1058778440883`。输入坐标按 WGS84 解释，不进行 GCJ-02 或 BD-09 转换。
- 测试范围：以中心为原点、水平半径 5000 米。实际地形覆盖 ENU 坐标 `[-5000, 5000] × [-5000, 5000]` 米的正方形，完整包含测试圆。
- 坐标轴：X 向东，Y 向北，Z 向上。中心地面为 Z=0；其他位置的地面 Z 随高程与地球曲率变化。
- 使用高度：距当地地面 200 米以下；环境本身不实施限高和地理围栏。飞行器的相对起飞点高度与当地离地高度不是同一数值。
- 地形：Mapzen Terrain Tiles 的 Terrarium 数据，zoom 12，在中心处的输入采样间距约 32 米；输出网格间距 31.25 米。重采样不增加源数据精度。
- 影像：Esri World Imagery，400 个地形块共享 16 张纹理，通过纹理坐标引用对应地理区域。每张纹理覆盖 2500 米见方的范围，有效影像 `2048 × 2048` 像素，输出采样约 1.22 米/像素；四周各增加 8 像素的相邻影像保护边，实际尺寸为 `2064 × 2064`。源影像精度、拍摄时间与实际地物现状可能不同，输出采样不代表原生分辨率或定位精度。
- 网格：20 × 20，共 400 块，每块 500 米见方、17 × 17 个顶点，共 204800 个三角形。视觉与碰撞使用同一地形表面；相邻块共用一致的边界高程和法线。模型名称为 `tile_00_00` 至 `tile_19_19`，两个索引分别沿东、北方向递增。
- 天空：使用 Gazebo Ogre2 自带的静态天空立方体贴图，包含云彩和太阳亮区。方向光固定；天空不按真实地点和日期计算太阳位置，不提供动态云、云影或气象模拟。

地表影像中的房屋、树木、车辆等是贴图内容，不产生对应的立体遮挡、视差或独立目标标签。河流水面随 DEM 地形显示，不模拟水动力或镜面反射。地形数据使用源高程作为近似地理高度，未进行大地水准面转换，不用于绝对高程精度验证。

## 加载

运行条件：Gazebo Sim 8、Ogre2 以及支持 OpenGL 3.3 的图形驱动。环境资源已随 `world.sdf` 保存，加载无需 Python、生成脚本或联网下载。

从仓库根目录直接加载：

```bash
gz sim -r worlds/geo_32_217857_119_105878/world.sdf
```

无界面运行：

```bash
gz sim -s -r worlds/geo_32_217857_119_105878/world.sdf
```

资源路径相对于 `world.sdf` 所在目录解析，移动环境时应保留 `assets/` 与 `world.sdf` 的相对位置。连接 PX4 时需另行加载无人机及云台模型。云台相机的远裁剪距离需覆盖目标观察距离；地理背景检查使用过 20 km 的远裁剪距离。

## 回到地图中心

Gazebo 3D 视图右上角提供“回到地图中心”按钮。点击后退出浏览相机的跟随模式，将视角移到地图中心 `(0, 0, 200)` 米，垂直俯视、北向上；对应地理中心为 `(32.21785737811048, 119.1058778440883)`。按钮只改变 Gazebo 浏览视角，不修改飞机、云台、目标或仿真状态。

按钮由 `gui/` 下的原生 Gazebo GUI 插件提供，调用 Gazebo 自带的 `CameraTracking` 相机定位服务。相机尚未就绪或服务不可用时，按钮显示重试提示。返回位置与朝向由 `world.sdf` 中 `MapCenter` 插件的 `center_pose` 配置，单位为米和弧度。

首次使用按钮时，需要 CMake、C++17 编译器、Gazebo GUI 8 / Transport 13 / Msgs 10 和 Qt 5 的开发库。在仓库根目录构建并安装到当前用户的 Gazebo GUI 插件目录：

```bash
cmake -S worlds/geo_32_217857_119_105878/gui \
  -B worlds/geo_32_217857_119_105878/gui/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$HOME/.gz/gui"
cmake --build worlds/geo_32_217857_119_105878/gui/build -j4
cmake --install worlds/geo_32_217857_119_105878/gui/build
```

安装后使用上面的 `gz sim` 命令启动即可显示按钮，已打开的 GUI 需要重新启动。复制该 world 的 GUI 配置到综合场景时，也会保留此按钮。插件源码随 world 保存，编译产物不入库；不需要 Python，也不改变静态地形资源的加载方式。无界面仿真不加载此 GUI 插件。

按钮测试覆盖 QML 点击、中心位置与朝向、重复点击和相机服务不可用的情况：

```bash
ctest --test-dir worlds/geo_32_217857_119_105878/gui/build --output-on-failure
```

## 静态文件

- `world.sdf`：地形布局、碰撞、天空、光照、地理原点和 Gazebo 系统插件配置。
- `assets/tile_*.dae`：地形网格。
- `assets/imagery_*.jpg`：共享的真实地表影像贴图，北向上。
- [metadata.json](metadata.json)：数据来源、源请求地址、源文件 SHA-256、中心高程及采样参数。
- `gui/`：地图中心按钮的 C++ / QML 插件源码和构建配置，不参与地形数据生成。

## 资源与数据来源

16 张纹理的单份 RGBA 解码数据约 260 MiB，连同完整缩小纹理层级约 347 MiB。该数值不包含网格、驱动缓存、GUI、相机渲染目标或算法进程，不能作为实际内存或显存测量值。图像画面细节受源影像限制，不适合直接验证小目标或高倍率变焦的真实识别效果。

Gazebo 默认一次加载全部 400 个地形块；500 米分块不代表按距离自动加载或卸载。

- 高程：[Mapzen Terrain Tiles / AWS Open Data](https://registry.opendata.aws/terrain-tiles/)。底层数据来源及署名要求见 [Tilezen attribution](https://github.com/tilezen/joerd/blob/master/docs/attribution.md)。
- 影像：Source: Esri, Vantor, Earthstar Geographics, and the GIS User Community。服务与使用条款见 [World Imagery](https://goto.arcgisonline.com/maps/World_Imagery)。影像权利属于对应提供方，未附加本仓库的软件许可；使用或再分发时适用原始数据条款。
- 天空：由本机 Gazebo Ogre2 安装资源提供，环境不复制该贴图。
