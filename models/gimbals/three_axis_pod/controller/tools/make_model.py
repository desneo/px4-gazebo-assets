#!/usr/bin/env python3
"""由 sim/models/gimbal_up/model.sdf 生成“云台插件版”模型。

与 gimbal_up 的差异：
- 模型名 three_axis_pod；
- 去掉三个 JointController 插件，改由 gz-gimbal-controller 插件直接以速度模式驱动关节；
- 网格改用本仓库内的 model://three_axis_pod/meshes/*，避免依赖 PX4 自带 gimbal 模型。

可重复执行。
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
PROJ = os.environ.get("GIMBAL_REFERENCE_ROOT", "")
if not PROJ:
    raise SystemExit("请设置 GIMBAL_REFERENCE_ROOT，指向包含 sim/models 的参考工程")
SRC = os.path.join(PROJ, "sim", "models", "gimbal_up", "model.sdf")
ASSETS = os.path.abspath(os.path.join(PKG, "../../../.."))
DST_DIR = os.path.join(ASSETS, "models", "gimbals", "three_axis_pod")

PLUGIN_BLOCK = """    <!-- 三轴云台插件：图像跟踪 + 视觉伺服 + 状态机 + 遥测 -->
    <!-- 直接以速度模式驱动关节，替换原 JointController + 外部服务 -->
    <plugin filename="gz-gimbal-controller" name="gimbal::GimbalController">
      <yaw_joint>cgo3_vertical_arm_joint</yaw_joint>
      <pitch_joint>cgo3_camera_joint</pitch_joint>
      <roll_joint>cgo3_horizontal_arm_joint</roll_joint>
      <camera_link>camera_link</camera_link>
      <camera_sensor>camera</camera_sensor>
      <command_topic>/gimbal/command</command_topic>
      <telemetry_topic>/gimbal/telemetry</telemetry_topic>
      <command_result_topic>/gimbal/command_result</command_result_topic>
      <control_rate>100</control_rate>
      <telemetry_rate>50</telemetry_rate>
      <max_rate_dps>100</max_rate_dps>
      <max_acc_dps2>160</max_acc_dps2>
      <forward_az_deg>0</forward_az_deg>
    </plugin>
  </model>
</sdf>
"""


def main():
    with open(SRC, encoding="utf-8") as f:
        s = f.read()

    s = s.replace("<model name='gimbal_up'>", "<model name='three_axis_pod'>")
    s = s.replace("model://gimbal/meshes/", "model://three_axis_pod/meshes/")

    # 去掉三个 JointController 与 JointStatePublisher 插件
    s = re.sub(
        r'\s*<plugin filename="gz-sim-joint-controller-system".*?</plugin>',
        "", s, flags=re.S)
    s = re.sub(
        r'\s*<plugin filename="gz-sim-joint-state-publisher-system".*?</plugin>',
        "", s, flags=re.S)

    # 用云台插件替换结尾
    s = re.sub(r"\s*<!-- Plugins -->\s*</model>\s*</sdf>\s*$",
               "\n" + PLUGIN_BLOCK, s)

    os.makedirs(os.path.join(DST_DIR, "meshes"), exist_ok=True)
    with open(os.path.join(DST_DIR, "model.sdf"), "w", encoding="utf-8") as f:
        f.write(s)
    with open(os.path.join(DST_DIR, "model.config"), "w", encoding="utf-8") as f:
        f.write(
            '<?xml version="1.0"?>\n<model>\n'
            '  <name>Three-axis Pod</name>\n'
            '  <version>1.0</version>\n'
            "  <sdf version='1.9'>model.sdf</sdf>\n"
            '  <description>Skyward 3-axis gimbal driven by the '
            'gz-gimbal-controller system plugin.</description>\n'
            '</model>\n')
    print("generated", os.path.join(DST_DIR, "model.sdf"))

    # 支架模型：把吊舱安装在 1.5m 高处（复用 sim/models/gimbal_stand 结构）
    stand_src = os.path.join(PROJ, "sim", "models", "gimbal_stand", "model.sdf")
    with open(stand_src, encoding="utf-8") as f:
        st = f.read()
    st = st.replace("<model name=\"gimbal_stand\">",
                    "<model name=\"three_axis_pod_stand\">")
    st = st.replace("<uri>model://gimbal_up</uri>", "<uri>model://three_axis_pod</uri>")
    stand_dir = os.path.join(ASSETS, "assemblies", "three_axis_pod_stand")
    os.makedirs(stand_dir, exist_ok=True)
    with open(os.path.join(stand_dir, "model.sdf"), "w", encoding="utf-8") as f:
        f.write(st)
    with open(os.path.join(stand_dir, "model.config"), "w", encoding="utf-8") as f:
        f.write(
            '<?xml version="1.0"?>\n<model>\n'
            '  <name>Three-axis Pod Stand</name>\n'
            '  <version>1.0</version>\n'
            "  <sdf version='1.9'>model.sdf</sdf>\n"
            '  <description>Fixed 1.5m stand mounting the three-axis pod.'
            '</description>\n'
            '</model>\n')
    print("generated", os.path.join(stand_dir, "model.sdf"))


if __name__ == "__main__":
    main()
