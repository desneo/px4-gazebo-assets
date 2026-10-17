# Planespotting YOLO11n 飞机检测模型

本目录保存两轴吊舱使用的单类别飞机检测模型。模型类别为 `aircraft`，固定输入为 `1×3×640×640`，输出为 `1×5×8400`，输出通道依次为中心点 X、中心点 Y、宽、高与 `aircraft` 置信度。模型输出不包含 NMS，调用方负责置信度过滤和 NMS。

## 来源与许可

- 上游仓库：<https://huggingface.co/nyuuzyou/Planespotting-YOLO11>
- 固定版本：`24eda2e19aa7efffed4b725a28430dc49bd89f83`
- 原始权重：[`yolo11n/weights/best.pt`](https://huggingface.co/nyuuzyou/Planespotting-YOLO11/blob/24eda2e19aa7efffed4b725a28430dc49bd89f83/yolo11n/weights/best.pt)
- 训练数据：`nyuuzyou/planespotting`
- 上游说明的训练集/验证集规模：9,885/93 张，训练输入尺寸为 640
- 许可证：AGPL-3.0，全文见 `LICENSE.txt`

原始 `best.pt` 的 SHA-256 为 `476ffc0a413a541f8b6f1503a34430ff926715b77e9c8a43697593f87f7d2b46`。

## ONNX 格式

`model.onnx` 使用 Ultralytics 8.4.178 从原始 YOLO11n 权重导出，配置为静态 640×640、batch 1、FP32、opset 17、图优化开启、动态维度关闭、内嵌 NMS 关闭。类别、张量与导出参数同时记录在 `metadata.json`。

模型已经通过 ONNX checker，并用 ONNX Runtime 1.31.0 和 OpenCV DNN 5.0.0 完成加载及零输入前向检查；两个运行时均得到单个有限值输出，形状为 `1×5×8400`。

文件校验值见 `SHA256SUMS`。
