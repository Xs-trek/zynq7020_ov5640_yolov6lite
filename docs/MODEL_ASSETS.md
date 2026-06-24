# cam_system 模型资产说明

版本：0.1
状态：a03 模型资产治理基线

## 1. 范围

本文档约束当前随 `cam_system` 发布的 NCNN 模型资产：

```text
assets/models/yolov6lite_s/yolov6lite_s.ncnn.param
assets/models/yolov6lite_s/yolov6lite_s.ncnn.bin
```

该模型是运行时功能依赖，不是普通测试数据。任何替换、重导出、量化、剪枝或类别裁剪都必须先更新本文档和 `docs/THIRD_PARTY_COMPONENTS.md`。

## 2. 当前模型资产

| 项目 | 当前值 |
| --- | --- |
| 模型名称 | `yolov6lite_s.ncnn` |
| 推理框架 | ncnn |
| 代码入口 | `src/vision/yolo_detect.cpp` |
| 模型部署目录 | `/usr/share/cam-system/models/yolov6lite_s/` |
| param 文件大小 | 36949 bytes |
| bin 文件大小 | 1128080 bytes |
| param sha256 | `05af6a2006eed92fc0996f72bfd42d82c903513e88530fd697580e840306c026` |
| bin sha256 | `8f825ded3d1e7d06f70734a3e2f91df039f60a2f81538f4b76818d765f9548aa` |
| 当前 provenance | P1+，能证明本地文件、运行契约和官方原始 `.pt` 候选来源，不能证明完整转换链路。 |

## 3. 运行契约

当前代码和 `.param` 共同约束如下：

| 项目 | 契约 |
| --- | --- |
| 输入层 | `in0` |
| 输出层 | `out0` |
| 输入尺寸 | `256x192` |
| 输入来源 | PS 侧 `640x480` packed RGB888 帧缓存。 |
| 输入像素转换 | `ncnn::Mat::PIXEL_RGB2BGR`，随后 `1/255` 归一化。 |
| 均值 | `{0, 0, 0}` |
| 归一化 | `{1/255, 1/255, 1/255}` |
| 类别数 | COCO 80 类。 |
| 输出解码 | 每行按 `[cx, cy, w, h, obj_conf, cls0..cls79]` 解码。 |
| 置信度阈值 | `YOLO_CONF_THRESH=0.5` |
| NMS 阈值 | `YOLO_NMS_THRESH=0.5` |
| 最大检测数 | `YOLO_MAX_DET=20` |
| 推理线程 | `CAM_CPU_YOLO_CORE=1`，ncnn `num_threads=1`。 |

`.param` 结构证据：

- NCNN magic: `7767517`。
- 网络结构计数行：`414 462`。
- 输入层：`Input in0`。
- 末端输出：`Concat cat_25 ... out0`。
- 最终候选数量由 head reshape 可见为 `768 + 192 + 48 + 12 = 1020`。

## 4. 当前精度和功能边界

已知事实：

- 用户报告当前 `YOLOv6Lite-S` 在 `256x192` 输入下 AP50 约为 `0.26`。
- 该数值不是本轮本地可复现实验产物，因此本文档将其记录为用户提供的历史基线，不能作为正式 benchmark 结论。
- 当前模型精度较低，跟踪框丢失或短时选错目标可能来自模型检测能力、低分辨率输入、当前 tracking ID 语义或场景条件，不能在没有专门实验前归因到单一代码 bug。

模型替换或优化不得降低当前已接受的前端功能：

- 相机开关、YOLO 开关、MJPEG 流、WebSocket 检测框发布必须保持兼容。
- `classes=[]` 必须保持当前语义：不做类别过滤，而不是关闭检测。
- 关闭 YOLO 后必须发布空检测状态。
- 若更改输出格式，必须同步修改 `src/vision/yolo_detect.cpp` 解码逻辑和测试用例。

## 5. 来源和许可证待确认

当前模型来源仍未闭环：

| 项目 | 当前状态 | A03 要求 |
| --- | --- | --- |
| 原始模型 | 用户确认 `.pt` 来自 Meituan `YOLOv6` 仓库；官方 release `0.4.0` 存在 `yolov6lite_s.pt`，大小 `1487033` bytes，sha256 为 `6e424f1defd72c6478e0b630977edcc2844d4e601aa96a3e2960a8a9dee1682d`。 | 本仓仍未保存 `.pt`，也未保存 `.pt -> NCNN` 中间产物。 |
| 上游项目 | https://github.com/meituan/YOLOv6，release `0.4.0`，tag commit `f2114f37ff12ca888cddb47baf6b532e04110251`。 | 需要确认本地 NCNN 文件是否确由该 release 权重转换得到。 |
| 导出流程 | 官方 `docs/Test_NCNN_speed.md` 说明存在 PyTorch -> ONNX -> NCNN 和 PyTorch -> TorchScript/PNNX -> ONNX -> NCNN 两条路径；本地 `.param` 出现 `pnnx_fold_*` 节点，说明很可能经过 PNNX 或相关折叠流程。 | 需要记录实际执行命令、输入尺寸 `256x192`、通道顺序和任何手工修改。 |
| 许可证 | YOLOv6 仓库 `0.4.0` 的 `LICENSE` 为 GPL-3.0；未看到针对 release 权重或 NCNN 导出物的单独授权声明。 | 需要确认模型权重、导出产物和再分发是否受 GPL-3.0 或其它条款约束。 |
| 数据集 | 官方 YOLOv6Lite-S release 表记录 `320x320`、COCO val2017、mAP 0.5:0.95 为 `22.4`；用户报告当前 `256x192` AP50 约 `0.26`。 | 当前 `256x192` AP50 不是本轮可复现实验产物，仍需正式验证记录。 |

在来源未闭环前，本模型只视为当前本地受控调试资产，不应描述为可量产分发资产。

## 6. 后续模型资产准入规则

新增或替换模型必须提交下列证据：

| 证据 | 要求 |
| --- | --- |
| 模型文件 | `.param`、`.bin` 或其它运行产物必须提供 sha256。 |
| 原始来源 | 记录原始 `.pt`、`.onnx`、训练仓库、tag/commit、下载链接。 |
| 转换命令 | 记录导出和 NCNN 转换命令，包含输入尺寸、通道顺序、归一化方式。 |
| 输出契约 | 明确输出层名、tensor 形状、每列语义、类别数。 |
| 精度基线 | 至少记录 AP50、测试集、输入尺寸、类别范围。 |
| 性能基线 | 至少记录 PS 单核推理耗时、预处理耗时、后处理耗时。 |
| 许可证 | 记录模型权重、代码仓库、数据集的许可证或使用限制。 |
| 回归验证 | 完成本地构建；必要时上板验证不超过 20 分钟。 |

## 7. 当前接受结论

当前 `yolov6lite_s.ncnn` 可以继续作为 A 阶段调试模型使用，理由是：

- 已经与当前代码路径和前端功能完成历史上板验证。
- 本地文件 hash 和运行契约已固定。
- 本轮没有修改模型文件，也没有改变推理逻辑。

但它不能视为完整规范化模型资产，直到 A03-Q005 中的来源、导出流程、精度基线和许可证问题得到确认或明确接受为偏差。
