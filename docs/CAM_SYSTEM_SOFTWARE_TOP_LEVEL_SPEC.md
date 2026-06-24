# cam_system 软件顶层规格说明

版本：1.5
状态：v0.1.0 正式冻结；OV5640 控制面和采集数据面均为 V4L2-only；旧裸 I2C/GPIO/MMIO 与开发期测试入口不进入正式程序，仅通过 git 历史追溯
范围：Zynq-7020 PetaLinux 用户态相机、YOLO、跟踪、MJPEG、REST、WebSocket 应用，以及调试态 package/rootfs 启动辅助组件

## 1. 文档定位

本文档是 `cam_system` 的唯一顶层软件约束文档。

本文档合并了当前项目最需要的需求规格说明和软件设计说明内容，用于明确：

- 程序必须实现什么功能
- 对外接口必须保持什么兼容行为
- 线程、模块、共享状态由谁拥有
- 编码、审查、测试需要遵守哪些规则
- 一个变更进入上板测试前需要哪些证据

后续补充文档、测试记录、补丁说明都应引用本文档中的需求 ID。若代码行为与本文档冲突，必须先解决冲突，再接受该变更。

## 2. 参考工程实践

本项目只采用下列工业实践中的可落地子集，不宣称完整认证合规：

- NVIDIA Holoscan / DeepStream 风格：将边缘 AI 应用组织为明确的数据流、组件和可观测 pipeline。
  - https://docs.nvidia.com/holoscan/sdk-user-guide/holoscan-core-concepts.html
  - https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_ref_app_deepstream.html
- NIST SSDF：安全软件开发中的准备、保护、生产、验证、残留风险跟踪。
  - https://csrc.nist.gov/pubs/sp/800/218/final
- Zephyr / MISRA-C 风格：受限 C 语言子集、显式边界、静态分析、偏差记录。
  - https://docs.zephyrproject.org/latest/contribute/coding_guidelines/index.html
- OpenSSF 编译加固建议：对既有 C/C++ 项目逐步启用 hardening 和 warning 管理。
  - https://best.openssf.org/Compiler-Hardening-Guides/Compiler-Options-Hardening-Guide-for-C-and-C%2B%2B.html

本项目不声明符合完整 MISRA、ISO 26262、NVIDIA DRIVE safety process 或任何安全认证生命周期。

## 3. 系统上下文

`cam_system` 是运行在 Zynq-7020 PS 侧 PetaLinux 上的用户态应用。

当前软硬件上下文：

- OV5640 通过 PL 侧 8-bit DVP 采集。
- PL 将传感器输入按 RGB565 类似格式拼接并扩展为 24-bit packed RGB888，再通过 AXI HP 写入 DDR。
- PS 侧通过 V4L2 `/dev/videoX` mmap capture 读取 packed 3 bytes/pixel 帧；正式程序不包含旧裸 MMIO frmbuf 路径。
- 应用在 `8080` 端口提供 HTTP/REST、MJPEG 和 WebSocket 接口。
- YOLOv6Lite-S 通过 NCNN FP32 路径在 PS CPU 上推理。
- 当前模型输入固定为 `256x192`，BGR，NCNN 输入路径等价于 NCHW 模型部署。

### 3.1 工作空间目录契约

项目源码和资源必须按职责边界组织：

| 路径 | 内容 |
| --- | --- |
| `include/cam_system/` | 对跨模块可见的公共头文件和全局配置。 |
| `src/app/` | 程序入口。 |
| `src/platform/` | Zynq/PL/OV5640/V4L2 capture 等平台相关实现。 |
| `src/capture/` | 帧采集主循环和帧发布。 |
| `src/media/` | JPEG 等媒体编码。 |
| `src/network/` | HTTP、REST、MJPEG、WebSocket 和命令解析。 |
| `src/vision/` | YOLO、光流等视觉算法。 |
| `src/control/` | tracker 和控制状态机。 |
| `assets/models/` | 随应用发布的只读模型资产。 |
| `third_party/` | 明确来源的三方源码或预构建依赖。 |
| `packaging/` | PetaLinux/Yocto recipe 模板和打包入口。 |
| `scripts/build/`、`scripts/petalinux/`、`scripts/board/` | 构建、PetaLinux 集成和板端测试脚本。 |
| `docs/THIRD_PARTY_COMPONENTS.md` | A03 程序资源、三方组件、模型资产和外部构建依赖治理清单。 |
| `docs/MODEL_ASSETS.md` | A03 模型文件、运行契约、来源和精度基线治理说明。 |
| `docs/CAM_LAUNCHER_WORKFLOW.md` | A06 PS_KEY1 物理启动入口设计、构建和测试说明。 |
| `docs/WEBSOCKET_PROTOCOL.md` | A07 WebSocket 受限协议栈边界、拒绝项和测试要求。 |
| `docs/DRIVER_INTEGRATION_A08.md` | A08 OV5640/PL video 与 8812BU/WiFi 驱动集成建模、学习路线和测试矩阵。 |
| `docs/DRIVER_INTEGRATION_A09.md` | A09 OV5640 控制面驱动化实现、问题归零、构建与上板回归说明。 |
| `docs/PROJECT_FREEZE_CHECK_A08.md` | A08 项目规范冻结检查记录，说明当前冻结口径、历史文档状态偏差和下一步讨论点。 |
| `docs/LEARNING_PLAN.md` | `v0.1.0` 固化后的系统学习计划，约束后续阶段推进、概念解释和计划变更说明。 |

### 3.2 目标 rootfs 路径契约

package 化后的板端路径必须遵循 Linux/FHS 和 Yocto 目录变量语义：

| 路径 | 内容 |
| --- | --- |
| `/usr/bin/cam_system` | 应用可执行文件。 |
| `/usr/sbin/cam-launcher` | PS_KEY1 input-event 启动辅助程序。 |
| `/etc/init.d/cam-launcher` | 启动辅助程序 SysV init 入口。 |
| `/usr/share/cam-system/models/yolov6lite_s/` | 随 rootfs 发布的只读 YOLO 模型。 |
| `/usr/share/cam-system/manifest/` | 产物版本和校验清单。 |
| `/etc/cam-system/` | 后续预留给设备配置；本阶段不新增配置文件。 |
| `/var/lib/cam-system/` | 后续预留给运行时持久状态、可更新模型或轻量 OTA 持久目录。 |
| `/run/cam-system/` | 后续预留给 PID、socket 等临时运行态。 |

当前模型不再安装到 `/home/petalinux`。正式冻结和板端回归必须使用 `/usr/bin/cam_system`；若开发阶段显式采用临时测试二进制，也必须记录为调试例外，并保证模型文件存在于上述 `/usr/share/cam-system/models/yolov6lite_s/` 路径。

本文档不覆盖：

- PL 加速器实现。
- 除 PS_KEY1 `gpio-keys` 节点和已受控 OTA/boot recipe 外的任意 kernel、device tree、boot image 修改。
- 模型重训、剪枝、蒸馏、量化。
- 前端应用源码。

## 4. 对外接口契约

除非显式做版本化升级，下列对外接口必须保持兼容。

| ID | 接口 | 约束 |
| --- | --- | --- |
| IF-001 | `GET /api/status` | 必须返回 `status` 对象，并为兼容旧前端保留顶层镜像字段；状态字段包含 `cam_enabled`、`yolo_enabled`、`tracking`、`target_id`、`fps_capture`、`fps_inference`、`fps_stream`、`uptime_sec`、`status_seq`。不得下发 `zoom`。 |
| IF-002 | `POST /api/camera` | 请求体 `{"enabled": true/false}` 必须控制相机采集状态。 |
| IF-003 | `POST /api/yolo` | 请求体 `{"enabled": true/false}` 必须控制 YOLO 推理状态。 |
| IF-004 | `POST /api/settings` | 必须接受合法范围内的 `classes` 和 `max_det`，非法类别必须拒绝。 |
| IF-005 | `POST /api/track` | 必须接受 `select`、`start`、`stop` 动作。开发期 `test` 动作不进入 v0.1.0 正式程序，必须按非法 action 拒绝。 |
| IF-006 | `GET /stream` | 有可用帧时必须提供 MJPEG multipart 流。 |
| IF-007 | `GET /ws` | 必须升级为 WebSocket，并发布检测与状态消息；客户端输入只支持 `WEBSOCKET_PROTOCOL.md` 定义的受限 text/ping/pong/close 子集，不承载业务控制命令。 |
| IF-008 | `POST /api/zoom` | 当前硬件链路不支持动态变焦跟踪；该接口仅为前端兼容保留，不得改变采集、检测或跟踪状态。 |
| IF-009 | PS_KEY1 | 当前工程实测对应 MIO10 的低有效按键经 `gpio-keys` 发布 `KEY_PROG1` press 事件；若 `cam_system` 未运行，`cam-launcher` 必须启动 `/usr/bin/cam_system`；若已运行，必须 no-op，不得停止程序或改变 camera/yolo/tracking 状态。 |

HTTP POST 分发前必须等待 `Content-Length` 指定的完整 body 到达，不能只收到 header 就调用 API handler。

## 5. 功能需求

| ID | 需求 |
| --- | --- |
| FR-001 | 应用必须初始化 OV5640 V4L2 控制面、V4L2 capture、JPEG 编码器、tracker 和 YOLO 模型后再提供控制服务。 |
| FR-002 | capture 线程必须发布最新完成帧，供 JPEG、YOLO、光流等消费者使用。 |
| FR-003 | YOLO 线程必须消费帧快照，执行 NCNN 推理、解码检测、类别过滤、NMS，并发布有界检测快照。 |
| FR-004 | YOLO 线程不得直接写 tracker 内部状态。 |
| FR-005 | tracker 状态转换和 ISP 写入必须由 capture/control 路径拥有。 |
| FR-006 | 网络 API handler 必须通过命令队列提交控制动作，不得直接修改 tracker 状态。 |
| FR-007 | `max_det` 必须限制在 `0..YOLO_MAX_DET`。 |
| FR-008 | class ID 必须限制在 `0..YOLO_NUM_CLASSES-1`，非法 class 必须返回错误。 |
| FR-008A | `classes=[]` 必须保持旧语义：表示不做类别过滤，而不是关闭检测。 |
| FR-009 | 关闭 YOLO 时，必须发布对前端可见的空检测状态。 |
| FR-010 | YOLO 关闭时若存在正在执行的推理，该 in-flight 结果不得在关闭后重新发布旧检测。 |
| FR-011 | 停止 tracking 时，必须清空 predicted target 状态并恢复 tracker 控制输出。 |
| FR-012 | 内部重构不得破坏前端可见 API 语义。 |
| FR-013 | 调试态必须提供不依赖 SSH 的本地物理启动入口，且该入口不得引入第二套前端控制语义。 |

## 6. 非功能需求

| ID | 需求 |
| --- | --- |
| NFR-001 | 当前 PS-only 部署中，YOLO 默认必须使用 NCNN 单线程。 |
| NFR-002 | 热路径日志默认必须关闭或限频。 |
| NFR-003 | 慢网络客户端不得无限期阻塞单线程网络事件循环。 |
| NFR-004 | 大帧数据不得在线程之间无控制复制；当前因硬件缓存/一致性约束保留的 copy 必须明确。 |
| NFR-005 | 运行时诊断和计时信息必须通过环境变量或显式测试版本控制。 |
| NFR-006 | 上板部署必须通过受控 recipe/staging 脚本进入 rootfs、device tree 或 boot 文件；不得手工覆盖板端系统文件作为固化结果。 |
| NFR-007 | 所有外部输入数值都必须做显式边界检查。 |
| NFR-008 | 变更进入上板测试前必须通过本地交叉编译和静态检查。 |
| NFR-009 | 用户态线程亲和性必须符合 7.1；亲和性设置失败不得静默继续。 |
| NFR-010 | 进入构建、链接、打包或运行路径的三方资源和模型资产必须记录来源、版本、许可证、hash 和治理状态。 |
| NFR-011 | WebSocket 异常输入不得长期阻塞 network 线程；受限协议边界必须可由主机侧测试验证。 |

## 7. 软件架构

本应用必须按轻量 pipeline 理解，而不是按无边界全局函数集合维护。

### 7.1 线程划分

| 线程 | 绑定/定位 | 职责 |
| --- | --- | --- |
| main/network 线程 | Core 0 | HTTP、REST、MJPEG、WebSocket、命令入队、状态发布。 |
| capture/control 线程 | Core 0 | 帧采集、缓存帧发布、tracker 命令消费、tracker 预测、光流更新、JPEG 编码。 |
| YOLO 线程 | Core 1 | 帧快照消费、NCNN 推理、检测快照发布。 |

### 7.2 状态所有权

| 状态 | 写入者 | 读取者 | 规则 |
| --- | --- | --- | --- |
| frame index / frame sequence / cached frame / frame offset | capture 线程 | YOLO 线程、network/status | capture 发布图像槽位、序号和 offset 时必须使用同一同步边界；YOLO 必须先复制私有帧快照再推理，不得在 NCNN 推理期间直接持有可被 capture 覆盖的共享缓存。 |
| detections | YOLO 线程发布原始结果；capture 线程可标注 target | network 线程、capture 线程 | 受 `det_mutex` 保护；`det_seq` 驱动 WS 发布。 |
| tracker 内部状态 | capture/control 路径 | network 只能读导出快照 | network 必须入队命令，不得直接改 tracker。 |
| predicted box | tracker/capture 路径 | network WS | 受 `pred_mutex` 保护。 |
| JPEG buffer | capture/JPEG 路径 | network MJPEG | 受 `jpeg_mutex` 保护。 |
| settings | network API | YOLO 线程 | 受 `settings_mutex` 保护。 |
| control command queue | network 生产，capture 消费 | 无直接外部读者 | 受 `cmd_mutex` 保护；固定长度 ring。 |

### 7.3 生命周期状态规则

所有对前端可见的 enable/disable 转换都必须发布收敛后的状态。

| 转换 | 必须达到的外部可见状态 |
| --- | --- |
| camera off | `status.cam_enabled=false`；HTTP 不崩溃、不阻塞。 |
| YOLO off | `status.yolo_enabled=false`；WS 发布 `detections: []`；in-flight YOLO 结果被丢弃。 |
| tracking stop | `status.tracking=false`；`target_id=null`；后续 WS 不再包含 `predicted_target`。 |
| settings changed | 非法 settings 被拒绝；合法 settings 影响后续检测。 |
| command queue full | API 返回显式错误，不覆盖已入队命令。 |

### 7.4 启动辅助组件

`cam-launcher` 是独立进程，不属于 `cam_system` 业务状态机。

| 组件 | 职责 | 非职责 |
| --- | --- | --- |
| `gpio-keys` | 通过 Zynq pinctrl 将 PS_KEY1/MIO10 切到 GPIO，并把低有效按键转换为 input event。 | 不承载业务命令。 |
| `cam-launcher` | 监听 `KEY_PROG1` press；在 `cam_system` 未运行时启动进程。 | 不控制 camera/yolo/tracking，不做进程 supervisor，不开放网络端口。 |
| `cam_system` | 启动后继续提供 8080 HTTP/REST/MJPEG/WebSocket。 | 不负责监听物理启动按键。 |

## 8. 编码与审查规则

后续变更必须遵守下列规则。

| ID | 规则 |
| --- | --- |
| CR-001 | 不允许新增无界数组、无界队列或未检查范围的客户端输入。 |
| CR-002 | network 线程不得直接修改 tracker；若调用函数当前是只读或 no-op，必须记录原因。 |
| CR-003 | 热路径不得默认每帧 `printf`，除非受显式诊断开关控制。 |
| CR-004 | network 事件循环不得加入无界阻塞等待。 |
| CR-005 | UI/network 重构不得夹带模型、框架、PL 行为变更。 |
| CR-006 | 每个状态生产者都必须定义 disable 后如何发布空状态或终止态。 |
| CR-007 | 每个 WebSocket 可见状态变化都必须有 sequence 或事件触发，使前端可以收敛。 |
| CR-008 | 静态分析告警必须修复或记录为已知偏差。 |
| CR-009 | 手工修改必须可由 `git diff` 审查；调试产物不得混入源码变更。 |
| CR-010 | 新增或替换三方资源前，必须先更新 `docs/THIRD_PARTY_COMPONENTS.md`，不得先放入构建路径再补来源说明。 |
| CR-011 | WebSocket 协议能力变更必须先更新 `docs/WEBSOCKET_PROTOCOL.md`，并补充或修改 `tests/ws_protocol_test.c`。 |
| CR-012 | 跨线程 flag、sequence 和退出状态不得使用裸 `volatile` 作为同步机制；必须使用 atomic accessor、mutex 或等价同步边界。 |
| CR-013 | signal handler 只允许设置异步安全退出标志；日志、socket 关闭和资源释放必须回到正常线程上下文执行。 |

## 9. 当前合规性评估

当前代码相对本文采用的工程子集评估如下。

| 领域 | 状态 | 证据 / 偏差 |
| --- | --- | --- |
| pipeline 模块划分 | 基本符合 | capture、YOLO、network、HAL、JPEG、tracker 已分模块。 |
| 单一状态所有者 | 基本符合 | tracker 写入大部分已收敛到 capture/control 路径；跨线程 flag、status/detection sequence 已通过轻量 atomic accessor 发布和读取；帧图像、offset、slot 和 frame sequence 已收敛到 `frame_mutex` 快照边界。 |
| 输入边界 | 基本符合 | `max_det`、classes、命令队列、HTTP body 长度已有边界。 |
| 热路径日志 | 基本符合 | YOLO 每帧日志默认关闭，可用 `CAM_YOLO_LOG_EVERY` 控制；tracker 高频匹配日志默认关闭，可用 `CAM_TRACKER_VERBOSE=1` 和 `CAM_TRACKER_DIAG_EVERY=<秒>` 显式开启。 |
| 网络健壮性 | 基本符合当前受限目标 | 慢发送、接收缓冲满、POST body 未完整到达已处理；普通 HTTP/WS 握手发送也必须受等待预算限制；WebSocket 已抽出受限 parser 并具备主机侧帧测试，但仍不是通用 HTTP/WS 服务器。 |
| OV5640 控制面 | 符合当前项目规范态 | sensor I2C/GPIO 由 `cam-ov5640-control` 内核模块拥有；用户态只通过 V4L2 subdev private controls 调用初始化、standby、ISP offset 和 scaler enable。 |
| 采集数据面 | 符合当前项目规范态 | 使用 Xilinx VIPP/frmbuf V4L2 capture，用户态通过 mmap buffer 取帧；V4L2 dequeue 必须优先返回最新完成帧，异常路径必须尽力归还已 dequeue buffer，避免控制闭环消费积压旧帧或耗尽队列。 |
| 程序资源治理 | 基本符合当前运行代码冻结目标 | cJSON、8812bu、libjpeg-turbo、ncnn、cam-opencv 已有来源/许可证/hash/recipe 证据；PetaLinux `cam-system` 已转为源码 recipe 构建；模型文件按输入资产纳入运行路径、hash 和接口契约，本轮不把模型来源或导出链作为运行代码冻结阻塞项。 |
| 物理启动入口 | 已通过当前板端快速验证 | `cam-launcher`、`gpio-keys`、MIO10 pinctrl state 和 recipe 已纳入项目；当前硬件上 PS_KEY1 可启动 `cam_system`，若硬件/XSA/device tree 变化必须重新取证。 |
| 静态分析 | 基本符合一方代码目标 | 用户态一方代码、当前 OV5640/PL bridge 驱动源码已清理本轮可处理告警；`cppcheck --project` 仍会报告三方 `cJSON.c` 上游源码告警，按三方治理偏差处理，不在本项目内直接改写上游源码。 |
| 安全/安全性生命周期 | 不完整符合 | 无正式 hazard analysis、无认证、无 CI、无 auth/TLS。 |
| 上板验证 | 当前冻结基线已完成快速回归 | 最近一次 PS_KEY1 启动和前端快速测试未发现功能问题；本轮定版优化未触及采集、驱动、模型、recipe、OTA 或启动链路，按变更范围不要求重新上板。后续若触及这些路径，必须重新执行快速回归。 |

结论：当前程序可用于本项目受控局域网内嵌入式原型测试，但不得描述为已完整符合大厂安全认证级软件规范。

## 10. 已知偏差

| ID | 偏差 | 风险 | 处理要求 |
| --- | --- | --- | --- |
| DEV-001 | 三方 `cJSON.c` 在 cppcheck 下存在上游源码告警。 | 当前输入长度、JSON body 和业务字段已有本项目边界；但不能声明三方源码零告警。 | 不直接改写上游单文件源码；如进入更严格产品态，优先通过 recipe 版本升级、替换库或三方偏差审批处理。 |
| DEV-002 | HTTP/WS 未认证。 | 仅适合可信本地局域网。 | 不得暴露到外网；若进入产品态，必须补认证、访问控制或隔离网络设计。 |
| DEV-003 | WebSocket 是受限协议栈，不支持分片、binary、64-bit payload length 或通用业务消息路由。 | 互操作能力有限。 | 以 `WEBSOCKET_PROTOCOL.md` 为准；扩展前先补测试和协议说明。 |
| DEV-004 | 当前没有正式 CI、覆盖率、fuzz、依赖 CVE 自动扫描和安全认证流程。 | 不能声明为完整大厂量产安全流程。 | 调试态接受；产品态必须接入自动化质量门禁。 |
| DEV-005 | 上板测试目前主要是人工/半自动，不是 CI。 | 回归风险。 | 接受行为变更前执行 `CAM_SYSTEM_TEST_CASES.md`。 |
| DEV-006 | PS_KEY1 的软件 MIO 依据来自当前板端实测：按键改变 GPIO bank0 bit10，而不是早期原理图推导的 bit16。 | 若更换硬件/XSA 后 MIO 对应关系变化，按键不会产生中断。 | A06 上板验证 `/proc/bus/input/devices`、`/proc/interrupts` 中 `cam-ps-key1` 计数和 `/tmp/cam-launcher.log`；硬件变更后重新执行 GPIO bank 扫描。 |

## 11. 接受准则

变更进入上板测试前必须满足：

- 源码变化可通过 `git diff` 审查。
- `cmake --build build -j2` 通过。
- `git diff --check` 通过。
- 若变更涉及 WebSocket 帧处理，`./scripts/build/test_ws_protocol.sh` 必须通过。
- 若变更涉及 PS_KEY1 启动入口，`./scripts/build/validate_cam_launcher.sh` 必须通过。
- `cppcheck` 不引入超过本文档已知偏差的新告警。
- 被触及模块的生命周期状态测试已定义。
- 上板测试结果已记录，或明确标记为未执行及原因。

若变更影响前端可见状态，还必须满足：

- REST 控制请求在真实前端兼容 HTTP body 传输下成功。
- WebSocket 在 enable/disable 状态转换后能收敛。
- 重复 REST/WS 操作后 MJPEG 仍可用。
