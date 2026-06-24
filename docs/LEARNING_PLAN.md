# cam_system 学习计划

版本：0.2
状态：基于 `v0.1.0` 正式冻结状态固化；作为后续学习推进的顶层计划
适用基线：以本仓库 `v0.1.0` release tag 指向的提交为准

## 1. 目标

本文档用于把 `cam_system` 从“已经可运行的项目”转化为“可系统学习的嵌入式 Linux 工程样板”。

后续学习必须围绕当前项目事实展开：

- Zynq-7020，PS + PL 架构。
- PetaLinux 2022.2，Linux 5.15.36-xilinx-v2022.2。
- OV5640 8-bit DVP 经 PL 写 DDR。
- PS 侧用户态 `cam_system` 提供 camera、YOLO、tracking、MJPEG、REST、WebSocket。
- OV5640 控制面和采集数据面已进入 V4L2-only 正式路径。
- `cam-launcher`、`cam-ota`、相关 recipe 和 OTA slot 状态已纳入 `v0.1.0` 管理。

学习目标不是背诵当前代码，而是形成可迁移的方法：

1. 能看懂嵌入式 Linux 项目的完整软硬件分层。
2. 能判断功能应放在 PL、内核驱动、系统服务、用户态应用、前端还是构建系统中。
3. 能从零设计项目目录、构建、recipe、驱动、协议、OTA、测试和发布流程。
4. 能区分调试态、工程样机态和量产态的不同要求。
5. 能基于事实证据、硬件资源、实时性、复杂度和维护成本做取舍。

## 2. 推进规则

每进入一个新学习阶段，必须先执行“阶段适用性检查”。

检查内容：

1. 当前阶段是否仍符合项目目标。
2. 当前阶段是否依赖已经过时的项目状态。
3. 是否需要补充代码、文档、板端状态或外部资料作为证据。
4. 是否有更高优先级的工程问题阻塞学习。
5. 本阶段是否会引入新的项目变更、测试需求或风险。

若需要调整学习计划，必须说明：

- 调整项。
- 调整原因。
- 对当前项目边界的影响。
- 对后续阶段顺序的影响。
- 是否需要修改本文档。

禁止事项：

- 不得把对话中的临时判断当作长期事实。
- 不得跳过概念解释直接进入命令或代码。
- 不得用“行业一般如此”替代当前项目证据。
- 不得为了学习而引入不适合当前项目的复杂实现。
- `v0.1.0` 正式发布后不得移动 release tag；后续学习文档和实验应在新提交中追踪。

## 3. 固定讲解格式

后续每个阶段按以下结构讲解：

1. 概念定义：先解释本阶段涉及的专业概念。
2. 背景来源：说明这些概念来自 Linux、Yocto、PetaLinux、V4L2、网络协议、OTA 或硬件设计中的哪一层。
3. 当前项目对应：映射到本项目的具体文件、模块、recipe、设备节点、启动文件或板端状态。
4. 当前设计原因：解释为什么当前项目这样做。
5. 替代方案：列出可选实现及其复杂度、风险、收益。
6. 调试态与量产态差异：说明当前项目接受了什么边界，量产项目通常还要补什么。
7. 可迁移原则：总结以后其它项目也适用的设计思想。
8. 验证方式：说明如何通过代码、构建日志、板端命令或测试用例证明结论。

## 4. 阶段 A：项目全貌与分层

目标：建立项目全局地图。

需要解释的概念：

- SoC：片上系统，把 CPU、外设控制器、内存控制器等集成在同一芯片中。
- Zynq PS：Processing System，Zynq 内的 ARM Cortex-A9 处理系统。
- Zynq PL：Programmable Logic，Zynq 内的 FPGA 可编程逻辑。
- BSP：Board Support Package，板级支持包，通常包括 bootloader、device tree、内核配置、驱动和 rootfs 集成。
- 用户态：Linux 中普通进程运行的空间，不能直接随意操作硬件，通常通过系统调用和设备节点访问资源。
- 内核态：Linux 内核和驱动运行的空间，负责硬件抽象、中断、DMA、调度和安全隔离。
- rootfs：根文件系统，包含 `/usr/bin`、`/etc`、`/usr/lib` 等运行时文件。
- FIT image / `image.ub`：PetaLinux 生成的启动镜像，当前包含 kernel、device tree、initramfs/rootfs。
- release tag：Git 中标记一个可追溯版本的标签，当前为 `v0.1.0`。

当前项目对应：

- PL 接收 OV5640 DVP 并写 DDR。
- 内核驱动提供 V4L2 控制面和视频数据面。
- 用户态 `cam_system` 处理业务逻辑、AI、网络接口和跟踪。
- PetaLinux/Yocto recipe 把程序、驱动、脚本、模型和配置装进 rootfs。
- OTA 把新的 `image.ub` 写入非活动 slot 并通过 U-Boot 启动钩子切换。

阶段产出：

- 系统分层图。
- 组件职责表。
- 当前项目“调试态、工程规范态、量产态”边界说明。

## 5. 阶段 B：PetaLinux/Yocto 构建与 package

目标：理解源码如何变成板端可启动系统。

需要解释的概念：

- Yocto：用于生成定制 Linux 发行版的元构建系统。
- OpenEmbedded：Yocto 使用的构建元数据和任务体系基础。
- BitBake：执行 recipe 任务的构建引擎。
- PetaLinux：Xilinx 对 Yocto、BSP、硬件描述和镜像生成流程的封装。
- recipe：`.bb` 文件，描述一个软件组件如何获取源码、编译、安装和打包。
- layer：Yocto 元数据分层，当前项目自定义内容位于 `meta-user`。
- `SRC_URI`：recipe 的源码或文件输入列表。
- `DEPENDS`：构建时依赖。
- `RDEPENDS`：运行时依赖。
- `do_fetch`：获取源码。
- `do_unpack`：解包源码。
- `do_patch`：应用补丁。
- `do_compile`：编译。
- `do_install`：安装到临时目标 rootfs 目录。
- `do_package`：生成包。
- `do_rootfs`：把包组合成 rootfs。
- `DL_DIR`：下载缓存。
- `SSTATE_DIR`：共享状态缓存，用于加速重复构建。
- `TMPDIR`：Yocto 构建临时目录。
- `rm_work`：构建后删除部分工作目录以节省空间。

当前项目对应：

- `packaging/petalinux/recipes-apps/cam-system/`
- `packaging/petalinux/recipes-apps/cam-launcher/`
- `packaging/petalinux/recipes-apps/cam-ota/`
- `packaging/petalinux/recipes-kernel/cam-ov5640-control/`
- `scripts/petalinux/stage_*.sh`
- `project-spec/meta-user/recipes-core/images/petalinux-image-minimal.bbappend`

阶段产出：

- 从 Git 源码到 `/usr/bin/cam_system` 的路径说明。
- 从 recipe 到 `image.ub` 的任务流说明。
- 当前 build cache、downloads、sstate-cache 的保留和清理策略。

## 6. 阶段 C：用户态应用架构

目标：理解 `cam_system` 的模块划分、线程设计和状态管理。

需要解释的概念：

- 进程：Linux 调度和资源隔离的基本单位。
- 线程：同一进程内共享地址空间的执行流。
- CPU affinity：限制线程在哪些 CPU core 上运行。
- 状态机：用明确状态和事件控制系统行为，避免隐式状态混乱。
- 模块边界：每个模块只拥有自己职责内的数据和操作。
- 接口契约：对外 API 和模块间 API 的稳定行为约束。
- fail fast：关键依赖缺失时立即失败，而不是静默降级到不受控路径。
- 资源生命周期：初始化、运行、关闭和异常退出时资源如何获得与释放。

当前项目对应：

- `src/app/`
- `src/capture/`
- `src/control/`
- `src/media/`
- `src/network/`
- `src/platform/`
- `src/vision/`
- `include/cam_system/`

阶段产出：

- 线程模型图。
- camera、YOLO、tracking 状态转换图。
- 模块职责和禁止跨界访问说明。

## 7. 阶段 D：Linux 驱动开发基础

目标：借当前 OV5640、DVP bridge、PS_KEY1、8812BU 理解驱动开发通用思路。

需要解释的概念：

- device tree：描述硬件连接和资源的树形数据结构。
- compatible：device tree 中用于匹配驱动的字符串。
- platform device：SoC 或板级外设在 Linux 中的设备对象。
- platform driver：匹配 platform device 的驱动。
- I2C client：I2C 总线上的从设备对象。
- V4L2：Linux 视频设备框架。
- V4L2 subdev：V4L2 中 sensor、bridge、decoder 等子设备抽象。
- media controller：用 entity、pad、link 描述视频硬件拓扑的框架。
- GPIO descriptor：新式 GPIO API，避免依赖 sysfs gpio 编号。
- input subsystem：Linux 输入子系统，`gpio-keys` 会把按键发布成 input event。
- sysfs GPIO deprecated：旧 GPIO sysfs ABI 已不推荐用于新设计。
- IRQ：中断请求，硬件通知 CPU 事件发生。
- workqueue：内核中把不能在中断上下文长时间执行的工作推迟执行的机制。
- DMA：设备直接访问内存的数据传输方式。
- cache coherency：CPU cache 与 DMA 访问内存之间的一致性问题。

当前项目对应：

- `cam-ov5640-control`
- `cam-dvp-bridge`
- `gpio-keys` PS_KEY1
- `8812bu`
- `system-user.dtsi`
- `/dev/v4l-subdevX`
- `/dev/video0`

阶段产出：

- 当前驱动拓扑图。
- device tree 到 driver probe 的生命周期说明。
- 当前驱动化设计与旧用户态裸控制方案的对比。

## 8. 阶段 E：视频链路与实时性

目标：理解一帧图像从 OV5640 到前端显示、YOLO 推理和跟踪控制的全生命周期。

需要解释的概念：

- DVP：并行数字视频接口。
- PCLK：pixel clock，像素采样时钟。
- VSYNC：帧同步信号。
- HREF/HSYNC：行有效或行同步信号。
- RGB565：16-bit RGB 格式。
- RGB888：24-bit RGB 格式。
- AXI4-Stream：面向流数据的 AXI 协议。
- `tvalid/tready`：AXI4-Stream 有效和反压握手。
- `tuser/tlast`：视频流中常用于帧起始和行结束。
- `v_frmbuf_wr`：Xilinx Video Frame Buffer Write，把视频流写入 DDR。
- V4L2 buffer：内核管理视频帧缓冲的对象。
- mmap capture：用户态映射内核视频 buffer 读取数据。
- MJPEG：连续 JPEG 图片组成的视频流。
- 端到端延迟：采集、缓冲、编码、传输、解码、显示的总延迟。
- 吞吐：单位时间处理多少帧。
- latency：单帧从产生到可见的时间。
- frame drop：为了实时性丢弃旧帧。

当前项目对应：

- OV5640 -> PL DVP bridge -> Video In -> frmbuf -> DDR。
- `/dev/video0` V4L2 capture。
- TurboJPEG 编码。
- `/stream` MJPEG。
- YOLOv6Lite-S NCNN FP32 推理。
- tracker 控制 OV5640 ISP offset/scaler。

阶段产出：

- 一帧生命周期图。
- 采集、编码、推流、推理、跟踪各段耗时表。
- 是否需要进一步优化显示链路的证据标准。

## 9. 阶段 F：OTA 与系统可靠性

目标：理解当前可靠最小 OTA，以及它和量产 OTA 的差距。

需要解释的概念：

- bootloader：Linux 启动前运行的引导程序，当前是 U-Boot。
- FSBL：Zynq 第一阶段启动程序。
- `BOOT.BIN`：包含 FSBL、bitstream、U-Boot 等早期启动内容。
- `boot.scr`：U-Boot 启动脚本。
- `uEnv.txt`：U-Boot 导入的环境脚本片段。
- A/B slot：保留两个系统镜像槽，一个运行，一个更新。
- pending：新镜像首次试启动状态。
- good：镜像已验证可用状态。
- rollback：pending 失败后回退到上一槽。
- mark-good：运行时确认当前镜像可固化。
- watchdog：系统无响应时触发复位的硬件或软件机制。
- FIT `iminfo`：U-Boot 对 FIT 镜像的基本校验。

当前项目对应：

- `cam-ota`
- `cam-ota-bootguard`
- `/ota/slot_a/image.ub`
- `/ota/slot_b/image.ub`
- `/uEnv.txt`
- `BOOTLOADER_OTA_WORKFLOW.md`

阶段产出：

- OTA 状态机图。
- 更新成功、更新失败、中途断电的路径说明。
- 调试态 OTA 到量产 OTA 的差距清单。

## 10. 阶段 G：网络、协议与安全

目标：理解当前 REST、MJPEG、WebSocket 的边界，以及安全性差距。

需要解释的概念：

- HTTP：请求/响应协议。
- REST：用 HTTP method 和路径表达资源操作的接口风格。
- MJPEG multipart：一个 HTTP 响应中连续发送多张 JPEG。
- WebSocket：HTTP upgrade 后的全双工帧协议。
- mask：WebSocket 客户端到服务端帧必须使用的掩码机制。
- ping/pong/close：WebSocket 控制帧。
- payload limit：限制输入大小以避免内存和解析风险。
- 协议栈裁剪：只实现项目需要的协议子集。
- 认证：证明客户端身份。
- 授权：决定客户端能执行哪些操作。
- TLS：传输层加密。
- 本地局域网调试态：只在可信 LAN 内使用，安全要求低于公网产品。

当前项目对应：

- `src/network/`
- `docs/WEBSOCKET_PROTOCOL.md`
- `/api/status`
- `/api/camera`
- `/api/yolo`
- `/api/track`
- `/stream`
- `/ws`

阶段产出：

- 当前协议规范表。
- WebSocket 受限协议栈裁剪项。
- 调试态安全边界和量产态补齐项。

## 11. 阶段 H：测试、审查、发布与维护

目标：形成可复用的软件工程工作流。

需要解释的概念：

- 静态分析：不运行程序，通过工具检查代码风险。
- 动态测试：运行程序验证行为。
- 单元测试：验证小函数或小模块。
- 集成测试：验证多个模块协作。
- 上板测试：在真实硬件上验证。
- 回归测试：确认修改没有破坏既有功能。
- release candidate：候选发布版本。
- release tag：正式发布标记。
- manifest：记录源码、模型、构建产物 hash 的清单。
- hash：内容校验摘要。
- 可追溯性：从板端文件能追到源码 commit、recipe 和构建产物。
- 变更管理：每个修改都应有原因、范围、验证和回退方式。
- 问题归零：问题修复后要解释根因、漏检原因和防复发措施。

当前项目对应：

- `docs/CAM_SYSTEM_TEST_CASES.md`
- `docs/CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md`
- `scripts/build/`
- `scripts/board/`
- `git tag v0.1.0`
- `/usr/share/cam-system/manifest/cam-system-artifacts.sha256`
- `cam-ota status`

阶段产出：

- 发布 checklist。
- 上板测试 checklist。
- 代码审查 checklist。
- 后续版本 `v0.1.x` 变更流程。

## 12. 阶段顺序

默认顺序：

1. 阶段 A：项目全貌与分层。
2. 阶段 B：PetaLinux/Yocto 构建与 package。
3. 阶段 C：用户态应用架构。
4. 阶段 D：Linux 驱动开发基础。
5. 阶段 E：视频链路与实时性。
6. 阶段 F：OTA 与系统可靠性。
7. 阶段 G：网络、协议与安全。
8. 阶段 H：测试、审查、发布与维护。

顺序允许调整，但必须说明理由。

常见调整条件：

- 若项目构建或板端状态异常，优先回到阶段 B 或 H。
- 若后续要新增硬件控制，优先回到阶段 D。
- 若前端显示、跟踪体验或 YOLO 延迟成为主要问题，优先回到阶段 E。
- 若要准备长期维护或多人协作，优先强化阶段 H。
- 若要从调试态走向量产态，阶段 F 和 G 必须提前。

## 13. 变更记录

| 版本 | 日期 | 变更 |
| --- | --- | --- |
| 0.1 | 2026-06-18 | 基于 `v0.1.0` 固化后续学习计划和阶段推进规则。 |
| 0.2 | 2026-06-24 | 将学习计划基线调整为 `v0.1.0` 正式冻结状态，移除对旧提交哈希的硬编码。 |
