# A08 驱动与系统集成建模说明

版本：0.1
状态：历史阶段记录；已被 A09 及后续 V4L2-only 正式程序形态取代，当前状态以 `CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md` 为准
范围：OV5640/PL video 链路、8812BU/WiFi 链路、PetaLinux/Yocto 驱动集成方式，以及后续驱动开发学习路线

## 1. 阶段目标

A08 的目标不是立即重写驱动，而是把当前项目中两条最有代表性的驱动链路统一建模：

- OV5640/PL video：固定传感器、PL 视频处理、frame buffer write、DDR 帧缓存、PS 用户态读取。
- 8812BU/WiFi：USB WiFi 芯片、内核外 kernel module、cfg80211/nl80211/wpa_supplicant、DHCP、SSH/前端调试通道。

A08-0 只完成下列内容：

- 记录当前本地实现事实。
- 整理可复用的驱动开发知识架构。
- 建立权威资料和工程案例索引。
- 明确后续是否进入驱动化实现前必须验证的决策点。
- 定义板端取证和测试矩阵。

A08-0 不做下列动作：

- 不修改 `cam_system` 运行代码。
- 不修改 device tree、kernel config、XSA、boot image。
- 不替换 8812BU 驱动。
- 不把 OV5640 迁移到 V4L2。
- 不改变前端、REST、MJPEG、WebSocket 或 PS_KEY1 行为。

## 2. 证据分级

后续讨论和实现必须区分证据等级。

| 等级 | 来源 | 用途 | 约束 |
| --- | --- | --- | --- |
| E0 | 本地源码、板端日志、上板测试结果 | 证明当前项目事实 | 优先级最高。若与外部案例冲突，以本地实测为准。 |
| E1 | Linux/Yocto/AMD/Xilinx/NVIDIA 官方文档 | 证明接口、框架和推荐实践 | 可作为设计依据，但仍需映射到当前 PetaLinux 2022.2 和 Zynq-7020。 |
| E2 | Linux 主线源码、官方内核树示例驱动 | 学习实际驱动写法 | 不能直接假设可无改动移植到当前板级 PL 链路。 |
| E3 | 8812BU 上游 vendor/out-of-tree 仓库 | 当前 WiFi 驱动来源和配置参考 | 只证明该仓库行为，不代表 Linux 无线驱动最佳实践。 |
| E4 | Bootlin、Linux Foundation、专业社区案例 | 学习路线、调试习惯和常见问题 | 不能单独作为本项目实现正确性的证明。 |

## 3. 当前本地架构事实

### 3.1 OV5640/PL video 链路

当时摄像头链路是“用户态 HAL + PL 固定视频链路”的实现，不是标准 V4L2 camera pipeline；该状态已被后续 V4L2 control/capture 实现取代。

| 环节 | 当前实现 | 本地证据 | 驱动化含义 |
| --- | --- | --- | --- |
| OV5640 power/reset | 用户态通过 `/sys/class/gpio` export GPIO 970/971 后写 direction/value。 | `src/platform/hal_ov5640.c` | 长期规范方向应改为 device tree + GPIO descriptor，由 kernel driver 获取 `reset-gpios`/`pwdn-gpios`。 |
| OV5640 I2C | 用户态打开 `/dev/i2c-2`，`ioctl(I2C_SLAVE, 0x3C)` 后写寄存器表。 | `src/platform/hal_ov5640.c` | 规范方向是 I2C client driver 的 `probe()`、regmap 或封装后的 read/write helper。 |
| OV5640 模式 | 当前固定 VGA 30fps RGB565 类初始化表，PS 侧最终按 packed RGB888 帧读取。 | `src/platform/hal_ov5640.c`、`include/cam_system/config.h` | 迁移到 V4L2 时必须把 mode table、mbus code、crop/scale、frame interval 显式化。 |
| PL DVP/格式转换 | OV5640 8-bit DVP 进入 PL，每两个 byte 拼 RGB565，再扩展 RGB888。 | 用户提供硬件链路说明，当前功能已由上板验证。 | 这部分不是普通 sensor driver 能单独描述的，需要 bridge/video IP 侧 device tree 和 media graph 配合。 |
| v_frmbuf_wr | 用户态直接 MMIO 配置 `FRMBUF_WR_BASE=0x43C10000`，单帧启动并轮询 `ap_done`。 | `src/platform/hal_frmbuf.c` | 规范方向可能是 Xilinx framebuffer DMA/Linux DMAEngine/V4L2 client，但必须先确认当前 XSA/DT/IP endpoint 是否匹配。 |
| DDR frame buffer | 固定物理地址 `0x3F000000`，16 MiB reserved-memory no-map。 | `include/cam_system/config.h`、`system-user.dtsi` | 若驱动化，buffer ownership、cacheability、DMA sync 和用户态访问接口必须重新定义。 |
| device tree 状态 | A08 当时 `v_frmbuf_wr@43c10000`、`ov5640_to_lcd@43c00000` 被置为 disabled。 | `packaging/petalinux/recipes-bsp/device-tree/files/system-user.dtsi` | 该阶段 kernel 尚未拥有 PL video 节点；当前正式形态以顶层规格为准。 |

结论：OV5640 链路是后续驱动学习价值最高的部分，但也是最高风险部分。不能直接把主线 `drivers/media/i2c/ov5640.c` 拿来替换当前代码；主线 sensor driver 只解决传感器端，不能自动解决本项目自定义 PL DVP/format/frmbuf 链路。

### 3.2 8812BU/WiFi 链路

当前 WiFi 链路已经完成基础工程化治理，但驱动本体仍是 out-of-tree vendor driver。

| 环节 | 当前实现 | 本地证据 | 驱动开发学习点 |
| --- | --- | --- | --- |
| 驱动来源 | `morrownr/88x2bu-20210702`，固定 `SRCREV=fecac340fb117eb979f4bb6d28e29730384c382b`。 | `docs/WIFI_DRIVER_WORKFLOW.md`、`docs/THIRD_PARTY_COMPONENTS.md` | 学习第三方内核外驱动治理：固定版本、许可证、构建参数、升级风险。 |
| 构建方式 | PetaLinux/Yocto recipe `inherit module`，随当前 kernel ABI 构建。 | `packaging/petalinux/recipes-kernel/8812bu/8812bu_git.bb` | 学习 kernel module recipe、`KERNEL_MODULE_AUTOLOAD`、`KERNEL_MODULE_PROBECONF`。 |
| 运行参数 | `CONFIG_BT_COEXIST=n`、`CONFIG_RTW_LOG_LEVEL=0`、`rtw_drv_log_level=0`。 | `8812bu_git.bb`、`S50wifi` | 学习功能裁剪、日志等级、module parameter 和可观测性取舍。 |
| 网络初始化 | `wifi-init` 启动 `wpa_supplicant`，等待 `wpa_state=COMPLETED` 后启动 `udhcpc`。 | `packaging/petalinux/recipes-apps/wifi-init/files/S50wifi` | 学习驱动与用户态网络栈的边界：驱动不负责 WPA 认证和 DHCP。 |
| 调试态配置 | 可从 `/etc/wifi-init.conf`、`/etc/wifi.conf`、SD 启动分区读取配置。 | `S50wifi`、`docs/WIFI_DRIVER_WORKFLOW.md` | 学习 RAM rootfs 场景下配置来源和安全边界。 |

结论：8812BU 适合学习“外部驱动如何进入嵌入式发行版”，但不适合当作“标准无线驱动设计”的主要范例。标准无线栈应重点学习 Linux `cfg80211`、`mac80211` 和主线 `rtw88/rtw89` 方向。

### 3.3 两条链路为什么合并到 A08

OV5640 和 8812BU 是不同外设，但它们共同覆盖驱动开发的通用模型：

- 硬件事实如何进入 device tree、Kconfig、Makefile 和 recipe。
- kernel 如何发现设备并调用 `probe()`。
- driver 如何申请 GPIO、I2C、USB、MMIO、IRQ、DMA 等资源。
- driver 如何注册到 V4L2、netdev/cfg80211、input、DMAEngine 等子系统。
- 用户态如何通过 `/dev/*`、netlink、ioctl、sysfs/debugfs、init script 消费驱动能力。
- 构建系统如何保证驱动、内核 ABI、rootfs、boot image 版本一致。

## 4. 权威资料与案例索引

以下资料用于 A08 后续学习和方案审查。表中“本项目结论”只代表当前建模推导，不代表已经完成本地实现。

| ID | 资料 | 证据等级 | 本项目结论 |
| --- | --- | --- | --- |
| R01 | Linux Driver Model: https://docs.kernel.org/driver-api/driver-model/index.html | E1 | 所有驱动迁移都应回到 device/driver/bus/class 的统一模型。 |
| R02 | Device Drivers: https://docs.kernel.org/driver-api/driver-model/driver.html | E1 | 驱动是面向一类设备的结构，不是某个应用私有函数集合。 |
| R03 | Devres managed resources: https://docs.kernel.org/driver-api/driver-model/devres.html | E1 | 新 kernel driver 应优先用 `devm_*` 降低错误路径泄漏风险。 |
| R04 | Device Tree schema: https://docs.kernel.org/devicetree/bindings/writing-schema.html | E1 | 新硬件节点应有 binding/schema 约束，至少本地文档要按 schema 思维描述属性。 |
| R05 | I2C client driver guide: https://docs.kernel.org/i2c/writing-clients.html | E1 | OV5640 规范入口应是 I2C client driver，而非用户态 `I2C_SLAVE` 长期持有。 |
| R06 | GPIO descriptor consumer API: https://docs.kernel.org/driver-api/gpio/consumer.html | E1 | 新驱动应使用 descriptor API；当前 sysfs GPIO 是历史调试路径。 |
| R07 | Pinctrl subsystem: https://docs.kernel.org/driver-api/pin-control.html | E1 | MIO/EMIO、复位脚、按键脚应通过 pinctrl/DT 声明，不应依赖运行时猜测 gpiochip base。 |
| R08 | Runtime PM: https://docs.kernel.org/power/runtime_pm.html | E1 | sensor、USB WiFi、video IP 的电源/时钟管理应明确 runtime lifecycle。 |
| R09 | DMA API: https://docs.kernel.org/core-api/dma-api.html | E1 | 若 PL frame buffer 改为驱动态，必须显式处理 DMA buffer、cache 和同步语义。 |
| R10 | DMAEngine client API: https://docs.kernel.org/driver-api/dmaengine/client.html | E1 | Xilinx frmbuf Linux 路线可能通过 DMAEngine 暴露给上层客户端。 |
| R11 | USB driver guide: https://docs.kernel.org/driver-api/usb/writing_usb_driver.html | E1 | 8812BU 首先是 USB device driver 问题，再进入无线栈。 |
| R12 | cfg80211: https://docs.kernel.org/driver-api/80211/cfg80211.html | E1 | 现代 WiFi driver 应通过 cfg80211/nl80211 与用户态一致交互。 |
| R13 | mac80211: https://docs.kernel.org/driver-api/80211/mac80211.html | E1 | 学标准无线驱动设计时，应看 mac80211/cfg80211，而不是只看 vendor driver。 |
| R14 | Camera sensor driver guide: https://docs.kernel.org/driver-api/media/camera-sensor.html | E1 | camera sensor driver 的关键是 mode、controls、power、subdev 和 firmware node。 |
| R15 | V4L2 subdev: https://docs.kernel.org/driver-api/media/v4l2-subdev.html | E1 | OV5640 进入规范 camera pipeline 时应成为 V4L2 subdev。 |
| R16 | V4L2 media controller: https://docs.kernel.org/driver-api/media/v4l2-mc.html | E1 | 自定义 PL video 链路若要标准化，需要 media graph 表达端点关系。 |
| R17 | Linux mainline OV5640 driver: https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov5640.c | E2 | 可学习 sensor driver 结构，但不能直接替换本项目 PL 链路。 |
| R18 | Linux mainline IMX219 driver: https://kernel.googlesource.com/pub/scm/linux/kernel/git/torvalds/linux/+/master/drivers/media/i2c/imx219.c | E2 | 可作为较清晰的 V4L2 sensor driver 学习样例。 |
| R19 | AMD/Xilinx PG278 v_frmbuf: https://docs.amd.com/r/2.4-English/pg278-v-frmbuf/Documentation | E1 | 本项目 frmbuf IP 与官方 PG278 v2.4 相关，驱动化必须对照 IP 版本和寄存器。 |
| R20 | Xilinx Video Framebuffer Write wiki: https://xilinx-wiki.atlassian.net/wiki/display/A/Video%20Framebuffer%20Write | E1/E2 | Linux 侧有 DMAEngine/V4L2 client 示例，但需确认当前 DT/IP 是否匹配。 |
| R21 | Xilinx embeddedsw v_frmbuf_wr: https://xilinx.github.io/embeddedsw.github.io/v_frmbuf_wr/doc/html/api/index.html | E1 | 可核对寄存器、格式枚举和裸机 layer-2 API，不等于 Linux 驱动方案。 |
| R22 | Yocto kernel development: https://docs.yoctoproject.org/5.0.14/kernel-dev/index.html | E1 | 外部 kernel module 应纳入 Yocto 构建，而不是手工拷贝 `.ko`。 |
| R23 | AMD PetaLinux building user modules: https://docs.amd.com/r/2022.2-English/ug1144-petalinux-tools-reference-guide/Building-User-Modules | E1 | 当前 8812BU recipe 方向与 PetaLinux 用户模块构建路径一致。 |
| R24 | Kernel testing overview: https://docs.kernel.org/dev-tools/testing-overview.html | E1 | 后续驱动变更应区分 KUnit、kselftest、板端集成测试和手工功能验证。 |
| R25 | Kernel selftests: https://docs.kernel.org/dev-tools/kselftest.html | E1 | 驱动调试不应只靠人工前端，应补能自动采集证据的测试。 |
| R26 | Kernel submitting patches: https://docs.kernel.org/process/submitting-patches.html | E1 | 即使本项目不向主线提交，也应学习小补丁、清晰提交、证据充分的习惯。 |
| R27 | Kernel license/SPDX: https://docs.kernel.org/process/license-rules.html | E1 | 后续新增 kernel code 必须有许可证和 SPDX，不混入来源不明代码。 |
| R28 | NVIDIA Jetson camera sensor guide: https://docs.nvidia.com/jetson/l4t/Tegra%20Linux%20Driver%20Package%20Development%20Guide/camera_sensor_prog.48.1.html | E4 | 可学习企业级 camera bring-up 文档组织、DT、controls、debug flow。 |
| R29 | morrownr 88x2bu upstream: https://github.com/morrownr/88x2bu-20210702 | E3 | 当前 WiFi 驱动来源；只能作为 vendor driver 治理依据。 |
| R30 | Linux mainline rtw8822bu source: https://codebrowser.dev/linux/linux/drivers/net/wireless/realtek/rtw88/rtw8822bu.c.html | E2 | 可学习主线 Realtek USB WiFi 的组织方式，但不直接证明 PetaLinux 5.15 可替换。 |
| R31 | Bootlin Linux kernel driver training 2024: https://bootlin.com/doc/training/sessions/ti-US.linux-kernel.jun2024/ | E4 | 学习路线应从 kernel/userspace 边界、driver model、bus、subsystem、debug 逐层推进。 |
| R32 | Linux Foundation LFD430: https://training.linuxfoundation.org/training/developing-linux-device-drivers/ | E4 | 驱动学习不局限外设，还包括机制/策略分离、调试、模块、power management。 |

## 5. 通用驱动开发知识架构

### 5.1 硬件层

必须先回答：

- 外设连接在哪条总线上：I2C、SPI、USB、PCIe、platform/MMIO、GPIO-only、PL AXI。
- 需要哪些时钟、复位、电源、pinmux、regulator。
- 是否产生 IRQ。
- 是否执行 DMA。
- 数据格式和时序由谁定义。

本项目映射：

- OV5640：I2C 控制面 + DVP 数据面 + GPIO power/reset。
- PL video：AXI4-Stream video + MMIO control + AXI HP DDR write。
- 8812BU：USB 数据面 + WiFi 射频/802.11 协议 + netdev 数据面。

### 5.2 设备发现与绑定层

常见绑定方式：

- platform device：通常来自 device tree，适合 SoC/PL MMIO IP。
- I2C/SPI client：由总线和 DT/board info 创建，driver 通过 address/compatible 匹配。
- USB device：由 USB 枚举和 id table 匹配。
- PCI/PCIe device：由 PCI 枚举和 id table 匹配。
- pseudo/input/gpio key：由 DT 声明后使用现成子系统驱动。

本项目映射：

- OV5640 应从 `/dev/i2c-2` 用户态访问，逐步转向 I2C client。
- A08 当时建议 `v_frmbuf_wr` 从用户态 MMIO 逐步评估 platform/DMAEngine/V4L2；后续已迁移到 V4L2 capture。
- 8812BU 已由 USB 枚举后进入 kernel module。
- PS_KEY1 已通过 `gpio-keys` 走 input 子系统，这是当前最规范的驱动化示例。

### 5.3 内核资源所有权层

规范驱动应在 `probe()` 中获取资源，在 `remove()` 或 devres 中释放资源：

- `devm_kzalloc()` 管理私有状态。
- `devm_gpiod_get()` 获取 GPIO。
- `devm_regulator_get()` 获取电源。
- `devm_clk_get()` 获取时钟。
- `devm_ioremap_resource()` 获取 MMIO。
- `devm_request_irq()` 获取中断。
- `pm_runtime_enable()` 建立 runtime PM。

当前 OV5640 HAL 直接在用户态持有 GPIO/I2C/MMIO，不具备 kernel resource ownership。后续驱动化时，第一目标不是“功能更多”，而是把资源所有权收回到内核。

### 5.4 子系统接口层

驱动不应随意自造用户态 ABI，优先接入成熟子系统：

| 子系统 | 适用内容 | 本项目状态 |
| --- | --- | --- |
| V4L2/media | sensor、bridge、video capture、controls、media graph | A08 当时未接入；后续正式程序已采用 V4L2 control/capture。 |
| DMAEngine | video DMA、AXI DMA、frmbuf 类 DMA 通道 | A08 当时未接入；后续正式程序不再直接控制 frmbuf MMIO。 |
| cfg80211/mac80211/netdev | WiFi 设备、扫描、认证协同、网络数据面 | 8812BU vendor driver 已能提供 wlan0，但不作为标准设计范例。 |
| input/gpio-keys | 低速按键、消抖、input event | PS_KEY1 已接入，是当前最好的本地示例。 |
| gpiolib/pinctrl | GPIO/pinmux 管理 | PS_KEY1 已使用；OV5640 power/reset 尚未使用。 |
| debugfs/sysfs | 调试和有限状态导出 | 后续可用于诊断，但不得成为复杂业务协议。 |

### 5.5 用户态边界层

用户态应看到“能力接口”，而不是直接拥有硬件细节：

- WiFi 用户态：`wpa_supplicant`、`wpa_cli`、`udhcpc`、`ip/ifconfig`。
- Camera 用户态理想路径：`/dev/videoX`、`media-ctl`、`v4l2-ctl`、V4L2 mmap/userptr/dmabuf。
- 当前 camera 用户态路径：`cam_system` 直接初始化 sensor/frmbuf 并读固定 DDR 地址。

当前 camera 方案能工作，但用户态承担了过多底层职责。后续驱动化的收益主要是规范性、可观测性、可替换性和学习价值，不应承诺立即提升性能。

### 5.6 构建与发布层

驱动开发不是只写 `.c` 文件，还必须闭环：

- kernel config：是否启用对应 subsystem。
- Kconfig/Makefile：驱动如何参与构建。
- device tree：硬件实例、compatible、资源属性。
- Yocto/PetaLinux recipe：源码获取、固定版本、许可证、构建、安装。
- rootfs/image install：模块、配置、init script 是否进入镜像。
- OTA/rollback：失败后能否恢复可启动状态。

当前 8812BU 已进入 recipe；OV5640/PL video 尚未进入 kernel/DT ownership。

## 6. 当前生命周期流程

### 6.1 启动到 WiFi 可达

1. U-Boot 加载 `BOOT.BIN`/`boot.scr`/`image.ub`。
2. Linux kernel 解析 device tree。
3. rootfs 进入 SysV init。
4. 8812BU 模块通过 Yocto autoload 或 `S50wifi` fallback 加载。
5. USB subsystem 枚举 WiFi dongle，驱动创建 `wlan0`。
6. `S50wifi` 读取 WiFi 配置，生成临时 `wpa_supplicant` 配置。
7. `wpa_supplicant` 完成认证和关联。
8. `udhcpc` 获取 IP。
9. Dropbear SSH 和前端连接具备网络路径。

### 6.2 按键到 cam_system 可用

1. device tree 中 `gpio-keys` 节点被 kernel 解析。
2. kernel `gpio-keys` 驱动注册 input event。
3. `cam-launcher` 监听 `KEY_PROG1` press。
4. 按下 PS_KEY1 后，如果 `cam_system` 未运行，launcher 启动 `/usr/bin/cam_system`。
5. `cam_system` 初始化状态、OV5640、frmbuf、JPEG、YOLO、HTTP/WS。
6. 前端连接 `8080`，通过 REST/MJPEG/WS 消费业务接口。

### 6.3 cam_system 内部 camera 初始化

1. `ov5640_init()` export GPIO，配置 PDN/RST。
2. 打开 `/dev/i2c-2`，设置 OV5640 I2C address。
3. 读取 chip id。
4. 写固定寄存器表。
5. `frmbuf_init()` MMIO 映射 reset GPIO 和 v_frmbuf_wr。
6. 配置 width/height/stride/format/address。
7. capture 线程启动单帧采集、等待 `ap_done`、发布帧。

该流程说明：当前 camera 的关键控制面都在用户态。若后续进入驱动化，必须逐步迁移，不能一次性替换。

## 7. A08 后续决策点

### D01：OV5640 是否先只做文档对照，不写驱动

建议：先做。

理由：

- 当前功能已可用。
- PL video 节点在 DT 中 disabled，说明 kernel 暂不拥有该链路。
- 主线 OV5640 driver 不能覆盖本项目自定义 PL DVP/format/frmbuf 细节。

进入下一步前需要补证：

- XSA 中 `ov5640_to_lcd`、`v_frmbuf_wr`、I2C、reset GPIO 的完整连接。
- 当前 PetaLinux kernel 是否启用 V4L2/media/DMAEngine/Xilinx video driver 相关配置。
- `/proc/device-tree` 中实际生成节点和当前 `system-user.dtsi` 是否一致。

### D02：OV5640 第一段驱动化从哪里开始

可选方案：

| 方案 | 内容 | 优点 | 风险 |
| --- | --- | --- | --- |
| A | 保留用户态 HAL，只补取证和文档 | 风险最低 | 学习驱动写作不够深入。 |
| B | 写最小 diagnostic platform/I2C driver，只 probe、读 chip id、控制 reset，不接管视频流 | 学习 `probe()`、DT、GPIO descriptor、I2C client，风险可控 | 与现有用户态 I2C 访问可能冲突，必须避免同时运行。 |
| C | 迁移为完整 V4L2 sensor subdev | 最规范 | 需要 media graph、bridge/frmbuf 配合，风险高。 |
| D | 迁移 PL frmbuf 到 DMAEngine/V4L2 client | 对 video pipeline 最规范 | 依赖 Xilinx driver、DT endpoint、buffer 模型，验证复杂。 |

建议 A08 后续先执行 A+B 的只读/诊断路线，不直接进入 C/D。

### D03：8812BU 是否迁移到主线 rtw88

当前不建议。

理由：

- 当前 PetaLinux kernel 为 5.15，项目已基于 out-of-tree 8812BU 验证可用。
- 上游和社区资料显示较新 kernel 有更标准的 in-kernel `rtw88` 方向，但这不等于当前 5.15 可稳定替换。
- WiFi 是 SSH/OTA/前端调试通道，稳定性优先于“形式上更主线”。

后续只在更换 kernel 或量产系统设计时重新评估。

### D04：WiFi 初始化是否引入 NetworkManager/ConnMan/systemd-networkd

当前不建议。

理由：

- 当前 RAM rootfs + 单 STA + 固定局域网调试路径，不需要完整网络管理守护进程。
- 当前 `wpa_supplicant + udhcpc + init.d` 已满足本地闭环。
- 引入更大框架会增加 rootfs、启动时序和调试变量，不符合当前项目约束。

### D05：是否把驱动测试与 YOLO 性能测试合并

建议只合并取证，不合并性能结论。

可以同轮采集：

- `/proc/interrupts`
- `lsmod`
- `modinfo 8812bu`
- `wpa_cli status`
- `dmesg` 驱动日志
- YOLO 单帧耗时

但是否存在 IRQ/USB/WiFi 对 YOLO 的显著影响，必须单独通过 A/B 测试证明，不能由架构推断替代。

## 8. A08 板端取证矩阵

后续每次进入驱动相关上板测试，建议先采集以下证据。单次上板时间仍受 20 分钟上限约束。

### 8.1 系统和内核

```sh
uname -a
cat /proc/cmdline
cat /proc/version
cat /proc/mounts
```

目标：确认 kernel 版本、启动参数、RAM rootfs/SD boot 分区状态。

### 8.2 device tree 和 pinctrl/GPIO

```sh
find /proc/device-tree -maxdepth 3 -type d | sort | head -n 120
grep -R . /sys/kernel/debug/gpio 2>/dev/null || true
cat /proc/bus/input/devices
```

目标：确认 `gpio-keys`、PS_KEY1、GPIO ownership、未来 OV5640 GPIO 迁移条件。

### 8.3 8812BU/WiFi

```sh
lsmod
modinfo /lib/modules/$(uname -r)/extra/8812bu.ko
cat /proc/modules | grep 8812
dmesg | grep -Ei '8812|88x2|rtl|wlan|cfg80211|usb' | tail -n 120
ip addr show wlan0
wpa_cli -i wlan0 status
```

目标：确认模块版本、加载路径、日志噪声、wlan0 状态、认证状态。

### 8.4 IRQ 和 core 分布

```sh
cat /proc/interrupts
cat /proc/softirqs
ps -eo pid,psr,comm,args
```

目标：观察 USB/WiFi/MMC/I2C/PL 中断是否集中到某个 core，作为后续性能讨论证据。

### 8.5 OV5640/PL video

```sh
i2cdetect -y 2 2>/dev/null || true
dmesg | grep -Ei 'i2c|ov5640|video|v4l2|media|frmbuf|dma|gpio' | tail -n 160
```

目标：确认当前 I2C bus 可见性、kernel 是否已有相关 video/media driver probe 日志。

注意：如果 `cam_system` 正在运行，不应同时用外部工具写 OV5640 寄存器，避免和用户态 HAL 冲突。

## 9. 后续实现规则

驱动相关变更必须遵守以下规则：

- 每次只能迁移一个 ownership 边界，例如 GPIO ownership、I2C probe、frmbuf ownership，不允许一次性替换整条链路。
- 每个新增 kernel/DT/recipe 变更必须有回滚路径。
- 每个新增驱动文件必须有 SPDX license identifier。
- 不使用来源不明的寄存器表、驱动片段或论坛代码直接进入构建路径。
- 任何改动如果可能影响 SSH/WiFi/OTA，必须先证明恢复路径。
- A08 当时要求保持旧路径可回退；当前正式程序不再内置旧实现，恢复依赖 git/OTA/镜像回滚。
- 任何“性能提升”结论必须来自板端计时，不能只来自经验判断。

## 10. A08-0 结论

当前项目已经具备进入驱动学习阶段的基础：

- PS_KEY1 已是一个合格的 `gpio-keys`/input 子系统学习样例。
- 8812BU 已完成源码 recipe 化，适合学习第三方 kernel module 的 Yocto/PetaLinux 集成。
- OV5640/PL video 仍是用户态 HAL，适合学习如何从可运行调试态逐步迁移到 Linux driver model。

当前仍不具备直接迁移完整 camera driver 的证据：

- PL video pipeline 的 DT endpoint、media graph、Xilinx driver 匹配关系尚未证明。
- 当前 `v_frmbuf_wr` 节点 disabled，kernel 不拥有该 IP。
- 主线 OV5640 driver 不能单独覆盖本项目 PL 采集链路。

因此，A08 当时建议先以“取证 + 最小诊断驱动 + 保留旧路径可回退”为原则推进。该建议已完成历史使命；当前正式程序已收敛为 V4L2-only，旧实现仅通过 git 历史追溯。
