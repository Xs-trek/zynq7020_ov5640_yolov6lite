# cam_system A03 程序资源与三方组件治理清单

版本：0.2
状态：A03 资源厘清与三方 recipe 基线；A08 作为历史资源治理基线引用
检索日期：2026-06-03

## 1. 目标

本文档用于约束 `cam_system` 的全部程序资源，包括第一方代码、参考编写代码、三方源码、预构建库、外部构建库、Yocto/rootfs 包、内核外驱动、模型资产和外部协议实现。

A03 的目标不是一次性把所有依赖复杂化，而是做到：

- 每个进入程序构建、链接、部署或运行路径的资源都有明确归属。
- 每个三方或外部资源都有来源、版本、许可证、构建方式、部署方式和 hash 记录。
- 无法证明的来源或版本必须显式列为待确认项，不能用经验判断替代证据。
- 后续 recipe 化、SBOM、OTA、模型替换或驱动迁移，都以本文档为资源基线。

## 2. 参考规范

本项目采用下列规范中的可落地子集：

| 来源 | 本项目采纳点 |
| --- | --- |
| Yocto recipe 与 license 文档 | recipe 必须记录 `SRC_URI`、`SRCREV` 或下载校验、`LICENSE`、`LIC_FILES_CHKSUM`；镜像构建应能生成 license manifest 或 SPDX SBOM。 |
| NIST SSDF SP 800-218 | 三方组件应评估来源、版本、维护状态、风险和验证证据。 |
| SPDX / SBOM 实践 | 使用统一清单记录组件名称、版本、许可证、供应方、hash 和关系。 |
| OpenSSF SLSA | 逐步提高依赖和构建产物的 provenance，可追溯到固定源码和固定构建流程。 |
| GitHub supply-chain security | 对 GitHub 下载资源采用固定版本、固定 commit、依赖清单和漏洞/维护状态跟踪。 |

参考链接：

- https://docs.yoctoproject.org/5.1.1/dev-manual/new-recipe.html
- https://docs.yoctoproject.org/dev/dev-manual/licenses.html
- https://docs.yoctoproject.org/dev-manual/sbom.html
- https://csrc.nist.gov/pubs/sp/800/218/final
- https://spdx.dev/about/overview/
- https://slsa.dev/spec/latest/
- https://docs.github.com/en/code-security/concepts/supply-chain-security/about-supply-chain-security

## 3. 资源分类

| 类别 | 名称 | 定义 | 当前处理策略 |
| --- | --- | --- | --- |
| R0 | 第一方代码 | 本项目直接编写并维护的源码、脚本、文档和 recipe 模板。 | 按本项目代码规范和 Git 变更管理。 |
| R1 | 参考编写实现 | 参考公开协议、芯片手册、示例或思路后自行编写的实现。 | 记录参考边界；若没有复制源码，不挂靠上游许可证。 |
| R2 | 内嵌三方源码 | 三方源码直接放入本仓并参与编译。 | 保留许可证头，记录版本、来源、hash。 |
| R3 | 内嵌预构建库 | 三方头文件和静态库直接放入本仓。 | 记录版本、构建来源、间接依赖和 hash；后续优先 recipe 化或重建。 |
| R4 | 外部构建库 | 本仓通过 CMake 路径引用工作空间外的安装产物。 | 记录外部路径、版本、构建 flags、hash；后续纳入 recipe 或固定源码包。 |
| R5 | 系统包依赖 | 由 PetaLinux/Yocto rootfs 提供的运行库或工具。 | 通过 `RDEPENDS` 和 rootfs manifest 管理。 |
| R6 | 内核外模块 | 独立源码 recipe 编译出的 kernel module。 | 固定 `SRCREV`，跟随 kernel ABI 重编译。 |
| R7 | 模型资产 | `.param`、`.bin` 等 AI 模型产物。 | 按 model artifact 管理，记录来源、输入输出、精度基线和 hash。 |
| R8 | 外部协议/标准 | HTTP、WebSocket、JSON、V4L/AXI/Xilinx IP 等协议或接口语义。 | 记录协议裁剪边界；不等同于引入三方代码。 |

## 4. Provenance 等级

| 等级 | 含义 | A03 接受性 |
| --- | --- | --- |
| P0 | 只有文件存在，无法说明来源、版本或许可证。 | 不接受，必须补证据或移除。 |
| P1 | 能说明本地版本、hash 和用途，但无法证明精确上游 commit 或构建流程。 | 可用于当前调试阶段，必须列入待确认。 |
| P2 | 能说明上游 URL、版本或 tag、许可证、本地 hash。 | 可作为 A03 基线。 |
| P3 | 有 Yocto recipe 或等价构建脚本，固定 `SRCREV` 或下载 hash，并能进入 package/rootfs manifest。 | 生产化方向。 |

A03 第 1 段以 P2 为目标；对构建复杂或当前不宜改动的组件，允许暂时停在 P1，但必须登记原因和后续动作。

## 5. 当前资源清单

### 5.1 第一方与参考编写资源

| 资源 | 类别 | 路径 | 状态 | 说明 |
| --- | --- | --- | --- | --- |
| cam_system 应用源码 | R0 | `src/`、`include/cam_system/` | P2 | 本项目维护。 |
| 构建和上板脚本 | R0 | `scripts/` | P2 | 本项目维护。 |
| PetaLinux recipe 模板 | R0 | `packaging/petalinux/recipes-apps/cam-system` | P3 | 模板已可同步到真实 PetaLinux 工程。 |
| `wifi-init` | R0 | `packaging/petalinux/recipes-apps/wifi-init` | P3 | 本项目脚本，通过 rootfs recipe 部署。 |
| `ssh-access-dev` | R0 | `packaging/petalinux/recipes-apps/ssh-access-dev` | P3 | 调试态 SSH 账号和 key 管理，禁止用于量产默认开放策略。 |
| HTTP/REST/MJPEG/WebSocket 实现 | R1/R8 | `src/network/`、`docs/WEBSOCKET_PROTOCOL.md` | P2 | 第一方实现。WebSocket 是受限协议栈，只服务当前前端，不声明通用 RFC 6455 完整兼容。 |
| OV5640/V4L2 capture HAL | R1 | `src/platform/` | P1 | 当前按板级硬件和既有调试结果编写；OV5640 控制面由项目内核模块持有，采集数据面通过 V4L2 capture 进入用户态。 |
| tracker/光流控制逻辑 | R0/R1 | `src/control/`、`src/vision/optical_flow.cpp` | P2 | tracker 为第一方实现；光流依赖 OpenCV API。 |

### 5.2 内嵌三方源码

| 组件 | 类别 | 本地路径 | 上游 | 版本证据 | 许可证 | 状态 |
| --- | --- | --- | --- | --- | --- | --- |
| cJSON | R2 | `third_party/cjson/cJSON.c`、`third_party/cjson/cJSON.h` | https://github.com/DaveGamble/cJSON | 本地头文件定义 `1.7.19` | MIT | P2 |

本地 hash：

| 文件 | sha256 |
| --- | --- |
| `third_party/cjson/cJSON.c` | `5120a690423c3c7964ed2acd3f3249ed3eecd13b7bf0d6666e3d68331aca193b` |
| `third_party/cjson/cJSON.h` | `25b0145150d500498e4d209cec69c18c42cf818bffcc54690be3b895a2a16dee` |
| `third_party/cjson/README.third_party` | `fda13e63f9b0e93b694bfca63caef923ca2c849f4b3c59cf6e59c998f3061566` |

处理策略：

- 继续以 vendored single-file 方式维护，原因是体积小、接口稳定、编译成本低。
- 后续更新时必须记录上游 tag 或 commit，并重新计算 hash。
- 禁止对 cJSON 源码做静默本地修改；若修改，必须在本文档标记为 fork。

### 5.3 内嵌预构建 OpenCV

| 组件 | 类别 | 本地路径 | 上游 | 版本证据 | 许可证 | 状态 |
| --- | --- | --- | --- | --- | --- | --- |
| OpenCV core/imgproc/video | R3/R4 | `packaging/petalinux/recipes-support/cam-opencv/cam-opencv_4.13.0.bb`；历史裁剪包 `third_party/opencv` | https://github.com/opencv/opencv | recipe 固定 `SRCREV=fe38fc608f6acb8b68953438a62305d8318f4fcd`，对应上游 tag `4.13.0`；仓内静态库 hash 与 `$HOME/opencv/build_arm` 一致 | OpenCV 4.5.0 及以上为 Apache-2.0 | P3 recipe 已建立并通过组件级构建验证；PetaLinux 应用 recipe 已切换到 recipe sysroot，后续快速回归未发现功能问题 |
| OpenCV 3rdparty openjp2 | R3 | `third_party/opencv/lib/opencv4/3rdparty/liblibopenjp2.a` | `$HOME/opencv/3rdparty/openjpeg` | 仓内静态库 hash 与 `$HOME/opencv/build_arm/3rdparty/lib/liblibopenjp2.a` 一致 | BSD-2-Clause | P2 |
| OpenCV 3rdparty tegra_hal | R3 | `third_party/opencv/lib/opencv4/3rdparty/libtegra_hal.a` | `$HOME/opencv/hal/carotene/hal/tegra_hal.hpp` | 仓内静态库 hash 与 `$HOME/opencv/build_arm/3rdparty/lib/libtegra_hal.a` 一致 | BSD-3-Clause | P2 |

本地 hash：

| 文件 | sha256 |
| --- | --- |
| `third_party/opencv/lib/libopencv_core.a` | `e69ebd8832c94c253d61c1caf70112e2deb70955610e2124749bfd6311809f00` |
| `third_party/opencv/lib/libopencv_imgproc.a` | `e9eba233bd11b8b2cd3f264ea8ed8fb5b4c57ac96c51d7e5a1d7e59fbcf95dc2` |
| `third_party/opencv/lib/libopencv_video.a` | `ddcf4495313fd757269d0e00f475a93f46939a2bd3b0fcbb04f0110a6777406e` |
| `third_party/opencv/lib/opencv4/3rdparty/liblibopenjp2.a` | `9bbee7be58561e9d50f866ceaa26208540c029e91912908447958ee922fbde3b` |
| `third_party/opencv/lib/opencv4/3rdparty/libtegra_hal.a` | `c66756e79e5871ebd08901f50e9025ddb36fbc16ea6ba01b93644343b73447ff` |
| PetaLinux recipe sysroot `cam-opencv/usr/lib/libopencv_core.a` | `5f837677e3ad0836629564c1b3f35599050e1c7d3779b1908a03acf60de8e74a`，官方 tag `4.13.0` recipe 产物 |
| PetaLinux recipe sysroot `cam-opencv/usr/lib/libopencv_imgproc.a` | `61767f16b99a7c56408aa250448e2a73f84488f415ddb38988e42e79630c5459`，官方 tag `4.13.0` recipe 产物 |
| PetaLinux recipe sysroot `cam-opencv/usr/lib/libopencv_video.a` | `19ea48b88d2ee57d546db64e181105f60ca7051147cae26ede64d2df39ab9b59`，官方 tag `4.13.0` recipe 产物 |
| PetaLinux recipe sysroot `cam-opencv/usr/lib/opencv4/3rdparty/libtegra_hal.a` | `19908f773c43d291c7b43c5c762dbc55fc7527ef2d4935f47acd8ad651741515`，官方 tag `4.13.0` recipe 产物 |

处理策略：

- 本地 SDK 调试构建默认仍使用历史 `third_party/opencv` 路径，作为已上板基线和回滚对照。
- PetaLinux `cam-system` 源码 recipe 使用 `CAM_SYSTEM_DEPENDENCY_MODE=YOCTO`，链接 `cam-opencv` recipe 产物；该路径必须通过本次上板回归后才能视为稳定发布路径。
- PetaLinux 基础层自带 `opencv_4.5.2.bb`，且存在多个 `opencv_*.bbappend`。A03 使用项目本地包名 `cam-opencv` 固定 OpenCV 4.13.0，避免全局覆盖系统 OpenCV 并被现有 bbappend 注入 dnn/python/ROS 等无关配置。
- `cam-opencv` recipe 只构建 `core,imgproc,video` 静态库，并关闭图像编解码、GUI、videoio、Python/Java、OpenCL、OpenMP、TBB 等当前程序不用的功能。
- `cam-opencv` 已通过 `petalinux-build -c cam-opencv -x cleansstate && petalinux-build -c cam-opencv` 组件级验证；`do_fetch`、`do_populate_lic`、`do_configure`、`do_compile`、`do_package_qa` 通过。
- `cam-opencv` recipe 产物不包含历史裁剪包里的 `liblibopenjp2.a`，因为 recipe 关闭了 OpenJPEG；当前 optical flow 不使用图像编解码，A03 上板需确认 tracking 功能和性能不退化。
- 预构建静态库中包含 OpenCV 3rdparty 产物，因此许可证记录不能只写 `OpenCV Apache-2.0`。
- `third_party/opencv/README.third_party` 已作为目录局部元数据，sha256 为 `7ddba5d5394c25043b075cc2481fbdbf1c79a488a61ae83090daf30043f361dc`。

### 5.4 外部构建后链接的库

| 组件 | 类别 | 本地路径 | 上游 | 本地版本证据 | 许可证 | 状态 |
| --- | --- | --- | --- | --- | --- | --- |
| ncnn | R4 | `packaging/petalinux/recipes-ai/ncnn/ncnn_20260526.bb`；历史外部路径 `$HOME/ncnn/build-arm/install` | https://github.com/Tencent/ncnn | recipe 固定 `SRCREV=e54f7b1f88434e1d844ea0551b880a1cfb079ce1`，对应上游 tag `20260526`；历史外部构建显示 `1.0.20260313`，但该版本号由 CMake 构建日期生成，不能证明源码 commit | BSD-3-Clause，内含 Zlib/BSD-2-Clause/BSD-3-Clause 组件 | P3 recipe 已建立并通过组件级构建验证；PetaLinux 应用 recipe 已切换到 recipe sysroot，后续快速回归未发现功能问题 |
| libjpeg-turbo | R4/R5 | `packaging/petalinux/recipes-graphics/jpeg/libjpeg-turbo_3.1.4.bb` | https://github.com/libjpeg-turbo/libjpeg-turbo | `SRCREV=e352b02f794f701407b39af08576035ba3360d60`，对应上游 tag `3.1.4` | IJG + BSD-3-Clause + Zlib | P3 recipe 已建立并通过组件级构建验证 |

本地 hash：

| 文件 | sha256 |
| --- | --- |
| `$HOME/ncnn/build-arm/install/lib/libncnn.a` | `97fb7545fe1a65cd90ec047409edb84d827317e2e7690921b4a6e0d9a460cc82` |
| PetaLinux recipe sysroot `ncnn/usr/lib/libncnn.a` | `d6a394feaa04c542960808c31417530bf08fbd36bb78b3551663a276d471d262`，官方 tag `20260526` recipe 产物 |
| `$HOME/workdir/libjpeg-turbo/build-arm/install/lib/libturbojpeg.so` | `58a6fac28491e79132ebc51285d4a8492f4734dfda9ecf1684eff9e5a6d425cc`，历史外部构建产物 |
| `$HOME/workdir/libjpeg-turbo/build-arm/install/lib/libturbojpeg.so.0.4.0` | `58a6fac28491e79132ebc51285d4a8492f4734dfda9ecf1684eff9e5a6d425cc`，历史外部构建产物 |

已知构建证据：

| 组件 | 本地构建参数证据 |
| --- | --- |
| ncnn | `Release`，`-O3 -DNDEBUG`，`-march=armv7-a -mfloat-abi=hard -mfpu=neon`，`NCNN_THREADS=ON`，`NCNN_BF16=ON`，`NCNN_VULKAN=OFF`。 |
| libjpeg-turbo | `WITH_SIMD=ON`，`ENABLE_SHARED=ON`，目标 sysroot 为 PetaLinux Cortex-A9 hard-float SDK。 |

处理策略：

- 本地 SDK 调试构建仍保留 ncnn 外部构建路径，因为该产物已经参与历史上板功能测试。
- PetaLinux `cam-system` 源码 recipe 已切换到 ncnn recipe sysroot 产物，必须通过本次上板回归确认模型加载、输出解码和耗时未出现不可接受退化。
- ncnn 已建立官方 tag `20260526` recipe，并通过 `petalinux-build -c ncnn -x cleansstate && petalinux-build -c ncnn` 组件级验证；`do_fetch`、`do_populate_lic`、`do_configure`、`do_compile`、`do_package_qa` 通过。
- 当前不能证明 ncnn recipe 产物与历史 `1.0.20260313` 二进制等价，因此替换应用链接必须经过构建、模型加载、上板推理和耗时回归。
- libjpeg-turbo 已建立本地 PetaLinux recipe，并通过 `petalinux-build -c libjpeg-turbo -x cleansstate && petalinux-build -c libjpeg-turbo` 组件级验证；`do_patch`、`do_package_qa` 通过。
- 对 libjpeg-turbo 还需通过 A03 上板验证应用链接版本和 rootfs 运行时版本一致。

### 5.5 Yocto/rootfs 系统包依赖

| 组件 | 类别 | 入口 | 作用 | 状态 |
| --- | --- | --- | --- | --- |
| `libturbojpeg` | R5 | `cam-system.bb` 的 `RDEPENDS` | JPEG runtime。 | P3，但需与链接版本一致性验证。 |
| `libcrypto` / OpenSSL | R5 | `cam-system.bb` 的 `RDEPENDS` 和 `target_link_libraries` | WebSocket SHA1/base64 辅助、crypto runtime。 | P3 |
| `zlib` | R5 | `cam-system.bb` 的 `RDEPENDS` | OpenCV/libjpeg-turbo 间接运行依赖。 | P3 |
| `libstdc++`、`libgcc` | R5 | `cam-system.bb` 的 `RDEPENDS` | C++/GCC runtime。 | P3 |
| `wpa-supplicant`、`wpa-supplicant-cli`、`wpa-supplicant-passphrase` | R5 | `wifi-init.bb` 的 `RDEPENDS` | WiFi 认证和配置生成。 | P3 |
| `dropbear` | R5 | `ssh-access-dev.bb` 的 `RDEPENDS` | 调试态 SSH。 | P3 |

处理策略：

- 系统包不复制到本仓，依赖闭环由 PetaLinux/Yocto rootfs 管理。
- 每次变更 `RDEPENDS` 后，需要通过 rootfs manifest 或板端 `ldd`/文件存在性验证。

### 5.6 8812BU 内核外驱动

| 组件 | 类别 | 本地 recipe | 上游 | 固定版本 | 许可证 | 状态 |
| --- | --- | --- | --- | --- | --- | --- |
| 8812bu kernel module | R6 | `packaging/petalinux/recipes-kernel/8812bu/8812bu_git.bb` | https://github.com/morrownr/88x2bu-20210702 | `SRCREV=fecac340fb117eb979f4bb6d28e29730384c382b`，`PV=5.13.1+git${SRCPV}` | GPL-2.0-only | P3 |

当前 recipe 已符合 A03 的基本方向：

- 使用源码 GitHub 仓库，不再使用手工下载 `.ko`。
- 固定 `SRCREV`，避免每次构建拉取不可预期版本。
- 使用 `inherit module` 跟随 PetaLinux kernel ABI 构建。
- 使用 `KERNEL_MODULE_AUTOLOAD` / `KERNEL_MODULE_PROBECONF` 声明模块自动加载和运行参数；`wifi-init` 保留 `modprobe`/`insmod` 兜底以适配当前 PetaLinux 调试态。
- 通过 `LIC_FILES_CHKSUM` 绑定许可证文件。

风险和后续动作：

- 上游说明 kernel 6.12 以后已有更标准的 in-kernel `rtw88` 方向，但当前 PetaLinux kernel 为 5.15，因此继续使用该 out-of-tree 驱动是现实选择。
- 后续若更换 kernel，需要重新评估是否迁移到内核内驱动。
- 驱动配置、本地化边界、调试态和量产态差异记录在 `docs/WIFI_DRIVER_WORKFLOW.md`。

### 5.7 YOLOv6Lite-S 模型资产

| 资产 | 类别 | 本地路径 | 上游/来源 | 本地证据 | 许可证 | 状态 |
| --- | --- | --- | --- | --- | --- | --- |
| `yolov6lite_s.ncnn.param` | R7 | `assets/models/yolov6lite_s/yolov6lite_s.ncnn.param` | 用户声明来自 Meituan YOLOv6Lite-S；官方 YOLOv6 release `0.4.0` 存在 `yolov6lite_s.pt` | 固定 `256x192` 输入，COCO 80 类，NCNN param | YOLOv6 仓库为 GPL-3.0；权重/导出物是否有单独授权未证明 | P1+ |
| `yolov6lite_s.ncnn.bin` | R7 | `assets/models/yolov6lite_s/yolov6lite_s.ncnn.bin` | 用户声明由 Meituan YOLOv6Lite-S `.pt` 转换；本地缺少转换命令和中间产物 | 与 `.param` 成对使用 | YOLOv6 仓库为 GPL-3.0；权重/导出物是否有单独授权未证明 | P1+ |

本地 hash：

| 文件 | sha256 |
| --- | --- |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.param` | `05af6a2006eed92fc0996f72bfd42d82c903513e88530fd697580e840306c026` |
| `assets/models/yolov6lite_s/yolov6lite_s.ncnn.bin` | `8f825ded3d1e7d06f70734a3e2f91df039f60a2f81538f4b76818d765f9548aa` |
| 官方 YOLOv6 release `0.4.0` 的 `yolov6lite_s.pt` | `6e424f1defd72c6478e0b630977edcc2844d4e601aa96a3e2960a8a9dee1682d`，大小 `1487033` bytes |

当前代码契约：

- 输入宽高：`YOLO_INPUT_W=256`，`YOLO_INPUT_H=192`。
- 输入像素：代码使用 `ncnn::Mat::PIXEL_RGB2BGR`，等价于从 PS 侧 RGB888 帧转为模型 BGR 输入。
- 类别数：`YOLO_NUM_CLASSES=80`。
- 部署路径：`/usr/share/cam-system/models/yolov6lite_s/`。

处理策略：

- 模型文件不按普通库处理，必须单独保留 model artifact 记录。
- 当前模型资产说明记录在 `docs/MODEL_ASSETS.md`。
- A03 已补齐官方 release `0.4.0` 中 `yolov6lite_s.pt` 的 URL、tag commit、大小和 sha256。
- 当前仍缺少 `.pt -> NCNN` 的实际转换命令、中间产物、手工修改记录、`256x192` 正式精度基线和权重/导出物再分发授权说明。
- 当前用户已提供历史量化/验证数据，但本模型作为 PS 路径使用的 S 模型仍缺少完整 model card，不能视为量产级模型资产。

## 6. 当前合规性结论

| 领域 | 结论 |
| --- | --- |
| 第一方代码结构 | 基本符合当前 A 阶段要求。 |
| PetaLinux package 闭环 | `cam-system` 已改为源码 recipe 构建；`wifi-init`、`ssh-access-dev`、`8812bu` 已进入 recipe 管理。 |
| 三方源码治理 | cJSON 基本合格。 |
| 预构建库治理 | OpenCV 已补齐本地源码、构建目录、hash 和间接许可证证据，并建立 `cam-opencv` recipe 且完成组件级构建验证；PetaLinux 应用 recipe 已切换，后续阶段已完成快速回归。 |
| 外部构建库治理 | ncnn、libjpeg-turbo 已建立 recipe 并完成组件级构建验证；PetaLinux 应用 recipe 已切换，后续阶段已完成快速回归。 |
| 驱动治理 | 8812bu recipe 方向合格，配置和维护策略已纳入 `docs/WIFI_DRIVER_WORKFLOW.md`。 |
| 模型治理 | 模型文件按运行输入资产管理；更完整的模型来源、导出链和精度治理属于独立资产任务，不作为当前运行代码冻结阻塞项。 |

## 7. A03 待确认项

| ID | 项目 | 当前证据 | 需要用户确认或后续补证 |
| --- | --- | --- | --- |
| A03-Q001 | ncnn 来源 | 本地版本 `1.0.20260313`，但 `$HOME/ncnn` 当前不是 Git 工作树；上游 tags 未发现 `20260313`，且 ncnn CMake 默认用构建日期生成版本号。 | 已选择官方 tag `20260526` 建立 recipe 并完成组件级构建验证；PetaLinux 应用 recipe 已切换，后续快速回归未发现功能问题。 |
| A03-Q002 | OpenCV 治理深度 | 已追溯到 `$HOME/opencv` HEAD `fe38fc608f6acb8b68953438a62305d8318f4fcd` 和 `build_arm` 产物；已建立 `cam-opencv_4.13.0.bb` 并完成组件级构建验证。 | PetaLinux 应用 recipe 已切换，后续快速回归未发现功能问题。 |
| A03-Q003 | OpenCV 构建裁剪 | 当前构建目录启用了多个本程序未直接使用的模块，但仓内只复制了 core/imgproc/video 及必要 3rdparty 静态库；`cam-opencv` recipe 仅构建 core/imgproc/video 和 tegra_hal，不包含 openjp2。 | 去 OpenCV 化必须单独证明性能和功能不下降，A03 不替换光流实现。 |
| A03-Q004 | libjpeg-turbo 来源 | 已建立 `3.1.4` recipe，固定上游 tag commit，并通过 PetaLinux 组件级构建验证。 | PetaLinux 应用 recipe 已切换，后续快速回归未发现运行库问题。 |
| A03-Q005 | YOLOv6Lite-S 模型资产 | 本地 NCNN 模型文件已按运行输入资产纳入 package 路径和 hash 管理。 | 模型来源、转换链和精度治理移入独立模型资产任务，不阻塞当前运行代码冻结。 |
| A03-Q006 | 参考编写代码边界 | 用户说明多数资源来自 GitHub 下载或参考编写。 | 需要确认是否存在直接复制第三方源码片段但未放入 `third_party/` 的情况。 |
| A03-Q007 | 量产/调试态 SSH | `ssh-access-dev` 明确是调试态。 | 后续量产态是否禁用 debug 用户、是否切换设备唯一 key，需要用户决策。 |

## 8. A03 后续执行顺序

1. 完成本文档和局部三方元数据固化，作为 A03 资源治理基线。
2. 使用源码 tarball 同步 `cam-system` recipe。
3. 执行 `petalinux-build -c cam-system` 和必要的 image 构建。
4. 执行一次不超过 20 分钟的上板回归，重点覆盖模型加载、YOLO 检测框、tracking、MJPEG/WS、前端开关语义和推理耗时。
5. 继续补齐模型来源、导出流程、精度基线和许可证，提升模型资产 provenance。
6. 根据 A03-Q006 确认是否存在直接复制但未登记的参考代码片段。

## 9. A03 接受准则

A03 完成时必须满足：

- 本文档中所有 P1 项要么提升到 P2/P3，要么保留为明确偏差并说明接受理由。
- 不能出现来源、许可证、用途完全未知的 P0 资源。
- 任何新增三方资源必须先进入本文档，再进入构建或打包路径。
- 任何从 GitHub 引入的源码 recipe 必须固定 commit，不使用漂移的 `AUTOREV`。
- 任何预构建二进制或模型资产必须有 sha256。
- 若三方资源变更影响链接、rootfs 或模型路径，必须通过本地构建，并在需要时安排上板测试。
