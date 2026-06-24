# A08 项目规范冻结检查记录

版本：0.1
状态：本地合并检查完成，等待 OV5640 驱动化方案讨论

## 1. 检查范围

本记录用于把 A03 到 A08 的阶段性结果收敛成当前可执行基线，避免旧文档中的历史“待上板”“待回归”被误读为当前阻塞项。

本轮检查覆盖：

- 第一方代码目录和模块边界是否仍符合顶层规格。
- PetaLinux/Yocto recipe 是否覆盖当前运行所需组件。
- WiFi、SSH、OTA、PS_KEY1、WebSocket 的项目化状态是否有未登记偏差。
- 历史文档与当前冻结口径是否冲突。

本轮不覆盖：

- 不做运行代码审查或静态分析；该项按用户要求放在最后执行。
- 不重新讨论模型来源、导出链或训练精度；模型文件在本阶段只作为运行输入资产处理。
- 不执行上板测试；本轮没有修改运行代码、recipe 或启动脚本。

## 2. 当前冻结基线

| 项目 | 当前状态 | 依据 |
| --- | --- | --- |
| Git 运行代码基线 | 冻结在 `cc0f6e0 Solidify restricted websocket protocol` 之后的文档变更阶段 | 本轮未修改 `src/`、`include/`、`packaging/` 或 `scripts/`。 |
| 第一方应用结构 | 符合当前顶层规格 | `src/app`、`src/platform`、`src/capture`、`src/media`、`src/network`、`src/vision`、`src/control` 分层仍保持。 |
| rootfs/package 路径 | 符合当前调试规范态 | `/usr/bin/cam_system`、`/usr/share/cam-system/models`、`/usr/share/cam-system/manifest` 已由 `cam-system.bb` 约束。 |
| 三方运行依赖 | 已 recipe 化或登记治理 | `ncnn`、`libjpeg-turbo`、`cam-opencv`、`8812bu`、`cJSON` 已在 A03/A04 文档和 recipe 中登记。 |
| WiFi/SSH 调试入口 | 符合调试态规范 | `wifi-init`、`ssh-access-dev` 纳入 recipe；禁止把该调试态默认开放策略当作量产策略。 |
| OTA | 已实现调试态可靠更新链路 | `cam-ota`、bootguard、双 boot slot 设计已纳入 recipe 和脚本；量产安全策略仍需另行设计。 |
| PS_KEY1 启动入口 | 当前硬件已快速验证 | `gpio-keys` + `cam-launcher` 已替代用户态猜测 gpiochip base；MIO10 来自当前板端实测。 |
| WebSocket | 已固化为受限协议栈 | 协议边界以 `docs/WEBSOCKET_PROTOCOL.md` 为准，不声明通用 WebSocket 服务器能力。 |
| OV5640/PL video | 功能冻结，驱动化待讨论 | 当前仍是用户态 HAL + PL 固定链路；不在未决策前改为 V4L2 或 DMAEngine。 |

## 3. 历史文档状态处理

以下旧状态不再作为当前阻塞项：

- A03 文档中针对三方 recipe 的“待上板回归”属于当时的历史门禁；当前 package 化链路已经过后续快速回归，后续只有在改动 recipe、rootfs、boot image 或相关运行代码时才重新执行。
- A06 文档中 PS_KEY1 的“待实测”已由后续板端取证修正为 MIO10 路径；硬件/XSA 变化时才需要重测。
- A07 文档限定 WebSocket 是受限协议栈；这不是当前缺陷，而是有意裁剪。
- 模型文件在本阶段按运行输入资产管理。模型来源、转换链、训练或精度治理可以作为单独资产治理任务，但不阻塞当前用户态运行代码冻结。

## 4. 历史偏差状态

本文件记录 A08 当时的冻结检查口径。当前有效偏差清单以
`docs/CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md` 为准。

| ID | A08 当时偏差 | 当前处理 |
| --- | --- | --- |
| DEV-001 | 含 float 的结构体仍存在 `memset` portability 告警 | 已在后续代码审查修复；不再作为当前偏差。 |
| DEV-002 | 部分跨线程 flag 仍使用 `volatile` | 已在后续代码审查迁移为 atomic accessor；不再作为当前偏差。 |
| DEV-003 | HTTP/WS 无认证和 TLS | 仍为当前调试态偏差；仅允许可信局域网调试，不对公网暴露。 |
| DEV-004 | camera 控制面仍在用户态 HAL | A09 后已迁移到 OV5640 V4L2 控制面；后续正式程序已移除旧用户态 I2C/GPIO fallback。 |
| DEV-005 | 三方 `cJSON.c` 在全项目 cppcheck 下存在上游源码告警 | 当前作为顶层规格 `DEV-001` 三方偏差治理，不直接改写上游源码。 |

## 5. 下一讨论点

下一步应进入 OV5640 驱动化方案讨论。讨论前必须维持两个约束：

- 不破坏当前前端、PS_KEY1、OTA、WiFi 和 package 化闭环。
- 不承诺驱动化带来性能提升；先以资源所有权、可观测性、可维护性和学习价值为目标。
