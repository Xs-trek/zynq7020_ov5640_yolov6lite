# 8812bu 与 WiFi Init 工作流

版本：0.3

## 1. 当前目标

本阶段把 `8812bu` 从“外部预编译 `.ko` 打包”迁移为“固定上游源码提交的 Yocto kernel module recipe”，同时重构 `wifi-init` 启动脚本。脚本保留原来的串口交互输入 SSID/密码流程，并支持从本地配置文件读取 WiFi 参数以减少人工启动步骤。

## 2. 上游来源

当前源码 recipe 固定：

```text
https://github.com/morrownr/88x2bu-20210702.git
SRCREV=fecac340fb117eb979f4bb6d28e29730384c382b
```

本地已有 `.ko` 的 `modinfo` 版本为：

```text
v5.13.1-20-gbd7c7eb9d.20210702_COEX20210316-18317b7b
```

上游仓库 `include/rtw_version.h` 中版本串一致。当前 recipe 通过 `CONFIG_BT_COEXIST=n` 保持模块名为 `8812bu.ko`，避免影响既有镜像包名和启动脚本。

## 2.1 A03 资源治理状态

`8812bu` 属于 `docs/THIRD_PARTY_COMPONENTS.md` 中的 R6 内核外模块。当前治理等级为 P3，依据如下：

| 项目 | 当前状态 |
| --- | --- |
| 上游仓库 | `https://github.com/morrownr/88x2bu-20210702.git` |
| 分支 | `main` |
| 固定提交 | `fecac340fb117eb979f4bb6d28e29730384c382b` |
| 许可证 | `GPL-2.0-only`，由 recipe 的 `LIC_FILES_CHKSUM` 绑定上游 `LICENSE` 文件。 |
| 构建方式 | Yocto `inherit module`，随当前 PetaLinux kernel ABI 构建。 |
| 部署路径 | `${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/8812bu.ko` |
| 启动入口 | recipe 声明 Yocto module autoload；`wifi-init` 在自动加载未生效时用 `modprobe`/`insmod` 兜底。 |

该方式是当前项目的本地化策略，不等同于通用 Linux 发行版推荐做法。上游 README 已说明在较新 kernel 上应优先考虑标准 in-kernel `rtw88` 路线；但当前板端 PetaLinux kernel 为 5.15，且历史功能测试基于 Realtek out-of-tree 驱动，因此本阶段继续使用源码 recipe 是合理的受控选择。

A03 约束：

- 禁止恢复为手工拷贝预编译 `.ko`。
- 禁止使用漂移的 `AUTOREV`。
- 禁止在无构建记录的情况下替换上游 commit。
- 修改驱动源码或 Makefile 参数时，必须同步更新 `docs/THIRD_PARTY_COMPONENTS.md` 和本文件。
- 量产态若升级到支持稳定 `rtw88` 的 kernel，需要重新评估是否移除该 out-of-tree 驱动。

## 3. 标准同步命令

```sh
./scripts/petalinux/stage_wifi.sh
```

默认 PetaLinux 工程：

```text
~/workdir/plnx_prj/zynq_plnx
```

路径变化时：

```sh
PLNX_PROJECT=/path/to/zynq_plnx ./scripts/petalinux/stage_wifi.sh
```

## 4. 设计约束

- `IMAGE_INSTALL` 继续使用 `8812bu wifi-init`，降低 rootfs 侧变更面。
- `8812bu` recipe 使用 `inherit module`，让 Yocto 使用当前内核构建目录和 ABI。
- `wifi-init` 不保存明文密码；`wpa_passphrase` 生成的临时配置在 `wpa_supplicant` 启动后删除。
- `wifi-init` 校验 WPA-PSK 长度，避免短密码错误文本被写入 `/tmp/wpa_supplicant.conf`。
- `8812bu` recipe 通过 `KERNEL_MODULE_AUTOLOAD` 声明启动自动加载，通过 `KERNEL_MODULE_PROBECONF` 固化模块参数。
- `wifi-init` 优先接受 Yocto/系统已加载的模块；若未加载，则先尝试 `modprobe`，最后按 `/lib/modules/*/extra/8812bu.ko` 路径 `insmod` 兜底。
- 旧预编译 `.ko` recipe 目录不再保留；反复执行 `stage_wifi.sh` 会清理 `meta-user/recipes-apps/8812bu`，避免源码 recipe 和旧二进制 recipe 并存。
- `wifi-init` 优先读取 `/etc/wifi-init.conf`、`/etc/wifi.conf`、SD 启动分区中的 `wifi-init.conf`/`wifi.conf`，配置无效或不存在时回退到串口交互输入。
- `wifi-init` 支持配置文件中的 `PSK=<64 hex>` 或 `PASSPHRASE=<8..63 chars>`；推荐调试时在 SD 启动分区放 `PSK`，避免直接保存明文口令。
- `wifi-init` 等待 `wpa_supplicant` 进入 `COMPLETED` 后再启动 `udhcpc`，减少 DHCP 早于无线关联完成导致的偶发失败。
- `wifi-init` 不引入 NetworkManager、ConnMan、systemd-networkd 等守护框架；当前板端只需要固定 STA 入网，`wpa_supplicant + udhcpc + init.d` 是更小的本地闭环。
- `wifi-init` 会从 `/proc/mounts` 发现 `/dev/mmcblk0p1` 的实际挂载点。PetaLinux 运行时常见路径是 `/run/media/mmcblk0p1`，不能假定固定为 `/mnt` 或 `/run/media/boot`。
- 系统启动阶段默认不进入串口交互，避免 WiFi 配置缺失时阻塞 boot。需要手动输入时执行 `WIFI_INTERACTIVE=1 /etc/init.d/S50wifi restart`。

## 4.1 驱动配置边界

当前 recipe 和启动脚本只冻结必要配置：

| 配置 | 位置 | 当前值 | 理由 |
| --- | --- | --- | --- |
| `CONFIG_BT_COEXIST` | recipe `EXTRA_OEMAKE` | `n` | 当前只使用 WiFi STA，不需要蓝牙共存路径；保持模块名和构建输出稳定。 |
| `CONFIG_RTW_LOG_LEVEL` | recipe `EXTRA_OEMAKE` | `0` | 降低内核日志噪声，避免影响调试输出和热路径观测。 |
| `KERNEL_MODULE_AUTOLOAD` | recipe | `8812bu` | 交给 Yocto/rootfs 启动流程尝试自动加载模块。 |
| `KERNEL_MODULE_PROBECONF` / `module_conf_8812bu` | recipe | `options 8812bu rtw_drv_log_level=0` | 用标准模块配置固化运行时日志等级。 |
| `rtw_drv_log_level` | `wifi-init` `insmod` 兜底参数 | `0` | 当自动加载和 `modprobe` 未生效时，仍沿用低日志等级。 |
| `WIFI_COUNTRY` | `wifi-init` 环境变量 | `CN` | 当前本地调试环境使用；若部署区域变化必须显式配置。 |

未冻结内容：

- 不在本阶段改写驱动内部电源管理、USB 模式、LED、AP/monitor/p2p 等功能。
- 不在本阶段写 `/proc/irq/*/smp_affinity`；IRQ 亲和性属于单独测试项，见 `docs/CORE_IRQ_AFFINITY_TEST_PLAN.md`。
- 不引入 DKMS。PetaLinux 镜像构建中，Yocto kernel module recipe 比 DKMS 更符合 rootfs/package 管理。

## 5. 可选 WiFi 配置文件

如果希望上电后自动连接 WiFi，可以在 SD 卡 FAT 启动分区放置 `wifi-init.conf`：

```text
SSID=your-ssid
PASSPHRASE=your-password
```

更推荐写入预计算的 WPA PSK：

```text
SSID=your-ssid
PSK=64_hex_wpa_psk
```

约束：

- `PASSPHRASE` 必须是 WPA-PSK 8..63 字符口令。
- `PSK` 是等价入网凭据，不能公开；但它避免在 SD 配置文件中直接出现原始口令。
- 配置文件适合当前本地受控调试场景，不应作为外场量产安全方案。
- 如果配置存在但关联失败，脚本会在串口交互启用时回退到手动输入。系统启动默认关闭交互，避免阻塞启动流程。

## 5.1 调试态与量产态差异

当前 `wifi-init` 是调试态最小闭环：

- 目标是上电后尽快进入本地局域网，供 SSH、前端和 Codex 远程调试使用。
- 默认不保存运行时明文密码。
- 允许 SD 启动分区提供临时 `wifi-init.conf`，便于 RAM rootfs 场景下无持久存储启动。

量产态应重新设计：

- WiFi 凭据应放在可靠持久存储、设备唯一配置区或受保护配置分区。
- 调试账号和调试 key 不应默认开放。
- 若需要在线配置 WiFi，应通过受控本地管理接口或安全 provisioning 流程，而不是依赖启动串口交互。
- 若系统切换到 eMMC 或 A/B rootfs，WiFi 配置也应纳入配置分区和 OTA 回滚策略。

## 6. 验证命令

```sh
./scripts/petalinux/stage_wifi.sh
petalinux-build -c 8812bu
petalinux-build -c wifi-init
petalinux-build
petalinux-package --boot --force --fsbl images/linux/zynq_fsbl.elf --fpga images/linux/system.bit --u-boot
```

本阶段本地构建通过后，仍必须上板验证：

- 串口交互输入 SSID/密码。
- `8812bu` 模块加载成功。
- `wpa_supplicant` 不再出现 `Passphrase must be 8..63 characters` 被当作配置行解析的问题。
- DHCP 获取到预期局域网 IP。
- SSH 可用。

额外 A03 验证：

- `modinfo /lib/modules/$(uname -r)/extra/8812bu.ko` 能显示期望版本和 GPL 许可证。
- `/proc/modules` 中存在 `8812bu`。
- `dmesg` 中没有持续刷屏的 8812bu 日志。
- `/etc/init.d/S50wifi status` 能显示 `wpa_supplicant` 状态或明确失败原因。
- 若执行 IRQ 亲和性实验，测试后必须恢复原始 `/proc/irq/*/smp_affinity`。
