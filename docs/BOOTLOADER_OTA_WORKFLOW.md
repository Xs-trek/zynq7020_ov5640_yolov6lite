# cam_system Bootloader OTA 工作流

版本：0.3

## 1. 目标

A05 实现调试态可靠最小 OTA，运行期更新范围限定为 `image.ub`。`image.ub` 内含 kernel、device tree 和 initramfs/rootfs，因此覆盖当前用户态程序、rootfs package 和内核配置类变更。`BOOT.BIN`、FSBL、U-Boot 本体和 bitstream 暂不纳入无人值守 OTA；这些文件损坏会发生在更早启动阶段，需要独立启动介质冗余设计。`boot.scr` 是 OTA 钩子的启动脚本依赖项，由项目纳入 PetaLinux recipe 管理，但不作为常规 OTA 载荷频繁更新。

## 2. 当前硬件与启动约束

- 当前 PetaLinux 工程使用 initramfs：`root=/dev/ram0 rw`。Linux 运行后根文件系统在内存中，SD FAT 启动分区可被用户态挂载并改写。
- SD FAT 启动分区保留原 `/image.ub` 作为最终兜底镜像。
- OTA 使用 `/ota/slot_a/image.ub` 和 `/ota/slot_b/image.ub` 作为 A/B 槽。
- 正常 `good` 状态只使用 PetaLinux `boot.scr` 已支持的 `/uEnv.txt` 钩子，不生成持久 `/uboot.env`。
- PetaLinux 2022.2 默认 `boot.scr` 中 `if test -n $uenvcmd; then` 会在导入长 `uenvcmd` 后把命令展开为多个 `test` 参数，导致 `uenvcmd` 不执行。项目通过 `u-boot-zynq-scr.bbappend` 把该行修正为 `if test -n "${uenvcmd}"; then`，并用 `validate_boot_scr.sh` 校验生成物。
- `pending` 首次启动时，`uenvcmd` 不依赖 U-Boot `saveenv`。它先把 `/ota/state/uEnv.rollback` 写回根目录 `/uEnv.txt`，再重新读取 `/uEnv.txt` 并校验文件大小，确认 rollback 钩子已预置后才允许启动 pending 槽。
- 当前 U-Boot FAT 写支持由 `CONFIG_FAT_WRITE=y` 提供；本方案依赖 `fatload + fatwrite + iminfo`，不依赖持久 `/uboot.env`。

## 3. 可靠性设计

OTA 状态机：

- `good`：当前槽已验证，`/uEnv.txt` 直接启动该槽。
- `pending`：新槽首次试启动，U-Boot 会先把 `/uEnv.txt` 覆盖为 rollback 钩子，再启动新槽。
- `rollback`：pending 槽未执行 `mark-good` 时，下一次启动会由 rollback 钩子切回上一槽。
- `disabled`：无持久 OTA 状态文件，默认 `/image.ub` 启动。该状态由 `cam-ota disable` 进入。

写入顺序：

1. 写非活动槽 `image.ub.tmp`。
2. 对临时文件做 SHA-256 校验。
3. 原子改名为 `image.ub`。
4. 写槽位 `manifest.env` 和全局 `current.env`。
5. pending 状态先写 `/ota/state/uEnv.rollback`。
6. 原子更新 `/uEnv.txt` 和 `/ota/state/uEnv.txt`。
7. 在 `good/rollback` 状态删除 `/uboot.env` 和 `/uboot-redund.env`，保证默认 U-Boot 环境生效。
8. `sync` 后等待人工或脚本重启。

回退链路：

- 正常启动：默认 `boot.scr` 导入 `/uEnv.txt` 后执行 `uenvcmd`，尝试从当前 OTA 槽加载 FIT；若槽加载失败，`uenvcmd` 把 `fitimage_name` 改回 `image.ub`，让默认脚本继续兜底启动。
- `pending` 首次启动：`uenvcmd` 先加载 `/ota/state/uEnv.rollback` 到内存，再用 `fatwrite` 覆盖根目录 `/uEnv.txt`。随后重新加载 `/uEnv.txt`，确认读取大小与 rollback 钩子大小一致，才继续启动 pending 槽。
- 所有 OTA 槽位 FIT 在 `bootm` 前必须先通过 U-Boot `iminfo`。pending bootargs 只能在 pending FIT `iminfo` 通过后设置，避免 pending FIT 启动失败后默认 `/image.ub` 携带错误的 pending cmdline。
- pending 槽未执行 `mark-good` 的情况下发生复位：下一次 U-Boot 导入的 `/uEnv.txt` 已经是 rollback 钩子，会尝试上一槽，最后尝试原始 `/image.ub`。
- 所有 fallback 到原始 `/image.ub` 的路径都会先恢复 baseline `bootargs`，避免 Linux 把 fallback 启动误判为 OTA 槽启动。
- `pending` 启动时 U-Boot 尝试启动 Cadence watchdog；Linux 早期 `cam-ota-bootguard` init 脚本若看到 `cam_ota_state=pending`，会启动 watchdog keepalive。`cam-ota mark-good` 会停止 keepalive，并把 `/uEnv.txt` 固化到当前槽。
- 若 rollback 钩子已进入上一槽并带 `cam_ota_state=rollback`，`cam-ota-bootguard` 会调用 `cam-ota mark-good rollback-recovered` 固化回退结果。
- 若 pending 槽在 U-Boot 阶段加载失败并 fallback 到原始 `/image.ub`，Linux cmdline 不含 OTA slot；`cam-ota-bootguard` 会检测 `state=pending` 并执行 `cam-ota rollback`，如果 rollback 目标不可用则执行 `cam-ota disable` 回到纯默认启动。
- `cam-ota install` 只允许从 `good` 状态进入 `pending`。若已有 pending 或 rollback 未收敛，必须先 `mark-good`、`rollback` 或 `disable`。

## 4. 本地同步与构建

从 `cam_system` 工作空间执行：

```sh
./scripts/build/validate_ota_env.sh
./scripts/build/validate_boot_scr.sh
./scripts/petalinux/stage_ota.sh
./scripts/petalinux/build_boot_scr.sh
```

`stage_ota.sh` 会自动调用 `validate_ota_env.sh` 和 `validate_boot_scr.sh`。`build_boot_scr.sh` 用当前 PetaLinux 构建目录中的 `u-boot-zynq-scr` 生成结果派生调试用 `images/linux/boot.scr`；完整 PetaLinux 构建则由 `u-boot-zynq-scr_%.bbappend` 生成同样的修正。

`validate_ota_env.sh` 会在本机模拟 `init` 和 `install pending`，验证：

- `good` 状态不会生成 `/uboot.env` 和 `/uboot-redund.env`。
- `/uEnv.txt` 与 `/ota/state/uEnv.txt` 内容一致。
- `good` 状态的 `uenvcmd` 只尝试当前槽，并在失败时回到默认 `image.ub`。
- `pending` 状态会生成 `/ota/state/uEnv.rollback`。
- `pending` 状态的 `uenvcmd` 会先用 `fatwrite` 预置 rollback 钩子，再重新读取 `/uEnv.txt` 校验大小，然后才启动 pending 槽。
- pending、good、rollback 槽位启动前都会先执行 `iminfo`。
- rollback 钩子会尝试上一槽，并在失败时尝试原始 `image.ub`。
- pending bootargs 不允许早于 pending FIT `iminfo` 设置。
- 所有默认 fallback 都会恢复 baseline `bootargs`。
- pending 状态下再次 `install` 会被拒绝。
- `cam-ota disable` 会移除 `/uEnv.txt`、`/ota/state/uEnv.txt`、`/ota/state/current.env`、rollback 钩子和持久 U-Boot env 文件。

`validate_boot_scr.sh` 会验证：

- `u-boot-zynq-scr.bbappend` 存在并包含对 `uenvcmd` 引用保护的修正。
- 从当前 PetaLinux `u-boot-zynq-scr` 生成目录派生出的 `boot.scr` 中存在 `if test -n "${uenvcmd}"; then`。
- 生成物中不存在 `if test -n $uenvcmd; then`。

该脚本同步：

```text
project-spec/meta-user/recipes-bsp/u-boot/u-boot-xlnx_%.bbappend
project-spec/meta-user/recipes-bsp/u-boot/u-boot-zynq-scr.bbappend
project-spec/meta-user/recipes-bsp/u-boot/files/bsp.cfg
project-spec/meta-user/recipes-bsp/u-boot/files/platform-top.h
project-spec/meta-user/recipes-apps/cam-ota/cam-ota.bb
project-spec/meta-user/recipes-apps/cam-ota/files/cam-ota
project-spec/meta-user/recipes-apps/cam-ota/files/cam-ota-bootguard
```

构建顺序：

```sh
cd ~/workdir/plnx_prj/zynq_plnx
petalinux-build -c u-boot-xlnx
petalinux-build -c cam-ota
petalinux-build
petalinux-package --boot --force \
  --fsbl images/linux/zynq_fsbl.elf \
  --fpga images/linux/system.bit \
  --u-boot
```

## 5. 板端命令

首次启用：

```sh
cam-ota init
cam-ota status
```

只读取持久状态：

```sh
cam-ota state
```

安装新镜像：

```sh
cam-ota install /tmp/image.ub <version>
sync
reboot
```

启动后验证通过：

```sh
cam-ota mark-good <version>
```

人工回退：

```sh
cam-ota rollback
sync
reboot
```

## 6. 上板验证边界

上板前必须通过本机脚本验证和 PetaLinux 构建验证。上板单轮不超过 20 分钟。

首轮上板只验证：

1. 默认 `/image.ub` 启动不进入 OTA 槽。
2. `cam-ota init` 后重启可从 `slot_a` 启动。
3. 安装同一 `image.ub` 到 `slot_b` 后，pending 启动成功。
4. `cam-ota mark-good` 清除 pending，并恢复 good 启动钩子。
5. 再次安装 pending 后不执行 `mark-good`，二次重启必须自动进入上一槽 rollback。
6. `cam-ota disable` 后重启必须回到默认 `/image.ub`。

故障注入验证在上述路径稳定后单独执行，避免在首轮引入长时间反复重启。
