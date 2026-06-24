# cam_system PS_KEY1 启动工作流

## 1. 目标

A06 新增一个最小启动辅助组件 `cam-launcher`，用于在调试态不依赖 SSH/Codex 的情况下，通过开发板物理按键 PS_KEY1 启动 `/usr/bin/cam_system`。

本阶段不改变 `cam_system` 的 HTTP、REST、MJPEG、WebSocket、camera/yolo/tracking 语义，也不让前端负责启动进程。

## 2. 设计边界

| 项 | 设计 |
| --- | --- |
| 硬件入口 | PS_KEY1，当前工程/实物上板实测对应 Zynq PS MIO10，低有效。 |
| 内核抽象 | `gpio-keys` 输入设备。 |
| pinmux | `gpio-keys` 绑定 `pinctrl_ps_key1_default`，将 `gpio0_10_grp` 切到 `gpio0`。 |
| 用户态入口 | `/usr/sbin/cam-launcher` 读取 `/dev/input/event*`。 |
| key code | `KEY_PROG1 = 148`。 |
| 触发动作 | press 事件启动 `/usr/bin/cam_system`。 |
| 重复按键 | 若 `cam_system` 已运行，仅记录并忽略。 |
| 停止动作 | 不提供按键停止，避免误触导致前端断流。 |
| 前端协议 | 不变；前端仍只连接已运行的 `cam_system:8080`。 |

## 3. 为什么使用 gpio-keys

当前内核配置已经启用 `CONFIG_KEYBOARD_GPIO`、`CONFIG_INPUT_EVDEV` 和 `CONFIG_GPIO_ZYNQ`。按键属于板级硬件输入，使用 `gpio-keys` 后，应用层不依赖 Linux sysfs GPIO 编号，也不依赖 `gpiochip` base 是否为 903。

Linux GPIO sysfs ABI 已被内核文档标记为 deprecated。当前实现只在 device tree 中声明 GPIO，由内核 `gpio-keys` 驱动完成消抖、边沿处理和 input event 发布，用户态只消费标准 input event。

上板实测表明，按下当前板上的 PS_KEY1 时，Zynq GPIO bank0 的 bit10 在 `1/0` 之间变化，而 bit16 不变化；因此当前工程以 MIO10 作为 PS_KEY1 的软件事实来源。若后续更换底板、XSA 或约束文件，必须重新执行 GPIO bank 扫描确认按键实际 MIO。

## 4. 文件与 recipe

受控源码侧：

```text
packaging/petalinux/recipes-bsp/device-tree/
packaging/petalinux/recipes-apps/cam-launcher/
scripts/build/validate_cam_launcher.sh
scripts/petalinux/stage_cam_launcher.sh
```

板端目标路径：

```text
/usr/sbin/cam-launcher
/etc/init.d/cam-launcher
/run/cam-system/cam-launcher.pid
/tmp/cam-launcher.log
/tmp/cam_system.log
```

## 5. 构建流程

从 `cam_system` 工作空间执行：

```sh
./scripts/build/validate_cam_launcher.sh
./scripts/petalinux/stage_cam_launcher.sh
cd ~/workdir/plnx_prj/zynq_plnx
petalinux-build -c device-tree
petalinux-build -c cam-launcher
petalinux-build
```

`stage_cam_launcher.sh` 会把 device-tree 片段和 `cam-launcher` recipe 同步到 PetaLinux `project-spec/meta-user`，并把 `cam-launcher` 加入 `petalinux-image-minimal.bbappend` 的 `IMAGE_INSTALL`。

## 6. 上板验证

上板后先不启动 `cam_system`，按顺序验证：

1. `/usr/sbin/cam-launcher` 和 `/etc/init.d/cam-launcher` 存在。
2. `cam-launcher` 进程运行。
3. `/proc/bus/input/devices` 中存在 `gpio-keys`，并包含 `KEY_PROG1` 对应能力。
4. `/tmp/cam-launcher.log` 中出现正在监听 `/dev/input/eventX`。
5. `/proc/interrupts` 中 `cam-ps-key1` 计数在按键后增加。
6. 按下 PS_KEY1 后，`pidof cam_system` 返回一个 PID。
7. 再按一次 PS_KEY1，`cam_system` PID 不变化，不产生第二个进程。
8. 前端连接 8080 后，camera/yolo/tracking 行为与 A05 固化状态一致。

如果第 3、4 或 5 步失败，优先检查 MIO10 pinmux、device tree 是否进入本次 FIT、以及 `gpio-keys` probe 日志；不得直接把问题归因到用户态 launcher。

## 7. 已知限制

- 本地无法证明 PS_KEY1 物理链路一定产生输入事件，必须上板用 input event 和 `/proc/interrupts` 验证。
- `cam-launcher` 不负责监控或重启崩溃的 `cam_system`。它只提供物理启动入口，不是完整 supervisor。
- 当前 RAM rootfs 模式下 `/tmp` 和 `/run` 均为易失状态，日志和 pidfile 下电后丢失，这是调试态可接受行为。
