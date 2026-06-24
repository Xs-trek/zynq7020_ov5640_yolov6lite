# Core0/Core1 与 IRQ Affinity 测试计划

版本：0.1

## 1. 背景

当前用户态程序线程亲和性为：

```text
main/network 线程 -> Core0
capture/control 线程 -> Core0
YOLO 线程 -> Core1
```

历史板端快照和 2026-06-01 短测显示，`e0002000.usb` 等 USB/WiFi 相关中断及 `TASKLET` softirq 主要落在 CPU0；前端 MJPEG 流活跃时，Core0 上的 USB/网络中断会抬升 YOLO 单帧时间约 10-14 ms。当前拓扑将 YOLO 移到 Core1，目标是让默认落在 CPU0 的 WiFi/USB 中断、网络事件循环、capture/JPEG/tracker 与 YOLO 推理分离。

## 2. 本阶段边界

本阶段默认只测试线程拓扑变化，不修改 IRQ affinity。真正写 `/proc/irq/*/smp_affinity` 必须有单独假设、在上板测试窗口内执行，并且测试后恢复。

## 3. 板端采集脚本

脚本：

```text
scripts/board/core_irq_probe.sh
```

推荐上板后复制到 `/tmp` 执行：

```sh
chmod 755 /tmp/core_irq_probe.sh
MODE=collect DURATION=60 INTERVAL=1 /tmp/core_irq_probe.sh /tmp/core_irq_swapped_idle
MODE=collect DURATION=60 INTERVAL=1 /tmp/core_irq_probe.sh /tmp/core_irq_swapped_stream
```

## 4. 需要采集的指标

- `/proc/interrupts`：确认 USB、WiFi、MMC、串口、I2C 等中断是否仍主要落在 Core0。
- `/proc/irq/*/smp_affinity`：确认哪些 IRQ 支持迁移、迁移后是否生效。
- `/proc/softirqs`：观察 NET_RX、TIMER、SCHED 等软中断是否集中到 Core0。
- `/proc/stat`：计算 CPU0/CPU1 总体忙闲变化。
- `/proc/<pid>/task/*/status`：确认线程 `Cpus_allowed_list` 是否符合设计。
- `/proc/<pid>/task/*/stat`：记录线程 utime/stime 和 last_cpu。
- `ps`：保留进程、内核线程、wpa_supplicant、udhcpc、dropbear 状态。

## 5. 判定逻辑

只有同时满足以下条件，才认为线程拓扑调换具备继续固化价值：

- YOLO 线程实际绑定到 CPU1，main/network 与 capture/control 实际绑定到 CPU0。
- 前端 MJPEG 流活跃时，YOLO 周期相比旧拓扑下降或至少不再出现明显中断相关抬升。
- Core0 未因 capture/JPEG/network/IRQ 聚合出现前端卡死、黑屏、WS 状态异常或命令响应明显退化。
- 前端画面、YOLO 开关、SSH/WiFi 不出现功能退化。

如果 YOLO 周期无明显改善，或前端/WiFi 稳定性下降，则不固化该线程拓扑。

## 6. 上板时间控制

单次上板窗口控制在 20 分钟以内：

```text
0-3 min   启动、登录、确认 IP/SSH
3-6 min   Camera+YOLO 无前端流 collect
6-9 min   Camera+YOLO+MJPEG stream collect
9-12 min  关闭 Camera/YOLO，恢复普通后台运行并快速功能检查
12-20 min  预留给前端人工确认或异常恢复
```

如果 WiFi 连接、SSH、前端任一项异常，先关闭 Camera/YOLO 并停止测试。
