# A09 OV5640 控制面驱动化说明

版本：0.2
状态：历史集成记录；当前正式程序已进一步收敛为 V4L2-only，旧用户态 I2C/GPIO 实现仅通过 git 历史追溯

## 1. 目标

A09 的目标是把 OV5640 的控制面从用户态裸 I2C/GPIO 迁移到内核驱动，同时保持现有视频数据面不变：

```text
OV5640 DVP -> PL ov5640_to_lcd -> v_frmbuf_wr -> DDR -> cam_system
```

本阶段不实现完整 V4L2 capture，不创建 `/dev/videoX`，不接管 `v_frmbuf_wr`，不改变前端协议、帧缓存地址、颜色格式或 YOLO/跟踪流程。

## 2. 最终型态

当前实现采用项目专用 `cam-ov5640-control` I2C kernel module：

- 设备树创建 `camera@3c`，绑定 `compatible = "cam-system,ov5640-control"`。
- 驱动通过 `devm_gpiod_get_optional()` 获取 `powerdown-gpios` 和 `reset-gpios`。
- 驱动通过 I2C 写入当前项目已验证的 640x480 RGB565 初始化表。
- 驱动注册最小 `v4l2_device` 和 `V4L2_SUBDEV_FL_HAS_DEVNODE` subdev，并调用 `v4l2_device_register_subdev_nodes()` 生成 `/dev/v4l-subdevX`。
- 用户态 `hal_ov5640.c` 扫描 `/sys/class/video4linux/v4l-subdev*/name`，通过 V4L2 private controls 调用初始化、standby、ISP offset 和 scaler enable。
- 默认运行路径要求 V4L2 control backend 存在并初始化成功；正式程序不再包含旧用户态 I2C/GPIO fallback。

这个型态是控制面驱动化，不是完整 camera media graph。它的设计目的，是先收回 sensor I2C/GPIO 资源所有权，同时避免一次性改动 PL frmbuf 数据路径。正常运行时用户态不再暴露单寄存器裸读写接口。

## 3. 控制接口

驱动和用户态共享以下 private control ID：

```c
CAM_OV5640_CID_BASE          = V4L2_CID_USER_BASE + 0x2000
CAM_OV5640_CID_INITIALIZE    = CAM_OV5640_CID_BASE + 0
CAM_OV5640_CID_STANDBY       = CAM_OV5640_CID_BASE + 1
CAM_OV5640_CID_ISP_OFFSET    = CAM_OV5640_CID_BASE + 2
CAM_OV5640_CID_SCALER_ENABLE = CAM_OV5640_CID_BASE + 3
```

`CAM_OV5640_CID_ISP_OFFSET` 使用一个 32-bit integer 打包：

```text
bits[27:16] = x offset
bits[11:0]  = y offset
```

驱动内部仍使用 OV5640 group write：

```text
0x3212 = 0x03
0x3810/0x3811 = x offset
0x3812/0x3813 = y offset
0x3212 = 0x13
0x3212 = 0xa3
```

## 4. 设备树边界

`system-user.dtsi` 中新增：

- `ov5640-xclk` fixed-clock，当前频率为 24 MHz。
- `i2c@41600000/camera@3c`，包含 clock、powerdown GPIO、reset GPIO。

本节记录 A09 当时的控制面迁移状态；后续阶段已把采集数据面也迁移到 V4L2 capture，当前状态以 `CAM_SYSTEM_SOFTWARE_TOP_LEVEL_SPEC.md` 为准。

## 5. 构建与同步

同步到 PetaLinux 工程：

```sh
./scripts/petalinux/stage_ov5640_control.sh
```

模块级构建：

```sh
petalinux-build -p ~/workdir/plnx_prj/zynq_plnx -c cam-ov5640-control -x cleansstate
petalinux-build -p ~/workdir/plnx_prj/zynq_plnx -c cam-ov5640-control
```

完整镜像构建：

```sh
petalinux-build -p ~/workdir/plnx_prj/zynq_plnx
```

## 6. 本地验证记录

本轮本地验证结果：

- `./scripts/build/build_arm.sh`：通过。
- `./scripts/build/verify_elf.sh build/cam_system`：通过。
- `petalinux-build -c cam-system`：通过。
- `petalinux-build -c cam-ov5640-control -x cleansstate`：通过。
- `petalinux-build -c cam-ov5640-control`：通过。
- `./scripts/build/validate_ota_env.sh`：通过。
- `./scripts/build/validate_boot_scr.sh`：通过。
- rootfs manifest 包含 `kernel-module-cam-ov5640-control-5.15.36-xilinx-v2022.2`。
- rootfs tar 包含 `/lib/modules/5.15.36-xilinx-v2022.2/extra/cam-ov5640-control.ko`。
- rootfs tar 包含 `/etc/modules-load.d/cam-ov5640-control.conf`。
- `modules.dep` 包含 `extra/cam-ov5640-control.ko`。

已固化镜像：

```text
image.ub sha256 = 45064e64e51b9cb2c5b7863d1d02fdeca020aa7f18a7709b5520ad101e0e6939
BOOT.BIN sha256 = 30b9b5553efd9807005d96b5de9ac3a3e39b0495e969e45baf4f8592f39c4808
```

`BOOT.BIN` 未变化，符合 A09 不改 PL 的边界。

## 7. 上板问题归零记录

第一次上板发现 `cam-ov5640-control` 能读取 `OV5640 chip id 0x5640`，但 probe 返回 `-34`，没有生成 `/dev/v4l-subdevX`。根因是项目 private controls 使用了 `v4l2_ctrl_new_std()`；该接口用于标准 controls，private controls 应使用 `v4l2_ctrl_new_custom()`。已改为显式 `struct v4l2_ctrl_config`。

第二次上板发现 subdev 已生成，但 `cam_system` 设置 `INITIALIZE` control 时返回 `I/O error`，legacy fallback 又因 GPIO 已归内核驱动所有而不可并行回退。已按旧 HAL 行为补齐 I2C 写寄存器 3 次 retry，并在每次初始化表写入前重新执行一次硬件 reset/power sequence，同时保留失败寄存器日志。

第三次上板验证结果：

- `cam_ov5640_control` 模块加载。
- `/dev/v4l-subdev0` 生成。
- `/sys/class/video4linux/v4l-subdev0/name = cam-ov5640-control 0-003c`。
- `dmesg` 显示 `OV5640 chip id 0x5640` 和 `cam ov5640 control subdev registered`。
- `cam_system` 日志显示 `Using V4L2 backend /dev/v4l-subdev0` 和 `V4L2 control backend initialized`。
- `cam_system` 可完成 frmbuf、JPEG、YOLO、HTTP server 初始化。
- `http://<board-ip>:8080/` 返回 HTTP 响应，说明 server 已监听；根路径 404 属于当前服务路由行为。
- 第三次测试只证明初始化和服务可启动，尚未证明 MJPEG 连续出帧。

第四次上板发现前端开启 camera 后只显示连接中或冻结一帧。A/B 验证显示卸载 `cam_ov5640_control` 后强制 legacy 路径可正常输出 MJPEG，说明 PL/frmbuf/HTTP/MJPEG 数据面未损坏。根因是 `CAM_OV5640_CID_INITIALIZE` 被实现为普通 boolean control；程序退出时会设置 standby，下一次启动再写 `INITIALIZE=1` 可能被 V4L2 control cache 判定为值未变化，驱动 `s_ctrl` 不再执行，sensor 停在 standby。修复为：

- `INITIALIZE` 改为 `V4L2_CTRL_TYPE_BUTTON`，利用 `EXECUTE_ON_WRITE` 语义确保每次写入都触发初始化。
- 用户态 V4L2 初始化完成后显式写 `STANDBY=0`。
- 默认路径不再静默 fallback，驱动缺失时 fail fast。

最终固化结果：

- `slot_a` 已执行 `cam-ota mark-good`。
- `image.ub sha256 = 45064e64e51b9cb2c5b7863d1d02fdeca020aa7f18a7709b5520ad101e0e6939`。
- 前端快速测试正常。
- 测试结束后停止 `cam_system`，仅保留 `cam-launcher`，避免 camera 空闲发热。

## 8. 前端验证计划

单次上板测试控制在 20 分钟以内。

1. 启动后确认 SSH 可达。
2. 查看模块和 probe 日志：

```sh
lsmod | grep cam_ov5640
dmesg | grep -Ei 'cam ov5640|ov5640|v4l'
ls -l /dev/v4l-subdev*
cat /sys/class/video4linux/v4l-subdev*/name
```

3. 启动 `cam_system`，日志应出现：

```text
[ov5640] Using V4L2 backend /dev/v4l-subdevX
[ov5640] V4L2 control backend initialized
```

4. 前端快速测试：

- camera on/off。
- YOLO on/off，关闭后检测框清除。
- 框选人物、启动跟踪、停止跟踪。
- 数字放大仍存在，颜色保持正常。

5. 若 V4L2 backend 失败，正常路径应 fail fast，不允许静默退回裸 I2C/GPIO。旧用户态控制实现已从正式程序移除；需要复盘时通过 git 历史查看旧实现。

## 9. 后续阶段

A09 完成后，OV5640 的控制面达到当前项目规范态。A10 才讨论完整 media graph 和 V4L2 capture：

- sensor endpoint。
- PL bridge endpoint。
- Xilinx frmbuf DMA/V4L2 capture。
- DMA buffer ownership 和 cache 同步。
- `/dev/videoX` mmap/DMABUF 用户态后端。

A10 会改变数据面所有权，风险明显高于 A09，必须单独设计和验证。
